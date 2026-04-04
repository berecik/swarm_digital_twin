use nalgebra::Vector3;

use crate::boids::{self, Boid, FlockingParams};
use crate::formation::FormationWaypointManager;

// ── Flight state machine ────────────────────────────────────────────────────

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum FlightState {
    /// Waiting on the ground.
    Disarmed,
    /// Requesting PX4 offboard mode (nav_state 14).
    OffboardRequested,
    /// Requesting arm (arming_state 2).
    Arming,
    /// Climbing to formation altitude directly above home.
    Takeoff,
    /// Flying to assigned formation slot position.
    FormUp,
    /// Following swarm-center waypoints while keeping formation.
    Formation,
    /// Returning to home position at current altitude.
    ReturnToLaunch,
    /// Descending toward ground.
    Landing,
    /// Hovering in place (simple fallback when no formation loaded).
    Loiter,
}

// ── Status and actions ──────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, Default)]
pub struct DriverStatus {
    pub nav_state: u8,
    pub arming_state: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DriverAction {
    RequestOffboard,
    RequestArm,
    PublishSetpoint(Vector3<f32>),
}

// ── Thresholds ──────────────────────────────────────────────────────────────

const FORM_UP_ACCEPT_RADIUS: f32 = 2.0;
const TAKEOFF_ACCEPT_M: f32 = 1.0;
const LAND_ALTITUDE_M: f32 = 0.5;
const RTL_ACCEPT_RADIUS: f32 = 3.0;

// ── Driver core ─────────────────────────────────────────────────────────────

pub struct DriverCore {
    pub flight_state: FlightState,
    pub status: DriverStatus,
    takeoff_height_m: f32,
    /// Latest ENU position from FMU odometry.
    position: Vector3<f32>,
    /// Home position (where the drone took off from).
    home: Vector3<f32>,
    /// Formation manager (None = simple loiter mode).
    formation: Option<FormationWaypointManager>,
    /// Flocking parameters for boids separation/cohesion.
    flocking_params: FlockingParams,
}

impl DriverCore {
    /// Create a driver in simple mode (takeoff → loiter, no formation).
    pub fn new(takeoff_height_m: f32) -> Self {
        Self {
            flight_state: FlightState::Disarmed,
            status: DriverStatus::default(),
            takeoff_height_m,
            position: Vector3::zeros(),
            home: Vector3::zeros(),
            formation: None,
            flocking_params: FlockingParams::default(),
        }
    }

    /// Create a driver with formation mission.
    pub fn with_formation(takeoff_height_m: f32, formation: FormationWaypointManager) -> Self {
        Self {
            flight_state: FlightState::Disarmed,
            status: DriverStatus::default(),
            takeoff_height_m,
            position: Vector3::zeros(),
            home: Vector3::zeros(),
            formation: Some(formation),
            flocking_params: FlockingParams::default(),
        }
    }

    pub fn update_status(&mut self, status: DriverStatus) {
        self.status = status;
    }

    /// Update the drone's current ENU position from FMU odometry.
    pub fn update_position(&mut self, pos: Vector3<f32>) {
        self.position = pos;
    }

    pub fn position(&self) -> Vector3<f32> {
        self.position
    }

    pub fn formation(&self) -> Option<&FormationWaypointManager> {
        self.formation.as_ref()
    }

    /// Run one control tick.  Returns actions to execute.
    /// `neighbors` is the list of other drones' boid states for flocking.
    pub fn tick(&mut self, neighbors: &[Boid]) -> Vec<DriverAction> {
        match self.flight_state {
            FlightState::Disarmed => {
                self.home = self.position;
                self.flight_state = FlightState::OffboardRequested;
                vec![DriverAction::RequestOffboard]
            }

            FlightState::OffboardRequested => {
                if self.status.nav_state == 14 {
                    self.flight_state = FlightState::Arming;
                    vec![DriverAction::RequestArm]
                } else {
                    vec![DriverAction::RequestOffboard]
                }
            }

            FlightState::Arming => {
                if self.status.arming_state == 2 {
                    self.flight_state = FlightState::Takeoff;
                    Vec::new()
                } else {
                    vec![DriverAction::RequestArm]
                }
            }

            FlightState::Takeoff => {
                let target = Vector3::new(self.home.x, self.home.y, self.takeoff_height_m);
                let dz = (self.position.z - self.takeoff_height_m).abs();

                if dz < TAKEOFF_ACCEPT_M {
                    if self.formation.is_some() {
                        self.flight_state = FlightState::FormUp;
                    } else {
                        self.flight_state = FlightState::Loiter;
                    }
                }
                vec![DriverAction::PublishSetpoint(target)]
            }

            FlightState::FormUp => {
                let target = match &self.formation {
                    Some(mgr) => mgr.form_up_target(),
                    None => {
                        self.flight_state = FlightState::Loiter;
                        return vec![DriverAction::PublishSetpoint(self.loiter_setpoint())];
                    }
                };

                let dx = self.position.x - target.x;
                let dy = self.position.y - target.y;
                let horiz_dist = (dx * dx + dy * dy).sqrt();

                if horiz_dist < FORM_UP_ACCEPT_RADIUS {
                    self.flight_state = FlightState::Formation;
                }
                vec![DriverAction::PublishSetpoint(target)]
            }

            FlightState::Formation => {
                let (target, mission_complete) = match &mut self.formation {
                    Some(mgr) => {
                        mgr.advance_if_reached(self.position);
                        let complete = mgr.is_complete();
                        (mgr.target_position(), complete)
                    }
                    None => {
                        self.flight_state = FlightState::Loiter;
                        return vec![DriverAction::PublishSetpoint(self.loiter_setpoint())];
                    }
                };

                if mission_complete {
                    self.flight_state = FlightState::ReturnToLaunch;
                    let rtl_target = Vector3::new(
                        self.home.x,
                        self.home.y,
                        self.takeoff_height_m,
                    );
                    return vec![DriverAction::PublishSetpoint(rtl_target)];
                }

                // Apply boids flocking correction
                let adjusted = self.apply_flocking(target, neighbors);
                vec![DriverAction::PublishSetpoint(adjusted)]
            }

            FlightState::ReturnToLaunch => {
                let target = Vector3::new(self.home.x, self.home.y, self.takeoff_height_m);
                let dx = self.position.x - target.x;
                let dy = self.position.y - target.y;
                let horiz_dist = (dx * dx + dy * dy).sqrt();

                if horiz_dist < RTL_ACCEPT_RADIUS {
                    self.flight_state = FlightState::Landing;
                }
                vec![DriverAction::PublishSetpoint(target)]
            }

            FlightState::Landing => {
                let target = Vector3::new(self.home.x, self.home.y, 0.0);
                if self.position.z < LAND_ALTITUDE_M {
                    self.flight_state = FlightState::Loiter;
                }
                vec![DriverAction::PublishSetpoint(target)]
            }

            FlightState::Loiter => {
                vec![DriverAction::PublishSetpoint(self.loiter_setpoint())]
            }
        }
    }

    /// Convenience: tick without neighbors (for simple mode / tests).
    pub fn tick_simple(&mut self) -> Vec<DriverAction> {
        self.tick(&[])
    }

    fn loiter_setpoint(&self) -> Vector3<f32> {
        Vector3::new(self.home.x, self.home.y, self.takeoff_height_m)
    }

    /// Apply boids flocking to a target setpoint.
    fn apply_flocking(&self, target: Vector3<f32>, neighbors: &[Boid]) -> Vector3<f32> {
        if neighbors.is_empty() {
            return target;
        }

        let me = Boid {
            drone_id: self.formation.as_ref()
                .map(|f| format!("drone_{}", f.drone_id()))
                .unwrap_or_else(|| "drone_0".to_string()),
            position: Vector3::new(
                self.position.x as f64,
                self.position.y as f64,
                self.position.z as f64,
            ),
            velocity: Vector3::zeros(),
            timestamp: 0,
        };

        let flock_vec = boids::calculate_flocking_vector(&me, neighbors, &self.flocking_params);

        // Scale flocking influence (keep it secondary to waypoint tracking)
        let flock_scale: f32 = 0.5;
        Vector3::new(
            target.x + (flock_vec.x as f32) * flock_scale,
            target.y + (flock_vec.y as f32) * flock_scale,
            target.z, // Keep altitude from formation, don't let boids change it
        )
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::formation::{FormationConfig, FormationPattern};

    // -- Simple mode (backwards-compatible) --

    #[test]
    fn simple_mode_transitions_to_loiter() {
        let mut core = DriverCore::new(5.0);

        assert_eq!(core.tick_simple(), vec![DriverAction::RequestOffboard]);
        assert_eq!(core.flight_state, FlightState::OffboardRequested);

        core.update_status(DriverStatus { nav_state: 14, arming_state: 0 });
        assert_eq!(core.tick_simple(), vec![DriverAction::RequestArm]);
        assert_eq!(core.flight_state, FlightState::Arming);

        core.update_status(DriverStatus { nav_state: 14, arming_state: 2 });
        assert!(core.tick_simple().is_empty());
        assert_eq!(core.flight_state, FlightState::Takeoff);

        // Simulate reaching takeoff altitude
        core.update_position(Vector3::new(0.0, 0.0, 5.0));
        let actions = core.tick_simple();
        assert_eq!(core.flight_state, FlightState::Loiter);
        assert_eq!(actions.len(), 1);
    }

    // -- Formation mode --

    fn make_formation_core() -> DriverCore {
        let cfg = FormationConfig {
            pattern: FormationPattern::Ring { radius: 8.0 },
            altitude: 20.0,
            waypoints: vec![[0.0, 0.0, 20.0], [40.0, 0.0, 20.0], [40.0, 40.0, 20.0]],
            waypoint_accept_radius: 3.0,
            cruise_speed: 4.0,
            n_drones: 6,
        };
        let mgr = FormationWaypointManager::new(cfg, 1);
        DriverCore::with_formation(20.0, mgr)
    }

    fn arm_core(core: &mut DriverCore) {
        core.tick_simple(); // Disarmed → OffboardRequested
        core.update_status(DriverStatus { nav_state: 14, arming_state: 0 });
        core.tick_simple(); // → Arming
        core.update_status(DriverStatus { nav_state: 14, arming_state: 2 });
        core.tick_simple(); // → Takeoff
    }

    #[test]
    fn formation_takeoff_to_formup() {
        let mut core = make_formation_core();
        arm_core(&mut core);
        assert_eq!(core.flight_state, FlightState::Takeoff);

        // Reach takeoff altitude
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple();
        assert_eq!(core.flight_state, FlightState::FormUp);
    }

    #[test]
    fn formup_to_formation() {
        let mut core = make_formation_core();
        arm_core(&mut core);
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple(); // Takeoff → FormUp

        // Drone 1 form-up target: wp0 (0,0,20) + ring offset at angle 0, r=8 → (8,0,20)
        let form_target = core.formation().unwrap().form_up_target();
        assert!((form_target.x - 8.0).abs() < 0.1);

        // Move to form-up position
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple();
        assert_eq!(core.flight_state, FlightState::Formation);
    }

    #[test]
    fn formation_advances_waypoints() {
        let mut core = make_formation_core();
        arm_core(&mut core);
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple(); // → FormUp
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple(); // → Formation

        assert_eq!(core.formation().unwrap().current_waypoint_index(), 0);

        // Reach wp 0: center(0,0,20) + offset(8,0,0) = (8,0,20)
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple();
        assert_eq!(core.formation().unwrap().current_waypoint_index(), 1);

        // Reach wp 1: center(40,0,20) + offset(8,0,0) = (48,0,20)
        core.update_position(Vector3::new(48.0, 0.0, 20.0));
        core.tick_simple();
        assert_eq!(core.formation().unwrap().current_waypoint_index(), 2);
    }

    #[test]
    fn formation_complete_triggers_rtl() {
        let mut core = make_formation_core();
        arm_core(&mut core);
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple(); // → FormUp
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple(); // → Formation

        // Advance through all waypoints
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple(); // wp 0
        core.update_position(Vector3::new(48.0, 0.0, 20.0));
        core.tick_simple(); // wp 1
        core.update_position(Vector3::new(48.0, 40.0, 20.0));
        core.tick_simple(); // wp 2 → mission complete → RTL

        assert_eq!(core.flight_state, FlightState::ReturnToLaunch);
    }

    #[test]
    fn rtl_to_landing() {
        let mut core = make_formation_core();
        arm_core(&mut core);

        // Fast-forward to RTL
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple();
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple();
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick_simple();
        core.update_position(Vector3::new(48.0, 0.0, 20.0));
        core.tick_simple();
        core.update_position(Vector3::new(48.0, 40.0, 20.0));
        core.tick_simple();
        assert_eq!(core.flight_state, FlightState::ReturnToLaunch);

        // Reach home
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple();
        assert_eq!(core.flight_state, FlightState::Landing);

        // Descend
        core.update_position(Vector3::new(0.0, 0.0, 0.3));
        core.tick_simple();
        assert_eq!(core.flight_state, FlightState::Loiter);
    }

    #[test]
    fn flocking_adjusts_setpoint() {
        let mut core = make_formation_core();
        arm_core(&mut core);
        core.update_position(Vector3::new(0.0, 0.0, 20.0));
        core.tick_simple(); // → FormUp
        core.update_position(Vector3::new(8.0, 0.0, 20.0));
        core.tick(&[]); // → Formation

        // Create a neighbor very close (should cause separation)
        let neighbor = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(8.5, 0.0, 20.0),
            velocity: Vector3::zeros(),
            timestamp: 0,
        };

        let actions_with = core.tick(&[neighbor.clone()]);
        // Reset state for comparison
        let mut core2 = make_formation_core();
        arm_core(&mut core2);
        core2.update_position(Vector3::new(0.0, 0.0, 20.0));
        core2.tick_simple();
        core2.update_position(Vector3::new(8.0, 0.0, 20.0));
        core2.tick(&[]);
        let actions_without = core2.tick(&[]);

        // The setpoints should differ (flocking pushes drone away from neighbor)
        if let (
            DriverAction::PublishSetpoint(sp_with),
            DriverAction::PublishSetpoint(sp_without),
        ) = (&actions_with[0], &actions_without[0])
        {
            assert!(
                (sp_with - sp_without).norm() > 0.01,
                "flocking should adjust setpoint"
            );
        }
    }
}
