/// Swarm formation geometry, slot assignment, and waypoint management.
///
/// Each drone in the swarm maintains a fixed offset (slot) from the swarm
/// center.  The swarm center follows a shared waypoint path.  Individual
/// drone targets are `swarm_center_wp + slot_offset`, with boids flocking
/// applied on top for smooth separation/cohesion.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

// ── Formation patterns ──────────────────────────────────────────────────────

/// Supported geometric formation patterns.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum FormationPattern {
    /// Drones equally spaced on a circle of given radius.
    Ring { radius: f32 },
    /// Classic V-formation with configurable arm spacing and opening angle.
    V { spacing: f32, angle_deg: f32 },
    /// Abreast line with fixed spacing between drones.
    Line { spacing: f32 },
}

impl Default for FormationPattern {
    fn default() -> Self {
        FormationPattern::Ring { radius: 8.0 }
    }
}

// ── Formation config ────────────────────────────────────────────────────────

/// Complete formation mission definition — shared by all drones in the swarm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FormationConfig {
    pub pattern: FormationPattern,
    /// Formation flight altitude AGL [m].
    pub altitude: f32,
    /// Swarm-center waypoints in ENU [m] (z is altitude).
    pub waypoints: Vec<[f32; 3]>,
    /// Horizontal distance to accept a waypoint as reached [m].
    pub waypoint_accept_radius: f32,
    /// Cruise speed toward waypoints [m/s].
    pub cruise_speed: f32,
    /// Number of drones in the swarm.
    pub n_drones: u64,
}

impl Default for FormationConfig {
    fn default() -> Self {
        Self {
            pattern: FormationPattern::default(),
            altitude: 20.0,
            waypoints: Vec::new(),
            waypoint_accept_radius: 3.0,
            cruise_speed: 4.0,
            n_drones: 6,
        }
    }
}

impl FormationConfig {
    /// Load a formation config from a JSON file.
    pub fn from_file(path: &str) -> Result<Self, String> {
        let data = std::fs::read_to_string(path)
            .map_err(|e| format!("read {path}: {e}"))?;
        serde_json::from_str(&data)
            .map_err(|e| format!("parse {path}: {e}"))
    }

    /// Compute the ENU offset for a given drone slot (1-based drone_id).
    pub fn slot_offset(&self, drone_id: u64) -> Vector3<f32> {
        let n = self.n_drones.max(1) as f32;
        let idx = (drone_id.saturating_sub(1)) as f32; // 0-based

        match &self.pattern {
            FormationPattern::Ring { radius } => {
                let angle = idx * 2.0 * std::f32::consts::PI / n;
                Vector3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
            }
            FormationPattern::V { spacing, angle_deg } => {
                if idx == 0.0 {
                    return Vector3::zeros(); // Leader at center
                }
                let half_angle = angle_deg.to_radians() / 2.0;
                let side = if (idx as i32) % 2 == 1 { 1.0 } else { -1.0 };
                let row = ((idx as i32 + 1) / 2) as f32;
                Vector3::new(
                    -row * spacing * half_angle.cos(),
                    side * row * spacing * half_angle.sin(),
                    0.0,
                )
            }
            FormationPattern::Line { spacing } => {
                let center = (n - 1.0) / 2.0;
                Vector3::new(0.0, (idx - center) * spacing, 0.0)
            }
        }
    }
}

// ── Waypoint manager ────────────────────────────────────────────────────────

/// Tracks progress along swarm-center waypoints and computes per-drone targets.
pub struct FormationWaypointManager {
    config: FormationConfig,
    current_wp: usize,
    drone_id: u64,
    slot: Vector3<f32>,
}

impl FormationWaypointManager {
    pub fn new(config: FormationConfig, drone_id: u64) -> Self {
        let slot = config.slot_offset(drone_id);
        Self {
            config,
            current_wp: 0,
            drone_id,
            slot,
        }
    }

    /// The formation slot (form-up) position: first waypoint + offset, at
    /// formation altitude.
    pub fn form_up_target(&self) -> Vector3<f32> {
        let base = if self.config.waypoints.is_empty() {
            Vector3::new(0.0, 0.0, self.config.altitude)
        } else {
            let wp = self.config.waypoints[0];
            Vector3::new(wp[0], wp[1], self.config.altitude)
        };
        base + self.slot
    }

    /// Current target position for this drone (swarm center wp + slot offset).
    pub fn target_position(&self) -> Vector3<f32> {
        let center = self.current_center();
        center + self.slot
    }

    /// Swarm-center position of the current waypoint.
    fn current_center(&self) -> Vector3<f32> {
        if self.current_wp < self.config.waypoints.len() {
            let wp = self.config.waypoints[self.current_wp];
            Vector3::new(wp[0], wp[1], wp[2])
        } else if let Some(last) = self.config.waypoints.last() {
            Vector3::new(last[0], last[1], last[2])
        } else {
            Vector3::new(0.0, 0.0, self.config.altitude)
        }
    }

    /// Check if the drone has reached the current waypoint (horizontal distance).
    /// If so, advance to the next waypoint and return true.
    pub fn advance_if_reached(&mut self, my_position: Vector3<f32>) -> bool {
        if self.current_wp >= self.config.waypoints.len() {
            return false;
        }
        let target = self.target_position();
        let dx = my_position.x - target.x;
        let dy = my_position.y - target.y;
        let horiz_dist = (dx * dx + dy * dy).sqrt();

        if horiz_dist < self.config.waypoint_accept_radius {
            self.current_wp += 1;
            true
        } else {
            false
        }
    }

    /// True when all waypoints have been reached.
    pub fn is_complete(&self) -> bool {
        self.config.waypoints.is_empty() || self.current_wp >= self.config.waypoints.len()
    }

    pub fn current_waypoint_index(&self) -> usize {
        self.current_wp
    }

    pub fn total_waypoints(&self) -> usize {
        self.config.waypoints.len()
    }

    pub fn drone_id(&self) -> u64 {
        self.drone_id
    }

    pub fn config(&self) -> &FormationConfig {
        &self.config
    }

    pub fn slot_offset(&self) -> Vector3<f32> {
        self.slot
    }

    /// Compute a velocity vector toward the current target, clamped to cruise speed.
    pub fn velocity_toward_target(&self, my_position: Vector3<f32>) -> Vector3<f32> {
        let target = self.target_position();
        let diff = target - my_position;
        let dist = diff.norm();
        if dist < 0.1 {
            return Vector3::zeros();
        }
        let speed = self.config.cruise_speed.min(dist); // slow down near target
        diff.normalize() * speed
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_six_drones_symmetric() {
        let cfg = FormationConfig {
            pattern: FormationPattern::Ring { radius: 10.0 },
            n_drones: 6,
            ..Default::default()
        };

        let offsets: Vec<Vector3<f32>> = (1..=6).map(|id| cfg.slot_offset(id)).collect();

        // All offsets should be at radius 10
        for (i, off) in offsets.iter().enumerate() {
            let r = (off.x * off.x + off.y * off.y).sqrt();
            assert!(
                (r - 10.0).abs() < 0.01,
                "drone {} radius = {}, expected 10.0",
                i + 1,
                r
            );
            assert!(off.z.abs() < 0.001);
        }

        // Drone 1 should be at angle 0 (positive x)
        assert!((offsets[0].x - 10.0).abs() < 0.01);
        assert!(offsets[0].y.abs() < 0.01);
    }

    #[test]
    fn v_formation_leader_at_center() {
        let cfg = FormationConfig {
            pattern: FormationPattern::V {
                spacing: 5.0,
                angle_deg: 60.0,
            },
            n_drones: 5,
            ..Default::default()
        };
        let leader = cfg.slot_offset(1);
        assert_eq!(leader, Vector3::zeros());
    }

    #[test]
    fn v_formation_symmetric_arms() {
        let cfg = FormationConfig {
            pattern: FormationPattern::V {
                spacing: 5.0,
                angle_deg: 60.0,
            },
            n_drones: 5,
            ..Default::default()
        };
        // Drones 2 and 3 should be symmetric about x axis
        let d2 = cfg.slot_offset(2);
        let d3 = cfg.slot_offset(3);
        assert!((d2.x - d3.x).abs() < 0.01);
        assert!((d2.y + d3.y).abs() < 0.01); // opposite y
    }

    #[test]
    fn line_formation_centered() {
        let cfg = FormationConfig {
            pattern: FormationPattern::Line { spacing: 4.0 },
            n_drones: 3,
            ..Default::default()
        };
        // Center drone (id=2, idx=1) should be at y=0
        let d2 = cfg.slot_offset(2);
        assert!(d2.y.abs() < 0.01);
        // Drone 1 at y=-4, drone 3 at y=+4
        let d1 = cfg.slot_offset(1);
        let d3 = cfg.slot_offset(3);
        assert!((d1.y - (-4.0)).abs() < 0.01);
        assert!((d3.y - 4.0).abs() < 0.01);
    }

    #[test]
    fn waypoint_manager_advance() {
        let cfg = FormationConfig {
            waypoints: vec![[0.0, 0.0, 20.0], [50.0, 0.0, 20.0], [50.0, 50.0, 20.0]],
            waypoint_accept_radius: 3.0,
            n_drones: 1,
            pattern: FormationPattern::Ring { radius: 0.0 },
            ..Default::default()
        };
        let mut mgr = FormationWaypointManager::new(cfg, 1);
        assert_eq!(mgr.current_waypoint_index(), 0);
        assert!(!mgr.is_complete());

        // Not reached yet
        assert!(!mgr.advance_if_reached(Vector3::new(10.0, 0.0, 20.0)));
        assert_eq!(mgr.current_waypoint_index(), 0);

        // Reached wp 0
        assert!(mgr.advance_if_reached(Vector3::new(1.0, 0.0, 20.0)));
        assert_eq!(mgr.current_waypoint_index(), 1);

        // Reached wp 1
        assert!(mgr.advance_if_reached(Vector3::new(50.0, 1.0, 20.0)));
        assert_eq!(mgr.current_waypoint_index(), 2);

        // Reached wp 2 (last)
        assert!(mgr.advance_if_reached(Vector3::new(50.0, 50.0, 20.0)));
        assert!(mgr.is_complete());
    }

    #[test]
    fn velocity_toward_target_clamps_speed() {
        let cfg = FormationConfig {
            waypoints: vec![[100.0, 0.0, 20.0]],
            cruise_speed: 4.0,
            n_drones: 1,
            pattern: FormationPattern::Ring { radius: 0.0 },
            ..Default::default()
        };
        let mgr = FormationWaypointManager::new(cfg, 1);
        let vel = mgr.velocity_toward_target(Vector3::new(0.0, 0.0, 20.0));
        let speed = vel.norm();
        assert!((speed - 4.0).abs() < 0.01, "speed={speed}, expected 4.0");
    }

    #[test]
    fn velocity_slows_near_target() {
        let cfg = FormationConfig {
            waypoints: vec![[1.0, 0.0, 20.0]],
            cruise_speed: 4.0,
            n_drones: 1,
            pattern: FormationPattern::Ring { radius: 0.0 },
            ..Default::default()
        };
        let mgr = FormationWaypointManager::new(cfg, 1);
        let vel = mgr.velocity_toward_target(Vector3::new(0.0, 0.0, 20.0));
        let speed = vel.norm();
        assert!(speed < 2.0, "should slow down near target, got {speed}");
    }

    #[test]
    fn form_up_target_uses_first_waypoint() {
        let cfg = FormationConfig {
            waypoints: vec![[10.0, 20.0, 25.0], [50.0, 0.0, 25.0]],
            altitude: 25.0,
            pattern: FormationPattern::Ring { radius: 5.0 },
            n_drones: 4,
            ..Default::default()
        };
        let mgr = FormationWaypointManager::new(cfg, 1);
        let target = mgr.form_up_target();
        // Drone 1 slot at angle 0, radius 5 → offset (5, 0, 0)
        // First wp center (10, 20, 25) + offset (5, 0, 0) = (15, 20, 25)
        assert!((target.x - 15.0).abs() < 0.01);
        assert!((target.y - 20.0).abs() < 0.01);
        assert!((target.z - 25.0).abs() < 0.01);
    }

    #[test]
    fn config_from_json_roundtrip() {
        let cfg = FormationConfig {
            pattern: FormationPattern::Ring { radius: 8.0 },
            altitude: 20.0,
            waypoints: vec![[0.0, 0.0, 20.0], [40.0, 0.0, 20.0]],
            waypoint_accept_radius: 3.0,
            cruise_speed: 4.0,
            n_drones: 6,
        };
        let json = serde_json::to_string_pretty(&cfg).unwrap();
        let parsed: FormationConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.n_drones, 6);
        assert_eq!(parsed.waypoints.len(), 2);
    }
}
