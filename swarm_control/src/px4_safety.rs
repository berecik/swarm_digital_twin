//! PX4 Safety Wrappers — high-performance bounds checking for 400 Hz control loops.
//!
//! Prevents invalid PX4 messages (NaN, Inf, out-of-bounds), enforces rate limits
//! between consecutive setpoints, and provides a watchdog timer that detects when
//! the control loop misses its deadline.

use std::time::{Duration, Instant};

use nalgebra::Vector3;
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint, VehicleCommand};

use crate::timing::TimingMetrics;
use crate::utils::{enu_to_ned, get_clock_microseconds};

// ── Command kinds ───────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Px4CommandKind {
    SetOffboard,
    Arm,
}

// ── Error types ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum Px4SafetyError {
    NonFiniteSetpoint,
    PositionOutOfBounds {
        axis: &'static str,
        value: f32,
        limit: f32,
    },
    RateLimitExceeded {
        field: &'static str,
        delta: f32,
        max_rate: f32,
    },
    WatchdogTimeout {
        elapsed_ms: f64,
        deadline_ms: f64,
    },
}

// ── Safety limits ───────────────────────────────────────────────────────────

/// Conservative operational bounds for PX4 setpoints.
#[derive(Debug, Clone)]
pub struct SafetyLimits {
    /// Max horizontal distance from origin [m].
    pub max_position_m: f32,
    /// Max altitude AGL [m] (regulatory / geofence).
    pub max_altitude_m: f32,
    /// Max velocity magnitude [m/s].
    pub max_velocity_mps: f32,
    /// Max position delta per second [m/s] (rate limiter).
    pub max_position_rate_mps: f32,
    /// Max yaw rate [rad/s].
    pub max_yaw_rate_radps: f32,
}

impl Default for SafetyLimits {
    fn default() -> Self {
        Self {
            max_position_m: 500.0,
            max_altitude_m: 120.0,
            max_velocity_mps: 15.0,
            max_position_rate_mps: 10.0,
            max_yaw_rate_radps: std::f32::consts::PI,
        }
    }
}

// ── Rate limiter ────────────────────────────────────────────────────────────

/// Clamps position and yaw deltas between consecutive setpoints.
pub struct SetpointRateLimiter {
    last_position: Option<[f32; 3]>,
    last_yaw: Option<f32>,
    last_time: Option<Instant>,
    limits: SafetyLimits,
}

impl SetpointRateLimiter {
    pub fn new(limits: SafetyLimits) -> Self {
        Self {
            last_position: None,
            last_yaw: None,
            last_time: None,
            limits,
        }
    }

    /// Check and clamp the next setpoint.  Returns the (possibly clamped) NED
    /// position and yaw.  First call always passes through.
    pub fn check_and_clamp(
        &mut self,
        position: [f32; 3],
        yaw: f32,
    ) -> Result<([f32; 3], f32), Px4SafetyError> {
        let now = Instant::now();

        let (clamped_pos, clamped_yaw) = match (self.last_position, self.last_yaw, self.last_time)
        {
            (Some(prev_pos), Some(prev_yaw), Some(prev_time)) => {
                let dt = now.duration_since(prev_time).as_secs_f32().max(1e-6);
                let max_delta = self.limits.max_position_rate_mps * dt;

                let mut out = position;
                for i in 0..3 {
                    let delta = position[i] - prev_pos[i];
                    if delta.abs() > max_delta {
                        out[i] = prev_pos[i] + delta.signum() * max_delta;
                    }
                }

                let max_yaw_delta = self.limits.max_yaw_rate_radps * dt;
                let yaw_delta = yaw - prev_yaw;
                let out_yaw = if yaw_delta.abs() > max_yaw_delta {
                    prev_yaw + yaw_delta.signum() * max_yaw_delta
                } else {
                    yaw
                };

                (out, out_yaw)
            }
            _ => (position, yaw),
        };

        self.last_position = Some(clamped_pos);
        self.last_yaw = Some(clamped_yaw);
        self.last_time = Some(now);
        Ok((clamped_pos, clamped_yaw))
    }

    /// Reset rate limiter state (e.g. on flight phase transition).
    pub fn reset(&mut self) {
        self.last_position = None;
        self.last_yaw = None;
        self.last_time = None;
    }
}

// ── Control loop watchdog ───────────────────────────────────────────────────

/// Detects when the control loop exceeds its deadline for too many
/// consecutive iterations, indicating a scheduling or performance problem.
pub struct ControlLoopWatchdog {
    deadline: Duration,
    last_kick: Option<Instant>,
    metrics: TimingMetrics,
    consecutive_misses: u32,
    max_consecutive_misses: u32,
    total_misses: u32,
}

impl ControlLoopWatchdog {
    /// Create a watchdog for a given loop rate.
    /// `max_consecutive_misses`: how many back-to-back overruns before error.
    pub fn new(target_hz: f64, max_consecutive_misses: u32) -> Self {
        Self {
            deadline: Duration::from_secs_f64(1.0 / target_hz),
            last_kick: None,
            metrics: TimingMetrics::new(target_hz),
            consecutive_misses: 0,
            max_consecutive_misses,
            total_misses: 0,
        }
    }

    /// Call at the top of each control loop iteration.
    /// Returns `Err(WatchdogTimeout)` when consecutive misses exceed threshold.
    pub fn kick(&mut self) -> Result<(), Px4SafetyError> {
        let now = Instant::now();
        if let Some(prev) = self.last_kick {
            let elapsed = now.duration_since(prev);
            self.metrics.record(elapsed);

            if elapsed > self.deadline {
                self.consecutive_misses += 1;
                self.total_misses += 1;
                if self.consecutive_misses > self.max_consecutive_misses {
                    return Err(Px4SafetyError::WatchdogTimeout {
                        elapsed_ms: elapsed.as_secs_f64() * 1000.0,
                        deadline_ms: self.deadline.as_secs_f64() * 1000.0,
                    });
                }
            } else {
                self.consecutive_misses = 0;
            }
        }
        self.last_kick = Some(now);
        Ok(())
    }

    pub fn metrics(&self) -> &TimingMetrics {
        &self.metrics
    }

    pub fn consecutive_misses(&self) -> u32 {
        self.consecutive_misses
    }

    pub fn total_misses(&self) -> u32 {
        self.total_misses
    }

    pub fn deadline(&self) -> Duration {
        self.deadline
    }
}

// ── Builder ─────────────────────────────────────────────────────────────────

pub struct Px4SafetyBuilder;

impl Px4SafetyBuilder {
    pub fn heartbeat_mode() -> OffboardControlMode {
        OffboardControlMode {
            timestamp: get_clock_microseconds(),
            position: true,
            velocity: false,
            acceleration: false,
            attitude: false,
            body_rate: false,
            ..Default::default()
        }
    }

    /// Build a trajectory setpoint with NaN/Inf guard only (fast path).
    pub fn trajectory_setpoint(
        enu_pos: Vector3<f32>,
    ) -> Result<TrajectorySetpoint, Px4SafetyError> {
        assert_finite_vec3(&enu_pos)?;
        let ned_pos = enu_to_ned(enu_pos.x, enu_pos.y, enu_pos.z);
        Ok(TrajectorySetpoint {
            timestamp: get_clock_microseconds(),
            position: ned_pos,
            yaw: 0.0,
            velocity: [f32::NAN; 3],
            acceleration: [f32::NAN; 3],
            ..Default::default()
        })
    }

    /// Build a trajectory setpoint with full bounds checking.
    pub fn trajectory_setpoint_checked(
        enu_pos: Vector3<f32>,
        limits: &SafetyLimits,
    ) -> Result<TrajectorySetpoint, Px4SafetyError> {
        assert_finite_vec3(&enu_pos)?;

        // Horizontal distance check
        let horiz = (enu_pos.x * enu_pos.x + enu_pos.y * enu_pos.y).sqrt();
        if horiz > limits.max_position_m {
            return Err(Px4SafetyError::PositionOutOfBounds {
                axis: "horizontal",
                value: horiz,
                limit: limits.max_position_m,
            });
        }

        // Altitude check (ENU z = up)
        if enu_pos.z > limits.max_altitude_m {
            return Err(Px4SafetyError::PositionOutOfBounds {
                axis: "altitude",
                value: enu_pos.z,
                limit: limits.max_altitude_m,
            });
        }
        if enu_pos.z < -1.0 {
            return Err(Px4SafetyError::PositionOutOfBounds {
                axis: "altitude_below_ground",
                value: enu_pos.z,
                limit: -1.0,
            });
        }

        let ned_pos = enu_to_ned(enu_pos.x, enu_pos.y, enu_pos.z);
        Ok(TrajectorySetpoint {
            timestamp: get_clock_microseconds(),
            position: ned_pos,
            yaw: 0.0,
            velocity: [f32::NAN; 3],
            acceleration: [f32::NAN; 3],
            ..Default::default()
        })
    }

    pub fn command(kind: Px4CommandKind) -> VehicleCommand {
        let (command, param1, param2) = match kind {
            Px4CommandKind::SetOffboard => (176, 1.0, 6.0),
            Px4CommandKind::Arm => (400, 1.0, 0.0),
        };

        VehicleCommand {
            timestamp: get_clock_microseconds(),
            command,
            param1,
            param2,
            target_system: 1,
            target_component: 1,
            source_system: 1,
            source_component: 1,
            from_external: true,
            ..Default::default()
        }
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn assert_finite_vec3(v: &Vector3<f32>) -> Result<(), Px4SafetyError> {
    if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
        return Err(Px4SafetyError::NonFiniteSetpoint);
    }
    Ok(())
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // -- existing tests (preserved) --

    #[test]
    fn rejects_non_finite_setpoint() {
        let err =
            Px4SafetyBuilder::trajectory_setpoint(Vector3::new(f32::NAN, 0.0, 1.0)).unwrap_err();
        assert_eq!(err, Px4SafetyError::NonFiniteSetpoint);
    }

    #[test]
    fn builds_ned_setpoint() {
        let setpoint =
            Px4SafetyBuilder::trajectory_setpoint(Vector3::new(1.0, 2.0, 3.0)).unwrap();
        assert_eq!(setpoint.position, [2.0, 1.0, -3.0]);
    }

    #[test]
    fn builds_arm_command() {
        let cmd = Px4SafetyBuilder::command(Px4CommandKind::Arm);
        assert_eq!(cmd.command, 400);
        assert_eq!(cmd.param1, 1.0);
    }

    // -- infinity rejected --

    #[test]
    fn rejects_positive_infinity() {
        let err = Px4SafetyBuilder::trajectory_setpoint(Vector3::new(f32::INFINITY, 0.0, 0.0))
            .unwrap_err();
        assert_eq!(err, Px4SafetyError::NonFiniteSetpoint);
    }

    #[test]
    fn rejects_negative_infinity() {
        let err =
            Px4SafetyBuilder::trajectory_setpoint(Vector3::new(0.0, f32::NEG_INFINITY, 0.0))
                .unwrap_err();
        assert_eq!(err, Px4SafetyError::NonFiniteSetpoint);
    }

    // -- bounds checking --

    #[test]
    fn checked_rejects_horizontal_out_of_bounds() {
        let limits = SafetyLimits::default(); // 500m
        let err = Px4SafetyBuilder::trajectory_setpoint_checked(
            Vector3::new(400.0, 400.0, 10.0), // ~566m
            &limits,
        )
        .unwrap_err();
        match err {
            Px4SafetyError::PositionOutOfBounds { axis, .. } => assert_eq!(axis, "horizontal"),
            _ => panic!("wrong error: {:?}", err),
        }
    }

    #[test]
    fn checked_rejects_altitude_over_limit() {
        let limits = SafetyLimits::default(); // 120m
        let err = Px4SafetyBuilder::trajectory_setpoint_checked(
            Vector3::new(0.0, 0.0, 150.0),
            &limits,
        )
        .unwrap_err();
        match err {
            Px4SafetyError::PositionOutOfBounds { axis, .. } => assert_eq!(axis, "altitude"),
            _ => panic!("wrong error: {:?}", err),
        }
    }

    #[test]
    fn checked_rejects_below_ground() {
        let limits = SafetyLimits::default();
        let err = Px4SafetyBuilder::trajectory_setpoint_checked(
            Vector3::new(0.0, 0.0, -5.0),
            &limits,
        )
        .unwrap_err();
        match err {
            Px4SafetyError::PositionOutOfBounds { axis, .. } => {
                assert_eq!(axis, "altitude_below_ground")
            }
            _ => panic!("wrong error: {:?}", err),
        }
    }

    #[test]
    fn checked_accepts_valid_position() {
        let limits = SafetyLimits::default();
        let sp = Px4SafetyBuilder::trajectory_setpoint_checked(
            Vector3::new(10.0, 20.0, 50.0),
            &limits,
        )
        .unwrap();
        // ENU (10,20,50) -> NED (20,10,-50)
        assert_eq!(sp.position, [20.0, 10.0, -50.0]);
    }

    #[test]
    fn safety_limits_default_values() {
        let l = SafetyLimits::default();
        assert_eq!(l.max_position_m, 500.0);
        assert_eq!(l.max_altitude_m, 120.0);
        assert_eq!(l.max_velocity_mps, 15.0);
    }

    // -- rate limiter --

    #[test]
    fn rate_limiter_first_call_passthrough() {
        let mut rl = SetpointRateLimiter::new(SafetyLimits::default());
        let (pos, yaw) = rl.check_and_clamp([10.0, 20.0, 30.0], 1.0).unwrap();
        assert_eq!(pos, [10.0, 20.0, 30.0]);
        assert_eq!(yaw, 1.0);
    }

    #[test]
    fn rate_limiter_clamps_large_jump() {
        let limits = SafetyLimits {
            max_position_rate_mps: 5.0,
            ..Default::default()
        };
        let mut rl = SetpointRateLimiter::new(limits);
        // First call — passthrough
        rl.check_and_clamp([0.0, 0.0, 0.0], 0.0).unwrap();
        // Immediate second call: dt ≈ 0, so max_delta ≈ 0 — clamps to previous
        let (pos, _) = rl.check_and_clamp([100.0, 0.0, 0.0], 0.0).unwrap();
        // The jump should be clamped (pos[0] << 100.0)
        assert!(pos[0] < 1.0, "position should be clamped, got {}", pos[0]);
    }

    #[test]
    fn rate_limiter_allows_slow_movement() {
        let limits = SafetyLimits {
            max_position_rate_mps: 10.0,
            ..Default::default()
        };
        let mut rl = SetpointRateLimiter::new(limits);
        rl.check_and_clamp([0.0, 0.0, 0.0], 0.0).unwrap();
        // Wait a simulated amount — we can't actually sleep in tests,
        // but two calls with tiny delta should pass through
        std::thread::sleep(Duration::from_millis(200)); // 0.2s → max delta = 2.0m
        let (pos, _) = rl.check_and_clamp([1.0, 0.0, 0.0], 0.0).unwrap();
        // 1.0m in 0.2s = 5 m/s < 10 m/s limit → passes through
        assert!((pos[0] - 1.0).abs() < 0.01);
    }

    #[test]
    fn rate_limiter_reset_clears_state() {
        let mut rl = SetpointRateLimiter::new(SafetyLimits::default());
        rl.check_and_clamp([0.0, 0.0, 0.0], 0.0).unwrap();
        rl.reset();
        // After reset, next call should pass through (no previous state)
        let (pos, _) = rl.check_and_clamp([100.0, 100.0, 100.0], 3.0).unwrap();
        assert_eq!(pos, [100.0, 100.0, 100.0]);
    }

    // -- watchdog --

    #[test]
    fn watchdog_normal_operation() {
        let mut wd = ControlLoopWatchdog::new(100.0, 3); // 10ms deadline
        // First kick — no previous, always OK
        assert!(wd.kick().is_ok());
        // Quick second kick
        std::thread::sleep(Duration::from_millis(2));
        assert!(wd.kick().is_ok());
        assert_eq!(wd.consecutive_misses(), 0);
    }

    #[test]
    fn watchdog_single_miss_no_error() {
        let mut wd = ControlLoopWatchdog::new(100.0, 3); // 10ms deadline
        wd.kick().unwrap();
        std::thread::sleep(Duration::from_millis(15)); // overrun
        assert!(wd.kick().is_ok()); // 1 miss, threshold is 3
        assert_eq!(wd.consecutive_misses(), 1);
        assert_eq!(wd.total_misses(), 1);
    }

    #[test]
    fn watchdog_consecutive_misses_trigger_error() {
        let mut wd = ControlLoopWatchdog::new(100.0, 2); // threshold = 2
        wd.kick().unwrap();
        std::thread::sleep(Duration::from_millis(15));
        wd.kick().unwrap(); // miss 1
        std::thread::sleep(Duration::from_millis(15));
        wd.kick().unwrap(); // miss 2
        std::thread::sleep(Duration::from_millis(15));
        let result = wd.kick(); // miss 3 > threshold 2
        assert!(result.is_err());
        match result.unwrap_err() {
            Px4SafetyError::WatchdogTimeout { .. } => {}
            e => panic!("wrong error: {:?}", e),
        }
    }

    #[test]
    fn watchdog_resets_on_good_tick() {
        let mut wd = ControlLoopWatchdog::new(100.0, 3);
        wd.kick().unwrap();
        std::thread::sleep(Duration::from_millis(15));
        wd.kick().unwrap(); // miss 1
        assert_eq!(wd.consecutive_misses(), 1);
        std::thread::sleep(Duration::from_millis(2));
        wd.kick().unwrap(); // good tick
        assert_eq!(wd.consecutive_misses(), 0);
        assert_eq!(wd.total_misses(), 1); // total preserved
    }

    #[test]
    fn watchdog_metrics_integration() {
        let mut wd = ControlLoopWatchdog::new(100.0, 3);
        wd.kick().unwrap();
        for _ in 0..5 {
            std::thread::sleep(Duration::from_millis(2));
            wd.kick().unwrap();
        }
        assert_eq!(wd.metrics().count(), 5);
        assert!(wd.metrics().mean() > Duration::ZERO);
        let report = wd.metrics().report();
        assert!(report.contains("TimingMetrics:"));
    }

    #[test]
    fn watchdog_deadline_accessor() {
        let wd = ControlLoopWatchdog::new(400.0, 3);
        let deadline = wd.deadline();
        // 400Hz → 2.5ms
        assert!((deadline.as_secs_f64() * 1000.0 - 2.5).abs() < 0.01);
    }
}
