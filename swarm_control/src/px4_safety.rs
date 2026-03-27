use nalgebra::Vector3;
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint, VehicleCommand};

use crate::utils::{enu_to_ned, get_clock_microseconds};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Px4CommandKind {
    SetOffboard,
    Arm,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Px4SafetyError {
    NonFiniteSetpoint,
}

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

    pub fn trajectory_setpoint(enu_pos: Vector3<f32>) -> Result<TrajectorySetpoint, Px4SafetyError> {
        if !enu_pos.x.is_finite() || !enu_pos.y.is_finite() || !enu_pos.z.is_finite() {
            return Err(Px4SafetyError::NonFiniteSetpoint);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_non_finite_setpoint() {
        let err = Px4SafetyBuilder::trajectory_setpoint(Vector3::new(f32::NAN, 0.0, 1.0)).unwrap_err();
        assert_eq!(err, Px4SafetyError::NonFiniteSetpoint);
    }

    #[test]
    fn builds_ned_setpoint() {
        let setpoint = Px4SafetyBuilder::trajectory_setpoint(Vector3::new(1.0, 2.0, 3.0)).unwrap();
        assert_eq!(setpoint.position, [2.0, 1.0, -3.0]);
    }

    #[test]
    fn builds_arm_command() {
        let cmd = Px4SafetyBuilder::command(Px4CommandKind::Arm);
        assert_eq!(cmd.command, 400);
        assert_eq!(cmd.param1, 1.0);
    }
}
