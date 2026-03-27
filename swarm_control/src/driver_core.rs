use nalgebra::Vector3;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum FlightState {
    Disarmed,
    OffboardRequested,
    Arming,
    Takeoff,
    Loiter,
}

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

pub struct DriverCore {
    pub flight_state: FlightState,
    pub status: DriverStatus,
    takeoff_height_m: f32,
}

impl DriverCore {
    pub fn new(takeoff_height_m: f32) -> Self {
        Self {
            flight_state: FlightState::Disarmed,
            status: DriverStatus::default(),
            takeoff_height_m,
        }
    }

    pub fn update_status(&mut self, status: DriverStatus) {
        self.status = status;
    }

    pub fn tick(&mut self) -> Vec<DriverAction> {
        match self.flight_state {
            FlightState::Disarmed => {
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
                self.flight_state = FlightState::Loiter;
                vec![DriverAction::PublishSetpoint(Vector3::new(
                    0.0,
                    0.0,
                    self.takeoff_height_m,
                ))]
            }
            FlightState::Loiter => vec![DriverAction::PublishSetpoint(Vector3::new(
                0.0,
                0.0,
                self.takeoff_height_m,
            ))],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transitions_to_loiter_after_offboard_and_arm() {
        let mut core = DriverCore::new(5.0);

        assert_eq!(core.tick(), vec![DriverAction::RequestOffboard]);
        assert_eq!(core.flight_state, FlightState::OffboardRequested);

        core.update_status(DriverStatus {
            nav_state: 14,
            arming_state: 0,
        });
        assert_eq!(core.tick(), vec![DriverAction::RequestArm]);
        assert_eq!(core.flight_state, FlightState::Arming);

        core.update_status(DriverStatus {
            nav_state: 14,
            arming_state: 2,
        });
        assert!(core.tick().is_empty());
        assert_eq!(core.flight_state, FlightState::Takeoff);

        let actions = core.tick();
        assert_eq!(actions.len(), 1);
        assert_eq!(core.flight_state, FlightState::Loiter);
        assert_eq!(
            actions,
            vec![DriverAction::PublishSetpoint(Vector3::new(0.0, 0.0, 5.0))]
        );
    }
}
