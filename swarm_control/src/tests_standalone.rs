use crate::driver_core::{DriverAction, DriverCore, DriverStatus, FlightState};
use crate::utils::{enu_to_ned, get_clock_microseconds};
use nalgebra::Vector3;

#[test]
fn test_driver_core_initial_state() {
    let core = DriverCore::new(5.0);
    assert_eq!(core.flight_state, FlightState::Disarmed);
}

#[test]
fn test_driver_core_transitions() {
    let mut core = DriverCore::new(5.0);

    assert_eq!(core.tick(), vec![DriverAction::RequestOffboard]);

    core.update_status(DriverStatus {
        nav_state: 14,
        arming_state: 0,
    });
    assert_eq!(core.tick(), vec![DriverAction::RequestArm]);

    core.update_status(DriverStatus {
        nav_state: 14,
        arming_state: 2,
    });
    assert!(core.tick().is_empty());

    let takeoff = core.tick();
    assert_eq!(
        takeoff,
        vec![DriverAction::PublishSetpoint(Vector3::new(0.0, 0.0, 5.0))]
    );
    assert_eq!(core.flight_state, FlightState::Loiter);
}

#[test]
fn test_enu_to_ned_conversion_standalone() {
    let enu = Vector3::new(1.0, 2.0, 3.0);
    let ned = enu_to_ned(enu.x, enu.y, enu.z);
    assert_eq!(ned, [2.0, 1.0, -3.0]);
}

#[test]
fn test_timestamp_micro() {
    let ts1 = get_clock_microseconds();
    std::thread::sleep(std::time::Duration::from_millis(1));
    let ts2 = get_clock_microseconds();
    assert!(ts2 > ts1);
}
