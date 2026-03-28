/// Swarm Digital Twin - Swarm Control Library
/// Author: beret <beret@hipisi.org.pl>
/// Company: Marysia Software Limited <ceo@marysia.app>
/// Domain: app.marysia.drone
/// Website: https://marysia.app

use rclrs;

pub mod boids;
pub mod communication;
pub mod consensus;
pub mod consensus_transport;
pub mod driver_core;
pub mod px4_safety;
pub mod timing;
pub mod transport;
pub mod utils;
pub mod search;
#[path = "main.rs"]
pub mod main_module;
pub use driver_core::{DriverAction, DriverCore, DriverStatus, FlightState};
pub use px4_safety::{
    ControlLoopWatchdog, Px4CommandKind, Px4SafetyBuilder, Px4SafetyError,
    SafetyLimits, SetpointRateLimiter,
};
pub use main_module::OffboardController;
pub use timing::TimingMetrics;
pub use transport::Transport;
pub use consensus_transport::{ConsensusTransportLoop, TransportLoopConfig, DedupCache};

#[cfg(test)]
pub mod tests;
#[cfg(test)]
pub mod tests_standalone;

pub fn hello_swarm() {
    println!("Hello from SAR Swarm Control!");
}

/// Calculate the target position based on leader position and offset.
pub fn calculate_target_pos(leader_pos: [f32; 3], offset: [f32; 3]) -> [f32; 3] {
    [
        leader_pos[0] + offset[0],
        leader_pos[1] + offset[1],
        leader_pos[2] + offset[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hello_swarm() {
        // Simple test to verify test infra
        hello_swarm();
    }

    #[test]
    fn test_calculate_target_pos() {
        let leader_pos = [1.0, 2.0, 3.0];
        let offset = [0.5, -1.0, 2.0];
        let expected = [1.5, 1.0, 5.0];
        assert_eq!(calculate_target_pos(leader_pos, offset), expected);
    }
}
