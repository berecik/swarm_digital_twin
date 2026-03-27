use rclrs::{self, QosProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QOS_PROFILE_DEFAULT};
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use nalgebra::Vector3;
use tokio;

use swarm_control_core::driver_core::{DriverAction, DriverCore, DriverStatus};

mod utils;
use utils::{enu_to_ned, get_clock_microseconds};

pub struct OffboardController {
    core: DriverCore,
    offboard_control_mode_publisher: Arc<rclrs::Publisher<OffboardControlMode>>,
    trajectory_setpoint_publisher: Arc<rclrs::Publisher<TrajectorySetpoint>>,
    vehicle_command_publisher: Arc<rclrs::Publisher<VehicleCommand>>,
}

impl OffboardController {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let qos_reliable = QosProfile {
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::TransientLocal,
            history: HistoryPolicy::KeepLast,
            depth: 1,
            ..QOS_PROFILE_DEFAULT
        };

        let offboard_control_mode_publisher = node.create_publisher::<OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            qos_reliable.clone(),
        )?;
        let trajectory_setpoint_publisher = node.create_publisher::<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            qos_reliable.clone(),
        )?;
        let vehicle_command_publisher = node.create_publisher::<VehicleCommand>(
            "/fmu/in/vehicle_command",
            qos_reliable,
        )?;

        Ok(Self {
            core: DriverCore::new(5.0),
            offboard_control_mode_publisher,
            trajectory_setpoint_publisher,
            vehicle_command_publisher,
        })
    }

    pub fn publish_heartbeat(&self) -> Result<(), rclrs::RclrsError> {
        let mut msg = OffboardControlMode::default();
        msg.timestamp = get_clock_microseconds();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        self.offboard_control_mode_publisher.publish(&msg)
    }

    pub fn publish_trajectory_setpoint(&self, enu_pos: Vector3<f32>) -> Result<(), rclrs::RclrsError> {
        let mut msg = TrajectorySetpoint::default();
        msg.timestamp = get_clock_microseconds();
        let ned_pos = enu_to_ned(enu_pos.x, enu_pos.y, enu_pos.z);
        msg.position = ned_pos;
        msg.yaw = 0.0; // North in NED
        msg.velocity = [f32::NAN; 3];
        msg.acceleration = [f32::NAN; 3];
        self.trajectory_setpoint_publisher.publish(&msg)
    }

    pub fn send_command(&self, command: u32, param1: f32, param2: f32) -> Result<(), rclrs::RclrsError> {
        let mut msg = VehicleCommand::default();
        msg.timestamp = get_clock_microseconds();
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        self.vehicle_command_publisher.publish(&msg)
    }

    pub fn update_status(&mut self, msg: VehicleStatus) {
        self.core.update_status(DriverStatus {
            nav_state: msg.nav_state,
            arming_state: msg.arming_state,
        });
    }

    pub fn run_control_tick(&mut self) -> Result<(), rclrs::RclrsError> {
        for action in self.core.tick() {
            match action {
                DriverAction::RequestOffboard => {
                    self.send_command(176, 1.0, 6.0)?;
                }
                DriverAction::RequestArm => {
                    self.send_command(400, 1.0, 0.0)?;
                }
                DriverAction::PublishSetpoint(enu_pos) => {
                    self.publish_trajectory_setpoint(enu_pos)?;
                }
            }
        }
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::Node::new(&context, "swarm_control_core")?;

    let controller = Arc::new(Mutex::new(OffboardController::new(&node)?));

    // Heartbeat Loop (20Hz) - Async and Robust
    let hb_controller = Arc::clone(&controller);
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(50));
        loop {
            interval.tick().await;
            let c = hb_controller.lock().unwrap();
            if let Err(e) = c.publish_heartbeat() {
                eprintln!("Failed to publish heartbeat: {}", e);
            }
        }
    });

    // State Subscriber
    let sub_controller = Arc::clone(&controller);
    let qos_best_effort = QosProfile {
        reliability: ReliabilityPolicy::BestEffort,
        durability: DurabilityPolicy::Volatile,
        history: HistoryPolicy::KeepLast,
        depth: 1,
        ..QOS_PROFILE_DEFAULT
    };
    let _status_sub = node.create_subscription::<VehicleStatus, _>(
        "/fmu/out/vehicle_status",
        qos_best_effort,
        move |msg: VehicleStatus| {
            let mut c = sub_controller.lock().unwrap();
            c.update_status(msg);
        },
    )?;

    // Main Control Loop (10Hz)
    let main_controller = Arc::clone(&controller);
    let mut interval = tokio::time::interval(Duration::from_millis(100));

    // Use a separate thread for spinning ROS 2 to avoid blocking async runtime
    let node_inner = Arc::new(node);
    let spin_node = Arc::clone(&node_inner);
    std::thread::spawn(move || {
        rclrs::spin(&spin_node).unwrap();
    });

    loop {
        interval.tick().await;
        let mut c = main_controller.lock().unwrap();
        c.run_control_tick()?;
    }
}
