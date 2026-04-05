use px4_msgs::msg::{VehicleOdometry, VehicleStatus};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use nalgebra::Vector3;

use raft::storage::MemStorage;

use swarm_control_core::boids::Boid;
use swarm_control_core::driver_core::{DriverAction, DriverCore, DriverStatus};
use swarm_control_core::formation::{FormationConfig, FormationWaypointManager};
use swarm_control_core::px4_safety::{Px4CommandKind, Px4SafetyBuilder};
use swarm_control_core::consensus::MissionConsensus;
use swarm_control_core::consensus_transport::{ConsensusTransportLoop, TransportLoopConfig};
use swarm_control_core::communication::ZenohManager;

const FORMATION_CONFIG_PATH: &str = "/root/workspace/swarm_mission.json";
const DEFAULT_FORMATION_ALT: f32 = 20.0;

/// Publishes a PX4 heartbeat (offboard control mode) via Zenoh.
async fn publish_heartbeat(zenoh: &ZenohManager, drone_id: u64) -> Result<(), String> {
    let msg = Px4SafetyBuilder::heartbeat_mode();
    let payload = bincode::serialize(&msg).map_err(|e| e.to_string())?;
    let topic = format!("swarm/drone_{drone_id}/fmu/in/offboard_control_mode");
    zenoh.session().put(&topic, payload).await.map_err(|e| e.to_string())
}

/// Publishes a trajectory setpoint via Zenoh.
async fn publish_trajectory(zenoh: &ZenohManager, drone_id: u64, enu_pos: Vector3<f32>) -> Result<(), String> {
    let Ok(msg) = Px4SafetyBuilder::trajectory_setpoint(enu_pos) else {
        return Ok(());
    };
    let payload = bincode::serialize(&msg).map_err(|e| e.to_string())?;
    let topic = format!("swarm/drone_{drone_id}/fmu/in/trajectory_setpoint");
    zenoh.session().put(&topic, payload).await.map_err(|e| e.to_string())
}

/// Publishes a vehicle command via Zenoh.
async fn send_command(zenoh: &ZenohManager, drone_id: u64, command: u32, param1: f32, param2: f32) -> Result<(), String> {
    let msg = match command {
        176 => Px4SafetyBuilder::command(Px4CommandKind::SetOffboard),
        400 => Px4SafetyBuilder::command(Px4CommandKind::Arm),
        _ => {
            let mut cmd = Px4SafetyBuilder::command(Px4CommandKind::SetOffboard);
            cmd.command = command;
            cmd.param1 = param1;
            cmd.param2 = param2;
            cmd
        }
    };
    let payload = bincode::serialize(&msg).map_err(|e| e.to_string())?;
    let topic = format!("swarm/drone_{drone_id}/fmu/in/vehicle_command");
    zenoh.session().put(&topic, payload).await.map_err(|e| e.to_string())
}

/// Execute driver actions produced by a control tick.
async fn execute_actions(zenoh: &ZenohManager, drone_id: u64, actions: Vec<DriverAction>) -> Result<(), String> {
    for action in actions {
        match action {
            DriverAction::RequestOffboard => {
                send_command(zenoh, drone_id, 176, 1.0, 6.0).await?;
            }
            DriverAction::RequestArm => {
                send_command(zenoh, drone_id, 400, 1.0, 0.0).await?;
            }
            DriverAction::PublishSetpoint(enu_pos) => {
                publish_trajectory(zenoh, drone_id, enu_pos).await?;
            }
        }
    }
    Ok(())
}

/// Try to load formation config from the well-known path.
fn load_formation(drone_id: u64) -> Option<FormationWaypointManager> {
    match FormationConfig::from_file(FORMATION_CONFIG_PATH) {
        Ok(cfg) => {
            let n_wp = cfg.waypoints.len();
            let n_drones = cfg.n_drones;
            eprintln!("[swarm_node_{drone_id}] Formation loaded: {n_wp} waypoints, {n_drones} drones");
            Some(FormationWaypointManager::new(cfg, drone_id))
        }
        Err(e) => {
            eprintln!("[swarm_node_{drone_id}] No formation config ({e}), using simple loiter mode");
            None
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let drone_id: u64 = env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);

    println!("[swarm_node_{drone_id}] Starting...");

    // ── Zenoh session ─────────────────────────────────────────────────
    let zenoh = Arc::new(ZenohManager::new(format!("drone_{drone_id}")).await);
    println!("[swarm_node_{drone_id}] Zenoh connected");

    // ── Formation or simple mode ─────────────────────────────────────
    let core = match load_formation(drone_id) {
        Some(mgr) => {
            let alt = mgr.config().altitude;
            Arc::new(Mutex::new(DriverCore::with_formation(alt, mgr)))
        }
        None => Arc::new(Mutex::new(DriverCore::new(DEFAULT_FORMATION_ALT))),
    };

    // ── Raft consensus ─────────────────────────────────────────────────
    let peers: Vec<u64> = (1..=6).collect();
    let storage = MemStorage::new_with_conf_state((peers.clone(), vec![]));
    let consensus = MissionConsensus::new(drone_id, &peers, storage)
        .expect("consensus init failed");
    let consensus_shared = Arc::new(Mutex::new(consensus));

    let _transport_loop = ConsensusTransportLoop::new(
        Arc::clone(&consensus_shared),
        drone_id,
        TransportLoopConfig::default(),
    );

    // Subscribe to incoming Raft messages via Zenoh
    let raft_sub = zenoh.subscribe_raft().await;
    let consensus_for_rx = Arc::clone(&consensus_shared);
    tokio::spawn(async move {
        while let Ok(sample) = raft_sub.recv_async().await {
            let payload = sample.payload().to_bytes();
            if let Ok(msg) = swarm_control_core::consensus::deserialize_raft_message(&payload) {
                let mut c = consensus_for_rx.lock().unwrap();
                let _ = c.step(msg);
            }
        }
    });

    // Subscribe to vehicle status via Zenoh
    let status_topic = format!("swarm/drone_{drone_id}/fmu/out/vehicle_status");
    let status_sub = zenoh.session().declare_subscriber(&status_topic).await.unwrap();
    let core_for_status = Arc::clone(&core);
    tokio::spawn(async move {
        while let Ok(sample) = status_sub.recv_async().await {
            let payload = sample.payload().to_bytes();
            if let Ok(msg) = bincode::deserialize::<VehicleStatus>(&payload) {
                let mut c = core_for_status.lock().unwrap();
                c.update_status(DriverStatus {
                    nav_state: msg.nav_state,
                    arming_state: msg.arming_state,
                });
            }
        }
    });

    // Subscribe to vehicle odometry for position feedback
    let odom_topic = format!("swarm/drone_{drone_id}/fmu/out/vehicle_odometry");
    let odom_sub = zenoh.session().declare_subscriber(&odom_topic).await.unwrap();
    let core_for_odom = Arc::clone(&core);
    tokio::spawn(async move {
        while let Ok(sample) = odom_sub.recv_async().await {
            let payload = sample.payload().to_bytes();
            if let Ok(msg) = bincode::deserialize::<VehicleOdometry>(&payload) {
                // PX4 publishes NED; convert to ENU for internal use
                let enu = Vector3::new(msg.position[1], msg.position[0], -msg.position[2]);
                let mut c = core_for_odom.lock().unwrap();
                c.update_position(enu);
            }
        }
    });

    // Consensus tick + outbox publish loop (10Hz)
    let consensus_for_tick = Arc::clone(&consensus_shared);
    let zenoh_for_raft = Arc::clone(&zenoh);
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(100));
        loop {
            interval.tick().await;
            let messages = {
                let mut c = consensus_for_tick.lock().unwrap();
                c.tick();
                c.drain_outgoing()
            };
            for msg in &messages {
                if let Ok(bytes) = swarm_control_core::consensus::serialize_raft_message(msg) {
                    let _ = zenoh_for_raft.publish_raft_message(msg.get_to(), bytes).await;
                }
            }
        }
    });

    // Heartbeat Loop (20Hz) — no lock needed
    let zenoh_for_hb = Arc::clone(&zenoh);
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(50));
        loop {
            interval.tick().await;
            if let Err(e) = publish_heartbeat(&zenoh_for_hb, drone_id).await {
                eprintln!("[swarm_node_{drone_id}] heartbeat error: {e}");
            }
        }
    });

    // Boid state broadcast loop (5Hz) — publish own position for neighbors
    let zenoh_for_boid = Arc::clone(&zenoh);
    let core_for_boid = Arc::clone(&core);
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(200));
        loop {
            interval.tick().await;
            let pos = {
                let c = core_for_boid.lock().unwrap();
                c.position()
            };
            let boid = Boid {
                drone_id: format!("drone_{drone_id}"),
                position: Vector3::new(pos.x as f64, pos.y as f64, pos.z as f64),
                velocity: Vector3::zeros(),
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_micros() as u64,
            };
            zenoh_for_boid.publish_state(&boid).await;
        }
    });

    println!("[swarm_node_{drone_id}] Control loop running");

    // Main Control Loop (10Hz) — collect neighbors + tick + execute
    let zenoh_for_ctrl = Arc::clone(&zenoh);
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    let mut last_state_log = std::time::Instant::now();

    loop {
        interval.tick().await;

        // Gather neighbor boid states
        let neighbors: Vec<Boid> = {
            let n = zenoh.neighbors.lock().unwrap();
            n.values().cloned().collect()
        };

        let (actions, state, wp_info) = {
            let mut c = core.lock().unwrap();
            let actions = c.tick(&neighbors);
            let state = c.flight_state;
            let wp_info = c.formation().map(|f| {
                (f.current_waypoint_index(), f.total_waypoints())
            });
            (actions, state, wp_info)
        };

        if let Err(e) = execute_actions(&zenoh_for_ctrl, drone_id, actions).await {
            eprintln!("[swarm_node_{drone_id}] control tick error: {e}");
        }

        // Periodic state logging (every 5s)
        if last_state_log.elapsed() >= Duration::from_secs(5) {
            last_state_log = std::time::Instant::now();
            let wp_str = match wp_info {
                Some((cur, total)) => format!(" wp={cur}/{total}"),
                None => String::new(),
            };
            println!(
                "[swarm_node_{drone_id}] state={:?} neighbors={}{wp_str}",
                state,
                neighbors.len(),
            );
        }
    }
}
