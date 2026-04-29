# 🏗 System Architecture - Swarm Digital Twin

This document provides a detailed technical overview of the **Swarm Digital Twin** system architecture, components, and communication protocols.

---

## 🚁 Project Essence
**Swarm Digital Twin** is a two-mode autonomous mission system designed for rapid deployment and high-capacity evacuation.

1.  **Scout Swarm:**
    - **Hardware:** Agile, man-portable drones (e.g., Holybro X500 V2).
    - **Objective:** Autonomous area search and human detection.
    - **Autonomy:** Boids-inspired flocking, state-machine-driven search patterns (Lawnmower/Boustrophedon).
2.  **Heavy Lift:**
    - **Hardware:** Coaxial X8 heavy-lift agents.
    - **Objective:** Evacuate 100kg+ casualties using a Distributed Lift System (DLS).
    - **Control:** 6-agent minimum for 6-DOF payload control, Admittance Control, and Distributed Control Allocation (DCA).

---

## 🛠 Software Stack

| Layer | Technology | Responsibility |
| :--- | :--- | :--- |
| **Firmware** | PX4 Autopilot | Real-time stabilization, motor control, and EKF. |
| **Logic (Rust)** | `rclrs` / `swarm_control` | Safety-critical coordination, FSM, and swarm algorithms. |
| **Logic (Python)** | `rclpy` / `perception` | AI perception, 3D localization, and computer vision. |
| **Middleware** | ROS 2 (Humble/Jazzy) | Intra-drone node communication. |
| **Global Mesh** | Eclipse Zenoh | Inter-drone communication and GCS link. |
| **Simulation** | Python (NumPy) | Standalone physics engine for rapid development. |
| **3D Visualization (Live)** | FastAPI + uvicorn + Three.js | Launcher, WebSocket telemetry, live HUD and viewport. |
| **3D Visualization (Replay)** | Matplotlib | Post-flight terrain surface, wind indicator, AGL tracking, telemetry panels. |
| **Full Simulation** | Gazebo Harmonic / PX4 SITL | Hardware-in-the-loop testing with world models. |

---

## 🏗 Component Breakdown

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity running on an NVIDIA Jetson compute platform (Orin Nano/AGX).

- **Low-Level (PX4):** Handles the "Dirty Air" — stabilization and basic flight.
- **High-Level (ROS 2):** Handles the "Smart Brain" — perception, path planning, and swarm logic.
- **Zenoh Bridge:** Bridges local ROS 2 topics to the global swarm mesh using `zenoh-bridge-ros2dds`.

### 2. Swarm Control (`swarm_control`)
Written in **Rust** for memory safety and deterministic performance.
- **FSM:** Manages mission states (IDLE, TAKEOFF, SEARCH, FOUND, RTL).
- **Boids Engine:** Computes separation, alignment, and cohesion forces.
- **PX4 Interface:** Communicates with PX4 via `TrajectorySetpoint` (NED frame).

### 3. Perception & Localization (`perception`)
Written in **Python** to leverage the AI ecosystem.
- **Detection:** YOLO-based human detection.
- **3D Projection:** Deprojects 2D pixel coordinates to 3D world coordinates using depth maps and drone odometry.
- **Coordination:** Publishes global human positions to the swarm.

### 4. Distributed Lift System (Heavy Lift mode)
The DLS is a critical innovation allowing multiple drones to act as a single "virtual crane".
- **6-DOF Control:** Requires 6 agents to control both position and orientation of the payload.
- **Admittance Control:** Drones admit external forces (tether tension) to prevent fighting against each other.
- **Emergency Detach:** Safety logic for immediate tether release.

### 5. Standalone Physics Engine (`simulation/`)
A high-fidelity rigid-body UAV simulator for rapid algorithm development without ROS 2 or Gazebo.

- **Rigid-Body Dynamics** (`drone_physics.py`): Dual-path physics — body-frame dynamics with quadratic drag and Coriolis coupling (Valencia et al. Eq. 3/5) when `AeroCoefficients` are set, or legacy world-frame linear drag for backward compatibility. ISA atmosphere model for altitude-dependent air density. Full inertia tensor with off-diagonal products of inertia. SVD re-orthogonalization for rotation matrix drift prevention. Cascaded PID controller (position -> acceleration -> attitude -> torque).
- **Fixed-Wing Aerodynamics** (`FixedWingAero`): AoA-dependent CL/CD with pre/post-stall model (paper Table 3, Fig. 5). Lift perpendicular to velocity in body XZ plane. `compute_aoa()` for angle-of-attack computation.
- **Wind Perturbation** (`wind_model.py`): Constant wind, Dryden turbulence (MIL-F-8785C), and flight-log replay modes (paper Eq. 5-7). Wind force includes both drag and lift components (Eq. 7).
- **Terrain Model** (`terrain.py`): Elevation maps from flat surfaces, 2D arrays, analytical functions, or STL meshes. Bilinear interpolation for smooth elevation queries. Collision detection integrated into `physics_step()`.
- **Validation** (`validation.py`, `flight_log.py`): RMSE metrics, Ardupilot CSV log parsing, and comparison plots for verification against real flight data (paper Table 5, Fig. 13 style).
- **MAVLink Bridge** (`mavlink_bridge.py`): MAVLink v2 UDP bridge to QGroundControl. Sends heartbeat, attitude, GPS, HUD, system status. Receives arm/disarm commands and guided-mode waypoints. Pure Python, no external dependencies.
- **Airframe Presets**: `make_generic_quad()`, `make_holybro_x500()`, and `make_fixed_wing()` with realistic mass, inertia, and aerodynamic parameters.

See [Physics Engine](physics.md) for an overview and [Physics Details](physics_details.md) for full equations.

### 6. Gazebo SITL Integration (`gazebo/`)
Full Gazebo Harmonic integration for hardware-in-the-loop testing.

K8s + Gazebo baseline implementation artifacts are present. Physics-parity
foundations are implemented in `simulation/physics_parity.py` and covered
by `TestPhysicsParity`. Collision-detection and safety KPI foundations
live in `simulation/safety.py` (`SeparationMonitor`, `TerrainMonitor`,
`SafetyReport`) and the state machine in `simulation/safety_response.py`.
Open follow-ups are tracked in `ROADMAP.md` and `TODO.md`.

- **Worlds**: `empty.world` (flat ground, ISA atmosphere), `terrain.world` (terrain mesh with wind plugin).
- **Models**: Holybro X500 V2 SDF with 4 rotors, LiftDrag plugin, and ArduPilot SITL plugin.
- **Launch**: ROS 2 launch file (`sitl_empty.launch.py`) orchestrating Gazebo, SITL, XRCE-DDS agent, and wind node.
- **Wind Node** (`wind_node.py`): ROS 2 node publishing `WindField` forces to `/wind/velocity` and `/wind/force`.

#### SITL Swarm Startup Profile (Reproducible)

Use the dedicated compose profile for the DT operation loop:

```bash
docker compose --profile swarm_sitl up -d
```

Expected stack state:
- `sitl_drone_*` containers are `healthy`.
- `swarm_node_1` is `healthy` and running `cargo run`.
- Gazebo launch (`ros2 launch gazebo sitl_empty.launch.py`) provides world + model + XRCE + wind node.

Health-check contract (must pass before mission replay):
1. **Core services**: `docker compose --profile swarm_sitl ps` shows `running/healthy`.
2. **ROS topics**: `ros2 topic list` includes `/wind/velocity` and `/wind/force`.
3. **MAVLink UDP**: bridge endpoint `udp://127.0.0.1:14550` receives `HEARTBEAT` at ~1 Hz.

### 7. Live Run-time View (`simulation/runtime_view/`)
Browser-based live visualization stack:

- FastAPI server (`server.py`) serves launcher and live viewport.
- WebSocket route `/ws/telemetry` streams `LiveTelemetrySample` at runtime.
- Vanilla Three.js frontend (`web/live.js`) renders drone pose, trail, and HUD.
- Mission launcher (`web/index.html` + `missions.json`) provides scenario cards.

### 8. Post-Flight 3D Replay (`simulation/visualize_drone_3d.py`)
Animated matplotlib visualization of recorded simulation data:
- Terrain surface rendered with elevation colormap.
- Flight trajectory with drone attitude axes and velocity vector.
- Wind direction indicator with speed label.
- Ground shadow projected onto terrain surface.
- Waypoint markers with terrain-projected ground posts.
- Timeline panels: altitude (MSL), AGL, speed, thrust.
- Falls back gracefully when terrain or wind data is absent.

---

## 📡 Communication Protocol

### Intra-Drone (ROS 2)
Standard ROS 2 publishers/subscribers for local sensing and control.

### Inter-Drone (Zenoh Mesh)
- **Mesh Connectivity:** Low-latency, decentralised communication.
- **Namespaces:** Per-drone namespace contract `/swarm/drone_n/*` (`n ∈ [1..6]`) to isolate state/control channels.
- **Topics:**
    - `/swarm/drone_n/state`: Position and mission status publication.
    - `/swarm/drone_n/consensus/raft_tx`: serialized Raft protobuf messages.
    - `/swarm/drone_n/consensus/propose`: leader proposal stream for mission command updates.
    - `/swarm/drone_n/consensus/raft_rx`: inbound Raft replication stream.
- **Discovery Storm Guardrail:** `docker/zenoh/drone_{1..6}.json5` now uses strict `allow` + `deny` lists to filter out high-bandwidth/unneeded discovery topics (`/tf*`, image/lidar, rosout).

---

## 📍 Coordinate Systems

| System | Orientation | Origin | Usage |
| :--- | :--- | :--- | :--- |
| **ENU** | East-North-Up | Launch Point | ROS 2 default frame. |
| **NED** | North-East-Down | Launch Point | PX4 firmware default frame. |
| **Body** | Forward-Left-Up | Drone Center | IMU and sensor orientation. |

**Conversion:** All high-level logic works in ENU. Conversions to NED are performed at the PX4 interface layer.

---

## 🔬 Testing & Verification

The project maintains comprehensive automated tests:

- **603 Python tests** split across `test_drone_physics/` (16 files), `test_runtime_view/` (10 files), `test_terrain.py`, `test_acceptance_matrix.py`, `test_ml/` (12 files, 150 tests): Rotation math, gravity, hover, drag, PID, position control, energy, atmosphere, wind, inertia, body-frame dynamics, validation, terrain, fixed-wing aero, MAVLink, telemetry, runtime-view HTTP/WebSocket, physics-live replay, multi-drone, post-flight replay, browser launch, DataFlash recorder, safety monitor, physics parity, SAR detection ML pipeline (catalogue, COCO annotator, augmentation, model zoo, inference logger, hard-example miner, registry, KPI gates, integration flows), and single-drone PID-policy waypoint optimisation (PolicyGains, episode runner, random search trainer, acceptance + promotion gates).
- **13 Perception Tests** (`perception/test/`): Detection pipeline and 3D localization.
- **1 Simulation Placeholder** (`test_sim.py`): ROS 2 swarm flight smoke test.
- **Rust Swarm Tests** (`swarm_control/`): Boids, FSM, PX4 interface (requires ROS 2 env).

See [Testing Guide](testing.md) and [TESTING.md](../TESTING.md) for the full catalog.

---

## 🤖 ML / Computer-Vision Pipeline (`simulation/ml/`)

SAR detection + single-drone control optimisation scaffolding. All modules are pure Python — the
heavy backends (PyTorch training, ONNX/TensorRT export, Jetson
deployment, Gazebo SAR-world capture) are registered as deferred stubs
and land through the [nightly lane](nightly_lane.md).

| Module | Purpose |
| :--- | :--- |
| `sar_targets.py`         | 21-class target catalogue (COCO-aligned IDs + SAR-specific 100-113) |
| `coco_annotator.py`      | Pinhole projection → COCO 2017 JSON dataset |
| `image_augment.py`       | Seeded Pillow-based domain randomisation (brightness, contrast, blur, cutout, rain/snow) |
| `model_zoo.py`           | `Detector` ABC + registry (`mock` active, 6 backends deferred) |
| `inference_logger.py`    | JSONL capture of every detection with frame metadata |
| `hard_example_miner.py`  | Uncertainty-band + missed-target heuristics → re-label queue |
| `model_registry.py`      | Append-only JSON registry with lineage walker and cycle detection |
| `kpi.py`                 | Acceptance thresholds (`mAP_50 ≥ 0.75`, `recall ≥ 0.85`, `inference_ms ≤ 50`) and promotion gate |
| `waypoint_optimizer.py`  | Single-drone PID tuner: `PolicyGains` (18 floats), `run_episode`, `evaluate_policy`, `random_search` |
| `waypoint_kpi.py`        | Waypoint acceptance gate (`completion ≥ 0.95`, `rmse_settled ≤ 1 m`, `t_first ≤ 30 s`, `overshoot ≤ 6 m`) and policy-promotion logic |

Driver scripts: `scripts/ml_run_pipeline.sh` (detection demo),
`scripts/ml_train_waypoint.sh` (PID trainer),
`scripts/ml_evaluate_waypoint.sh` (policy evaluator).

Developer walkthrough: [ML Tutorial](ml_tutorial.md) (Polish:
[ml_tutorial.pl.md](ml_tutorial.pl.md)).
Full reference: [ML Pipeline](ml_pipeline.md).
