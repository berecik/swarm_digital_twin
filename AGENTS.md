# AGENTS.md - Technical Context & Development Guide

This document provides a comprehensive technical overview and context for autonomous agents (AI) and developers working on the **Swarm Digital Twin** project. It serves as a "brain dump" of architectural decisions, system constraints, and development patterns.

---

## 👥 Authors & Context

- **Author:** beret ([beret@hipisi.org.pl](mailto:beret@hipisi.org.pl))
- **Company:** Marysia Software Limited ([ceo@marysia.app](mailto:ceo@marysia.app))
- **Website:** [https://marysia.app](https://marysia.app)
- **Primary Domain:** `app.marysia.drone`

---

## 🚁 Project Essence
**Swarm Digital Twin** (DAS-SAR) is a dual-phase Search and Rescue (SAR) system:
1.  **Phase 1 (Scout Swarm):** Agile, man-portable drones (Holybro X500 V2) for autonomous area search and human detection.
2.  **Phase 2 (Heavy Lift):** A **Distributed Lift System (DLS)** using a minimum of **6 heavy-lift agents** (coaxial X8) to evacuate human casualties (100kg+ payload).

**Core Philosophy:** Decentralization, Determinism (via Rust), and Fail-Operational Redundancy (6 agents for 6-DOF payload control).

---

## 🏗 System Architecture

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity.
-   **Low-Level (Firmware):** PX4 Autopilot on Pixhawk 6C/X. Handles real-time stabilization.
-   **High-Level (Compute):** NVIDIA Jetson Orin Nano/AGX. Runs the ROS 2 workspace.
-   **Middleware:** `zenoh-bridge-ros2dds` bridges local ROS 2 topics to the global swarm mesh.

### 2. Software Stack & Language Choice
-   **Rust (`rclrs`):** Used for `swarm_control` and `heavy_lift_core`. **Why?** Memory safety, zero-cost abstractions, and predictable performance. Essential for safety-critical coordination and the Distributed Control Allocation (DCA) layer.
-   **Python (`rclpy`):** Used for `perception` and `simulation`. **Why?** AI ecosystem (PyTorch, YOLOv8/11).
-   **Zenoh:** Used for Inter-Drone (swarm) and Ground-to-Swarm comms. **Why?** Low-latency mesh networking; avoids DDS discovery overhead in wireless environments.

---

## 🛠 Repository & Workspace Structure

-   `/swarm_control`: The Rust swarm logic (Boids, FSM, formation).
-   `/heavy_lift_core`: Phase 2 core logic (DCA, Admittance Control, 6-agent redundancy).
-   `/perception`: Python nodes for vision and 3D localization.
-   `/simulation`: Mock simulators, standalone physics engine, and test runners.
    -   `drone_physics.py`: Full physics engine — rigid-body dynamics with two modes: legacy linear drag (world-frame) or quadratic drag with body-frame dynamics (Valencia et al. Eq. 3/5). ISA atmosphere, airframe presets (quad + fixed-wing), AoA-dependent lift with stall model, terrain-aware ground collision.
    -   `wind_model.py`: Wind perturbation model (constant, Dryden turbulence, flight-log replay) following paper Eq. 5-7.
    -   `terrain.py`: Terrain elevation model (flat, grid, STL, analytical function) with bilinear interpolation and collision detection.
    -   `flight_log.py`: Ardupilot flight log parser (CSV) for validation against real data.
    -   `validation.py`: RMSE metrics and comparison plots (paper Table 5, Fig. 13 style).
    -   `drone_scenario.py`: Full-featured scenario over terrain with wind and quadratic drag. 7 AGL-relative waypoints with terrain clearance verification.
    -   `visualize_drone_3d.py`: 3D animated visualization with terrain surface, wind indicator, AGL tracking, ground shadow projected onto terrain, and telemetry panels.
    -   `mavlink_bridge.py`: MAVLink v2 UDP bridge connecting the standalone sim to QGroundControl. Sends heartbeat, attitude, GPS, HUD, SYS_STATUS. Receives COMMAND_LONG and position targets.
    -   `test_drone_physics.py`: 64 pytest tests covering rotation math, gravity, hover, drag (linear + quadratic), PID, position control, simulation, energy conservation, atmosphere, wind, inertia, body-frame dynamics, validation, terrain, fixed-wing aerodynamics (AoA, stall, lift), and MAVLink bridge.
-   `/gazebo`: Gazebo SITL integration (worlds, models, launch files, wind ROS node).
-   `/docs`: Detailed project documentation.
    -   [`docs/architecture.md`](docs/architecture.md): System design and components.
    -   [`docs/testing.md`](docs/testing.md): Test strategy and catalog.
    -   [`docs/development.md`](docs/development.md): Setup & coding standards.
    -   [`docs/physics.md`](docs/physics.md): Physics engine overview.
    -   [`docs/physics_details.md`](docs/physics_details.md): Full equations, derivations, and parameter tables.
    -   [`docs/REFACTOR_PLAN.md`](docs/REFACTOR_PLAN.md): Refactoring roadmap vs Valencia et al. (2025).

---

## 🧩 Key Integration Points (How to Develop)

### Adding a New Swarm Behavior
1.  **Modify `swarm_control` (Rust):** Implement the logic in a new module.
2.  **State Machine:** Add new states to the FSM (e.g., `APPROACH_TARGET`).
3.  **PX4 Interface:** Use `px4_msgs::msg::TrajectorySetpoint` (ENU -> NED conversion required).

### Phase 2: Distributed Lift System (DLS)
-   **6-Agent Minimum:** Phase 2 operations **require** 6 agents to maintain 6-DOF control of the slung payload.
-   **Admittance Control:** Drones must "admit" tether forces to prevent rigid position fighting.
-   **Emergency Detach:** Safety logic must handle immediate tether release in case of critical failure.

### Inter-Drone Communication
-   **Mechanism:** Publish to a local ROS 2 topic mapped to Zenoh.
-   **Data types:** `VehicleOdometry` or custom lightweight messages. Avoid raw video/LIDAR over the mesh.

### AI Detection Pipeline
-   Detections must be transformed from Image Coordinates -> Camera Coordinates (using Depth Map) -> Drone Body Frame -> World Frame (using Odometry).
-   Publish global coordinates to `/perception/human_found`.

---

## 🚦 Development & Testing Workflow

### 1. Maintenance Task ("do maintenance")
When an agent receives the "do maintenance" command, it must follow this iterative protocol:
1.  **Run All Tests:**
    -   **Rust:** Run `cargo test` in `swarm_control/`.
    -   **Python:** Run `pytest` in `perception/test/` and `simulation/` (includes `test_drone_physics.py` — 41 physics tests and `test_sim.py`).
    -   **Simulation:** Execute `test_swarm_flight.py` to verify swarm flight logic (requires ROS 2). Run `drone_scenario.py` for standalone physics verification.
2.  **Fix Issues:** Analyze any failures (compilation errors, test regressions, or linter warnings) and apply fixes.
3.  **Iterate:** Repeat steps 1 and 2 until all tests pass and no issues remain.
4.  **Update Documentation:**
    -   **AGENTS.md:** Ensure this guide reflects the latest architectural changes or maintenance procedures.
    -   **ROADMAP.md:** Update milestones and current status based on completed tasks.
        -   **Status vocabulary:** Use `[ ]` (to do), `[/]` (in progress), or `[x]` (done/cover by tests).
        -   **Format:** Start each task line with the status checkbox (e.g., `- [x] Task description`) or update the `Status` column in tables.
        -   **Test Coverage:** Update the `(Test Coverage: X%)` next to Phase headers. Calculate this as the percentage of tasks within that phase marked as `(cover by tests)`.
        -   **Subtask Coverage:** For complex tasks with multiple subtasks, describe the specific coverage for each subtask if it is possible and makes sense (e.g., `- [x] (cover by tests) **Task Name**: Description (Coverage: Unit tests for X and Y)`).
    -   **Project Docs:** Update `README.md` and ensure translations are synchronized to keep them synchronized with the codebase.
5.  **Log Maintenance:** Record the date and summary of changes in the project's history or a dedicated `MAINTENANCE.log`.

### 2. Testing Task ("do tests")
When an agent receives the "do tests" command, it must prioritize coverage and robustness:
1.  **Expand Test Coverage:**
    -   **Unit Tests:** Identify untested functions (e.g., in `boids.rs`, `utils.rs`, `communication.rs`) and write comprehensive unit tests.
    -   **Integration Tests:** Create or update tests in `swarm_control/tests/` to verify multi-module interactions, such as the PX4 handshake logic or Zenoh communication.
    -   **Edge Cases:** Add tests for negative altitudes, disconnected peers, and malformed messages.
2.  **Execute & Verify:**
    -   Run `cargo test` and ensure coverage is as high as possible.
    -   In environments where `rclrs` cannot compile, use a `tests_standalone` approach to verify logic.
3.  **Fix & Repeat:**
    -   Fix all discovered issues immediately.
    -   Repeat the process until coverage is satisfactory and no tests fail.
3.  **Update Documentation:**
    -   **TESTING.md:** Update the global test status and provide a **detailed explanation** for all tests (Purpose, Input, Expected Outcome).
    -   **Module Testing Docs:** Update specific files like `swarm_control/TESTING.md` with detailed test descriptions.
    -   **Project Docs:** Ensure documentation reflects any changes in system behavior discovered during testing.

*Last Maintenance: 2026-03-24 - Phase 3 complete: fixed-wing aerodynamics (`FixedWingAero` with AoA-dependent CL/CD and stall), MAVLink v2 bridge (`mavlink_bridge.py`). All 64 physics + MAVLink tests pass.*

### 3. Simulation First
Always validate logic in simulation.
-   `drone_physics.py`: **Standalone physics engine** — use for rapid algorithm development without ROS 2 or Docker. Supports quadratic drag, ISA atmosphere, wind perturbation, body-frame dynamics, terrain collision, and airframe presets. Run `./run_scenario.sh` for the full pipeline (tests -> sim -> visualization).
-   `mock_drone_sim.py`: Fast ROS 2-based mock for swarm protocol/logic testing.
-   `Gazebo SITL`: Integration ready via `gazebo/` directory (worlds, X500 model with LiftDrag + ArduPilot plugins, wind ROS node). Launch with `gazebo/launch/sitl_empty.launch.py`.

### 3. Dockerized Environment
The `Dockerfile` in the root contains all dependencies (Rust, ROS 2, Python libs).
-   Use `docker-compose up` to spin up a multi-drone test environment.

### 4. Safety First
-   **Heartbeat:** All high-level nodes should monitor the link to the Pixhawk.
-   **Failsafes:** Logic must default to `HOVER` or `RTL` (Return to Launch) if the perception or control node crashes.

---

## 📈 Roadmap Context for Agents
If you are tasked with moving the project forward, prioritize:
1.  **Raft Consensus Implementation:** Moving from simple Leader-Follower to a robust consensus for task allocation.
2.  **Zenoh-based Telemetry:** Finalizing the bridge configuration to ensure reliable GCS feedback.
3.  **Admittance Control (Phase 2):** Implementing force-feedback logic for tethered flight.
4.  **Multi-drone physics simulation:** Extend `drone_physics.py` to support multiple drones with Boids flocking (port from `boids.rs`) and inter-drone collision avoidance in the standalone simulator.
5.  ~~**Wind & turbulence model:**~~ **DONE** — Constant wind, Dryden turbulence, and flight-log replay implemented in `wind_model.py`.
6.  ~~**Phase 3 — Fixed-wing aerodynamics:**~~ **DONE** — `FixedWingAero` with AoA-dependent CL/CD, pre/post-stall model, lift force, and `make_fixed_wing()` preset.
7.  ~~**Phase 3 — MAVLink bridge:**~~ **DONE** — `mavlink_bridge.py` with MAVLink v2 UDP bridge (heartbeat, attitude, GPS, HUD, commands). No external dependency — pure Python.

---

## 📝 Coding Standards
-   **Rust:** Follow `clippy` and `rustfmt`. Use `Arc<Mutex<T>>` for shared state in ROS 2 nodes.
-   **Python:** PEP 8 compliance. Use type hints.
-   **Documentation:** All new modules must be added to the relevant `README.md` and referenced in the main documentation.
-   **Languages:** Maintain translations in `README_PL.md`, `README_UA.md`, `README_HE.md`, etc.

---
*Generated for the Swarm Digital Twin Project. Use this context to maintain architectural integrity.*
