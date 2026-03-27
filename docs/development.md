# 🛠 Development & Setup Guide

This guide provides everything needed to set up the development environment, follow the coding standards, and use the project's specialized workflows.

---

## 🚀 Quick Start — Standalone Development

If you only want to develop and test algorithms using the physics engine (no ROS 2/Docker required):

1. **Prerequisites:**
   - Python 3.10+
   - `pip`
2. **Run Scenario:**
   ```bash
   ./run_scenario.sh --all
   ```
   This will install dependencies in a virtual environment (`.venv`), run 19 physics tests, execute a flight scenario, and open a 3D visualization.

---

## 🐋 Full Swarm Development (Docker)

To develop with the full stack (ROS 2, Zenoh, PX4 Messages):

1. **Build Environment:**
   ```bash
   docker-compose build
   ```
2. **Launch Swarm:**
   ```bash
   docker-compose up
   ```
   *Note: This starts multiple drone containers and a Zenoh router.*
3. **Visualize on Host:**
   ```bash
   python3 visualize_on_host.py
   ```

### QGroundControl Mission Replay (Phase B Protocol)

1. **Start operation stack (profile):**
   ```bash
   docker compose --profile phase_b_stack up -d
   ```
2. **Launch Gazebo SITL world (separate terminal):**
   ```bash
   ros2 launch gazebo sitl_empty.launch.py
   ```
3. **Run standalone MAVLink bridge (separate terminal):**
   ```bash
   cd simulation
   python mavlink_bridge.py
   ```
4. **Connect QGroundControl:**
   - Verify UDP auto-connect to `127.0.0.1:14550`.
   - Wait for heartbeat lock and GPS fix indicator.
5. **Upload and replay mission:**
   - Open `Plan` view, load mission, click `Upload`.
   - Switch to `Fly` view and start mission replay.
6. **Capture replay evidence:**
   - Save QGC telemetry log (`.tlog`).
   - Capture terminal output from `mavlink_bridge.py` for command/target receipt.
7. **Pass criteria:**
   - Mission replay executes without link drops.
   - Telemetry checklist in `docs/testing.md` is fully satisfied.

---

## 📝 Coding Standards

### Rust (`swarm_control`, `heavy_lift_core`)
- **Safety:** Avoid `unsafe` blocks.
- **Style:** Run `cargo fmt` and `cargo clippy`.
- **ROS 2:** Use `Arc<Mutex<T>>` for shared node state.
- **Conversion:** Always perform NED (PX4) to ENU (ROS 2) conversions at the interface boundary.
- **PX4 Flight-Loop Safety Wrapper:** Use `swarm_control::px4_safety::Px4SafetyBuilder` for 400Hz command construction to guarantee finite setpoints and zero-panic message construction.

### Swarm Network Emulation (packet loss + latency)

To emulate unstable mesh conditions during local validation:

```bash
./scripts/swarm_netem.sh lo0 apply
```

This applies UDP shaping for `100ms` latency and `5%` packet loss (macOS `dummynet`/`pf`).
Clear it with:

```bash
./scripts/swarm_netem.sh lo0 clear
```

### Python (`perception`, `simulation`)
- **Style:** PEP 8 compliance.
- **Typing:** Use type hints for all public functions.
- **Dependencies:** List new dependencies in `requirements.txt`.
- **Async:** Use `rclpy`'s executor-based node structures.

---

## 🚦 Git & Commit Workflow

- **Branching:** Use descriptive branch names (e.g., `feature/raft-consensus`, `fix/depth-outliers`).
- **Commits:** Write clear commit messages.
- **Co-authorship:** When using AI assistants (like Junie), ensure the following trailer is added to commits:
  `Co-authored-by: Junie <junie@jetbrains.com>`

---

## 🤖 AI Agent Protocols

Autonomous agents working on this repository MUST follow the protocols defined in `AGENTS.md`:
1. **"do maintenance":** Run all tests, fix issues, update `ROADMAP.md` and `AGENTS.md`.
2. **"do tests":** Expand coverage, update `TESTING.md`, and verify green status.

---

## 📁 Repository Structure

```text
.
├── swarm_control/              # Rust (Safety-critical logic)
├── heavy_lift_core/            # Rust (Phase 2 heavy lift)
├── perception/                 # Python (Computer Vision & AI)
├── simulation/                 # Python (Physics & Scenarios)
├── docs/                       # Detailed Documentation
├── docker/                     # Environment Configs
└── AGENTS.md                   # Agent-specific workflows
```
