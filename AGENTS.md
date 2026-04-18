# AGENTS.md - Technical Context & Development Guide

This document provides a comprehensive technical overview and context for autonomous agents (AI) and developers working on the **Swarm Digital Twin** project. It serves as a "brain dump" of architectural decisions, system constraints, and development patterns.

**IMPORTANT RULES:**
1. **Never commit or push automatically.** The user manages the git repository. Only stage/commit/push when explicitly asked.
2. **After finishing any scenario**, always run the MAINTENANCE scenario (see below).

---

## Authors & Context

- **Author:** beret ([beret@hipisi.org.pl](mailto:beret@hipisi.org.pl))
- **Company:** Marysia Software Limited ([ceo@marysia.app](mailto:ceo@marysia.app))
- **Website:** [https://marysia.app](https://marysia.app)
- **Primary Domain:** `app.marysia.drone`

---

## Project Essence

**Swarm Digital Twin** is a dual-phase autonomous mission system:
1. **Phase 1 (Scout Swarm):** Agile, man-portable drones (Holybro X500 V2) for autonomous area search and human detection.
2. **Phase 2 (Heavy Lift):** A **Distributed Lift System (DLS)** using a minimum of **6 heavy-lift agents** (coaxial X8) to evacuate human casualties (100kg+ payload).

**Core Philosophy:** Decentralization, Determinism (via Rust), and Fail-Operational Redundancy (6 agents for 6-DOF payload control).

---

## System Architecture

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity.
- **Low-Level (Firmware):** PX4 Autopilot on Pixhawk 6C/X. Handles real-time stabilization.
- **High-Level (Compute):** NVIDIA Jetson Orin Nano/AGX. Runs the ROS 2 workspace.
- **Middleware:** `zenoh-bridge-ros2dds` bridges local ROS 2 topics to the global swarm mesh.

### 2. Software Stack & Language Choice
- **Rust (`rclrs`):** Used for `swarm_control` and `heavy_lift_core`. Memory safety, zero-cost abstractions. Essential for safety-critical coordination.
- **Python (`rclpy`):** Used for `perception` and `simulation`. AI ecosystem (PyTorch, YOLOv8/11).
- **Zenoh:** Inter-drone and ground-to-swarm comms. Low-latency mesh networking.

### 3. Live Run-time View
- **FastAPI + Three.js** web app at `http://127.0.0.1:8765/live`
- Multi-drone support with per-drone demux, colours, trails, labels
- WebSocket telemetry at 50 Hz from MAVLink bridge
- Post-flight replay from `.npz` files via launcher file picker
- Browser-driven mission launch from mission catalogue
- DataFlash `.BIN` recording for ArduPilot compatibility

---

## Repository & Workspace Structure

- `/swarm_control`: Rust swarm logic (Boids, FSM, formation, Raft consensus).
- `/heavy_lift_core`: Phase 2 core logic (DCA, Admittance Control, 6-agent redundancy).
- `/perception`: Python nodes for vision and 3D localization.
- `/simulation`: Physics engine, live viewer, and test runners.
  - `drone_physics.py`: Full physics engine — rigid-body dynamics, quadratic drag, ISA atmosphere, wind, terrain, motor dynamics, battery model. 242+ physics tests.
  - `live_telemetry.py`: MAVLink v2 receiver, per-drone demux by system_id, `TelemetryQueue`, `LiveTelemetrySample`.
  - `physics_live_replay.py`: Runs physics simulation and streams to the live viewer. Supports single/swarm/replay modes.
  - `dataflash_recorder.py`: ArduPilot-compatible `.BIN` file writer.
  - `runtime_view/server.py`: FastAPI server — REST API, WebSocket telemetry, mission launch, file replay.
  - `runtime_view/web/live.js`: Three.js multi-drone scene with dynamic meshes, trails, waypoints.
  - `mavlink_bridge.py`: MAVLink v2 UDP bridge with per-drone system_id support.
  - `test_drone_physics.py`: **299 tests** covering physics, RTV, multi-drone, replay, launch, recorder.
- `/gazebo`: Gazebo SITL integration (worlds, models, launch files).
- `/docs`: Project documentation.
- [`ROADMAP.md`](ROADMAP.md): Strategic roadmap (K8s Gazebo phases).
- [`TODO.md`](TODO.md): Execution backlog with links to detailed `/todo` instructions.
- [`todo/`](todo/): Detailed per-phase instruction files.

---

## MAINTENANCE Scenario

This is the most important workflow. It must be run after finishing any implementation, feature, or bug fix. It can be triggered with:

- **"do maintenance"**
- **"do test-fix loop"**
- **"let do maintenance"**

### The Protocol (follow in order, do not skip steps)

#### Step 1 — Finish the current task
Complete the requested implementation or fix before entering maintenance.

#### Step 2 — Run all tests
```bash
# Python physics + RTV (must show 299+ passed, 0 warnings)
.venv/bin/python -m pytest simulation/test_drone_physics.py -q

# Rust (if swarm_control was modified)
cd swarm_control && cargo test

# Perception (if perception was modified)
.venv/bin/python -m pytest perception/test/

# Shell syntax
bash -n run_scenario.sh

# JS syntax
node --check simulation/runtime_view/web/live.js
```

#### Step 3 — Test code safety
- Verify safety-critical behavior: `HOVER`/`RTL` fallbacks, finite setpoints, collision/clearance protections.
- Check for security issues: no command injection, no XSS in web UI, no unbounded inputs.

#### Step 4 — Simplify and clean up
- Remove dead code, unused imports, legacy aliases.
- Reduce duplication — extract shared helpers.
- Fix linter warnings (`clippy`/PEP8).
- Do NOT add speculative abstractions or unnecessary comments.

#### Step 5 — Run all tests again
Re-run the full test set after cleanup changes.

#### Step 6 — Fix issues
Analyze any failures and apply minimal, surgical fixes.

#### Step 7 — Iterate (test-fix loop)
Repeat steps 2–6 until **all tests pass with 0 failures and 0 warnings**.

#### Step 8 — Update and synchronize documentation
Update **all** of the following to match the current state:
- **`AGENTS.md`**: This file — test counts, file list, architecture changes.
- **`ROADMAP.md`**: Phase status checkboxes, test coverage percentages.
- **`TODO.md`**: Mark completed items, add discovered tasks.
- **`CHANGELOG.md`**: Add entries for new features/fixes.
- **`MAINTENANCE.log`**: Record date, summary, and test count.
- **`TESTING.md`**: Update test counts and verification status.
- **`README.md`**: Update quick-start, test counts, feature list.
- **`docs/architecture.md`**, **`docs/testing.md`**: Keep in sync.

#### Step 9 — Present changes (DO NOT commit)
Show the user what was changed. **Do not commit, push, or create branches** unless the user explicitly asks. The user manages the git repository.

### Summary Flow

```
finish task → run tests → test safety → simplify/cleanup → run tests
→ fix issues → repeat until clean → sync all docs → present to user
```

---

## Git Policy

**NEVER commit, push, create branches, or manage the git repository automatically.**

The user controls all git operations. When asked to commit, follow the user's instructions exactly. When not asked, just present the changes and wait.

This applies to all agents: Claude, Junie, and any other AI tool working on this project.

---

## Key Integration Points

### Adding a New Swarm Behavior
1. Modify `swarm_control` (Rust): implement logic in a new module.
2. State Machine: add new states to the FSM.
3. PX4 Interface: use `TrajectorySetpoint` (ENU -> NED conversion required).

### Phase 2: Distributed Lift System (DLS)
- **6-Agent Minimum:** Phase 2 requires 6 agents for 6-DOF control.
- **Admittance Control:** Drones must "admit" tether forces.
- **Emergency Detach:** Safety logic must handle immediate tether release.

### Inter-Drone Communication
- Publish to local ROS 2 topic mapped to Zenoh.
- Avoid raw video/LIDAR over the mesh.

---

## Testing Commands

```bash
# Full physics + RTV test suite
.venv/bin/python -m pytest simulation/test_drone_physics.py -q

# Quick subset
.venv/bin/python -m pytest simulation/test_drone_physics.py -q -k "MAVLink"

# Rust tests
cd swarm_control && cargo test

# Perception tests
.venv/bin/python -m pytest perception/test/

# Run the live viewer (default mode)
./run_scenario.sh

# Run specific modes
./run_scenario.sh --physics-live
./run_scenario.sh --physics-swarm-live
./run_scenario.sh --replay-live simulation/scenario_data.npz
./run_scenario.sh --test
```

---

## Roadmap Context for Agents

If tasked with moving the project forward, check:
1. [`ROADMAP.md`](ROADMAP.md) — strategic phases (K8s Gazebo integration).
2. [`TODO.md`](TODO.md) — actionable backlog with links to `todo/` instructions.
3. [`todo/`](todo/) — detailed per-phase implementation guides.

Current priorities: K8s + Gazebo realistic simulation (Phases 1–6), live view terrain rendering (Phase 7).

---

## Coding Standards
- **Rust:** Follow `clippy` and `rustfmt`. Use `Arc<Mutex<T>>` for shared state.
- **Python:** PEP 8. Type hints. No speculative abstractions.
- **JavaScript:** Vanilla ES modules. No build step. `escapeHtml()` for any dynamic content.
- **Documentation:** All new modules must be referenced in `AGENTS.md` and `README.md`.

---

## Safety First
- **Heartbeat:** All high-level nodes monitor the Pixhawk link.
- **Failsafes:** Default to `HOVER` or `RTL` if perception or control crashes.
- **Collision detection:** Inter-drone separation monitor + terrain AGL enforcement.

---

*Last Maintenance: 2026-04-19 — 299 tests passing, 0 warnings. Live Run-time View with multi-drone, post-flight replay, browser launch, DataFlash recording. Full roadmap expanded with per-phase todo/ instructions.*
