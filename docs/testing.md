# 🧪 Testing & Verification Guide

This document provides a comprehensive guide to the testing strategy, catalog, and verification protocols for the Swarm Digital Twin project.

---

## 🧪 Testing Strategy

Our strategy follows a "Swiss Cheese" model with multiple layers of defense:
1. **Unit Tests (Fast):** Verify individual mathematical functions, coordinate transforms, and state-machine transitions.
2. **Component Integration (Medium):** Verify interactions between nodes (e.g., PX4 handshake, Zenoh messaging).
3. **Physics Simulation (Fast/Accurate):** Standalone rigid-body simulation for rapid algorithm iteration.
4. **End-to-End Simulation (Comprehensive):** Docker-based SITL (Software-In-The-Loop) with ROS 2 and Zenoh.

---

## 📂 Test Catalog

### 1. `swarm_control_core` (Rust)
Safety-critical flight and swarm logic.

| Test Name | Component | Purpose |
| :--- | :--- | :--- |
| `test_enu_to_ned_conversion` | Math Utils | Validates ROS 2 (ENU) to PX4 (NED) translation. |
| `test_separation_force` | Boids Engine | Verifies repulsion when drones are too close. |
| `test_process_raw_data` | Communication | Validates Zenoh message serialization and handling. |
| `test_handshake_logic` | FSM | Verifies the state sequence for taking offboard control. |

### 2. `perception_core` (Python)
AI detection and 3D vision logic.

| Test Name | Component | Purpose |
| :--- | :--- | :--- |
| `test_get_depth_at` | Vision | Verifies windowed averaging and outlier filtering in depth maps. |
| `test_deproject` | Projection | Validates Pinhole Camera Model math (Pixel → 3D). |
| `test_generate_lawnmower_path` | Planning | Verifies Boustrophedon waypoint generation for search. |

### 3. `drone_physics` (Python Standalone Sim)
Standalone physics engine tests for rigid-body quadrotor dynamics.

| Category | Description | Key Test |
| :--- | :--- | :--- |
| **Rotation** | Euler/Matrix math correctness. | `test_roundtrip` |
| **Gravity** | Freefall and ground constraints. | `test_ground_constraint` |
| **Control** | PID convergence and saturation. | `test_converges_to_zero` |
| **Stability** | Hover equilibrium and drift. | `test_hover_thrust` |
| **Dynamics** | Aerodynamic drag and energy. | `test_drag_slows_horizontal` |

### 4. Reproducible Validation Baseline (Paper-Aligned)

| Command | Purpose | Expected Outcome |
| :--- | :--- | :--- |
| `./run_scenario.sh --benchmark` | Single-entry benchmark verification aligned with deterministic validation gates and swarm benchmark gates. Runs single-drone profiles (`moderate`, `strong_wind`, `crosswind`, `storm`) and swarm profiles (`baseline`, `crosswind`, `gusty`, `tight_ring`, `high_altitude`). | Command exits with success only when all single-drone and swarm benchmark profiles pass their configured validation envelopes. |

### 5. SITL Mission Replay Verification (GCS)

| Check | Command / Action | Expected Outcome |
| :--- | :--- | :--- |
| Stack profile | `docker compose --profile swarm_sitl up -d` | SITL stack starts with no failed containers. |
| Container health | `docker compose --profile swarm_sitl ps` | `sitl_drone_*` and `swarm_node_1` show `running` and healthy status. |
| Gazebo + ROS topics | `ros2 launch gazebo sitl_empty.launch.py` then `ros2 topic list` | `/wind/velocity` and `/wind/force` are present. |
| MAVLink telemetry | Open QGroundControl on UDP `14550` | Continuous `HEARTBEAT`, `ATTITUDE`, `GLOBAL_POSITION_INT`, `VFR_HUD`, `SYS_STATUS`. |
| Replay proof | Upload and execute a mission in QGC | Replay completes and `.tlog` is saved without link dropouts. |

### 7. Real-Log Validation (Paper Table 5)

| Check | Command / Action | Expected Outcome |
| :--- | :--- | :--- |
| Real-log end-to-end gate | `./run_scenario.sh --real-log` | Required OSSITLQUAD logs are present (auto-downloaded if missing) and all mission profiles (`quad_carolina_40`, `quad_carolina_20`, `quad_epn_30`, `quad_epn_20`) pass `rmse_z/x/y <= 2x` paper Table 5 values. |
| Profile catalog sanity | `cd simulation && pytest -q test_drone_physics.py::TestPaperValidation::test_real_log_mission_catalog_has_required_profiles` | Mission catalog includes all four required paper-aligned quadrotor windows and valid source/segment metadata. |
| Acceptance gate behavior | `cd simulation && pytest -q test_drone_physics.py::TestPaperValidation::test_real_log_acceptance_gate_passes_within_2x_paper test_drone_physics.py::TestPaperValidation::test_real_log_acceptance_gate_fails_over_2x_paper` | Gate accepts metrics within `2x` envelope and rejects metrics above envelope. |

### 6. Swarm-Ready Standalone Twin Verification

| Check | Command / Action | Expected Outcome |
| :--- | :--- | :--- |
| 6-agent standalone sim safety gate | `cd simulation && pytest -q test_drone_physics.py -k test_six_agent_run_maintains_min_separation` | Test passes and minimum pairwise separation stays above the configured threshold (no collisions). |
| Boids parity reference | `cd simulation && pytest -q test_drone_physics.py -k test_flocking_vector_matches_rust_reference_case` | Python boids steering vector matches the Rust equation reference case within tolerance. |
| Boids edge parity (empty + radius boundary) | `cd simulation && pytest -q test_drone_physics.py -k "test_flocking_vector_returns_zero_without_neighbors or test_flocking_vector_excludes_neighbor_at_radius_boundary"` | Empty-neighbor case returns zero vector, and a neighbor exactly at `neighbor_radius` is excluded (`<` contract parity with Rust). |
| Swarm deterministic benchmark profiles | `cd simulation && pytest -q test_drone_physics.py -k test_swarm_benchmark_profiles_are_deterministic` | All canonical swarm profiles (`baseline`, `crosswind`, `gusty`, `tight_ring`, `high_altitude`) pass gates and remain deterministic within profile tolerance across two consecutive runs. |
| Swarm benchmark envelope compliance | `cd simulation && pytest -q test_drone_physics.py -k test_swarm_benchmark_profiles_stay_within_envelopes` | Each swarm benchmark profile satisfies configured safety and quality envelopes (`min_separation`, `p05_separation`, tracking errors, mean/p90 speed). |
| Swarm scenario risk ordering sanity check | `cd simulation && pytest -q test_drone_physics.py -k test_swarm_profile_risk_ordering` | Baseline remains easier than gusty for tracking/speed, while tight-ring yields lower minimum separation than baseline. |
| Standalone swarm demo run | `python simulation/swarm_scenario.py` | Simulation completes, prints record count and minimum separation for 6 agents in shared wind/terrain. |

---

## 🚦 Execution Protocols

### Running All Tests
```bash
# Physics & Scenario Simulation
./run_scenario.sh --test

# Deterministic benchmark baseline (validation gates)
./run_scenario.sh --benchmark

# Real-flight-data validation (paper Table 5)
./run_scenario.sh --real-log

# Swarm Control (Rust)
cd swarm_control && cargo test

# Perception (Python)
cd perception && pytest test/

# SITL stack health snapshot
docker compose --profile swarm_sitl ps

# Standalone swarm checks
cd simulation && pytest -q test_drone_physics.py -k SwarmStandaloneTwin
python simulation/swarm_scenario.py
```

### Protocol for Autonomous Agents
Agents receiving the "do tests", "do maintenance", or "do test-fix loop" commands must:
1. **Use MAINTENANCE scenario when requested:**
   - finish the current scenario,
   - test code,
   - test safety behavior,
   - simplify/cleanup code,
   - run all tests,
   - fix issues,
   - repeat until no issues remain,
   - then update/synchronize documentation.
2. **Identify untested code:** Run coverage reports.
3. **Add unit tests:** For every new function, add a corresponding test.
4. **Document in `TESTING.md`:** Keep the global registry updated.
5. **Fix failures immediately:** No submission with broken tests.

---

## 📈 Verification Status (Last Update: 2026-04-18)

| Module | Unit Tests | Integration | SITL |
| :--- | :---: | :---: | :---: |
| Swarm Control | ✅ 17 Pass | ⏳ Pending | ✅ Pass |
| Perception | ✅ 13 Pass | ⏳ Pending | ✅ Pass |
| Heavy Lift | ✅ 1 Pass | ⏳ Pending | ⏳ Pending |
| Physics Engine + Run-time View | ✅ 318 Pass | ✅ Pass | N/A |
| **Total** | **349+ Pass** | **Green** | **Green** |

## Phase 1 (K8s + Gazebo Baseline) audit status (2026-04-19)

- Baseline artifacts are present (`values-playground.yaml`, topology Helm test,
  and `docs/k8s_runbook.md`).
- A required-fixes pass is now tracked in `ROADMAP.md` (Phase 1 implementation
  audit) and `TODO.md` (section 1.1).
- Maintenance expectation: when Phase 1 implementation changes, update
  `ROADMAP.md`, `TODO.md`, `README.md`, `TESTING.md`, and `CHANGELOG.md`
  together in the same scenario.

## Phase 2 (Real Physics in Kubernetes Loop) audit status (2026-04-19)

- Phase 2 parity foundations are implemented (`simulation/physics_parity.py`,
  `TestPhysicsParity` in `simulation/test_drone_physics.py`).
- A required-fixes pass is now tracked in `ROADMAP.md` (Phase 2 implementation
  audit) and `TODO.md` (section 2.1).
- Maintenance expectation: when Phase 2 parity implementation changes, update
  `ROADMAP.md`, `TODO.md`, `README.md`, `TESTING.md`, and `CHANGELOG.md`
  together in the same scenario.

## Phase 4 (Collision Detection & Safety) audit status (2026-04-19)

- Phase 4 detection/KPI foundations are implemented (`simulation/safety.py`,
  plus related coverage in `simulation/test_drone_physics.py`).
- A required-fixes pass is now tracked in `ROADMAP.md` (Phase 4
  implementation audit) and `TODO.md` (section 4.1).
- Maintenance expectation: when Phase 4 safety implementation changes, update
  `ROADMAP.md`, `TODO.md`, `README.md`, `TESTING.md`, and `CHANGELOG.md`
  together in the same scenario.
