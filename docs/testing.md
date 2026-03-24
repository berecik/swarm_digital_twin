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

### 1. `sar_swarm_control` (Rust)
Safety-critical flight and swarm logic.

| Test Name | Component | Purpose |
| :--- | :--- | :--- |
| `test_enu_to_ned_conversion` | Math Utils | Validates ROS 2 (ENU) to PX4 (NED) translation. |
| `test_separation_force` | Boids Engine | Verifies repulsion when drones are too close. |
| `test_process_raw_data` | Communication | Validates Zenoh message serialization and handling. |
| `test_handshake_logic` | FSM | Verifies the state sequence for taking offboard control. |

### 2. `sar_perception` (Python)
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

### 4. Phase A Reproducible Validation Baseline (Paper-Aligned)

| Command | Purpose | Expected Outcome |
| :--- | :--- | :--- |
| `./run_scenario.sh --phase-a` | Single-entry verification for Phase A (`A1+A2+A3`). Runs unit tests + deterministic benchmarks (`moderate`, `strong_wind`). | Command exits with success only when tests pass and both benchmark profiles pass `assert_validation_pass(...)` gates. |

---

## 🚦 Execution Protocols

### Running All Tests
```bash
# Physics & Scenario Simulation
./run_scenario.sh --test

# Phase A deterministic benchmark baseline (tests + validation gates)
./run_scenario.sh --phase-a

# Swarm Control (Rust)
cd swarm_control && cargo test

# Perception (Python)
cd perception && pytest test/
```

### Protocol for Autonomous Agents
Agents receiving the "do tests" or "do maintenance" commands must:
1. **Identify untested code:** Run coverage reports.
2. **Add unit tests:** For every new function, add a corresponding test.
3. **Document in TESTING.md:** Keep the global registry updated.
4. **Fix failures immediately:** No submission with broken tests.

---

## 📈 Verification Status (Last Update: 2026-03-23)

| Module | Unit Tests | Integration | SITL |
| :--- | :---: | :---: | :---: |
| Swarm Control | ✅ 17 Pass | ⏳ Pending | ✅ Pass |
| Perception | ✅ 13 Pass | ⏳ Pending | ✅ Pass |
| Heavy Lift | ✅ 1 Pass | ⏳ Pending | ⏳ Pending |
| Physics Engine | ✅ 19 Pass | ✅ 1 Pass | N/A |
| **Total** | **50 Pass** | **1 Pass** | **Green** |
