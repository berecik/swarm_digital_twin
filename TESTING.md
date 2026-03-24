# TESTING.md - Swarm System Test Status

This document tracks the high-level testing status and provides detailed explanations of the verification suite across the Swarm Digital Twin project.

## 🧪 Current Status (2026-03-24)

| Module | Unit Tests | Integration Tests | SITL / Hardware | Status |
| :--- | :---: | :---: | :---: | :--- |
| `sar_swarm_control` (Rust) | ✅ Pass (17)* | ⏳ Pending | ✅ Pass (Sim) | Boids & Mission FSM Verified. |
| `sar_perception` (Python) | ✅ Pass (13) | ⏳ Pending | ✅ Pass (Sim) | 3D Localization & Lawnmower Verified |
| `heavy_lift_core` (Rust) | ✅ Pass (1) | ⏳ Pending | ⏳ Pending | Extraction State Machine Verified |
| **Drone Physics** (Python) | ✅ Pass (35) | ✅ Pass (Scenario) | N/A | Quadratic drag, wind, body-frame dynamics |
| **Swarm Simulation** | - | ✅ Pass (3) | ✅ Pass (Sim) | Mock Drone Flight Logic Verified |

\* *Note: Rust tests for `sar_swarm_control` require a sourced ROS 2 environment for compilation due to `rclrs` dependency.*

## 📂 Detailed Test Catalog

### 1. `sar_swarm_control` (Rust Core)

The following tests verify the mission-critical flight logic and swarm coordination:

#### A. Coordinate Systems & Math
- **`test_enu_to_ned_conversion`**:
    - **Purpose**: Validates the translation between ROS 2 ENU (East-North-Up) and PX4 NED (North-East-Down) frames.
- **`test_calculate_target_pos`**:
    - **Purpose**: Verifies leader-offset translation for swarm formation.
- **`test_timestamp_generation`**:
    - **Purpose**: Ensures monotonically increasing timestamps in microseconds.

#### B. Swarm Intelligence & Search
- **`test_separation_force`**, **`test_alignment_force`**, **`test_cohesion_force`**: Core flocking behaviors.
- **`test_boids_edge_cases`**: Neighbors at exactly 5.0m, identical positions, etc.
- **`test_boids_many_neighbors`**: Stress test with 100 neighbors.
- **`test_boids_empty_neighbors`**: Verification of zero-neighbor behavior.
- **`test_lawnmower_pattern`**:
    - **Purpose**: Verifies the generation of waypoints for a lawnmower search pattern.
    - **Verification**: Checks waypoint count and specific coordinates for alternating passes.

#### C. Communication & Middleware
- **`test_boid_serialization`**: Validates `bincode` encoding/decoding.
- **`test_process_raw_data`**:
    - **Purpose**: Verifies inter-drone message processing logic without a live Zenoh session.
    - **Verification**: Tests valid messages, messages from self (ignored), and malformed payloads.
- **`test_handshake_logic`**: Verifies PX4 Offboard handshake state machine.

### 2. `sar_perception` (Python AI)
The following tests verify the AI-driven human detection and 3D localization logic:

#### A. Detector Logic (`test_detector.py`)
- **`test_get_depth_at`**:
    - **Purpose**: Verifies robust depth sampling with window averaging and NaN/zero filtering.
- **`test_deproject`**:
    - **Purpose**: Validates the Pinhole Camera Model math for pixel-to-3D conversion.
- **`test_transform_and_publish_success`**:
    - **Purpose**: Ensures detection points are correctly transformed to the `map` frame and published.
- **`test_transform_and_publish_tf_error`**:
    - **Purpose**: Verifies graceful handling of TF2 lookup exceptions.

#### B. Search Planning (`test_search_planner.py`)
- **`test_generate_lawnmower_path`**:
    - **Purpose**: Validates the Boustrophedon waypoint generation logic.
- **`test_pose_callback_advances_waypoint`**:
    - **Purpose**: Ensures the mission advances to the next waypoint upon reaching the current target.
- **`test_pose_callback_ignores_far_pose`**:
    - **Purpose**: Verifies that waypoints are NOT advanced when the drone is too far.
- **`test_publish_target_at_end`**:
    - **Purpose**: Checks behavior when the search mission is complete.

### 3. `heavy_lift_core` (Rust Heavy Lift)
- **`test_state_transitions`**:
    - **Purpose**: Verifies the extraction sequence state machine (IDLE -> EN_ROUTE -> DESCENDING -> LIFTING -> RETURN -> IDLE).
    - **Verification**: Validates correctness of the `transition()` function and `next_state()` logic.

### 4. `sar_simulation` — Drone Physics Engine (`test_drone_physics.py`)

Run with: `./run_scenario.sh --test` or `pytest simulation/test_drone_physics.py`

#### A. Rotation Math
- **`test_identity`**: `euler_to_rotation(0,0,0)` produces identity matrix.
- **`test_roundtrip`**: Euler → rotation → Euler → rotation roundtrip (50 random angles, atol=1e-10).
- **`test_orthogonality`**: Rotation matrix satisfies R·R^T = I and det(R) = 1.

#### B. Gravity & Ground
- **`test_freefall_no_thrust`**: Zero-thrust drone falls (z decreases, vz < 0).
- **`test_freefall_analytical`**: Drag-free freefall matches z = z₀ - ½gt², vz = -gt (atol=0.05).
- **`test_ground_constraint`**: Drone cannot fall below z=0; velocity clamped at ground.

#### C. Hover Equilibrium
- **`test_hover_thrust`**: Thrust = mg holds altitude at 10 m for 10 seconds (atol=0.01 m).
- **`test_hover_xy_stable`**: Hover does not drift in XY (atol=0.001 m).

#### D. Aerodynamic Drag
- **`test_drag_slows_horizontal`**: Horizontal velocity decays under drag during hover (< 0.1 m/s after 20 s).
- **`test_more_drag_slows_faster`**: Higher drag coefficient → less horizontal displacement.

#### E. PID Controller
- **`test_converges_to_zero`**: PID loop drives error from 10 to < 0.1 within 5000 steps.
- **`test_limit`**: PID output is clamped to the configured limit.

#### F. Position Controller
- **`test_reach_target`**: Cascaded controller flies drone from origin to (0, 0, 5) within 0.5 m.
- **`test_horizontal_flight`**: Controller reaches (10, 5, 8) within 1.0 m.

#### G. Full Simulation
- **`test_simple_flight`**: Two-waypoint mission completes, final position within 1.0 m of target.
- **`test_records_have_increasing_time`**: All timestamps are strictly monotonically increasing.
- **`test_thrust_always_positive`**: Thrust is never negative (physical constraint).
- **`test_altitude_stays_positive`**: Position z ≥ 0 throughout the simulation.

#### H. Energy Conservation
- **`test_freefall_energy_conservation`**: KE + PE conserved within 0.5% during drag-free freefall.

#### I. Quadratic Drag (Phase 1)
- **`test_quadratic_drag_scales_with_v_squared`**: Drag force quadruples when velocity doubles (V² scaling).
- **`test_high_altitude_less_drag`**: Same velocity at 4500m ASL produces ~63% of sea-level drag.
- **`test_terminal_velocity`**: Freefall with quadratic drag converges to finite terminal velocity (variation < 5%).

#### J. Atmosphere Model (Phase 1)
- **`test_sea_level_density`**: ISA sea level density = 1.225 kg/m³.
- **`test_high_altitude_density`**: ISA at 4500m = ~0.77 kg/m³.

#### K. Wind Model (Phase 1)
- **`test_no_wind_unchanged`**: `turbulence_type="none"` produces zero force.
- **`test_constant_wind_deflects_hover`**: Hovering drone drifts > 1m downwind in 10s with 5 m/s wind.
- **`test_stronger_wind_more_force`**: Wind force scales with V² (10 m/s wind → 4x force of 5 m/s).
- **`test_wind_from_log_matches_data`**: `from_log` interpolates altitude profile correctly.

#### L. Inertia & Presets (Phase 1)
- **`test_off_diagonal_inertia_coupling`**: Off-diagonal inertia terms cause cross-axis angular velocity coupling.
- **`test_preset_loading`**: `make_generic_quad()` and `make_holybro_x500()` return correct parameters.

#### M. Body-Frame Dynamics (Phase 1)
- **`test_body_frame_hover_equivalent`**: Body-frame quadratic drag hover matches world-frame linear drag hover at ~10m.
- **`test_body_frame_freefall`**: Body-frame freefall (C_D=0) matches analytical kinematics.

#### N. Validation Module (Phase 1)
- **`test_rmse_identical`**: RMSE of identical trajectories = 0.
- **`test_rmse_known_offset`**: Constant 1m X-offset → RMSE_x = 1.0.
- **`test_flight_log_csv_roundtrip`**: FlightLog parses CSV, returns correct shape, origin at (0,0,0).

### 5. `sar_simulation` — Swarm Sim (`test_swarm_flight.py`)
- **`test_swarm_flight`**:
    - **Purpose**: Validates the end-to-end swarm flight logic in the mock simulator.
    - **Verification**: Ensures drones reach targets and maintain boid constraints.

## 📂 Detailed Documentation

- [**Complete Testing Guide**](docs/testing.md) — Strategy, Catalog, and Verification.
- [Physics Engine Deep Dive](docs/physics.md)
- [swarm_control Testing Guide](swarm_control/TESTING.md)

---

## 🛠 Testing Protocol

Autonomous agents and developers must follow the protocol defined in [AGENTS.md](AGENTS.md#2-testing-task-do-tests).

1. **Expand Coverage**: Add tests for new features and edge cases.
2. **Detailed Explanation**: When adding or updating tests, you **MUST** update this file (`TESTING.md`) with a detailed entry in the Test Catalog (Purpose, Input, Expected Outcome).
3. **Verify**: Run the full test suite (or `tests_standalone` if environment constrained).
4. **Document**: Always update this file (`TESTING.md`) and module-specific `TESTING.md` files after changes.
5. **Fix**: Ensure all tests pass before submission.
