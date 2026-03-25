# TESTING.md - Swarm System Test Status

This document tracks the high-level testing status and provides detailed explanations of the verification suite across the Swarm Digital Twin project.

## 🧪 Current Status (2026-03-25)

| Module | Unit Tests | Integration Tests | SITL / Hardware | Status |
| :--- | :---: | :---: | :---: | :--- |
| `swarm_control_core` (Rust) | ✅ Pass (17)* | ⏳ Pending | ✅ Pass (Sim) | Boids & Mission FSM Verified. |
| `perception_core` (Python) | ✅ Pass (13) | ⏳ Pending | ✅ Pass (Sim) | 3D Localization & Lawnmower Verified |
| `heavy_lift_core` (Rust) | ✅ Pass (1) | ⏳ Pending | ⏳ Pending | Extraction State Machine Verified |
| **Drone Physics** (Python) | ✅ Pass (73) | ✅ Pass (Scenario + Benchmarks + Swarm parity) | N/A | Full physics + terrain + fixed-wing + MAVLink + Phase A/B/C validation gates |
| **Swarm Simulation** | - | ✅ Pass (3) | ✅ Pass (Sim) | Mock Drone Flight Logic Verified |

\* *Note: Rust tests for `swarm_control_core` require a sourced ROS 2 environment for compilation due to `rclrs` dependency.*

## 📂 Detailed Test Catalog

### 1. `swarm_control_core` (Rust Core)

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

### 2. `perception_core` (Python AI)
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

### 4. `simulation` — Drone Physics Engine (`test_drone_physics.py`)

Run with: `./run_scenario.sh --test` or `pytest simulation/test_drone_physics.py`

#### A0. Phase A Reproducible Validation Baseline
- **`./run_scenario.sh --benchmark`**:
    - **Purpose**: Runs one-command deterministic benchmark validation gates for both single-drone and swarm profile sets.
    - **Input**: Single-drone canonical profiles (`moderate`, `strong_wind`, `crosswind`, `storm`) from `simulation/validation.py` and swarm profiles (`baseline`, `crosswind`, `gusty`) from `simulation/swarm_scenario.py`, all with fixed seeds.
    - **Expected Outcome**: All profiles pass configured envelopes; repeated runs on same commit produce equivalent metrics within configured tolerance.

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
- **`test_validation_gate_passes_within_envelope`**: Gate accepts metrics that remain below all configured envelope thresholds.
- **`test_validation_gate_fails_outside_envelope`**: Gate rejects metrics above threshold and raises `AssertionError`.
- **`test_benchmark_profiles_are_deterministic`**: All canonical benchmark profiles (`moderate`, `strong_wind`, `crosswind`, `storm`) produce repeatable RMSE values within profile tolerance across two consecutive runs.
- **`test_swarm_benchmark_profiles_are_deterministic`**: All canonical swarm benchmark profiles (`baseline`, `crosswind`, `gusty`) produce repeatable safety/tracking metrics within profile tolerance across two consecutive runs.

#### N2. Phase B Runtime Guardrails
- **`test_runtime_warning_for_out_of_range_aero_params`**: Out-of-range aerodynamic coefficients emit one-shot runtime warning tied to documented validity envelopes.

#### O. Terrain (Phase 2)
- **`test_flat_terrain_elevation`**: Flat terrain returns constant elevation at all query points.
- **`test_from_array_elevation_query`**: Grid-based terrain uses bilinear interpolation (exact + midpoint checks).
- **`test_terrain_collision_detection`**: `check_collision()` detects below/at/above terrain surface.
- **`test_terrain_in_physics_step`**: Drone with zero thrust lands on 20m terrain, not z=0.
- **`test_from_function_terrain`**: `from_function(lambda x,y: 0.1*x)` slope gives correct elevation.
- **`test_stl_file_not_found`**: Nonexistent STL raises FileNotFoundError.

#### P. Fixed-Wing Aerodynamics (Phase 3)
- **`test_aoa_computation_level_flight`**: Level forward flight (V=[20,0,0]) produces AoA=0.
- **`test_aoa_computation_climbing`**: Climbing flight (V_z > 0) produces negative AoA.
- **`test_aoa_computation_descending`**: Descending flight (V_z < 0) produces positive AoA.
- **`test_aoa_zero_velocity`**: Zero velocity returns AoA=0 (safe default).
- **`test_pre_stall_cl_linear`**: CL=0 at alpha_0; linear C_La slope at 10° AoA.
- **`test_post_stall_cl_drops`**: CL decreases after stall angle (negative post-stall slope).
- **`test_cd_increases_with_aoa`**: CD increases with AoA due to induced drag.
- **`test_post_stall_cd_increases_faster`**: Post-stall CD slope (C_Da_stall) is steeper than pre-stall.
- **`test_lift_force_perpendicular_to_velocity`**: Lift vector is orthogonal to velocity (dot product ≈ 0).
- **`test_lift_zero_for_quadrotor`**: AeroCoefficients with C_L=0 produces zero lift force.
- **`test_fixed_wing_preset`**: `make_fixed_wing()` returns FixedWingAero with correct parameters.
- **`test_fixed_wing_generates_lift_in_flight`**: Forward flight with descent generates upward lift.
- **`test_quadratic_drag_uses_aoa_dependent_cd`**: Drag uses `get_CD(alpha)` — higher AoA → more drag.

#### Q. MAVLink Bridge (Phase 3)
- **`test_crc_computation`**: MAVLink X.25 CRC is consistent and in range [0, 0xFFFF].
- **`test_heartbeat_encoding`**: Heartbeat has MAVLink v2 STX byte (0xFD) and correct length.
- **`test_heartbeat_decode_roundtrip`**: Encoded heartbeat decodes to msg_id=0, non-empty payload.
- **`test_attitude_encode_decode`**: Attitude message roundtrips: time, roll, pitch, yaw match.
- **`test_global_position_encode_decode`**: GPS message encodes lat/lon in 1e-7 degrees correctly.
- **`test_vfr_hud_encode_decode`**: VFR HUD roundtrips airspeed and groundspeed.
- **`test_command_long_parse`**: COMMAND_LONG payload parses command_id and param1 correctly.
- **`test_enu_to_gps_conversion`**: ENU→GPS: zero offset returns ref point; +Y increases latitude.
- **`test_sim_state_from_record`**: SimRecord converts to SimState with correct thrust %.
- **`test_invalid_message_returns_none`**: Garbage data and truncated messages return None.

#### R. Swarm Standalone Twin (Phase C)
- **`test_flocking_vector_returns_zero_without_neighbors`**: Empty neighbor list returns zero steering vector (stable no-neighbor behavior).
- **`test_flocking_vector_excludes_neighbor_at_radius_boundary`**: Neighbor exactly at `neighbor_radius` is excluded, mirroring Rust `<` condition.
- **`test_flocking_vector_matches_rust_reference_case`**: Python boids helper matches hand-derived Rust reference vector.
- **`test_six_agent_run_maintains_min_separation`**: 6-agent deterministic run remains above separation threshold (no collisions).

### 5. `simulation` — Swarm Sim (`test_swarm_flight.py`)
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
