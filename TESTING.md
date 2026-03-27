# TESTING.md - Swarm System Test Status

This document tracks the high-level testing status and provides detailed explanations of the verification suite across the Swarm Digital Twin project.

## Current Status (2026-03-27)

| Module | Unit Tests | Integration Tests | SITL / Hardware | Status |
| :--- | :---: | :---: | :---: | :--- |
| `swarm_control_core` (Rust) | ✅ Pass (17)* | ⏳ Pending | ✅ Pass (Sim) | Boids & Mission FSM + Transport + Timing Verified. |
| `perception_core` (Python) | ✅ Pass (13) | ⏳ Pending | ✅ Pass (Sim) | 3D Localization & Lawnmower Verified |
| `heavy_lift_core` (Rust) | ✅ Pass (1) | ⏳ Pending | ⏳ Pending | Extraction State Machine Verified |
| **Drone Physics** (Python) | ✅ Pass (184) | ✅ Pass (Scenario + 6 FW/IRS-4 Benchmarks + Swarm parity) | N/A | Full physics + terrain + fixed-wing + MAVLink + Phase A-N validation gates |
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
- **`test_swarm_benchmark_profiles_are_deterministic`**: All canonical swarm benchmark profiles (`baseline`, `crosswind`, `gusty`, `tight_ring`, `high_altitude`) produce repeatable safety/tracking metrics within profile tolerance across two consecutive runs.
- **`test_swarm_benchmark_profiles_stay_within_envelopes`**: Every swarm profile satisfies its configured envelope for separation (`min_separation`, `p05_separation`), tracking (`mean/p75/max error`), and kinematics (`mean_speed`, `p90_speed`).
- **`test_swarm_profile_risk_ordering`**: Relative scenario difficulty remains coherent: gusty conditions are harder than baseline (tracking/speed), while tight-ring reduces minimum separation versus baseline.

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

#### S. Pitching Moment (Phase D)
- **`test_pitching_moment_pre_stall`**: C_M is negative and proportional to alpha below stall (stabilizing).
- **`test_pitching_moment_post_stall`**: C_M uses C_Ma_stall slope above stall angle.
- **`test_pitching_moment_zero_at_zero_alpha`**: No pitching moment at zero AoA.
- **`test_pitching_moment_applied_in_physics_step`**: Physics step applies aero pitching moment on pitch axis for fixed-wing.
- **`test_quadrotor_has_no_pitching_moment`**: Base AeroCoefficients returns zero C_M.

#### T. Valencia Paper-Exact Preset (Phase D)
- **`test_valencia_fixed_wing_mass`**: Mass matches paper Table 2 (2.5 kg).
- **`test_valencia_fixed_wing_ref_area`**: Wing reference area = 0.3997 m^2.
- **`test_valencia_fixed_wing_chord`**: Chord = 0.235 m.
- **`test_valencia_fixed_wing_aero_coefficients`**: All Table 3 CFD coefficients match.
- **`test_valencia_fixed_wing_high_altitude_atmosphere`**: Antisana atmosphere at 4500m MSL, rho ~0.77.
- **`test_valencia_fixed_wing_passes_provenance_check`**: No out-of-range warnings.

#### U. Gamma-Term Equivalence (Phase D)
- **`test_gamma_terms_match_matrix_form`**: Expanded paper Eq. 4 Gamma terms match matrix-form `I_inv @ (tau - omega x I@omega)` within 1e-12 for non-diagonal inertia tensor.

#### V. Flight Log Binary Parser (Phase E)
- **`test_bin_parser_creates_flight_log`**: Synthetic DataFlash binary with FMT+GPS messages parses into FlightLog with correct positions.
- **`test_bin_parser_file_not_found`**: Missing .bin file raises FileNotFoundError.
- **`test_get_elevator_returns_pitch_rate`**: `get_elevator()` returns pitch rate derivative as elevator estimate.
- **`test_extract_waypoints_from_dwell`**: `extract_waypoints()` detects dwell points where speed drops below threshold.

#### W. Sim-vs-Real Comparison (Phase E)
- **`test_compare_sim_real_identical`**: Identical sim and real trajectories produce zero RMSE in all axes.
- **`test_compare_sim_real_known_offset`**: Constant 5m Z-offset produces expected RMSE and percentile metrics.
- **`test_compare_signals_perfect_match`**: Identical signals produce RMSE=0 and correlation=1.0.
- **`test_compare_signals_anticorrelated`**: Anticorrelated signals produce correlation=-1.0.

#### X. SRTM Terrain Pipeline (Phase F)
- **`test_from_hgt_synthetic`**: Synthetic .hgt file (3601x3601 int16 big-endian) loads into TerrainMap with correct GPS origin.
- **`test_gps_elevation_query`**: `get_elevation_gps(lat, lon)` returns correct elevation for known GPS coordinates.
- **`test_gps_query_without_origin_raises`**: GPS query on TerrainMap without `origin_gps` raises ValueError.
- **`test_hgt_parser_file_not_found`**: Missing .hgt file raises FileNotFoundError.

#### Y. Gazebo Model Integration (Phase G)
- **`test_sdf_model_exists`**: `gazebo/models/valencia_fixed_wing/model.sdf` file exists.
- **`test_sdf_model_config_exists`**: `gazebo/models/valencia_fixed_wing/model.config` file exists.
- **`test_sdf_contains_liftdrag_plugin`**: SDF file contains `<plugin>` with LiftDrag configuration.
- **`test_sdf_mass_matches_paper`**: SDF inertial mass matches paper Table 2 (2.5 kg).
- **`test_antisana_world_exists`**: `gazebo/worlds/antisana.world` file exists.
- **`test_antisana_world_gps_origin`**: World file contains Antisana GPS coordinates (-0.508333, -78.141667).
- **`test_parm_file_exists`**: `gazebo/models/valencia_fixed_wing/valencia_fw.parm` file exists.

#### Z. SITL Mission Lifecycle (Phase H)
- **`test_sitl_script_exists`**: `scripts/run_sitl_mission.sh` exists and is executable.
- **`test_sitl_script_has_required_steps`**: Script contains all 6 lifecycle steps (start, health check, upload, arm, capture, validate).

#### AA. 3D Wind Estimation (Phase H2)
- **`test_still_trajectory_gives_zero_wind`**: Stationary drone produces near-zero 3D wind estimate.
- **`test_constant_drift_detected`**: Sinusoidal lateral deviation produces non-zero Y-component wind.
- **`test_3d_profile_has_correct_shape`**: 3D wind profile returns Nx4 array [t, wx, wy, wz].
- **`test_3d_profile_too_short_returns_zero`**: Trajectory with < 3 points returns zero wind.
- **`test_from_log_3d_wind_replay`**: WindField with `from_log_3d` correctly interpolates 3D wind profile.

#### AB. CI Pipeline (Phase I1)
- **`test_ci_workflow_exists`**: `.github/workflows/ci.yml` file exists.
- **`test_ci_workflow_has_required_jobs`**: Workflow contains pytest, benchmark, and upload-artifact steps.
- **`test_ci_workflow_triggers_on_push`**: Workflow triggers on push to master/main.

#### AC. IRS-4 Quadrotor Preset (Phase J1)
- **`test_irs4_mass`**: IRS-4 mass is in [1.0, 3.0] kg range for compact quadrotor.
- **`test_irs4_atmosphere_quito`**: Default atmosphere at 2800m MSL, density ~0.93 kg/m^3.
- **`test_irs4_custom_altitude`**: Accepts custom altitude for different experiment sites.
- **`test_irs4_has_aero`**: Uses quadratic drag model (not linear fallback), C_L=0.
- **`test_irs4_symmetric_inertia`**: Roll/pitch inertia are symmetric (X-frame).
- **`test_irs4_hover_stable`**: Maintains hover within 0.5m after settling.
- **`test_irs4_waypoint_tracking`**: Tracks square waypoint pattern and returns home.

#### AD. Mission Replay Pipeline (Phase J2)
- **`test_replay_returns_metrics`**: `replay_mission()` returns dict with all standard metric keys.
- **`test_replay_quad_produces_valid_rmse`**: Quadrotor replay produces finite, positive RMSE values.
- **`test_replay_fixed_wing_produces_valid_rmse`**: Fixed-wing replay produces finite RMSE.
- **`test_replay_with_wind_from_log`**: Replay with from_log wind completes without error.
- **`test_replay_rejects_short_log`**: Rejects flight logs with fewer than 10 data points.

#### AE. Paper Table 5 Acceptance (Phase J3)
- **`test_quadrotor_carolina_hover_accuracy[20/40]`**: Hover accuracy at Carolina-like 20m and 40m AGL.
- **`test_quadrotor_epn_hover_accuracy[20/30]`**: Hover accuracy at EPN-like 20m and 30m AGL.
- **`test_fixed_wing_deterministic`**: Fixed-wing sim is deterministic (identical inputs → RMSE≈0).
- **`test_quadrotor_deterministic`**: Quadrotor sim is deterministic.
- **`test_paper_table5_format`**: Validation pipeline produces all Table 5 metric fields.
- **`test_quadrotor_hover_no_wind_rmse`**: No-wind hover converges within generic PID threshold.
- **`test_paper_table5_thresholds_documented`**: All 7 paper Table 5 RMSE values documented and validated.

#### AF. IRS-4 Benchmark Determinism
- **`test_irs4_benchmark_profiles_are_deterministic[irs4_carolina]`**: Carolina quadrotor benchmark is repeatable (identical RMSE across runs).
- **`test_irs4_benchmark_profiles_are_deterministic[irs4_epn]`**: EPN quadrotor benchmark is repeatable.

#### AF. Simulation Bridge (Phase R1)
- **`test_bridge_message_contract`**: ActionMessage and StatusMessage JSON contract matches Rust wire format.
- **`test_bridge_process_actions`**: SimBridge correctly processes RequestOffboard, RequestArm, PublishSetpoint action batches.
- **`test_bridge_physics_step`**: Armed bridge advances physics upward toward target altitude.
- **`test_bridge_status_message_format`**: Status response contains nav_state, arming_state, position fields.
- **`test_bridge_udp_roundtrip`**: Full UDP roundtrip — client sends actions, bridge processes and returns status.

#### AG. Real-Time Timing Contract (Phase R2)
- **`test_physics_step_latency`**: Single `physics_step()` mean < 1ms, p95 < 2ms (CI gate for real-time viability).
- **`test_controller_step_latency`**: `PositionController.compute()` p95 < 1ms.

#### AH. IRS-4 Gazebo Model (Phase K2)
- **`test_irs4_sdf_exists`**: IRS-4 SDF model file exists.
- **`test_irs4_model_config_exists`**: Model config XML exists.
- **`test_irs4_parm_exists`**: ArduPilot parameter file exists.
- **`test_irs4_sdf_mass_matches_preset`**: SDF mass matches `make_irs4_quadrotor()` (1.8 kg).
- **`test_irs4_sdf_has_4_rotors`**: SDF contains 4 rotor link/joint pairs.
- **`test_irs4_sdf_has_ardupilot_plugin`**: ArduPilot SITL plugin present.
- **`test_irs4_sdf_has_liftdrag_plugin`**: LiftDrag aero plugin with C_D=1.0.
- **`test_irs4_sdf_inertia_matches_preset`**: Inertia matches Ixx=0.025, Iyy=0.025, Izz=0.042.
- **`test_irs4_parm_copter_frame`**: Parameter file configures copter frame (FRAME_CLASS=1).
- **`test_irs4_parm_carolina_origin`**: GPS origin at Carolina Park (-0.189, 2800m).

#### AI. Docker SITL Configuration (Phase K1)
- **`test_dockerfile_sitl_exists`**: Dockerfile.sitl exists.
- **`test_dockerfile_builds_copter_and_plane`**: Dockerfile builds both arducopter and arduplane.
- **`test_dockerfile_exposes_ports`**: Required UDP/TCP ports (9002, 9003, 14550) exposed.
- **`test_dockerfile_has_healthcheck`**: MAVLink heartbeat health check configured.
- **`test_compose_has_sitl_service`**: docker-compose.yml contains ardupilot_sitl service.
- **`test_compose_sitl_ports`**: SITL compose service maps correct ports.
- **`test_sitl_entrypoint_exists`**: Entrypoint script exists and is executable.

#### AJ. Mission Waypoint Files (Phase N1)
- **`test_fw_158_exists`**: FW mission 158 waypoint file exists.
- **`test_fw_178_exists`**: FW mission 178 waypoint file exists.
- **`test_fw_185_exists`**: FW mission 185 waypoint file exists.
- **`test_fw_missions_qgc_format`**: All FW missions start with QGC WPL 110 header.
- **`test_fw_missions_have_antisana_origin`**: FW missions reference Antisana GPS coords.
- **`test_fw_missions_have_waypoints`**: Each FW mission has >=5 waypoints.
- **`test_quad_missions_module_exists`**: Quadrotor missions Python module exists.
- **`test_quad_missions_importable`**: Module importable, contains 4 missions.
- **`test_quad_mission_to_qgc_wpl`**: `mission_to_qgc_wpl()` produces valid QGC format.
- **`test_quad_missions_have_correct_origins`**: Carolina=-0.189, EPN=-0.210.

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
