# TESTING.md - Swarm System Test Status

This document tracks the high-level testing status and provides detailed explanations of the verification suite across the Swarm Digital Twin project.

## Current Status (2026-04-21)

| Module | Unit Tests | Integration Tests | SITL / Hardware | Status |
| :--- | :---: | :---: | :---: | :--- |
| `swarm_control_core` (Rust) | ✅ Pass (17)* | ⏳ Pending | ✅ Pass (Sim) | Boids & Mission FSM + Transport + Timing Verified. |
| `perception_core` (Python) | ✅ Pass (13) | ⏳ Pending | ✅ Pass (Sim) | 3D Localization & Lawnmower Verified |
| `heavy_lift_core` (Rust) | ✅ Pass (1) | ⏳ Pending | ⏳ Pending | Extraction State Machine Verified |
| **Drone Physics + Run-time View + ML/CV** (Python) | ✅ Pass (603) | ✅ Pass (Scenario + 6 FW/IRS-4 Benchmarks + Swarm parity + real-log gate + aero-area gate + battery/energy gate + terrain satellite-texture gate + wind auto-tuning gate + trajectory-tracking validation + live telemetry/runtime-view integration + ML pipeline cross-module + single-drone PID waypoint tuning) | N/A | Full physics + terrain + fixed-wing + MAVLink + sensor noise + motor dynamics + fixed-wing control surfaces + validation gates + real flight log validation + trajectory tracking + quadrotor effective aero-area model + battery & energy model + satellite-texture terrain overlay + wind disturbance auto-tuning + FastAPI/Three.js live view telemetry pipeline + SAR detection ML pipeline + single-drone waypoint-achievement optimiser (`simulation/ml/`, 150 tests across 12 files) |
| **Swarm Simulation** | - | ✅ Pass (3) | ✅ Pass (Sim) | Mock Drone Flight Logic Verified |

## K8s + Gazebo Baseline (verified 2026-04-19)

All baseline items implemented. Docs synchronized.

| Check | Command | Status |
| :--- | :--- | :--- |
| Namespace commands idempotent | Review `todo/k8s_namespace_lifecycle.md` | ✅ Uses `--dry-run=client \| kubectl apply` |
| Helm playground profile | `helm lint helm/swarm-digital-twin/ -f helm/swarm-digital-twin/values-playground.yaml` | ✅ Lint passes |
| ResourceQuota template | `helm template sim helm/swarm-digital-twin/ -f helm/swarm-digital-twin/values-playground.yaml \| grep ResourceQuota` | ✅ Rendered |
| Topology Helm test | `test -f helm/swarm-digital-twin/templates/tests/test-service-topology.yaml` | ✅ Exists |
| Operational runbook | `test -f docs/k8s_runbook.md` | ✅ Exists |

## Real Physics Parity (verified 2026-04-19)

All parity items implemented. 9 tests in `TestPhysicsParity`.

| Check | Command | Status |
| :--- | :--- | :--- |
| Parity tests pass | `.venv/bin/python -m pytest simulation/test_drone_physics.py::TestPhysicsParity -v` | ✅ 9 passed |
| SDF parameter match | `.venv/bin/python -m pytest ...::test_sdf_parameter_match` | ✅ All params match |
| Trajectory RMSE gates | `.venv/bin/python -m pytest ...::test_trajectory_comparison_*` | ✅ Pass/fail verified |
| Timing determinism | `.venv/bin/python -m pytest ...::test_timing_determinism_*` | ✅ Stable/jittery verified |
| Truth CSV export | `.venv/bin/python -m pytest ...::test_truth_csv_roundtrip` | ✅ Roundtrip verified |
| Helm parity profile | `helm lint helm/swarm-digital-twin/ -f helm/swarm-digital-twin/values-parity.yaml` | ✅ Lint passes |

Remaining (requires K8s cluster):
- [ ] Attitude RMSE gate (< 5 deg) — threshold defined, not yet wired as test
- [ ] Energy consumption delta (< 15%) — threshold defined, not yet wired as test
- [ ] End-to-end K8s parity against real Gazebo traces

## Collision Detection & Safety (verified 2026-04-19)

Detection and KPI foundations implemented. 10 tests in `TestSafetyMonitor`.

| Check | Command | Status |
| :--- | :--- | :--- |
| Safety tests pass | `.venv/bin/python -m pytest simulation/test_drone_physics.py::TestSafetyMonitor -v` | ✅ 10 passed |
| SeparationMonitor | `...::test_collision_detected`, `...::test_near_miss_detected` | ✅ Verified |
| TerrainMonitor | `...::test_terrain_collision_detected`, `...::test_clearance_violation_detected` | ✅ Verified |
| SafetyReport | `...::test_safety_report_from_monitors`, `...::test_safety_report_unsafe_on_collision` | ✅ Verified |
| Swarm benchmark | `...::test_full_swarm_simulation_produces_report` | ✅ Produces valid report |
| Safety module exists | `test -f simulation/safety.py` | ✅ Exists |

Remaining (requires PX4 integration):
- [ ] Safety response playbook (HOVER/RTL triggers on critical events)
- [ ] Safety response latency < 2 s
- [ ] KPI export in acceptance report JSON format

\* *Note: Rust tests for `swarm_control_core` require a sourced ROS 2 environment for compilation due to `rclrs` dependency.*

Additional consensus/safety validation introduced:
- `consensus::tests::mission_command_round_trip`
- `consensus::tests::raft_message_round_trip`
- `consensus::tests::rejects_cluster_smaller_than_six`
- `px4_safety::tests::rejects_non_finite_setpoint`

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

#### A1. Real Flight Data Validation (Paper Table 5)
- **`./run_scenario.sh --real-log`** / **`python simulation/drone_scenario.py --real-log`**:
    - **Purpose**: Reproduce paper-aligned Table 5 validation against real OSSITLQUAD flight logs for Carolina/EPN mission windows.
    - **Input**: `Carolina_quad_40m_plus_20m.bin` and `EPN_quad_30m_plus_20m.bin` (auto-downloaded to `data/flight_logs/` if missing), mission profiles `quad_carolina_40`, `quad_carolina_20`, `quad_epn_30`, `quad_epn_20`.
    - **Expected Outcome**: Each mission passes RMSE gate `rmse_z/x/y <= 2x` paper-reported Table 5 values.

#### A0. Reproducible Validation Baseline
- **`./run_scenario.sh --benchmark`**:
    - **Purpose**: Runs one-command deterministic benchmark validation gates for both single-drone and swarm profile sets.
    - **Input**: Single-drone canonical profiles (`moderate`, `strong_wind`, `crosswind`, `storm`) from `simulation/validation.py` and swarm profiles (`baseline`, `crosswind`, `gusty`) from `simulation/swarm_scenario.py`, all with fixed seeds.
    - **Expected Outcome**: All profiles pass configured envelopes; repeated runs on same commit produce equivalent metrics within configured tolerance.

#### A2. Quadrotor Effective Aerodynamic Area
- **`TestQuadrotorAeroArea::test_effective_area_increases_with_tilt`**:
    - **Purpose**: Verifies effective drag area is attitude-dependent.
    - **Input**: `QuadrotorAero` at 0° and 35° tilt with fixed thrust ratio.
    - **Expected Outcome**: Effective area at 35° is greater than at level attitude.
- **`TestQuadrotorAeroArea::test_effective_area_increases_with_prop_wash`**:
    - **Purpose**: Verifies prop-wash influence on effective drag area.
    - **Input**: Same tilt angle, thrust ratio sweep from 0.0 to 0.9.
    - **Expected Outcome**: Effective area at high thrust is greater than at idle thrust.
- **`TestQuadrotorAeroArea::test_drag_force_responds_to_tilt_and_prop_wash`**:
    - **Purpose**: Verifies aerodynamic drag force reflects the new effective-area model.
    - **Input**: Constant body velocity, varying tilt and thrust-ratio inputs to `_compute_quadratic_drag()`.
    - **Expected Outcome**: Drag magnitude increases from level/low-thrust → tilted/low-thrust → tilted/high-thrust.
- **`TestIRS4Preset::test_irs4_hover_stable`**:
    - **Purpose**: Regression check that IRS-4 hover remains stable after aero-area changes.
    - **Input**: `make_irs4_quadrotor()` simulation hover run.
    - **Expected Outcome**: Hover vertical RMSE remains within configured threshold (`< 1.5 m`).

#### A3. Satellite Texture Terrain Overlay
- **`TestTerrainSatelliteTexture::test_satellite_tile_download_has_offline_fallback`**:
    - **Purpose**: Verifies satellite texture acquisition is available for SRTM region and deterministic offline fallback exists.
    - **Input**: `download_satellite_tile(-0.508333, -78.141667, cache_dir, tile_size=64)`.
    - **Expected Outcome**: Returns a valid `.ppm` texture path and creates tile asset in cache directory.
- **`TestTerrainSatelliteTexture::test_export_obj_with_uv_contains_uv_and_faces`**:
    - **Purpose**: Verifies UV-mapped mesh export path used for textured Gazebo terrain.
    - **Input**: Small synthetic terrain grid + `export_obj_with_uv()` with a test texture.
    - **Expected Outcome**: Exported OBJ contains `vt` UV coordinates and `f` faces; MTL includes diffuse texture binding (`map_Kd`).
- **`TestTerrainSatelliteTexture::test_export_assets_fallbacks_to_height_material_without_texture`**:
    - **Purpose**: Verifies fallback behavior when satellite tile is unavailable.
    - **Input**: `export_gazebo_terrain_assets(..., texture_path=<missing>)`.
    - **Expected Outcome**: Returned material is `AntisanaTerrain/HeightColored` and mesh assets are still generated.
- **`TestTerrainSatelliteTexture::test_world_and_material_include_satellite_reference`**:
    - **Purpose**: Verifies Gazebo world/material wiring for textured terrain rendering.
    - **Input**: `gazebo/worlds/antisana.world` and `gazebo/media/materials/scripts/antisana_terrain.material`.
    - **Expected Outcome**: Material defines `AntisanaTerrain/SatelliteTextured` with satellite diffuse texture; world includes the textured material reference.

#### A4. Wind Disturbance Auto-Tuning
- **`TestWindAutoTuning::test_auto_tuning_converges_and_recovers_scale`**:
    - **Purpose**: Verifies iterative wind-force scaling optimization converges and minimizes altitude RMSE.
    - **Input**: Synthetic reference trajectory with known wind scale (`1.7`), deterministic simulation callback, convergence tolerance `0.01 m`.
    - **Expected Outcome**: Auto-tuning converges, `best_rmse_z < 0.01 m`, and recovered scale is within `±0.05` of ground truth.
- **`TestWindAutoTuning::test_auto_tuning_is_reproducible`**:
    - **Purpose**: Verifies deterministic, reproducible per-mission calibration constants.
    - **Input**: Identical reference signal and simulation callback evaluated in two consecutive runs.
    - **Expected Outcome**: Both runs produce identical `best_scale`, `best_rmse_z`, and full optimization history.

#### A5. Trajectory Tracking and Real-Log Validation
- **`TestTrajectoryTracking::test_trajectory_tracking_follows_reference`**:
    - **Purpose**: Verifies `run_trajectory_tracking` follows a reference trajectory with sub-meter RMSE.
    - **Input**: Gentle helical path, IRS-4 quadrotor, no wind.
    - **Expected Outcome**: RMSE_z < 1.0m, RMSE_x < 1.5m, RMSE_y < 1.5m.
- **`TestTrajectoryTracking::test_trajectory_tracking_starts_at_ref_origin`**:
    - **Purpose**: Simulation initial position matches the reference trajectory start point.
    - **Input**: Reference starting at (10, 20, 5).
    - **Expected Outcome**: First sim record position matches within 0.01m.
- **`TestTrajectoryTracking::test_trajectory_tracking_rejects_short_ref`**:
    - **Purpose**: Validates input guard for minimum reference length.
    - **Input**: Single-point reference.
    - **Expected Outcome**: `ValueError` raised.
- **`TestTrajectoryTracking::test_real_log_segments_have_data`**:
    - **Purpose**: All 4 mission segment boundaries yield sufficient GPS data points.
    - **Input**: `REAL_LOG_MISSIONS` segment masks applied to downloaded flight logs.
    - **Expected Outcome**: Each segment has >= 10 GPS points (catches gap/boundary misalignment).

#### A6. Battery and Energy Model
- **`TestBatteryModel::test_lipo_voltage_curve_is_soc_dependent`**:
    - **Purpose**: Verifies Li-Po open-circuit voltage follows SoC (full > mid > empty) and known endpoint voltages.
    - **Input**: `BatteryModel(cells=6)` evaluated at `SoC={1.0, 0.5, 0.0}`.
    - **Expected Outcome**: Monotonic voltage decrease with discharge; full pack ≈ `25.2V`, empty pack ≈ `19.8V`.
- **`TestBatteryModel::test_motor_power_draw_reduces_soc`**:
    - **Purpose**: Verifies battery discharge is driven by motor mechanical power `P=τ·ω` through the physics loop.
    - **Input**: IRS-4-like quad config with motor dynamics and battery model under sustained thrust command.
    - **Expected Outcome**: Positive battery power/current, `SoC` decreases from 1.0, and finite remaining-time estimate is produced.
- **`TestBatteryModel::test_fixed_wing_autonomy_estimate_matches_table2`**:
    - **Purpose**: Verifies fixed-wing endurance estimate aligns with paper Table 2 mission autonomy target.
    - **Input**: `make_valencia_fixed_wing()` battery model with representative cruise power (`152W`).
    - **Expected Outcome**: Estimated endurance is approximately `85 min` (within configured tolerance).
- **`TestMAVLink::test_sys_status_encode_decode_battery_fields`**:
    - **Purpose**: Verifies MAVLink `SYS_STATUS` battery telemetry fields are correctly encoded/decoded.
    - **Input**: `build_sys_status(voltage_mv=22200, current_ca=1234, battery_pct=76)`.
    - **Expected Outcome**: Decoded fields preserve voltage/current/remaining battery values.

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

#### I. Quadratic Drag
- **`test_quadratic_drag_scales_with_v_squared`**: Drag force quadruples when velocity doubles (V² scaling).
- **`test_high_altitude_less_drag`**: Same velocity at 4500m ASL produces ~63% of sea-level drag.
- **`test_terminal_velocity`**: Freefall with quadratic drag converges to finite terminal velocity (variation < 5%).

#### J. Atmosphere Model (ISA)
- **`test_sea_level_density`**: ISA sea level density = 1.225 kg/m³.
- **`test_high_altitude_density`**: ISA at 4500m = ~0.77 kg/m³.

#### K. Wind Model
- **`test_no_wind_unchanged`**: `turbulence_type="none"` produces zero force.
- **`test_constant_wind_deflects_hover`**: Hovering drone drifts > 1m downwind in 10s with 5 m/s wind.
- **`test_stronger_wind_more_force`**: Wind force scales with V² (10 m/s wind → 4x force of 5 m/s).
- **`test_wind_from_log_matches_data`**: `from_log` interpolates altitude profile correctly.

#### L. Inertia & Presets
- **`test_off_diagonal_inertia_coupling`**: Off-diagonal inertia terms cause cross-axis angular velocity coupling.
- **`test_preset_loading`**: `make_generic_quad()` and `make_holybro_x500()` return correct parameters.

#### M. Body-Frame Dynamics (Eq. 3)
- **`test_body_frame_hover_equivalent`**: Body-frame quadratic drag hover matches world-frame linear drag hover at ~10m.
- **`test_body_frame_freefall`**: Body-frame freefall (C_D=0) matches analytical kinematics.

#### N. Validation Module (RMSE / Envelopes)
- **`test_rmse_identical`**: RMSE of identical trajectories = 0.
- **`test_rmse_known_offset`**: Constant 1m X-offset → RMSE_x = 1.0.
- **`test_flight_log_csv_roundtrip`**: FlightLog parses CSV, returns correct shape, origin at (0,0,0).
- **`test_validation_gate_passes_within_envelope`**: Gate accepts metrics that remain below all configured envelope thresholds.
- **`test_validation_gate_fails_outside_envelope`**: Gate rejects metrics above threshold and raises `AssertionError`.
- **`test_benchmark_profiles_are_deterministic`**: All canonical benchmark profiles (`moderate`, `strong_wind`, `crosswind`, `storm`) produce repeatable RMSE values within profile tolerance across two consecutive runs.
- **`test_swarm_benchmark_profiles_are_deterministic`**: All canonical swarm benchmark profiles (`baseline`, `crosswind`, `gusty`, `tight_ring`, `high_altitude`) produce repeatable safety/tracking metrics within profile tolerance across two consecutive runs.
- **`test_swarm_benchmark_profiles_stay_within_envelopes`**: Every swarm profile satisfies its configured envelope for separation (`min_separation`, `p05_separation`), tracking (`mean/p75/max error`), and kinematics (`mean_speed`, `p90_speed`).
- **`test_swarm_profile_risk_ordering`**: Relative scenario difficulty remains coherent: gusty conditions are harder than baseline (tracking/speed), while tight-ring reduces minimum separation versus baseline.

#### N2. Runtime Guardrails (Aero Parameter Warnings)
- **`test_runtime_warning_for_out_of_range_aero_params`**: Out-of-range aerodynamic coefficients emit one-shot runtime warning tied to documented validity envelopes.

#### O. Terrain Model
- **`test_flat_terrain_elevation`**: Flat terrain returns constant elevation at all query points.
- **`test_from_array_elevation_query`**: Grid-based terrain uses bilinear interpolation (exact + midpoint checks).
- **`test_terrain_collision_detection`**: `check_collision()` detects below/at/above terrain surface.
- **`test_terrain_in_physics_step`**: Drone with zero thrust lands on 20m terrain, not z=0.
- **`test_from_function_terrain`**: `from_function(lambda x,y: 0.1*x)` slope gives correct elevation.
- **`test_stl_file_not_found`**: Nonexistent STL raises FileNotFoundError.

#### P. Fixed-Wing Aerodynamics (AoA / Stall)
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

#### P2. Fixed-Wing Control Surfaces
- **`test_elevator_pitch_response_matches_control_effectiveness`**: Elevator deflection produces pitch response consistent with the configured control-effectiveness model within 10% tolerance.
- **`test_control_surface_rate_limit_prevents_instant_step`**: Elevator/aileron/rudder deflections are rate-limited and cannot jump instantaneously in one physics step.

#### Q. MAVLink Bridge
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

#### S. Pitching Moment (Paper Table 3)
- **`test_pitching_moment_pre_stall`**: C_M is negative and proportional to alpha below stall (stabilizing).
- **`test_pitching_moment_post_stall`**: C_M uses C_Ma_stall slope above stall angle.
- **`test_pitching_moment_zero_at_zero_alpha`**: No pitching moment at zero AoA.
- **`test_pitching_moment_applied_in_physics_step`**: Physics step applies aero pitching moment on pitch axis for fixed-wing.
- **`test_quadrotor_has_no_pitching_moment`**: Base AeroCoefficients returns zero C_M.

#### T. Valencia Paper-Exact Preset (Table 2/3)
- **`test_valencia_fixed_wing_mass`**: Mass matches paper Table 2 (2.5 kg).
- **`test_valencia_fixed_wing_ref_area`**: Wing reference area = 0.3997 m^2.
- **`test_valencia_fixed_wing_chord`**: Chord = 0.235 m.
- **`test_valencia_fixed_wing_aero_coefficients`**: All Table 3 CFD coefficients match.
- **`test_valencia_fixed_wing_high_altitude_atmosphere`**: Antisana atmosphere at 4500m MSL, rho ~0.77.
- **`test_valencia_fixed_wing_passes_provenance_check`**: No out-of-range warnings.

#### U. Gamma-Term Equivalence (Eq. 4)
- **`test_gamma_terms_match_matrix_form`**: Expanded paper Eq. 4 Gamma terms match matrix-form `I_inv @ (tau - omega x I@omega)` within 1e-12 for non-diagonal inertia tensor.

#### V. Flight Log Binary Parser (DataFlash .bin)
- **`test_bin_parser_creates_flight_log`**: Synthetic DataFlash binary with FMT+GPS messages parses into FlightLog with correct positions.
- **`test_bin_parser_file_not_found`**: Missing .bin file raises FileNotFoundError.
- **`test_get_elevator_returns_pitch_rate`**: `get_elevator()` returns pitch rate derivative as elevator estimate.
- **`test_extract_waypoints_from_dwell`**: `extract_waypoints()` detects dwell points where speed drops below threshold.

#### W. Sim-vs-Real Comparison
- **`test_compare_sim_real_identical`**: Identical sim and real trajectories produce zero RMSE in all axes.
- **`test_compare_sim_real_known_offset`**: Constant 5m Z-offset produces expected RMSE and percentile metrics.
- **`test_compare_signals_perfect_match`**: Identical signals produce RMSE=0 and correlation=1.0.
- **`test_compare_signals_anticorrelated`**: Anticorrelated signals produce correlation=-1.0.

#### X. SRTM Terrain Pipeline
- **`test_from_hgt_synthetic`**: Synthetic .hgt file (3601x3601 int16 big-endian) loads into TerrainMap with correct GPS origin.
- **`test_gps_elevation_query`**: `get_elevation_gps(lat, lon)` returns correct elevation for known GPS coordinates.
- **`test_gps_query_without_origin_raises`**: GPS query on TerrainMap without `origin_gps` raises ValueError.
- **`test_hgt_parser_file_not_found`**: Missing .hgt file raises FileNotFoundError.

#### Y. Gazebo Model Integration
- **`test_sdf_model_exists`**: `gazebo/models/valencia_fixed_wing/model.sdf` file exists.
- **`test_sdf_model_config_exists`**: `gazebo/models/valencia_fixed_wing/model.config` file exists.
- **`test_sdf_contains_liftdrag_plugin`**: SDF file contains `<plugin>` with LiftDrag configuration.
- **`test_sdf_mass_matches_paper`**: SDF inertial mass matches paper Table 2 (2.5 kg).
- **`test_antisana_world_exists`**: `gazebo/worlds/antisana.world` file exists.
- **`test_antisana_world_gps_origin`**: World file contains Antisana GPS coordinates (-0.508333, -78.141667).
- **`test_parm_file_exists`**: `gazebo/models/valencia_fixed_wing/valencia_fw.parm` file exists.

#### Z. SITL Mission Lifecycle
- **`test_sitl_script_exists`**: `scripts/run_sitl_mission.sh` exists and is executable.
- **`test_sitl_script_has_required_steps`**: Script contains all 6 lifecycle steps (start, health check, upload, arm, capture, validate).

#### AA. 3D Wind Estimation
- **`test_still_trajectory_gives_zero_wind`**: Stationary drone produces near-zero 3D wind estimate.
- **`test_constant_drift_detected`**: Sinusoidal lateral deviation produces non-zero Y-component wind.
- **`test_3d_profile_has_correct_shape`**: 3D wind profile returns Nx4 array [t, wx, wy, wz].
- **`test_3d_profile_too_short_returns_zero`**: Trajectory with < 3 points returns zero wind.
- **`test_from_log_3d_wind_replay`**: WindField with `from_log_3d` correctly interpolates 3D wind profile.

#### AB. CI Pipeline
- **`test_ci_workflow_exists`**: `.github/workflows/ci.yml` file exists.
- **`test_ci_workflow_has_required_jobs`**: Workflow contains pytest, benchmark, and upload-artifact steps.
- **`test_ci_workflow_triggers_on_push`**: Workflow triggers on push to master/main.

#### AC. IRS-4 Quadrotor Preset
- **`test_irs4_mass`**: IRS-4 mass is in [1.0, 3.0] kg range for compact quadrotor.
- **`test_irs4_atmosphere_quito`**: Default atmosphere at 2800m MSL, density ~0.93 kg/m^3.
- **`test_irs4_custom_altitude`**: Accepts custom altitude for different experiment sites.
- **`test_irs4_has_aero`**: Uses quadratic drag model (not linear fallback), C_L=0.
- **`test_irs4_symmetric_inertia`**: Roll/pitch inertia are symmetric (X-frame).
- **`test_irs4_hover_stable`**: Maintains hover within 0.5m after settling.
- **`test_irs4_waypoint_tracking`**: Tracks square waypoint pattern and returns home.

#### AD. Mission Replay Pipeline
- **`test_replay_returns_metrics`**: `replay_mission()` returns dict with all standard metric keys.
- **`test_replay_quad_produces_valid_rmse`**: Quadrotor replay produces finite, positive RMSE values.
- **`test_replay_fixed_wing_produces_valid_rmse`**: Fixed-wing replay produces finite RMSE.
- **`test_replay_with_wind_from_log`**: Replay with from_log wind completes without error.
- **`test_replay_rejects_short_log`**: Rejects flight logs with fewer than 10 data points.

#### AE. Paper Table 5 Acceptance
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

#### AF. Simulation Bridge (UDP)
- **`test_bridge_message_contract`**: ActionMessage and StatusMessage JSON contract matches Rust wire format.
- **`test_bridge_process_actions`**: SimBridge correctly processes RequestOffboard, RequestArm, PublishSetpoint action batches.
- **`test_bridge_physics_step`**: Armed bridge advances physics upward toward target altitude.
- **`test_bridge_status_message_format`**: Status response contains nav_state, arming_state, position fields.
- **`test_bridge_udp_roundtrip`**: Full UDP roundtrip — client sends actions, bridge processes and returns status.

#### AG. Real-Time Timing Contract
- **`test_physics_step_latency`**: Single `physics_step()` mean < 1ms, p95 < 2ms (CI gate for real-time viability).
- **`test_controller_step_latency`**: `PositionController.compute()` p95 < 1ms.

#### AH. IRS-4 Gazebo Model (SDF / Sensors)
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

#### AI. Docker SITL Configuration
- **`test_dockerfile_sitl_exists`**: Dockerfile.sitl exists.
- **`test_dockerfile_builds_copter_and_plane`**: Dockerfile builds both arducopter and arduplane.
- **`test_dockerfile_exposes_ports`**: Required UDP/TCP ports (9002, 9003, 14550) exposed.
- **`test_dockerfile_has_healthcheck`**: MAVLink heartbeat health check configured.
- **`test_compose_has_sitl_service`**: docker-compose.yml contains ardupilot_sitl service.
- **`test_compose_sitl_ports`**: SITL compose service maps correct ports.
- **`test_sitl_entrypoint_exists`**: Entrypoint script exists and is executable.

#### AJ. Mission Waypoint Files
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

#### AK. Terrain STL Export
- **`test_export_stl_creates_file`**: export_stl writes a binary STL with correct triangle count (2 per grid cell).
- **`test_export_stl_expected_size`**: STL file size = 84 + n_triangles × 50 bytes.
- **`test_export_stl_roundtrip`**: export → from_stl preserves center elevation within 2m.
- **`test_export_stl_with_scale`**: Scale factor correctly multiplies vertex coordinates.
- **`test_export_stl_rejects_1x1`**: 1×1 grid raises ValueError (cannot tessellate).
- **`test_export_stl_normals_point_up`**: Triangle normals on flat terrain point upward (nz > 0.99).

#### AL. Terrain Coloring (Gazebo Shaders)
- **`test_material_file_exists`**: Gazebo material script exists.
- **`test_vertex_shader_exists`**: GLSL vertex shader file exists.
- **`test_fragment_shader_exists`**: GLSL fragment shader file exists.
- **`test_material_references_shaders`**: Material file references both shaders and HeightColored name.
- **`test_fragment_has_elevation_bands`**: Fragment shader contains green_max, brown_max, snow_min uniforms.
- **`test_world_references_material`**: antisana.world references the terrain material.

#### AM. Position-Aware Wind
- **`test_wind_node_has_pose_subscription`**: wind_node.py subscribes to /mavros/local_position/pose.
- **`test_wind_node_has_pose_callback`**: _pose_callback method exists.
- **`test_wind_node_uses_drone_pos`**: Hardcoded `np.zeros(3)` TODO removed, uses self.drone_pos.
- **`test_wind_node_altitude_density`**: _get_altitude_density method and base_altitude_msl parameter exist.
- **`test_isa_density_at_altitude`**: ISA density at 4500m is 55–70% of sea level.

#### AN. Euler Rate Kinematics (Eq. 2)
- **`test_identity_at_zero_attitude`**: At phi=theta=0, euler rates equal body rates.
- **`test_pure_roll_rate`**: Pure p maps to phi_dot only.
- **`test_pitched_yaw_coupling`**: Body yaw rate couples into phi_dot and psi_dot at nonzero pitch.
- **`test_numerical_differentiation`**: Euler rates self-consistent under small perturbation.
- **`test_gimbal_lock_raises`**: ValueError at theta=π/2 (cos(theta)≈0).
- **`test_symmetric_roll`**: Negating phi changes coupling sign on theta_dot.

#### AO. Motor Dynamics (Eq. 4 Rotor Model)
- **`test_motor_step_response_reaches_63pct_near_tau`**: Motor speed reaches ~63% of commanded value after one time-constant (τ=0.05s), validating first-order spin-up dynamics.
- **`test_motor_steady_state_matches_kt_omega_squared`**: At steady state, motor thrust matches `k_T·ω²` relationship from paper Eq. 4.
- **`test_default_physics_step_without_motor_model_stays_backward_compatible`**: With `motor_dynamics_enabled=False`, physics_step produces identical results to legacy behavior.

#### AP. Sensor Noise Models (GPS / IMU / Barometer)
- **`test_gps_noise_quantization_and_statistics`**: GPS output quantized to 1e-7 deg; horizontal CEP < 3.5m; altitude σ < 6m.
- **`test_gps_noise_drift_growth_and_zero_dt_bias_behavior`**: GPS random-walk drift magnitude grows over long horizon (`~sqrt(t)` trend via early/late windows), and `dt=0.0` keeps internal bias unchanged.
- **`test_imu_noise_density_matches_order_of_magnitude`**: Accelerometer/gyro white noise σ matches configured density within 20%.
- **`test_imu_noise_bias_random_walk_develops_slow_offset`**: With nonzero bias RW and zero true signals, IMU outputs develop nonzero mean and slowly varying offset across windows.
- **`test_baro_noise_quantization_and_lag`**: Barometer output quantized to 0.12 hPa; first-order lag visible on step input; sea-level altitude noise σ ≤ 1m.
- **`test_baro_noise_drift_and_multi_dt_lag_consistency`**: Baro bias random walk broadens long-horizon spread, and responses at matched physical time remain consistent across `dt=0.1` vs `dt=1.0` lag regimes.

#### R. Swarm Standalone Twin
- **`test_flocking_vector_returns_zero_without_neighbors`**: Empty neighbor list returns zero steering vector (stable no-neighbor behavior).
- **`test_flocking_vector_excludes_neighbor_at_radius_boundary`**: Neighbor exactly at `neighbor_radius` is excluded, mirroring Rust `<` condition.
- **`test_flocking_vector_matches_rust_reference_case`**: Python boids helper matches hand-derived Rust reference vector.
- **`test_six_agent_run_maintains_min_separation`**: 6-agent deterministic run remains above separation threshold (no collisions).

### 5. `simulation` — Swarm Sim (`test_swarm_flight.py`)
- **`test_swarm_flight`**:
    - **Purpose**: Validates the end-to-end swarm flight logic in the mock simulator.
    - **Verification**: Ensures drones reach targets and maintain boid constraints.

## 🤖 ML/CV Pipeline Tests (verified 2026-04-21)

`simulation/test_ml/` ships **150 tests across 12 files** covering the
ML pipeline scaffolding (`simulation/ml/`) — both the SAR detection
side and the single-drone PID-policy waypoint-achievement optimiser:

| File | Tests | Surface |
| :--- | :---: | :--- |
| `test_sar_targets.py`            | 10 | 21-class catalogue, COCO mapping, frozen dataclass |
| `test_coco_annotator.py`         | 10 | Pinhole projection, multi-frame IDs, partial-bbox clipping, write side-effects |
| `test_image_augment.py`          | 13 | Seeded reproducibility, all 5 augmentations, weather kinds, default spec |
| `test_model_zoo.py`              | 17 | Detector ABC, registry, custom backends, kwargs forwarding, all 6 deferred stubs |
| `test_inference_logger.py`       |  9 | JSONL round-trip, append-mode resume, corrupted-line tolerance |
| `test_hard_example_miner.py`     | 11 | Uncertainty + missed heuristics, score ordering, `to_dict` |
| `test_model_registry.py`         | 13 | Persistence, duplicate rejection, lineage walking, cycle detection |
| `test_kpi.py`                    | 16 | Acceptance thresholds, promotion gate, exact-delta acceptance |
| `test_pipeline_integration.py`   |  7 | Cross-module: annotate→write, augment+annotate, infer→mine, registry+kpi promotion |
| `test_waypoint_optimizer.py`     | 18 | PolicyGains round-trip + apply, episode determinism, baseline completes patrol/lawnmower, divergence handling, multi-mission averaging, search bounds, seeded reproducibility |
| `test_waypoint_kpi.py`           | 17 | Threshold constants, every failure path, promotion gate (energy regression, completion-tie rmse fallback, identical-metrics rejection) |

Reference docs:
- [`docs/ml_pipeline.md`](docs/ml_pipeline.md) ([`pl`](docs/ml_pipeline.pl.md)) — per-module API reference
  with data flow diagram and KPI tables.
- [`docs/ml_tutorial.md`](docs/ml_tutorial.md) ([`pl`](docs/ml_tutorial.pl.md)) — end-to-end developer
  walkthrough: dataset → augment → infer → mine → register → deploy.

Run subsets:
- All ML: `.venv/bin/python -m pytest simulation/test_ml/ -q`
  (141 passed in ~12 s — no GPU/PyTorch required).
- Just detection: `.venv/bin/python -m pytest simulation/test_ml/ -q --ignore=simulation/test_ml/test_waypoint_optimizer.py --ignore=simulation/test_ml/test_waypoint_kpi.py`
- Just waypoint optimizer: `.venv/bin/python -m pytest simulation/test_ml/test_waypoint_optimizer.py simulation/test_ml/test_waypoint_kpi.py -q`

Driver scripts (under `scripts/`):
- `ml_run_pipeline.sh` — end-to-end SAR detection demo.
- `ml_train_waypoint.sh` — random-search PID tuner.
- `ml_evaluate_waypoint.sh` — policy evaluator.

## 📂 Detailed Documentation

- [**Complete Testing Guide**](docs/testing.md) — Strategy, Catalog, and Verification.
- [**ML Pipeline Reference**](docs/ml_pipeline.md) — `simulation/ml/` module reference.
- [**ML Tutorial**](docs/ml_tutorial.md) — Develop & deploy SAR detection on the drones.
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
