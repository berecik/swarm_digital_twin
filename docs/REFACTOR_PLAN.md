# Refactoring Plan (v4): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only gaps still open against the paper workflow. Completed items are in section 1. Remaining gaps and new phases are in sections 2-3.

---

## 1) What is already implemented

The following paper-aligned capabilities are in the codebase and covered by `simulation/test_drone_physics.py` (127 tests passing):

| Paper element | Implementation | Tests |
|:---|:---|:---|
| 6-DOF body-frame dynamics (Eq. 3) with Coriolis coupling | `drone_physics.py:physics_step()` body-frame path | TestBodyFrame |
| Euler's rotation equation with full inertia tensor (Eq. 4) | `I_inv @ (torque - cross(omega, I@omega))` — algebraically equivalent to Gamma-term formulation | TestRotation, TestGammaTermEquivalence |
| Quadratic aerodynamic drag, ISA atmosphere (Eq. 5) | `_compute_quadratic_drag()`, `Atmosphere.rho` | TestQuadraticDrag, TestAtmosphere |
| AoA-dependent CL/CD with pre/post-stall (Table 3, Fig. 5) | `FixedWingAero` with `get_CL(alpha)`, `get_CD(alpha)` | TestFixedWingAero (14 tests) |
| Aerodynamic lift perpendicular to velocity (Eq. 6) | `_compute_lift()` in body XZ plane | TestFixedWingAero |
| Aerodynamic pitching moment C_M(alpha) with stall (Table 3) | `FixedWingAero.get_CM()`, applied in `physics_step()` pitch axis | TestPitchingMoment (5 tests) |
| Wind perturbation model (Eq. 5-7): constant, Dryden, from-log, from-log-3d | `wind_model.py:WindField` with drag+lift+3D replay | TestWind (4 tests), TestWindEstimation3D (5 tests) |
| Paper-exact fixed-wing preset (Table 2/3) | `make_valencia_fixed_wing()` with all CFD coefficients + Antisana atmosphere | TestValenciaPreset (6 tests) |
| Gamma-term equivalence verified (Eq. 4 vs matrix form) | Algebraic proof via test with non-diagonal inertia tensor | TestGammaTermEquivalence |
| Flight log parsing (CSV + ArduPilot .bin DataFlash) | `flight_log.py:from_csv()`, `from_bin()` with FMT-based message parsing | TestValidation, TestFlightLogBin (4 tests) |
| Wind profile extraction from altitude deviations + 3D lateral | `get_wind_profile()`, `get_wind_profile_3d()` | TestValidation, TestWindEstimation3D |
| Sim-vs-real comparison pipeline (Table 5 format) | `validation.py:compare_sim_real()` producing per-axis RMSE, median, percentiles | TestSimVsRealComparison (4 tests) |
| Throttle/elevator signal comparison (Fig. 9/10) | `validation.py:compare_signals()`, `plot_signal_comparison()` | TestSimVsRealComparison |
| Terrain model: flat, grid, STL, analytical, SRTM .hgt | `terrain.py:TerrainMap` with bilinear interpolation, collision checks | TestTerrain (6 tests), TestTerrainSRTM (4 tests) |
| GPS-referenced terrain queries | `terrain.py:get_elevation_gps(lat, lon)` with local coordinate conversion | TestTerrainSRTM |
| RMSE validation with per-axis quality gates | `validation.py:assert_validation_pass()`, `compute_rmse()` | TestValidation |
| Deterministic benchmark scenarios (4 single-drone + 5 swarm profiles) | `drone_scenario.py`, `swarm_scenario.py` with acceptance envelopes | TestValidation |
| MAVLink v2 bridge (pure Python) | `mavlink_bridge.py`: heartbeat, attitude, GPS, HUD, command parsing | TestMAVLink (10 tests) |
| Cascaded PID position controller | `PositionController` in `drone_physics.py` | TestPositionController |
| SVD re-orthogonalization for rotation matrix drift | `physics_step()` SVD path | TestRotation |
| Airframe presets with parameter provenance + runtime warnings | `make_generic_quad()`, `make_fixed_wing()`, `make_holybro_x500()`, `make_valencia_fixed_wing()` | TestFixedWingAero, TestInertiaPresets, TestValenciaPreset |
| N-drone swarm simulation with boids flocking + avoidance | `run_swarm_simulation()`, `calculate_flocking_vector()` | TestSwarmStandaloneTwin (4 tests) |
| Boids parity with Rust reference | Python mirror matches Rust `calculate_flocking_vector` | TestSwarmStandaloneTwin |
| Gazebo aircraft SDF with LiftDrag plugin (Table 2/3 params) | `gazebo/models/valencia_fixed_wing/model.sdf` | TestGazeboModels (7 tests) |
| Antisana Gazebo world + ROS launch file | `gazebo/worlds/antisana.world`, `gazebo/launch/antisana.launch` | TestGazeboModels |
| ArduPilot parameter file | `gazebo/models/valencia_fixed_wing/valencia_fw.parm` | TestGazeboModels |
| ROS 2 wind perturbation node | `gazebo/scripts/wind_node.py` — publishes velocity+force at 50Hz | manual (requires ROS 2) |
| SITL mission lifecycle script | `scripts/run_sitl_mission.sh` — 6-step lifecycle with exit codes 0-5 | TestSITLLifecycle (2 tests) |
| Docker compose stack (3 drones + swarm + perception) | `docker-compose.yml` with `phase_b_stack` profile | documented |
| CI validation pipeline | `.github/workflows/ci.yml` — pytest + benchmarks + artifact upload | TestCIPipeline (3 tests) |

Verification:

```bash
cd simulation && pytest -q test_drone_physics.py    # 148 tests
```

---

## 2) Remaining gaps vs the paper framework

| # | Area | Current status | Gap vs paper | Priority |
|:--|:---|:---|:---|:---|
| J1 | Paper-exact quadrotor preset (IRS-4) | `make_holybro_x500()` is generic, not matching paper's quadrotor | Paper validates with IRS-4 quadrotor at Carolina (2800m) and EPN campuses. Need preset with correct mass, motor config, PID tuning for Section 3.2 experiments | **P0** |
| J2 | Automated mission replay pipeline | `compare_sim_real()` exists but no end-to-end replay from .bin | Paper Table 4 has 7 missions (158, 178, 185, Carolina-40/20, EPN-30/20). Need: load .bin → extract waypoints + wind → run sim → produce Table 5 metrics automatically | **P0** |
| J3 | Paper Table 5 acceptance targets | No tests validate against paper's specific RMSE values | Fixed-wing RMSE_Z < 2.0m per mission, quadrotor RMSE_Z < 0.1m. Need parametrized tests with paper's exact thresholds | **P0** |
| K1 | ArduPilot SITL in Docker compose | Current compose uses placeholder containers (`tail -f /dev/null`) | Paper Fig. 1 pipeline requires actual ArduPilot SITL container with proper UDP ports (9003/9004, 14550/14555) connecting to Gazebo | **P1** |
| K2 | Quadrotor Gazebo model (IRS-4) | Only `x500` basic model exists | Paper Fig. 7 shows IRS-4 digital twin. Need SDF with correct mass, motor layout, PID params for urban flight validation | **P1** |
| L1 | SRTM → STL mesh conversion for Gazebo | `from_srtm()` downloads .hgt but no STL export | Paper Section 2.1 pipeline: SRTM → BlenderGIS → STL → Gazebo collision mesh. Need automated heightmap-to-STL converter | **P1** |
| L2 | Terrain texture pipeline | Ground plane is flat green color | Paper Fig. 3 uses Google Earth satellite textures on terrain mesh. At minimum, need elevation-based coloring on the terrain | **P2** |
| M1 | Wind node position subscription | `wind_node.py` uses `pos = np.zeros(3)` (hardcoded) | Wind force should vary with drone position (altitude-dependent density, terrain-aware wind). Need ROS subscription to `/mavros/local_position/pose` | **P1** |
| M2 | Euler angle rate kinematics (Eq. 2) | Not explicitly implemented (rotation matrix used instead) | Paper Eq. 2 provides `[phi_dot, theta_dot, psi_dot] = E(phi,theta) @ [p,q,r]`. For telemetry/analysis, expose an `euler_rates_from_body_rates()` utility | **P2** |
| N1 | Paper mission waypoint files (Table 4) | No mission files for the 7 paper experiments | Need QGC-format waypoint files or Python mission defs for missions 158, 178, 185, Carolina-40/20, EPN-30/20 — enables repeatable validation | **P1** |

---

## 3) Active refactor roadmap

### Phase J — Quadrotor preset and mission replay (**P0**)

#### J1. Paper-exact quadrotor preset

- **Target files:** `simulation/drone_physics.py`
- **Deliverables:**
  - `make_irs4_quadrotor()` preset matching paper's IRS-4 platform (Fig. 7b)
  - Correct mass, motor layout, PID tuning, aero coefficients matching default ArduPilot IRS-4 config
  - `Atmosphere(altitude_msl=2800.0)` for Carolina/EPN experiments
- **Acceptance criteria:**
  - RMSE_Z < 0.1m for simulated quadrotor hover in calm conditions (matching paper Section 3.2)

#### J2. Automated mission replay pipeline

- **Target files:** `simulation/drone_scenario.py` (new replay mode)
- **Deliverables:**
  - `replay_mission(bin_path, airframe, wind_source)` function
  - Loads .bin → extracts GPS waypoints + timestamps → extracts wind profile from altitude deviations
  - Runs simulation with matching airframe preset + atmosphere + wind
  - Returns `compare_sim_real()` metrics + optional plot generation
- **Acceptance criteria:**
  - `replay_mission("mission_185.bin", make_valencia_fixed_wing())` produces RMSE_Z < 2.0m

#### J3. Paper Table 5 acceptance tests

- **Target files:** `simulation/test_drone_physics.py`
- **Deliverables:**
  - Parametrized test `TestPaperValidation` with synthetic mission data matching paper thresholds
  - Fixed-wing: RMSE_Z ≤ 2.0m, RMSE_X ≤ 1.8m, RMSE_Y ≤ 1.3m
  - Quadrotor: RMSE_Z ≤ 0.1m, RMSE_X ≤ 0.07m, RMSE_Y ≤ 0.07m
- **Acceptance criteria:**
  - Tests validate that sim-vs-real pipeline produces metrics within paper's reported ranges

### Phase K — Docker SITL and quadrotor Gazebo model (**P1**)

#### K1. ArduPilot SITL Docker container

- **Target files:** `docker-compose.yml`, `Dockerfile.sitl` (new)
- **Deliverables:**
  - Dockerfile building ArduPilot SITL (plane + copter)
  - Compose service `ardupilot_sitl` with proper UDP ports (9002/9003 for Gazebo, 14550 for QGC)
  - Health check: wait for MAVLink heartbeat on port 14550
- **Acceptance criteria:**
  - `docker compose up ardupilot_sitl` starts SITL and responds to MAVLink heartbeat within 30s

#### K2. IRS-4 quadrotor Gazebo model

- **Target files:** `gazebo/models/irs4_quadrotor/model.sdf`, `model.config`
- **Deliverables:**
  - SDF with 4 rotors, correct mass/inertia for paper's IRS-4 platform
  - ArduPilot plugin configured for copter frame
  - `.parm` file with PID gains matching paper's urban experiments
- **Acceptance criteria:**
  - Model loads in Gazebo, hover stable, responds to MAVLink commands

### Phase L — Terrain mesh pipeline (**P1/P2**)

#### L1. SRTM heightmap to STL converter

- **Target files:** `simulation/terrain.py` (new `export_stl()` method)
- **Deliverables:**
  - `TerrainMap.export_stl(path)` — writes binary STL mesh from elevation grid
  - Proper triangle tessellation of the heightmap grid
  - Scale factor to match Gazebo world coordinates
- **Acceptance criteria:**
  - Generated STL loads in Gazebo and matches SRTM elevations within 1m

#### L2. Elevation-based terrain coloring (optional)

- **Target files:** `gazebo/worlds/antisana.world` or separate material file
- **Deliverables:**
  - Height-based color gradient (green lowlands → brown highlands → white peaks)
  - Or satellite texture UV-mapped onto terrain mesh
- **Acceptance criteria:**
  - Visual terrain distinguishable from flat ground in Gazebo

### Phase M — Wind node and kinematic utilities (**P1/P2**)

#### M1. Position-aware wind node

- **Target files:** `gazebo/scripts/wind_node.py`
- **Deliverables:**
  - Subscribe to `/mavros/local_position/pose` for drone position
  - Wind force varies with altitude (density changes) and optionally terrain proximity
- **Acceptance criteria:**
  - Wind force at 4500m differs from sea level by the ISA density ratio

#### M2. Euler angle rate utility (Eq. 2)

- **Target files:** `simulation/drone_physics.py`
- **Deliverables:**
  - `euler_rates_from_body_rates(phi, theta, p, q, r)` implementing paper Eq. 2
  - Useful for telemetry analysis and comparison with real ATT log messages
- **Acceptance criteria:**
  - Test verifies `euler_rates` matches numerical differentiation of Euler angles from rotation matrix

### Phase N — Paper mission definitions (**P1**)

#### N1. Mission waypoint files

- **Target files:** `missions/` directory (new)
- **Deliverables:**
  - QGC WPL 110 format files for Antisana missions (158, 178, 185)
  - Python dict definitions for Carolina-40/20 and EPN-30/20 quadrotor missions
  - GPS origins, altitudes, and approximate waypoints from paper Fig. 8/11
- **Acceptance criteria:**
  - Mission files load with `run_sitl_mission.sh --mission missions/fw_185.waypoints`

---

## 4) Verification matrix

| Item category | Verification method | Current status |
|:---|:---|:---|
| Physics + wind + terrain + fixed-wing + MAVLink + swarm | `pytest simulation/test_drone_physics.py` (150 tests) | **Done** |
| Deterministic benchmarks + acceptance gates | `bash run_scenario.sh --benchmark` (4 + 5 profiles) | **Done** |
| Paper-exact fixed-wing preset (Table 2/3) | TestValenciaPreset + TestPitchingMoment + TestGammaTermEquivalence (18 tests) | **Done** |
| Flight log parsing (.csv + .bin) | TestValidation + TestFlightLogBin (8 tests) | **Done** |
| Sim-vs-real comparison | TestSimVsRealComparison (4 tests) | **Done** |
| SRTM terrain + GPS queries | TestTerrainSRTM (4 tests) | **Done** |
| Gazebo models + world | TestGazeboModels (7 tests) | **Done** |
| SITL lifecycle + CI | TestSITLLifecycle (2 tests) + TestCIPipeline (3 tests) | **Done** |
| 3D wind estimation | TestWindEstimation3D (5 tests) | **Done** |
| IRS-4 quadrotor preset (Phase J1) | TestIRS4Preset (7 tests) — mass, atmosphere, aero, hover, tracking | **Done** |
| Mission replay pipeline (Phase J2) | TestMissionReplay (5 tests) — metrics, quad/FW, wind, validation | **Done** |
| Paper Table 5 RMSE acceptance (Phase J3) | TestPaperValidation (9 tests) — hover accuracy, determinism, format | **Done** |
| ArduPilot SITL Docker (Phase K1) | `docker compose up ardupilot_sitl` + heartbeat | **Pending** |
| IRS-4 Gazebo model (Phase K2) | TestGazeboModels extended | **Pending** |
| STL terrain export (Phase L1) | Test STL loads + elevation check | **Pending** |
| Wind node position-aware (Phase M1) | Manual test with ROS 2 | **Pending** |
| Euler rate utility (Phase M2) | TestEulerRates vs numerical differentiation | **Pending** |
| Mission files (Phase N1) | TestMissionFiles — files exist + parseable | **Pending** |

---

## 5) Execution order

1. **Phase J (P0)** — Quadrotor preset + mission replay + Table 5 acceptance. Core paper validation.
2. **Phase K (P1)** — ArduPilot SITL Docker + quadrotor Gazebo. Makes the full Fig. 1 pipeline real.
3. **Phase N (P1)** — Mission waypoint files. Enables repeatable validation.
4. **Phase L (P1/P2)** — Terrain mesh export. Visual fidelity for Gazebo.
5. **Phase M (P1/P2)** — Wind node improvements + kinematic utilities. Polish.

---

## 6) Notes

- Keep this document as a **delta backlog** only; move completed items to section 1.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
- Previous v3 plan phases D-I are all completed and moved to section 1.
