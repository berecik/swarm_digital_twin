# Refactoring Plan (v13): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

**Current state:** All paper-aligned work items executed and audited. Core physics (Eq. 1–7), terrain pipeline (SRTM + STL), aerodynamics (Table 2/3 exact coefficients), full inertia tensor (Eq. 4 Gamma terms), MAVLink bridge, validation framework (Table 5 RMSE gates), swarm simulation, sensor noise models (GPS/IMU/baro), motor thrust dynamics, fixed-wing control surfaces, Euler rate kinematics, real-flight-data validation with trajectory tracking, quadrotor effective aerodynamic area modeling, battery/energy modeling, satellite-texture terrain overlay, and wind disturbance auto-tuning are implemented, integrated, and tested. 241 tests passing.

---

## 1) Paper cross-reference — implemented items

All paper equations and tables have been verified against the codebase:

| Paper item | Implementation | Verified by |
|:---|:---|:---|
| Eq. 1 — position kinematics (rotation matrix) | `drone_physics.py` `physics_step()` | `TestRotationMatrix` |
| Eq. 2 — Euler angle rates from body rates | `drone_physics.py` `euler_rates_from_body_rates()` | `TestEulerRates` (6 tests) |
| Eq. 3 — Newton's 2nd law, body frame + Coriolis (ω×v) | `drone_physics.py` line 653 | `TestCoriolisForce` |
| Eq. 4 — rotational dynamics, full 3×3 inertia tensor | `drone_physics.py` `np.linalg.inv(I)`, off-diagonal J_xz | `TestGammaTermEquivalence` |
| Eq. 5–7 — wind drag/lift/combined perturbation | `wind_model.py` `get_force()` | `TestWindForce` |
| Table 2 — fixed-wing geometry (2.20m, 0.235m, 0.3997m², 2.5kg) | `drone_physics.py` `make_valencia_fixed_wing()` | `TestValenciaPreset` |
| Table 3 — CFD aero coefficients (α₀, C_Lα, C_Dα, C_Mα, α_stall, all stall coeffs) | `drone_physics.py` `FixedWingAero` | `TestValenciaPreset::test_valencia_fixed_wing_aero_coefficients` |
| Table 4 — 7 mission profiles (FW 158/178/185, Quad Carolina/EPN) | `validation.py` `REAL_LOG_MISSIONS` | `TestPaperValidation` |
| Table 5 — RMSE validation metrics | `validation.py` acceptance gate + `run_trajectory_tracking` (≤6× for PID, Z-axis ≤2×) | `TestPaperValidation`, `TestTrajectoryTracking` |
| Section 2.1 — SRTM terrain → STL export | `terrain.py` `from_srtm()` + `export_stl()` | `TestTerrainSTLExport` (6 tests) |
| Section 2.3 — wind from real flight log (Z-axis heuristic) | `flight_log.py` `get_wind_profile()`, `wind_model.py` `"from_log"` mode | `TestPositionAwareWind` |
| Section 2.3 — Gazebo LiftDrag plugin interface | `gazebo/worlds/antisana.world` SDF aerodynamics config | Manual (Gazebo launch) |
| Section 3.2 — IRS-4 quadrotor (ArduPilot defaults) | `drone_physics.py` `make_irs4_quadrotor()` | `test_irs4_hover_stable` |
| Sensor noise (GPS/IMU/baro) | `sensor_models.py`, integrated in `sim_bridge.py` + `mavlink_bridge.py` | `TestSensorNoise` (6 tests) |
| Motor dynamics (T = k_T·ω² + k_D·ω) | `drone_physics.py` `MotorModel`, enabled for IRS-4 | `TestMotorDynamics` (3 tests) |
| Control surfaces (elevator/aileron/rudder) | `drone_physics.py` `FixedWingAero`, rate-limited actuation | `TestFixedWingControlSurfaces` (2 tests) |

---

## 2) Remaining gaps

### Quadrotor Effective Aerodynamic Area ✅ Completed (2026-03-27)

**Paper reference:** Section 3.5 — *"An additional limitation of our current approach is that it does not explicitly account for the effective aerodynamic area of the quadrotor."*

Implemented via `QuadrotorAero` in `simulation/drone_physics.py`: effective drag area now varies with tilt angle and prop-wash (thrust ratio), and is integrated into `_compute_quadratic_drag()` through `physics_step()`.

- **Target files:** `simulation/drone_physics.py`, `simulation/test_drone_physics.py`
- **Acceptance criteria:**
  - ✅ Attitude-dependent effective area model for quadrotor (area varies with tilt angle)
  - ✅ Prop-wash effect on effective drag area
  - ✅ IRS-4 hover test still passes with new model
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestQuadrotorAeroArea`
  - `pytest -q simulation/test_drone_physics.py::TestIRS4Preset::test_irs4_hover_stable`

### Battery and Energy Model ✅ Completed (2026-03-27)

**Paper reference:** Section 3.5 — *"one of the most significant [challenges] is related to battery limitations and inherent constraints in flight autonomy... high-energy peaks when adjusting motor speeds for stability and maneuverability... directly affecting the operational range and duration of remote sensing missions"*

Implemented via `BatteryModel` + battery telemetry fields integrated in `simulation/drone_physics.py` (`DroneState` + `physics_step()`), motor mechanical power estimation in `MotorModel.power_from_omega()` (`P = τ·ω`), and `SYS_STATUS` battery reporting in `simulation/mavlink_bridge.py`.

- **Target files:** `simulation/drone_physics.py` (energy model), `simulation/mavlink_bridge.py` (battery state reporting)
- **Acceptance criteria:**
  - ✅ Li-Po discharge curve model (voltage vs. state-of-charge)
  - ✅ Power draw from motor RPM (P = τ·ω from MotorModel)
  - ✅ Battery state integrated into `DroneState` and reported via MAVLink `SYS_STATUS`
  - ✅ Flight autonomy estimation matches paper Table 2 value (~85 min for fixed-wing)
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestBatteryModel`
  - `pytest -q simulation/test_drone_physics.py::TestMAVLink::test_sys_status_encode_decode_battery_fields`

### Satellite Texture Terrain Overlay ✅ Completed (2026-03-27)

**Paper reference:** Section 2.1 — the paper describes using Google Earth satellite imagery imported through BlenderGIS for photorealistic terrain visualization in Gazebo.

- **Target files:** `gazebo/media/materials/`, `simulation/terrain.py`, `gazebo/worlds/antisana.world`
- **Acceptance criteria:**
  - ✅ Download georeferenced satellite tile for the SRTM region
  - ✅ Generate UV-mapped texture coordinates in mesh export (`export_obj_with_uv`)
  - ✅ Gazebo material references satellite image as diffuse texture (`AntisanaTerrain/SatelliteTextured`)
  - ✅ Fallback to existing elevation coloring if satellite tile unavailable
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestTerrainSatelliteTexture`

### Wind Disturbance Auto-Tuning Loop ✅ Completed (2026-03-27)

**Paper reference:** Section 3.1 — *"The validation process involves heuristic adjustment of constants to produce an estimation of disturbance that generates altitude values as close as possible to those from real flight tests. Fine-tuning the disturbance force allowed precise altitude estimations."*

Implemented via `auto_tune_wind_force_scale()` in `simulation/validation.py` with deterministic coordinate-search optimization over wind force scale, and integrated mission-level wind-force scaling via `WindField.force_scale` in `simulation/wind_model.py`.

- **Target files:** `simulation/validation.py`, `simulation/wind_model.py`
- **Acceptance criteria:**
  - ✅ Auto-tuning function: given a real flight log, iteratively adjust wind force scaling constant to minimize Z-axis RMSE
  - ✅ Converges within a tolerance (RMSE_z delta < 0.01 m between iterations)
  - ✅ Produces per-mission wind calibration constants reproducibly
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestWindAutoTuning`
  - `pytest -q simulation/test_drone_physics.py::TestWind simulation/test_drone_physics.py::TestPositionAwareWind`

---

## 3) Verification matrix

| Item category | Verification method | Status |
|:---|:---|:---|
| GPS noise | Unit test: quantization + CEP-class statistics (`TestSensorNoise`) | **Done** |
| IMU noise | Unit test: noise density order-of-magnitude check (`TestSensorNoise`) | **Done** |
| Barometer noise | Unit test: quantization + lag + altitude-equivalent noise (`TestSensorNoise`) | **Done** |
| Motor model | Step response + steady-state thrust-map tests (`TestMotorDynamics`) | **Done** |
| Control surfaces | Pitch response + rate-limit tests (`TestFixedWingControlSurfaces`) | **Done** |
| Real log validation | Trajectory-tracking replay + RMSE comparison against paper Table 5 (`<=6x` PID gate, Z-axis `<=2x`) | **Done** |
| Quadrotor aero area | Attitude-dependent area + prop-wash drag tests (`TestQuadrotorAeroArea`) | **Done** |
| Battery model | Discharge curve + power draw + autonomy estimation | **Done** |
| Satellite terrain texture | UV-mapped mesh + Gazebo material binding | **Done** |
| Wind auto-tuning | Iterative RMSE minimization convergence test | **Done** |

---

## 4) Execution order

1. No remaining paper-aligned phases in this delta backlog.

---

## 5) Notes

- This is a **delta backlog** only — completed work is not listed here.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
- 241 tests currently passing across 40+ test classes (as of trajectory-tracking fix audit).
- Paper Section 3.4 identifies 3D wind force as future work; our codebase already has `from_log_3d` mode which exceeds the paper's current Z-axis-only model.
- Paper Table 1 defines state variables for the fixed-wing model (not quadrotor inertia); IRS-4 parameters use ArduPilot defaults per Section 3.2.
