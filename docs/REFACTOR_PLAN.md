# Refactoring Plan (v13): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

**Current state:** Phases S/T/U/V/W/X/Y/Z executed and audited. Core physics (Eq. 1–7), terrain pipeline (SRTM + STL), aerodynamics (Table 2/3 exact coefficients), full inertia tensor (Eq. 4 Gamma terms), MAVLink bridge, validation framework (Table 5 RMSE gates), swarm simulation, sensor noise models (GPS/IMU/baro), motor thrust dynamics, fixed-wing control surfaces, Euler rate kinematics, real-flight-data validation, quadrotor effective aerodynamic area modeling, battery/energy modeling, satellite-texture terrain overlay, and wind disturbance auto-tuning are implemented, integrated, and tested. 234 tests passing.

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
| Table 5 — RMSE validation metrics | `validation.py` acceptance gate (≤2× paper values) | `TestPaperValidation` |
| Section 2.1 — SRTM terrain → STL export | `terrain.py` `from_srtm()` + `export_stl()` | `TestTerrainSTLExport` (6 tests) |
| Section 2.3 — wind from real flight log (Z-axis heuristic) | `flight_log.py` `get_wind_profile()`, `wind_model.py` `"from_log"` mode | `TestPositionAwareWind` |
| Section 2.3 — Gazebo LiftDrag plugin interface | `gazebo/worlds/antisana.world` SDF aerodynamics config | Manual (Gazebo launch) |
| Section 3.2 — IRS-4 quadrotor (ArduPilot defaults) | `drone_physics.py` `make_irs4_quadrotor()` | `test_irs4_hover_stable` |
| Sensor noise (GPS/IMU/baro) | `sensor_models.py`, integrated in `sim_bridge.py` + `mavlink_bridge.py` | `TestSensorNoise` (6 tests) |
| Motor dynamics (T = k_T·ω² + k_D·ω) | `drone_physics.py` `MotorModel`, enabled for IRS-4 | `TestMotorDynamics` (3 tests) |
| Control surfaces (elevator/aileron/rudder) | `drone_physics.py` `FixedWingAero`, rate-limited actuation | `TestFixedWingControlSurfaces` (2 tests) |

---

## 2) Remaining gaps

No remaining paper-aligned gaps in this delta backlog.

---

## 3) Verification matrix

All previously planned delta items have been verified and completed (see `TESTING.md` and `MAINTENANCE.log` for detailed historical evidence).

---

## 4) Execution order

1. No remaining paper-aligned phases in this delta backlog.

---

## 5) Notes

- This is a **delta backlog** only — completed work is tracked in `MAINTENANCE.log` and summarized in `TESTING.md`.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
- 234 tests currently passing across 40+ test classes (latest audited state).
- Paper Section 3.4 identifies 3D wind force as future work; our codebase already has `from_log_3d` mode which exceeds the paper's current Z-axis-only model.
- Paper Table 1 defines state variables for the fixed-wing model (not quadrotor inertia); IRS-4 parameters use ArduPilot defaults per Section 3.2.
