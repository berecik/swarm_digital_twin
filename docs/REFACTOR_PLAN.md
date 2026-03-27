# Refactoring Plan (v11): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

**Current state:** ~95% paper coverage. Core physics (Eq. 1–7), terrain pipeline, aerodynamics, MAVLink, validation framework, swarm simulation, sensor noise models (GPS/IMU/baro), motor thrust dynamics, and fixed-wing control-surface dynamics are implemented and tested. 215 tests passing.

---

## 1) Remaining gaps

| # | Area | Current status | Gap | Priority |
|:--|:---|:---|:---|:---|
| V1 | Real flight log validation | Comparison pipeline ready | Need actual .bin logs from OSSITLQUAD repo for Table 5 numbers | **P3** |

---

## 2) Active roadmap

### Phase S — Sensor Noise Models (**Completed 2026-03-27**)

Paper Section 2.4 sensor-noise gap has been closed.

- **Implemented files:**
  - `simulation/sensor_models.py` (new): `GPSNoise`, `IMUNoise`, `BaroNoise`
  - `simulation/test_drone_physics.py`: `TestSensorNoise` coverage
- **Delivered capabilities:**
  - GPS quantization (`1e-7` deg), white noise, and slow drift bias
  - IMU accelerometer/gyro white-noise density + bias random walk model
  - Barometer quantization (`0.12` hPa), low-pass lag, and drift bias
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestSensorNoise`
  - `pytest -q simulation/test_drone_physics.py`

### Phase T — Motor Dynamics (**Completed 2026-03-27**)

#### T1. Explicit motor thrust model

- **Implemented files:**
  - `simulation/drone_physics.py`: `MotorModel`, optional motor dynamics in `physics_step()`, motor state in `DroneState`
  - `simulation/test_drone_physics.py`: `TestMotorDynamics` coverage
- **Delivered capabilities:**
  - Motor model: `T = k_T·ω² + k_D·ω`
  - First-order motor spin-up: `dω/dt = (ω_cmd - ω)/τ_motor`
  - Optional integration path (`motor_dynamics_enabled`) preserving legacy default dynamics
  - IRS-4 preset motor constants (`k_T=9.2e-6`, `τ_motor=0.05s`, `num_motors=4`)
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestMotorDynamics`
  - `pytest -q simulation/test_drone_physics.py`

### Phase U — Fixed-Wing Control Surfaces (**Completed 2026-03-27**)

#### U1. Control surface deflection model

- **Implemented files:**
  - `simulation/drone_physics.py`: control-surface commands/states, per-axis rate limits, and control-effectiveness moment mapping in `physics_step()`
  - `simulation/test_drone_physics.py`: `TestFixedWingControlSurfaces` coverage
- **Delivered capabilities:**
  - Elevator, aileron, rudder deflection states with command clipping and rate-limited actuation
  - Control effectiveness coefficients (`Cl_δa`, `Cm_δe`, `Cn_δr`) in `FixedWingAero`
  - Deflection-to-moment mapping for roll/pitch/yaw aerodynamic torque channels
- **Verification:**
  - `pytest -q simulation/test_drone_physics.py::TestFixedWingControlSurfaces::test_elevator_pitch_response_matches_control_effectiveness`
  - `pytest -q simulation/test_drone_physics.py::TestFixedWingControlSurfaces::test_control_surface_rate_limit_prevents_instant_step`
  - `pytest -q simulation/test_drone_physics.py`

### Phase V — Real Flight Data Validation (**P3**)

#### V1. Paper Table 5 validation with real logs

- **Target files:** `simulation/validation.py`, `data/flight_logs/` (new directory)
- **Deliverables:**
  - Download and parse .bin flight logs from `github.com/estebanvt/OSSITLQUAD`
  - Run sim-vs-real comparison for Carolina 40m, Carolina 20m, EPN 30m, EPN 20m
  - Compare RMSE position/velocity/attitude against paper Table 5 values
- **Acceptance criteria:**
  - Simulation RMSE within 2× of paper-reported values (accounting for model simplifications)

---

## 3) Verification matrix

| Item category | Verification method | Status |
|:---|:---|:---|
| GPS noise (S1) | Unit test: quantization + CEP-class statistics (`TestSensorNoise`) | **Done** |
| IMU noise (S2) | Unit test: noise density order-of-magnitude check (`TestSensorNoise`) | **Done** |
| Barometer noise (S3) | Unit test: quantization + lag + altitude-equivalent noise (`TestSensorNoise`) | **Done** |
| Motor model (T1) | Step response + steady-state thrust-map tests (`TestMotorDynamics`) | **Done** |
| Control surfaces (U1) | Pitch response + rate-limit tests (`TestFixedWingControlSurfaces`) | **Done** |
| Real log validation (V1) | RMSE comparison against paper Table 5 | **Pending** |

---

## 4) Execution order

1. **Phase V (P3)** — Real flight data validation. End-to-end paper reproduction.

---

## 5) Notes

- This is a **delta backlog** only — completed work is not listed here.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
- 215 tests currently passing across 40 test classes (as of Phase U completion).
