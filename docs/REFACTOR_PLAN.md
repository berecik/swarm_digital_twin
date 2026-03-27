# Refactoring Plan (v8): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

**Current state:** ~85% paper coverage. Core physics (Eq. 1–7), terrain pipeline, aerodynamics, MAVLink, validation framework, swarm simulation, and all parameter tables are fully implemented. 207 tests passing.

---

## 1) Remaining gaps

| # | Area | Current status | Gap | Priority |
|:--|:---|:---|:---|:---|
| S1 | GPS sensor noise model | GPS messages defined, no noise injection | Need GPS quantization/bias/drift noise (paper Section 2.4) | **P1** |
| S2 | IMU sensor noise model | IMU messages defined, no noise injection | Need accelerometer/gyroscope bias + white noise (Section 2.4) | **P1** |
| S3 | Barometer noise model | Pressure computed via ISA, no noise | Need quantization, lag, and bias noise for baro readings | **P2** |
| T1 | Motor thrust dynamics | Direct thrust command (ideal) | Need `T_i = k_T·ω_i²` motor model with spin-up lag (Eq. 4) | **P2** |
| U1 | Fixed-wing control surfaces | Lift/drag/stall implemented | Need elevator/aileron/rudder deflection models + rate limits | **P2** |
| V1 | Real flight log validation | Comparison pipeline ready | Need actual .bin logs from OSSITLQUAD repo for Table 5 numbers | **P3** |

---

## 2) Active roadmap

### Phase S — Sensor Noise Models (**P1/P2**)

Paper Section 2.4 describes sensor models for GPS, IMU, and barometer. The message types exist (`SensorGps.msg`, `VehicleImu.msg`, `SensorBaro.msg`) but no noise injection is implemented.

#### S1. GPS noise model

- **Target files:** `simulation/sensor_models.py` (new file)
- **Deliverables:**
  - `GPSNoise` class with configurable horizontal/vertical accuracy (default: ±5m H, ±10m V)
  - Random walk bias drift (~0.5m/hour)
  - Quantization to 1e-7 degree resolution
  - `apply(true_lat, true_lon, true_alt) → noisy_lat, noisy_lon, noisy_alt`
- **Acceptance criteria:**
  - Noise statistics match typical u-blox M8N GPS (CEP 2.5m, altitude ±5m)
  - Injected noise does not break validation envelopes when moderate

#### S2. IMU noise model

- **Target files:** `simulation/sensor_models.py`
- **Deliverables:**
  - `IMUNoise` class with accelerometer and gyroscope noise parameters
  - Allan variance-based noise model: bias instability + angle random walk
  - Temperature-dependent bias drift (optional)
  - `apply_accel(true_accel) → noisy_accel`, `apply_gyro(true_gyro) → noisy_gyro`
- **Acceptance criteria:**
  - Accelerometer noise density ~400 µg/√Hz (ICM-20689 class)
  - Gyroscope noise density ~0.01 °/s/√Hz

#### S3. Barometer noise model

- **Target files:** `simulation/sensor_models.py`
- **Deliverables:**
  - `BaroNoise` class with quantization (±0.12 hPa) and low-pass lag
  - Slow drift bias (~1 hPa/hour)
  - `apply(true_pressure) → noisy_pressure`
- **Acceptance criteria:**
  - Altitude noise equivalent ≤ ±1m at sea level conditions

### Phase T — Motor Dynamics (**P2**)

#### T1. Explicit motor thrust model

- **Target files:** `simulation/drone_physics.py`
- **Deliverables:**
  - `MotorModel` class: `T = k_T·ω² + k_D·ω`, first-order spin-up: `dω/dt = (ω_cmd - ω)/τ_motor`
  - Integration into `physics_step()` as optional motor dynamics layer
  - Motor constants from paper Table 1 for IRS-4 quadrotor
- **Acceptance criteria:**
  - Motor spin-up visible in step response (τ ~ 50ms)
  - Thrust output matches `k_T·ω²` steady-state relationship

### Phase U — Fixed-Wing Control Surfaces (**P2**)

#### U1. Control surface deflection model

- **Target files:** `simulation/drone_physics.py` (extend `FixedWingAero`)
- **Deliverables:**
  - Elevator, aileron, rudder deflection angles with rate limits
  - Control effectiveness coefficients (Cl_δa, Cm_δe, Cn_δr)
  - Deflection-to-moment mapping for 3-axis control
- **Acceptance criteria:**
  - Pitch response to elevator matches thin airfoil theory within 10%
  - Control surface rate limit prevents instantaneous deflection

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
| GPS noise (S1) | Unit test: noise statistics match CEP spec | **Pending** |
| IMU noise (S2) | Allan variance test on synthetic noise output | **Pending** |
| Barometer noise (S3) | Altitude noise equivalent test | **Pending** |
| Motor model (T1) | Step response test: spin-up τ ≈ 50ms | **Pending** |
| Control surfaces (U1) | Pitch response to elevator step input | **Pending** |
| Real log validation (V1) | RMSE comparison against paper Table 5 | **Pending** |

---

## 4) Execution order

1. **Phase S (P1/P2)** — Sensor noise models (GPS, IMU, baro). Enables realistic closed-loop validation.
2. **Phase T (P2)** — Motor dynamics. Improves transient response fidelity.
3. **Phase U (P2)** — Fixed-wing control surfaces. Completes fixed-wing model.
4. **Phase V (P3)** — Real flight data validation. End-to-end paper reproduction.

---

## 5) Notes

- This is a **delta backlog** only — completed work is not listed here.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
- 207 tests currently passing across 37 test classes (as of Phase L/M completion).
