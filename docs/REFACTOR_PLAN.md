# Refactoring Plan (v2): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** this is no longer a "build-from-zero" plan. It is a **delta plan** that removes already implemented capabilities and focuses only on gaps still open against the paper workflow.

---

## 1) What is already implemented (removed from active refactor backlog)

The following paper-aligned foundations are already in the codebase and covered by tests in `simulation/test_drone_physics.py`:

- Quadratic drag model and ISA atmosphere in `simulation/drone_physics.py`
- Wind perturbation module in `simulation/wind_model.py` (constant + Dryden + log replay)
- Full inertia tensor handling and body-frame dynamics path
- Terrain model and terrain-aware collision checks in `simulation/terrain.py`
- Flight log parsing + validation utilities in `simulation/flight_log.py` and `simulation/validation.py`
- Fixed-wing aerodynamic model (`FixedWingAero`) with stall behaviour
- MAVLink v2 bridge for simulator ↔ GCS integration in `simulation/mavlink_bridge.py`

Baseline verification command for the above:

```bash
cd simulation && pytest -q test_drone_physics.py
```

---

## 2) Remaining gaps vs the paper framework

| Area | Current status | Gap vs paper workflow | Priority |
|:---|:---|:---|:---|
| End-to-end SITL orchestration | Components exist (`gazebo/`, ArduPilot-oriented assets, MAVLink bridge), but no single reproducible pipeline proving full chain | Need one-command reproducible run that demonstrates mission lifecycle (spawn → arm → mission → log capture) | **P0** |
| Real-flight scenario replication | Generic scenario exists (`drone_scenario.py`), no locked paper-style benchmark scenario package | Need canonical benchmark scenarios with fixed seeds, wind envelopes, and acceptance thresholds | **P0** |
| Quantitative acceptance gates | RMSE utilities exist, but no strict pass/fail quality gate tied to known scenarios | Need automated metric gate (RMSE axis + percentile checks) to prevent regression | **P0** |
| Mission-planning loop with GCS | MAVLink primitives implemented, but no documented QGroundControl mission replay procedure | Need validated mission upload/replay workflow and expected telemetry contract | **P1** |
| Aerodynamic parameter provenance | Parameters are present, but traceability to source (CFD/manual/estimate) is weak | Need parameter registry with provenance + validity ranges per airframe | **P1** |
| Multi-UAV digital twin (paper-inspired extension for this project) | Current standalone physics is mostly single-aircraft | Need multi-agent standalone simulation with inter-drone avoidance and synchronized wind/terrain | **P1** |
| Continuous validation in CI | Tests can be run manually | Need repeatable CI jobs for simulation validation tiers and artifacts | **P2** |

---

## 3) Active refactor roadmap (new)

### Phase A — Reproducible paper-style validation baseline (**P0**)

#### A1. Create canonical benchmark scenarios
- **Target files:**
  - `simulation/drone_scenario.py`
  - `simulation/validation.py`
  - `docs/testing.md`
- **Deliverables:**
  - At least two fixed benchmark profiles (moderate and strong-wind conditions)
  - Deterministic seeds and fixed simulation parameters
  - Stored expected metric envelopes (RMSE per axis + dispersion)
- **Acceptance criteria:**
  - Running the benchmark twice on same commit yields equivalent metrics within tolerance

#### A2. Add validation quality gates
- **Target files:**
  - `simulation/validation.py`
  - `simulation/test_drone_physics.py`
- **Deliverables:**
  - Programmatic `assert_validation_pass(...)` gate function
  - Tests for failing and passing thresholds
- **Acceptance criteria:**
  - CI/local run fails if benchmark metrics exceed configured envelope

#### A3. Document and script full verification entrypoint
- **Target files:**
  - `run_scenario.sh`
  - `README.md`
  - `TESTING.md`
- **Deliverables:**
  - Single documented command to run tests + scenario + metrics summary
- **Acceptance criteria:**
  - Fresh contributor can reproduce validation results without ad-hoc steps

### Phase B — Full open-source DT operation loop (**P1**)

#### B1. SITL + Gazebo + bridge workflow hardening
- **Target files:**
  - `gazebo/launch/`
  - `gazebo/worlds/`
  - `docker-compose.yml`
  - `docs/architecture.md`
- **Deliverables:**
  - Stable startup profile for simulator stack and bridge
  - Health checks for core services/topics/UDP links
- **Acceptance criteria:**
  - Mission stack starts reproducibly and passes health checks

#### B2. QGroundControl mission replay protocol
- **Target files:**
  - `docs/development.md`
  - `docs/testing.md`
  - `simulation/mavlink_bridge.py` (only if protocol fixes are needed)
- **Deliverables:**
  - Step-by-step mission upload/replay doc
  - Expected telemetry checklist (heartbeat, attitude, GPS, HUD, status)
- **Acceptance criteria:**
  - Mission replay is repeatable and log capture works end-to-end

#### B3. Aerodynamic parameter registry and provenance
- **Target files:**
  - `docs/physics_details.md`
  - `simulation/drone_physics.py`
- **Deliverables:**
  - Per-airframe parameter table with source/provenance tags and valid ranges
  - Runtime warnings when parameters are outside valid range
- **Acceptance criteria:**
  - Every preset has documented origin and uncertainty notes

### Phase C — Project-priority extension: swarm-ready standalone twin (**P1/P2**)

#### C1. Multi-drone standalone simulation core
- **Target files:**
  - `simulation/drone_physics.py`
  - `simulation/drone_scenario.py`
  - new `simulation/swarm_scenario.py`
- **Deliverables:**
  - N-drone simulation loop with shared terrain/wind field
  - Collision-avoidance constraints in standalone mode
- **Acceptance criteria:**
  - Demonstration run with at least 6 agents without inter-vehicle collisions

#### C2. Boids behaviour parity checks (Rust vs Python standalone)
- **Target files:**
  - `swarm_control/src/boids.rs`
  - `simulation/` parity test utilities
  - `swarm_control/TESTING.md`
- **Deliverables:**
  - Reference trajectories/metrics for behavioural parity
- **Acceptance criteria:**
  - Defined error bounds between `boids.rs` and standalone implementation

---

## 4) Verification matrix for this plan

| Item category | Verification method | Current status |
|:---|:---|:---|
| Implemented physics + wind + terrain + flight-log + fixed-wing + MAVLink | `pytest simulation/test_drone_physics.py` | **Available now** |
| End-to-end SITL reproducibility | scripted startup + health checks + mission replay log | **To implement** |
| Quantitative acceptance gates | automated threshold checks in tests/CI | **To implement** |
| Multi-drone standalone behaviour | dedicated scenario + collision/formation metrics | **To implement** |

---

## 5) Execution order

1. **Phase A (P0)** — make validation deterministic and enforceable first.
2. **Phase B (P1)** — harden full DT operation loop with SITL/Gazebo/GCS workflows.
3. **Phase C (P1/P2)** — extend to swarm-ready standalone twin aligned with project roadmap.

---

## 6) Notes for future maintenance updates

- Keep this document as a **delta backlog** only; move completed items into "already implemented" section instead of keeping them in active phases.
- Any new phase must include:
  - exact target files,
  - measurable acceptance criteria,
  - explicit verification command(s).
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log` in the same change.
