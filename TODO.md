# TODO — Kubernetes Gazebo Playground Execution Backlog

This task list translates [`ROADMAP.md`](ROADMAP.md) into implementation-ready
work items. Each section links to detailed instructions in [`todo/`](todo/).

---

## 0) Project Controls

- [ ] Define owners for `simulation`, `gazebo`, `helm`, and `docs` deliverables.
- [ ] Establish weekly milestone review with measurable KPI updates.
- [ ] Add a single source-of-truth scenario manifest for K8s simulation runs.

## 1) Gazebo as Kubernetes Playground

**Instructions:** [`todo/gazebo_k8s_playground.md`](todo/gazebo_k8s_playground.md)
and [`todo/k8s_namespace_lifecycle.md`](todo/k8s_namespace_lifecycle.md)

- [ ] Maintain `todo/gazebo_k8s_playground.md` as the canonical operational scenario.
- [ ] Add scripted workflow: cluster prep → deploy → run mission → collect artifacts → teardown.
- [ ] Support local (`minikube`/`kind`) and cloud cluster profiles.
- [ ] Document required ports/services for telemetry, visualization, and operator control.
- [ ] Create `values-playground.yaml` Helm profile.
- [ ] Create namespace quota manifest (`k8s/sim-quota.yaml`).

## 2) Real Physics in Kubernetes

**Instructions:** [`todo/physics_parity.md`](todo/physics_parity.md)

- [ ] Mirror critical physics parameters between standalone and Gazebo:
  - [ ] mass / inertia tensor
  - [ ] motor response dynamics (`MotorModel.tau_spinup`)
  - [ ] aerodynamic drag/lift settings (`AeroCoefficients`)
  - [ ] atmosphere settings (density, gravity)
  - [ ] battery model (`BatteryModel`)
- [ ] Create `simulation/k8s_parity_test.py` — automated comparison script.
- [ ] Create `helm/swarm-digital-twin/values-parity.yaml` matching standalone params.
- [ ] Verify `gazebo/models/x500/model.sdf` inertia matches `DroneParams`.
- [ ] Add pass/fail thresholds: RMSE < 2.0 m (XY), < 1.0 m (Z), attitude < 5 deg.

## 3) Terrain Model

**Instructions:** [`todo/terrain_integration.md`](todo/terrain_integration.md)

- [ ] Add terrain asset workflow (SRTM/grid/STL) with versioned manifest.
- [ ] Create `gazebo/worlds/terrain/manifest.yaml` with checksums.
- [ ] Validate terrain import into Gazebo worlds.
- [ ] Enforce AGL and clearance checks in mission controllers.
- [ ] Add regression tests for flat + rolling + steep terrains.
- [ ] Align live viewer terrain mesh with simulation elevation data.

## 4) Collision Detection

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [ ] Create `simulation/safety.py` — `SeparationMonitor` with near-miss and collision events.
- [ ] Implement terrain/obstacle collision detector (AGL-based).
- [ ] Define safety response playbook (warn → mitigate → safe mode).
- [ ] Add test scenarios: crossing trajectories, formation compression, terrain clipping.
- [ ] Track KPIs: collision count, near-miss count, min separation, recovery latency.

## 5) Wind in Kubernetes Simulation

**Instructions:** [`todo/wind_simulation.md`](todo/wind_simulation.md)

- [ ] Expose wind profiles (constant, Dryden, replay) via scenario configuration.
- [ ] Add deterministic seeded gust generation for reproducible CI runs.
- [ ] Add spatial wind gradients for multi-drone distributed missions.
- [ ] Verify mission robustness across baseline/crosswind/gust/storm classes.
- [ ] Define stress envelope KPIs per wind profile class.

## 6) Full Kubernetes Testing Scenario

**Instructions:** [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md)

- [ ] Build scenario matrix: 4 drone counts x 3 terrains x 4 winds x 4 missions x 5 faults.
- [ ] Define CI subset (20 scenarios) for nightly runs.
- [ ] Define acceptance KPIs with pass/fail thresholds.
- [ ] Build `kpis.json` + `summary.md` report generator.
- [ ] Automate result collection: logs + metrics + replay artifacts.
- [ ] Integrate into CI/nightly pipeline with trend tracking.

## 7) Documentation Synchronization

- [ ] Keep `README.md` K8s simulation section aligned with scenario workflow.
- [ ] Update `TESTING.md` with K8s simulation test purposes and expected outcomes.
- [ ] Update `MAINTENANCE.log` after each major validation cycle.
- [ ] Keep `ROADMAP.md` phase status in sync with implementation progress.

## 8) Live View & Replay Backlog

**Instructions:** [`todo/live_view_backlog.md`](todo/live_view_backlog.md)

- [ ] Terrain rendering in live viewer:
  - [ ] Add `GET /api/terrain` endpoint serving elevation mesh.
  - [ ] Load mesh into Three.js scene, hide flat grid when present.
- [ ] Real mission thumbnails:
  - [ ] Capture screenshots from SITL runs.
  - [ ] Replace Pillow placeholders in `web/img/`.
- [ ] Authentication (if non-localhost exposure needed):
  - [ ] Add `--auth` flag with API key or JWT.
  - [ ] Add CSRF protection for mutating endpoints.
- [ ] Swarm SITL live telemetry:
  - [ ] Wire `run_swarm_mission_live()` in `run_scenario.sh`.
  - [ ] Per-drone `system_id` forwarding from `sitl_orchestrator.py`.
- [ ] `.BIN` replay in web loader:
  - [ ] Add `parse_bin_to_records()` for ArduPilot DataFlash format.
  - [ ] Extend `POST /api/load` to accept `.BIN` files.
