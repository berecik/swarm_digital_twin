# TODO.md — Kubernetes Gazebo Playground Execution Backlog

This task list translates `ROADMAP.md` into implementation-ready work items.

## 0) Project Controls

- [ ] Define owners for `simulation`, `gazebo`, `helm`, and `docs` deliverables.
- [ ] Establish weekly milestone review with measurable KPI updates.
- [ ] Add a single source-of-truth scenario manifest for Kubernetes simulation runs.

## 1) Gazebo as Kubernetes Playground (`/todo` scenario integration)

- [ ] Create and maintain `/todo/gazebo_k8s_playground.md` as the canonical operational scenario.
- [ ] Add scripted workflow for: cluster prep → deploy → run mission → collect artifacts → teardown.
- [ ] Ensure scenario supports local (`minikube`/`kind`) and cloud cluster profiles.
- [ ] Document required ports/services for telemetry, visualization, and operator control.

## 2) Real Physics in Kubernetes

- [ ] Mirror critical physics parameters between standalone and Gazebo pipelines:
  - [ ] mass/inertia tensor
  - [ ] motor response dynamics
  - [ ] aerodynamic drag/lift settings
  - [ ] atmosphere settings (density, gravity assumptions)
- [ ] Define parity test suite comparing Gazebo traces against `simulation/drone_physics.py` references.
- [ ] Add pass/fail thresholds for trajectory error, attitude stability, and energy usage.

## 3) Terrain Model

- [ ] Add terrain asset workflow (SRTM/grid/STL) with reproducible dataset versioning.
- [ ] Validate terrain import into Gazebo worlds and publish terrain metadata for analysis.
- [ ] Enforce AGL and clearance checks in mission controllers.
- [ ] Add terrain regression tests for flat + moderate + steep terrains.

## 4) Collision Detection

- [ ] Implement inter-drone separation monitor with near-miss and collision events.
- [ ] Implement terrain/obstacle collision detector based on pose and geometry queries.
- [ ] Define standardized safety response playbook (warn → mitigate → safe mode).
- [ ] Add test scenarios for crossing trajectories, formation compression, and terrain clipping.

## 5) Wind in Kubernetes Simulation

- [ ] Expose wind profiles (constant, Dryden, replay) via scenario configuration.
- [ ] Add deterministic seeded gust generation for reproducible CI runs.
- [ ] Add spatial wind gradients for multi-drone distributed missions.
- [ ] Verify mission robustness across baseline/crosswind/gust/storm classes.

## 6) Detailed Kubernetes Testing Scenario (Full Realistic Simulation)

- [ ] Build scenario matrix by dimensions:
  - [ ] drone count: `1`, `3`, `6`, `12`
  - [ ] terrain: flat, rolling, steep
  - [ ] wind: calm, crosswind, gusty, storm-like
  - [ ] mission: patrol, lawnmower search, payload escort, heavy-lift mock
  - [ ] faults: pod restart, packet loss, delayed telemetry, sensor dropout
- [ ] Define acceptance KPIs:
  - [ ] mission completion rate
  - [ ] mean/min separation distance
  - [ ] collision and near-miss count
  - [ ] trajectory RMSE and AGL violation count
  - [ ] failover recovery time and success rate
- [ ] Automate result collection:
  - [ ] logs + metrics + traces
  - [ ] replay artifacts (`.npz`, `.BIN` where available)
  - [ ] machine-readable summary (`json`) + human report (`md`)
- [ ] Integrate scenario execution into CI/nightly runs with trend tracking.

## 7) Documentation Synchronization

- [ ] Keep `README.md` Kubernetes simulation section aligned with scenario workflow.
- [ ] Update `TESTING.md` with detailed purpose/input/expected-outcome for new K8s simulation tests.
- [ ] Keep `AGENTS.md` maintenance/testing protocol in sync with Kubernetes-first workflow.
- [ ] Update `MAINTENANCE.log` after each major validation cycle.

## 8) Migrated Actions from former `docs/REFACTOR_PLAN.md`

- [ ] Live viewer terrain parity:
  - [ ] Load SRTM/STL terrain mesh into `simulation/runtime_view/web/live.js` scene.
  - [ ] Reuse `simulation/terrain.py` elevation source to keep live/post-flight consistency.
- [ ] Replace placeholder mission thumbnails:
  - [ ] Capture screenshots from real SITL missions.
  - [ ] Add deterministic thumbnail generation/update script for docs/runtime view assets.
- [ ] Runtime view auth hardening (if non-localhost exposure is required):
  - [ ] Add authentication and authorization model.
  - [ ] Add CSRF protection for launch/replay endpoints.
  - [ ] Add per-user session isolation and audit fields.
- [ ] Swarm SITL live telemetry forwarding:
  - [ ] Wire `simulation/sitl_orchestrator.py` for N-drone forwarding via `--telemetry-forward`.
  - [ ] Verify per-drone mapping by `system_id` in browser live view.
- [ ] DataFlash replay in web loader:
  - [ ] Extend `POST /api/load` to parse `.BIN` logs.
  - [ ] Convert parsed data to replay stream compatible with current live telemetry pipeline.
