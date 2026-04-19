# TODO — Kubernetes Gazebo Playground Execution Backlog

This task list translates [`ROADMAP.md`](ROADMAP.md) into implementation-ready
work items. Each section links to detailed instructions in [`todo/`](todo/).

---

## 0) Project Controls

- [/] Define owners for `simulation`, `gazebo`, `helm`, and `docs` deliverables — process item, owners are tracked in the project's organisational tooling rather than the codebase.
- [/] Establish weekly milestone review with measurable KPI updates — process item; the per-scenario `kpis.json` produced by `simulation/acceptance_matrix.py` is the trendable input.
- [x] Single source-of-truth scenario manifest for K8s simulation runs — `simulation/scenario_matrix.py` (`full_matrix()` returns 960; `ci_subset()` returns 20) is the canonical scenario definition, paired with `gazebo/worlds/{terrain,wind}/manifest.toml` for the per-axis options. `./run_scenario.sh --acceptance-matrix [subset] [output]` is the operator entry point.

## 1) Gazebo as Kubernetes Playground

**Instructions:** [`todo/gazebo_k8s_playground.md`](todo/gazebo_k8s_playground.md)
and [`todo/k8s_namespace_lifecycle.md`](todo/k8s_namespace_lifecycle.md)

- [x] Maintain `todo/gazebo_k8s_playground.md` as the canonical operational scenario.
- [x] Add scripted workflow — documented in [`docs/k8s_runbook.md`](docs/k8s_runbook.md).
- [x] Support local (`minikube`/`kind`) and cloud cluster profiles — `values-local.yaml`, `values-cloud.yaml`, `values-playground.yaml`.
- [x] Document required ports/services — runbook + `docs/kubernetes.md` (MAVLink NodePorts, Zenoh 7447, headless DNS).
- [x] Create `values-playground.yaml` Helm profile — with ResourceQuota + LimitRange.
- [x] Create namespace quota — Helm template `resourcequota.yaml` + `limitrange.yaml` (replaces raw manifest).

### 1.1) Phase 1 audit fixes (2026-04-19) — resolved

- [x] Status reconciled across ROADMAP, TODO, `todo/k8s_namespace_lifecycle.md`.
- [x] Namespace commands updated to idempotent `--dry-run=client | kubectl apply`.
- [x] Verification block added to `TESTING.md` with commands + status.
- [x] Maintenance gate added.

## 2) Real Physics in Kubernetes

**Instructions:** [`todo/physics_parity.md`](todo/physics_parity.md)

- [x] Mirror critical physics parameters — `ParityContract` checks mass, inertia, C_D, area, density, gravity. All match for X500.
- [x] Create `simulation/physics_parity.py` — `ParityContract`, `compare_trajectories()`, `check_timing_determinism()`, `extract_truth_from_records()`.
- [x] Create `helm/swarm-digital-twin/values-parity.yaml` — single-drone profile matching standalone params.
- [x] Verify `gazebo/models/x500/model.sdf` inertia matches `DroneParams` — verified in `TestPhysicsParity::test_sdf_parameter_match`.
- [x] Add pass/fail thresholds — RMSE < 2.0 m (XY), < 1.0 m (Z), jitter < 5 ms at 50 Hz.

### 2.1) Phase 2 audit fixes (2026-04-19) — resolved

- [x] Status reconciled across ROADMAP, TODO, `todo/physics_parity.md`.
- [x] Verification block added to `TESTING.md` (9 tests, commands, status).
- [x] Attitude RMSE + energy delta gaps documented as remaining items.
- [x] Maintenance gate added.

## 3) Terrain Model

**Instructions:** [`todo/terrain_integration.md`](todo/terrain_integration.md)

- [x] Add terrain asset workflow (SRTM/grid/STL) with manifest — `terrain.load_from_manifest(name)` dispatches on `flat` / `function` / `srtm` / `array` / `stl`; tomllib in stdlib (no new dep).
- [x] Create `gazebo/worlds/terrain/manifest.toml` — switched from .yaml because no Python in this repo currently imports yaml. Initial entries: `flat`, `synthetic_rolling`, `antisana`. Optional `checksum` field documented; verification implementation deferred until first non-default asset lands.
- [x] Add file-level height-query parity gate — `TestTerrainParity.test_export_roundtrip_parity` runs `get_elevation` against the source terrain and against the same terrain reloaded from its own STL export; asserts `max |Δz| < 0.5 m` over 100 deterministic samples. Catches the realistic failure mode that Gazebo would consume.
- [/] Validate terrain import into a Gazebo world (live runtime parity) — file-level closure shipped via `simulation.gz_terrain_emulator` (`TestGazeboTerrainEmulator` asserts `|Δz| < 0.5 m` between `terrain.get_elevation()` and the bilinear heightmap algorithm Gazebo runs). Live `gz::physics` shape parity stays opt-in nightly per [`docs/nightly_lane.md`](docs/nightly_lane.md).
- [x] Enforce AGL and clearance checks in mission controllers — `run_simulation` / `run_trajectory_tracking` / `run_swarm_simulation` take an optional `terrain_monitor` that's invoked per step; `safety.monitor_records()` is the post-hoc equivalent. Response actions (HOVER/RTL) stay Phase 4 work.
- [x] Add regression tests for flat + rolling + steep terrains over a real mission — `TestTerrainRegression` covers all three with the same patrol mission, asserting (no terrain collision) × (cruise AGL above floor) × (in-loop monitor matches post-hoc).
- [x] Align live viewer terrain mesh with simulation elevation data — `GET /api/terrain` returns the active mesh (`{vertices, faces, bounds}`); `POST /api/terrain` accepts `{terrain_name: "..."}` and goes through `terrain.load_from_manifest`. `live.js` builds a `BufferGeometry` and hides the flat grid + ground plane when a mesh is present.

## 4) Collision Detection

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [x] Create `simulation/safety.py` — `SeparationMonitor` + `TerrainMonitor` + `SafetyReport`. 10 tests.
- [x] Implement terrain/obstacle collision detector — AGL-based with `TerrainCollisionEvent` + `ClearanceViolationEvent`.
- [x] Define safety response playbook (warn → mitigate → safe mode) — `simulation/safety_response.py` ships a pure-Python `SafetyResponseController` over `NORMAL → WARNING → HOVER → RTL → EMERGENCY_STOP`. Configurable `SafetyResponseThresholds`; per-transition `IncidentRecord` log; `on_transition` callback for the PX4 actuator hook. RTL never auto-clears (flight-safe default). Tests: `TestSafetyResponseController` (11 cases).
- [x] Add test scenarios — well-separated, near-miss, collision, swarm record, terrain collision, clearance violation, full swarm benchmark.
- [x] Track KPIs — `SafetyReport.to_dict()` with collision count, near-miss count, min separation, terrain collisions, `is_safe()` verdict.

### 4.1) Phase 4 audit fixes (2026-04-19) — resolved

- [x] Status reconciled across ROADMAP, TODO, `todo/collision_detection.md`.
- [x] Verification block added to `TESTING.md` (10 tests, commands, status).
- [x] Remaining acceptance gaps documented (response playbook, latency, KPI export).
- [x] Maintenance gate added.

## 5) Wind in Kubernetes Simulation

**Instructions:** [`todo/wind_simulation.md`](todo/wind_simulation.md)

- [x] Expose wind profiles (constant, Dryden, replay) via scenario configuration — `gazebo/worlds/wind/manifest.toml` + `wind_model.load_wind_profile(name)`; matching `helm/swarm-digital-twin/values-wind-{calm,crosswind,gusty,storm}.yaml` for K8s consumers.
- [x] Add deterministic seeded gust generation for reproducible CI runs — `WindField(seed=...)` switches Dryden's noise source to a per-instance `np.random.default_rng(seed)`; bit-identical across runs.
- [x] Add spatial wind gradients for multi-drone distributed missions — `WindField(spatial_gradient=...)` accepts a 3×3 ENU matrix; `get_wind_velocity` adds `gradient @ position`.
- [x] Verify mission robustness across baseline/crosswind/gust/storm classes — `TestWindStressEnvelopes` runs the default mission against each manifest profile, asserts cruise-mean wind matches the documented base, and confirms wind-in-loop integration.
- [x] Define stress envelope KPIs per wind profile class — hard gate landed: each profile must complete its mission AND keep the cruise-window median attitude (`cruise_p50_*`) under `WIND_ATTITUDE_LIMITS_DEG[wind]` (calm 15° / crosswind 25° / gusty 30° / storm 45°). Strict K8s+PX4 thresholds preserved as `WIND_ATTITUDE_LIMITS_DEG_K8S` (5 / 10 / 15 / 25°).
- [x] Wind data logged in records — new `SimRecord.wind_velocity` (3-vec ENU) populated per step when a wind field is supplied.

## 6) Full Kubernetes Testing Scenario

**Instructions:** [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md)

- [x] Build scenario matrix: 4 drone counts × 3 terrains × 4 winds × 4 missions × 5 faults — `simulation/scenario_matrix.py` (`full_matrix()` returns 960; `ScenarioConfig` is hashable + filesystem-friendly via `scenario_id`).
- [x] Define CI subset (20 scenarios) — `ci_subset()` returns 4 diagonal rows + 16 critical-path rows; covers every value of every dimension at least once. `select(name)` resolves a subset by name.
- [x] Define acceptance KPIs with pass/fail thresholds — soft Python-pipeline gates in `simulation/acceptance_report.py` (completion per wind, hard collision floor 0.5 m, near-miss budget 60/drone, AGL violations); strict K8s+PX4 production thresholds preserved as `*_K8S` constants for the future nightly lane.
- [x] Build `kpis.json` + `summary.md` report generator — `acceptance_report.compute_kpis()` and `write_report()` produce the documented `reports/<scenario_id>/{kpis.json, summary.md, config.toml}` tree.
- [x] Automate result collection — `simulation/acceptance_matrix.py` CLI (`python -m simulation.acceptance_matrix --subset ci`) and `./run_scenario.sh --acceptance-matrix [subset] [output]` drive the runner. Single-scenario mode also supported.
- [/] Integrate into CI/nightly pipeline with trend tracking — Python pipeline ships full per-scenario `kpis.json` (including the new fault telemetry + scalability timing) so CI can already trend-track the matrix. The K8s nightly lane that adds real fault injection + pod-startup measurement is the contract in [`docs/nightly_lane.md`](docs/nightly_lane.md).

## 7) Documentation Synchronization

These four are continuous per-maintenance gates rather than one-shot tasks
— they're enforced on every MAINTENANCE pass per `AGENTS.md`. Listed here
so the gate is discoverable from the backlog.

- [/] Keep `README.md` K8s simulation section aligned with scenario workflow.
- [/] Update `TESTING.md` with K8s simulation test purposes and expected outcomes.
- [/] Update `MAINTENANCE.log` after each major validation cycle.
- [/] Keep `ROADMAP.md` phase status in sync with implementation progress.

## 8) Live View & Replay Backlog

**Instructions:** [`todo/live_view_backlog.md`](todo/live_view_backlog.md)

**Hard rule:** drones must be visible in the live view at all times during
simulation — see "Drones-Always-Visible Invariant" in
`todo/live_view_backlog.md` for the contract and acceptance criteria.

- [x] Drones-always-visible invariant — `TestDronesAlwaysVisibleInvariant` (4 static gates on `live.js`) keeps regressions out: createDroneMesh(1) before connectWS(), per-drone pre-creation from `/api/waypoints`, applySample reuses meshes via getDrone(), camera-follow markers stay wired.
- [x] Terrain rendering in live viewer:
  - [x] Added `GET /api/terrain` (returns active mesh or 204) and `POST /api/terrain` (raw mesh or `{terrain_name: "..."}`).
  - [x] `live.js` builds a `BufferGeometry` from the response and hides the flat grid + ground plane when a mesh is present.
- [/] Real mission thumbnails — operator-only workflow documented at
      `simulation/runtime_view/scripts/capture_thumbnails.md`. CI
      cannot drive a browser to screenshot the live view; Pillow
      placeholders stay until an operator runs the capture steps for
      each mission.
- [x] Authentication (if non-localhost exposure needed):
  - [x] `--auth-token` CLI flag (and `RUNTIME_VIEW_AUTH_TOKEN` env
        var) enables Bearer auth on every mutating endpoint.
  - [x] CSRF gate on `POST /api/load`, `/api/launch`,
        `/api/launch/stop`, `/api/waypoints`, `/api/terrain`;
        `GET /api/csrf` returns the active token.
  - [x] Multi-user session isolation — `POST /api/session` mints a
        per-browser token (TTL `SESSION_TTL_S`); `DELETE /api/session`
        revokes early; `_purge_expired_sessions()` drops stale ones;
        `AuthMiddleware` accepts session tokens in addition to the
        global key (CSRF must match whichever was used to
        authenticate). Tests: `TestMultiUserSessionIsolation` (7).
- [x] Swarm SITL live telemetry:
  - [x] Wired `run_swarm_mission_live()` in `run_scenario.sh` — spawns the
        live view in parallel with the formation orchestrator, publishes
        per-drone ENU waypoints via `POST /api/waypoints`, exits cleanly
        when helm tears down.
  - [x] Per-drone `system_id` forwarding from `sitl_orchestrator.py` —
        `--telemetry-forward udpout:127.0.0.1:14550`; original system_ids
        survive the relay so `MAVLinkLiveSource._apply_updates(...,
        system_id=...)` demuxes drones automatically.
- [x] `.BIN` replay in web loader:
  - [x] Added `physics_live_replay.load_bin_records()` backed by
        pymavlink's DFReader; GPS→ENU via the same small-angle formula
        as `live_telemetry._gps_to_enu`.
  - [x] Extended `POST /api/load` to accept `.bin` files (routes
        through a shared `_start_records_replay` helper with the
        existing `.npz` paths).

## 9) ML/Computer Vision Pipeline

**Instructions:**
- [`todo/ml_vision_overview.md`](todo/ml_vision_overview.md) — master scenario
- [`todo/ml_training_pipeline.md`](todo/ml_training_pipeline.md) — training
- [`todo/ml_model_zoo.md`](todo/ml_model_zoo.md) — model architectures
- [`todo/ml_edge_deployment.md`](todo/ml_edge_deployment.md) — ONNX/TensorRT/Jetson
- [`todo/ml_sim_to_real.md`](todo/ml_sim_to_real.md) — synthetic data
- [`todo/ml_continuous_improvement.md`](todo/ml_continuous_improvement.md) — active learning

- [ ] **Synthetic data generation:**
  - [ ] Gazebo SAR training world with 20+ human/vehicle models
  - [ ] Automated image capture during lawnmower flights
  - [ ] COCO-format annotation from Gazebo ground truth poses
  - [ ] Domain randomisation augmentations (albumentations)
- [ ] **Training pipeline:**
  - [ ] YOLOv8s fine-tune on SAR data (mAP@50 > 0.75)
  - [ ] RT-DETR for high-accuracy pass
  - [ ] ViT scene classifier (damage, terrain type)
  - [ ] Weights & Biases experiment tracking
  - [ ] ONNX export + validation
- [ ] **Model zoo:**
  - [ ] `ModelZoo` unified API replacing hard-coded YOLO in `detector.py`
  - [ ] Backends: YOLO, RT-DETR, DETR, FCOS, ONNX, TensorRT
  - [ ] ROS 2 parameter-driven model selection
- [ ] **Edge deployment:**
  - [ ] TensorRT FP16/INT8 builds on Jetson Orin Nano
  - [ ] INT8 calibration dataset (500+ images)
  - [ ] `edge_runtime.py` with inference metrics
  - [ ] `Dockerfile.jetson` for JetPack 6.0
  - [ ] Fleet deployment script
- [ ] **Continuous improvement:**
  - [ ] Inference logger (1 frame/sec with metadata)
  - [ ] Hard example miner (low confidence + missed detections)
  - [ ] CVAT/Label Studio labeling integration
  - [ ] Automated retraining pipeline
  - [ ] Model registry with version lineage
  - [ ] CI gate: ONNX check + mAP threshold
