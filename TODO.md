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

- [ ] Add terrain asset workflow (SRTM/grid/STL) with versioned manifest.
- [ ] Create `gazebo/worlds/terrain/manifest.yaml` with checksums.
- [ ] Validate terrain import into Gazebo worlds.
- [ ] Enforce AGL and clearance checks in mission controllers.
- [ ] Add regression tests for flat + rolling + steep terrains.
- [ ] Align live viewer terrain mesh with simulation elevation data.

## 4) Collision Detection

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [x] Create `simulation/safety.py` — `SeparationMonitor` + `TerrainMonitor` + `SafetyReport`. 10 tests.
- [x] Implement terrain/obstacle collision detector — AGL-based with `TerrainCollisionEvent` + `ClearanceViolationEvent`.
- [ ] Define safety response playbook (warn → mitigate → safe mode) — requires PX4 integration.
- [x] Add test scenarios — well-separated, near-miss, collision, swarm record, terrain collision, clearance violation, full swarm benchmark.
- [x] Track KPIs — `SafetyReport.to_dict()` with collision count, near-miss count, min separation, terrain collisions, `is_safe()` verdict.

### 4.1) Phase 4 audit fixes (2026-04-19) — resolved

- [x] Status reconciled across ROADMAP, TODO, `todo/collision_detection.md`.
- [x] Verification block added to `TESTING.md` (10 tests, commands, status).
- [x] Remaining acceptance gaps documented (response playbook, latency, KPI export).
- [x] Maintenance gate added.

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
