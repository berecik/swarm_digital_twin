# ROADMAP — Kubernetes + Gazebo Realistic Swarm Simulation

This roadmap defines delivery phases for using Gazebo as a Kubernetes-native
playground for realistic multi-drone simulation.

Execution backlog: [`TODO.md`](TODO.md)
Detailed instructions per phase: [`todo/`](todo/)

## Status Legend

- `[ ]` to do
- `[/]` in progress
- `[x]` done / covered by tests

---

## Phase 1 — Kubernetes + Gazebo Baseline

**Instructions:** [`todo/k8s_namespace_lifecycle.md`](todo/k8s_namespace_lifecycle.md)
and [`todo/gazebo_k8s_playground.md`](todo/gazebo_k8s_playground.md)

- [x] K8s simulation namespace and lifecycle — ResourceQuota + LimitRange
      templates in Helm chart, `swarm-sim` namespace with quotas.
- [x] Helm profile for Gazebo playground mode — `values-playground.yaml`
      with resource quotas, 6-drone defaults, Quito SITL location.
- [x] Service topology for swarm networking — Helm test
      `test-service-topology.yaml` validates headless DNS, MAVLink, Zenoh
      ports across all pods.
- [x] Operational runbook — [`docs/k8s_runbook.md`](docs/k8s_runbook.md)
      with one-command quick reference, detailed workflows, troubleshooting.

### Phase 1 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/k8s_namespace_lifecycle.md`.
- [x] Namespace commands made idempotent (`--dry-run=client | kubectl apply`).
- [x] Verification evidence added to `TESTING.md` with commands + status.
- [x] Maintenance gate: verify Phase 1 docs consistency during each maintenance run.

## Phase 2 — Real Physics in Kubernetes Loop

**Instructions:** [`todo/physics_parity.md`](todo/physics_parity.md)

- [x] Physics parity contract — `ParityContract` in `simulation/physics_parity.py`
      compares DroneParams against Gazebo SDF (mass, inertia, C_D, area,
      density, gravity). X500 model verified: all parameters match.
- [x] Parity test suite — `compare_trajectories()` with RMSE thresholds
      (< 2.0 m XY, < 1.0 m Z). 9 tests in `TestPhysicsParity`.
- [x] Timing determinism checks — `check_timing_determinism()` validates
      control loop jitter (< 5 ms at 50 Hz). `TimingResult` with P99 jitter.
- [x] Telemetry truth pipeline — `TelemetryTruthRecord`,
      `extract_truth_from_records()`, `save_truth_csv()` for post-run comparison.

### Phase 2 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/physics_parity.md`.
- [x] Verification evidence added to `TESTING.md` (9 tests, commands, status).
- [x] Attitude RMSE and energy delta thresholds documented as remaining gates.
- [x] Maintenance gate: verify Phase 2 docs consistency during each maintenance run.

## Phase 3 — Terrain Model Integration

**Instructions:** [`todo/terrain_integration.md`](todo/terrain_integration.md)

- [ ] Terrain source workflow — support flat, grid, STL, and SRTM terrain
      assets with reproducible versioned datasets and a manifest file.
- [ ] Height-query consistency — align terrain elevation lookups across
      Gazebo world, physics replay, validation tooling, and live viewer.
- [ ] Mission safety envelope over terrain — enforce AGL/clearance
      constraints against terrain mesh during autonomous missions.
- [ ] Regression tests — flat + rolling + steep terrain profiles all pass.

## Phase 4 — Collision Detection & Safety

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [x] Inter-drone collision detection — `SeparationMonitor` with near-miss
      (< 3.0 m) and collision (< 1.5 m) events. 10 tests.
- [x] Drone-terrain collision detection — `TerrainMonitor` with AGL-based
      terrain collision and clearance violation events.
- [x] Safety KPIs — `SafetyReport` with collision count, near-miss count,
      min separation, terrain collisions, clearance violations, `is_safe()`,
      `summary()`, `to_dict()`.
- [ ] Safety response playbook — Warning → HOVER → RTL / emergency stop
      with configurable thresholds and logged incidents (requires PX4 integration).

### Phase 4 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/collision_detection.md`.
- [x] Verification evidence added to `TESTING.md` (10 tests, commands, status).
- [x] Remaining acceptance gaps documented (response playbook, latency, KPI export).
- [x] Maintenance gate: verify Phase 4 docs consistency during each maintenance run.

## Phase 5 — Wind Simulation in Kubernetes

**Instructions:** [`todo/wind_simulation.md`](todo/wind_simulation.md)

- [ ] Wind model mapping — map constant, Dryden turbulence, and flight-log
      replay profiles into Gazebo/K8s scenario definitions.
- [ ] Distributed wind injection — spatially varying wind fields for
      multi-drone area operations.
- [ ] Wind reproducibility — fixed seeds and profile snapshots for
      deterministic reruns (seeded Dryden must be bit-identical).
- [ ] Wind stress envelopes — pass/fail gates for baseline (calm),
      crosswind (5 m/s), gusty (8+), and storm-like (12 m/s) profiles.

## Phase 6 — Full-System Kubernetes Validation

**Instructions:** [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md)

- [ ] Test scenario matrix — 4 drone counts x 3 terrains x 4 wind profiles
      x 4 mission types x 5 fault types (960 combinations, CI subset of 20).
- [ ] Scalability gates — validate 1/3/6/12 drone swarm sizes with pod
      startup time and scheduling delay thresholds.
- [ ] Failure-mode verification — pod restart, packet loss, network delay,
      sensor dropout. Each must detect, respond, and recover within limits.
- [ ] Acceptance report automation — machine-readable KPIs (`kpis.json`),
      human report (`summary.md`), plots, logs, replay artifacts.

## Phase 7 — Live View & Replay Backlog

**Instructions:** [`todo/live_view_backlog.md`](todo/live_view_backlog.md)

- [ ] Terrain rendering in live viewer — replace flat Three.js grid with
      SRTM/STL mesh rendering aligned with `simulation/terrain.py`.
- [ ] Real mission thumbnails — replace Pillow placeholders with captures
      from real SITL runs.
- [ ] Authentication and multi-user — auth/CSRF/session isolation if the
      runtime view is exposed beyond localhost.
- [ ] Swarm SITL live view — wire `--swarm-live` mode with per-drone
      telemetry forwarding from `sitl_orchestrator.py`.
- [ ] `.BIN` replay in web viewer — extend `POST /api/load` to parse and
      replay ArduPilot DataFlash `.BIN` logs.

## Phase 8 — ML/Computer Vision Pipeline for Perception

**Instructions:**
- [`todo/ml_vision_overview.md`](todo/ml_vision_overview.md) — master scenario
- [`todo/ml_training_pipeline.md`](todo/ml_training_pipeline.md) — PyTorch training
- [`todo/ml_model_zoo.md`](todo/ml_model_zoo.md) — YOLO, ViT, DETR, FCOS, RT-DETR
- [`todo/ml_edge_deployment.md`](todo/ml_edge_deployment.md) — ONNX, TensorRT, Jetson
- [`todo/ml_sim_to_real.md`](todo/ml_sim_to_real.md) — synthetic data, domain randomisation
- [`todo/ml_continuous_improvement.md`](todo/ml_continuous_improvement.md) — active learning

### Phase 8A — Simulation Data Generation
- [ ] Gazebo SAR training world with 20+ target models (person, vehicle, equipment)
- [ ] RGB-D camera sensor on drone model for automatic capture
- [ ] Automated annotation pipeline producing COCO-format JSON
- [ ] Domain randomisation augmentations (lighting, weather, blur, cutout)
- [ ] Mixed dataset strategy: 5k synthetic + 500 real + public aerial datasets

### Phase 8B — Training Pipeline
- [ ] YOLOv8/v11 fine-tuning with Weights & Biases tracking
- [ ] RT-DETR training for high-accuracy secondary detection
- [ ] ViT scene classifier (damage assessment, terrain type)
- [ ] ONNX export with validation (checker + inference test + mAP parity)
- [ ] Acceptance KPIs: mAP@50 > 0.75, Recall > 0.85, Inference < 50 ms

### Phase 8C — Model Zoo
- [ ] Unified `ModelZoo` API: `detect(image) → List[Detection]`
- [ ] Backends: YOLO, RT-DETR, DETR, FCOS, ONNX Runtime, TensorRT
- [ ] `detector.py` uses `ModelZoo` with ROS 2 parameter-driven model swap
- [ ] Unit tests for each backend load + detect path

### Phase 8D — Edge Deployment
- [ ] ONNX → TensorRT FP16/INT8 build on Jetson Orin Nano
- [ ] INT8 calibration dataset (500+ representative images)
- [ ] Edge runtime with inference timing metrics (FPS, latency)
- [ ] Jetson Dockerfile (`Dockerfile.jetson`) with JetPack 6.0 stack
- [ ] Fleet deployment script (`deploy_model.sh`)
- [ ] Target: > 20 FPS on YOLOv8s FP16 (Orin Nano)

### Phase 8E — Continuous Improvement Loop
- [ ] Inference logger: capture 1 frame/sec with detections + metadata
- [ ] Hard example miner: uncertain (0.3-0.6 conf) and missed detections
- [ ] Labeling workflow with CVAT/Label Studio integration
- [ ] Automated retraining pipeline (train → evaluate → export if improved)
- [ ] Model registry with version lineage (`model_registry.json`)
- [ ] CI validation: ONNX check + deployed model mAP threshold gate

---

## Delivered Baseline

- [x] Paper equations 1–7, tables 1–5 — fully implemented and verified.
      See [`CHANGELOG.md`](CHANGELOG.md).
- [x] Live Run-time View — FastAPI + Three.js, multi-drone demux,
      post-flight replay, browser launch, DataFlash recording.
      318 tests, 0 warnings.
- [x] Historical verification commands preserved in `TESTING.md` and
      `CHANGELOG.md`.

## Definition of Done (Program-Level)

- [ ] End-to-end K8s scenario can launch and execute realistic Gazebo
      missions reproducibly.
- [ ] Physics, terrain, collision, and wind behaviors are covered by
      automated tests and scenario validations.
- [ ] Safety and mission KPIs are defined, measured, and enforced by
      CI-compatible gates.
- [ ] Documentation (`README.md`, `TESTING.md`, `ROADMAP.md`, `TODO.md`)
      is synchronized with implementation status.
