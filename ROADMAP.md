# ROADMAP — Kubernetes + Gazebo Realistic Swarm Simulation

This roadmap defines the delivery phases for using Gazebo as a
Kubernetes-native playground for realistic multi-drone simulation.

- Execution backlog: [`TODO.md`](TODO.md)
- Per-phase implementation history + verification commands: [`CHANGELOG.md`](CHANGELOG.md)
- Per-phase instructions: [`todo/`](todo/)
- Nightly K8s + Gazebo + Playwright lane (deferred runtime work): [`docs/nightly_lane.md`](docs/nightly_lane.md)

## Status Legend

- `[ ]` to do
- `[/]` in progress
- `[x]` done / covered by tests

---

## Phases 1–7 — Closed

All Phase 1–7 work has shipped on the Python/CI pipeline. The
per-phase implementation summaries (file lists, test names,
verification commands) live in `CHANGELOG.md`; the per-phase
instruction docs live in `todo/`.

| Phase | Focus | Instructions | Status |
|---|---|---|---|
| 1 | K8s + Gazebo Baseline (Helm, ResourceQuota, runbook) | [`todo/k8s_namespace_lifecycle.md`](todo/k8s_namespace_lifecycle.md), [`todo/gazebo_k8s_playground.md`](todo/gazebo_k8s_playground.md) | ✅ |
| 2 | Real Physics in K8s Loop (parity contract, RMSE gates, timing) | [`todo/physics_parity.md`](todo/physics_parity.md) | ✅ |
| 3 | Terrain Model Integration (manifest, parity gate, AGL hook, regression) | [`todo/terrain_integration.md`](todo/terrain_integration.md) | ✅ |
| 4 | Collision Detection & Safety (separation/terrain monitors, KPIs, response state machine) | [`todo/collision_detection.md`](todo/collision_detection.md) | ✅ |
| 5 | Wind Simulation in K8s (manifest, seeded Dryden, gradient, stress envelopes, plugin emulator) | [`todo/wind_simulation.md`](todo/wind_simulation.md) | ✅ |
| 6 | Full-System K8s Validation (matrix, KPIs, fault injection, scalability timing) | [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md) | ✅ |
| 7 | Live View & Replay Backlog (terrain mesh, auth + sessions, .BIN replay, drones-always-visible invariant, mission thumbnails) | [`todo/live_view_backlog.md`](todo/live_view_backlog.md) | ✅ |

The truly-runtime items deferred to the nightly lane:

- live `gz::physics::HeightmapShape::HeightAt` parity (Gazebo runtime)
- real `kubectl delete pod` + `tc qdisc` fault injection (K8s)
- pod-startup + scheduling-delay measurement (K8s)
- Playwright headless DOM smoke (browser toolchain)
- real-flight mission thumbnails (operator capture workflow)

Each one has a CI-deliverable Python equivalent already shipping; see
[`docs/nightly_lane.md`](docs/nightly_lane.md) for the contract and
report-artefact schema.

---

## Phase 8 — ML/Computer Vision Pipeline for Perception

The only open roadmap work. Deserves its own branch — six
per-sub-phase instruction docs live under `todo/`.

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

## Delivered Baseline (snapshot)

- Paper equations 1–7, tables 1–5 — fully implemented and verified.
- 453 Python physics + run-time-view tests passing, 3 skipped, 0
  warnings. Test surface organised as packages
  (`simulation/test_drone_physics/` × 16 files,
  `simulation/test_runtime_view/` × 10 files,
  `simulation/test_terrain.py`, `simulation/test_acceptance_matrix.py`).
- Live Run-time View: FastAPI + Three.js, multi-drone demux,
  post-flight replay, browser launch, DataFlash recording,
  terrain mesh rendering, .BIN replay, Bearer auth + CSRF +
  per-browser session tokens.
- Acceptance matrix runner (4×3×4×4×5 = 960 scenarios, 20-row CI
  subset) with `kpis.json` + `summary.md` per scenario.
- See [`CHANGELOG.md`](CHANGELOG.md) for the full per-phase
  implementation history.

## Definition of Done (Program-Level)

- [x] End-to-end Python-pipeline scenario can launch and execute
      realistic missions reproducibly. (`./run_scenario.sh
      --acceptance-matrix ci`.)
- [x] Physics, terrain, collision, and wind behaviors are covered by
      automated tests and scenario validations.
- [x] Safety and mission KPIs are defined, measured, and enforced by
      CI-compatible gates (`acceptance_report.py` thresholds; strict
      K8s+PX4 thresholds preserved as `*_K8S` constants).
- [x] Documentation (`README.md`, `TESTING.md`, `ROADMAP.md`,
      `TODO.md`, `CHANGELOG.md`, `MAINTENANCE.log`) synchronized with
      implementation status.
- [/] Same end-to-end scenario can launch and execute against real
      Kubernetes + Gazebo, with the strict thresholds. Tracked in
      [`docs/nightly_lane.md`](docs/nightly_lane.md).
