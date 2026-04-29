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

Phase 8 has a Python/CI side and a heavy-ML side. Everything that
runs without PyTorch / Ultralytics / ONNX / TensorRT / Jetson /
W&B / Gazebo / CVAT now ships in `simulation/ml/` (with
`simulation/test_ml/` covering it). The dependency-heavy items
remain deferred to a future ML branch — see
`docs/nightly_lane.md` for the contract.

**Instructions:**
- [`todo/ml_vision_overview.md`](todo/ml_vision_overview.md) — master scenario
- [`todo/ml_training_pipeline.md`](todo/ml_training_pipeline.md) — PyTorch training
- [`todo/ml_model_zoo.md`](todo/ml_model_zoo.md) — YOLO, ViT, DETR, FCOS, RT-DETR
- [`todo/ml_edge_deployment.md`](todo/ml_edge_deployment.md) — ONNX, TensorRT, Jetson
- [`todo/ml_sim_to_real.md`](todo/ml_sim_to_real.md) — synthetic data, domain randomisation
- [`todo/ml_continuous_improvement.md`](todo/ml_continuous_improvement.md) — active learning

### Phase 8A — Simulation Data Generation
- [/] Gazebo SAR training world with 20+ target models — target
      catalogue (`simulation/ml/sar_targets.py`, 21 classes, COCO IDs +
      footprints) ships; the actual SDF world authoring stays deferred.
- [ ] RGB-D camera sensor on drone model for automatic capture (deferred — needs Gazebo).
- [x] Automated annotation pipeline producing COCO-format JSON —
      `simulation/ml/coco_annotator.py` projects ground-truth target
      poses through a pinhole camera into COCO `xywh` bboxes; visible
      vs out-of-FOV vs occluded handled. Tests:
      `TestCocoAnnotator` (5 cases).
- [x] Domain randomisation augmentations — `simulation/ml/image_augment.py`
      ships brightness / contrast / blur / cutout / rain / snow via
      Pillow with seeded reproducibility. Tests: `TestAugmenter` (8 cases).
- [/] Mixed dataset strategy — synthesis side ships; real-flight 500
      images + public aerial datasets are the deferred half.

### Phase 8B — Training Pipeline
- [ ] YOLOv8/v11 fine-tuning with Weights & Biases tracking (deferred — needs PyTorch).
- [ ] RT-DETR training for high-accuracy secondary detection (deferred).
- [ ] ViT scene classifier (damage assessment, terrain type) (deferred).
- [ ] ONNX export with validation (deferred — needs ONNX runtime).
- [x] Acceptance KPIs: mAP@50 > 0.75, Recall > 0.85, Inference < 50 ms —
      thresholds shipped in `simulation/ml/kpi.py` (`ACCEPTANCE_THRESHOLDS`,
      `evaluate_kpis()`); strict K8s+Jetson values preserved as
      `*_K8S` constants. Tests: `TestEvaluate` (4 cases).

### Phase 8C — Model Zoo
- [x] Unified `ModelZoo` API — `simulation/ml/model_zoo.py` with
      `Detection` dataclass, `Detector` ABC, `MockDetector` always
      available, six deferred backends registered as stub loaders that
      raise `NotImplementedError` with a pointer at
      `docs/nightly_lane.md`. Tests: `TestModelZoo` (4 cases) +
      parametrized stub-failure assertions per backend.
- [/] Backends: YOLO, RT-DETR, DETR, FCOS, ONNX Runtime, TensorRT —
      stubs registered; real implementations deferred.
- [ ] `detector.py` uses `ModelZoo` with ROS 2 parameter-driven model swap (deferred — needs ROS 2 + ultralytics in CI).
- [x] Unit tests for each backend load + detect path — covered by
      `TestModelZoo` + `TestMockDetector`.

### Phase 8D — Edge Deployment
- [ ] ONNX → TensorRT FP16/INT8 build on Jetson Orin Nano (deferred — Jetson hardware).
- [ ] INT8 calibration dataset (deferred).
- [ ] Edge runtime with inference timing metrics (deferred).
- [ ] Jetson Dockerfile (`Dockerfile.jetson`) (deferred).
- [ ] Fleet deployment script (deferred).
- [ ] Target: > 20 FPS on YOLOv8s FP16 (Orin Nano) (deferred).

### Phase 8E — Continuous Improvement Loop
- [x] Inference logger — `simulation/ml/inference_logger.py` captures
      `(frame_id, t_wall_s, model_version, image_path, metadata,
      detections)` to JSONL; resumes frame_id across reopens. Tests:
      `TestInferenceLogger` (4 cases).
- [x] Hard example miner — `simulation/ml/hard_example_miner.py` flags
      uncertain detections (confidence in `[0.30, 0.60]`) and missed
      frames (zero detections with `metadata.expected_targets > 0`),
      sorted by descending re-labelling priority. Tests: 4 cases.
- [ ] Labeling workflow with CVAT/Label Studio integration (deferred — needs CVAT).
- [ ] Automated retraining pipeline (deferred — needs PyTorch + GPU).
- [x] Model registry with version lineage (`model_registry.json`) —
      `simulation/ml/model_registry.py` ships `ModelEntry` +
      `ModelRegistry` with `register / get / latest / best_by_kpi /
      lineage`. Tests: 6 cases.
- [x] CI validation: ONNX check + deployed model mAP threshold gate —
      `compare_models()` in `ml/kpi.py` enforces
      `PROMOTION_MIN_DELTA["mAP_50"] = 0.005` and a 0.5 ms latency
      regression cap; `evaluate_kpis()` enforces the absolute
      acceptance thresholds. Real ONNX-checker integration is the
      Jetson-side follow-up.

---

## Delivered Baseline (snapshot)

- Paper equations 1–7, tables 1–5 — fully implemented and verified.
- 603 Python physics + run-time-view + ML tests passing, 3 skipped, 0
  warnings. Test surface organised as packages
  (`simulation/test_drone_physics/` × 16 files,
  `simulation/test_runtime_view/` × 10 files,
  `simulation/test_ml/` × 12 files (150 tests — detection + control),
  `simulation/test_terrain.py`, `simulation/test_acceptance_matrix.py`).
- ML pipeline reference & tutorial published at
  [`docs/ml_pipeline.md`](docs/ml_pipeline.md) and
  [`docs/ml_tutorial.md`](docs/ml_tutorial.md) (PL:
  [`docs/ml_tutorial.pl.md`](docs/ml_tutorial.pl.md)).
- Driver scripts: `scripts/ml_run_pipeline.sh`,
  `scripts/ml_train_waypoint.sh`, `scripts/ml_evaluate_waypoint.sh`.
- Phase 8F (single-drone waypoint optimiser) shipped:
  `simulation/ml/waypoint_optimizer.py` and `waypoint_kpi.py` with
  `PolicyGains`, episode runner, bounded random search, and the
  acceptance + promotion gates.
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
