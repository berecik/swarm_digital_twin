# TODO — Swarm Digital Twin Backlog

This file is the actionable execution backlog for [`ROADMAP.md`](ROADMAP.md).
It tracks **open work only** — completed items and audit-fix history live
in [`CHANGELOG.md`](CHANGELOG.md), not here.

---

## 0) Project Controls

- [/] Define owners for `simulation`, `gazebo`, `helm`, and `docs`
      deliverables — process item; tracked in the project's
      organisational tooling.
- [/] Establish weekly milestone review with measurable KPI updates —
      process item; the per-scenario `kpis.json` produced by
      `simulation.acceptance_matrix` is the trendable input.

## 1) Phases 1–7 — Closed

All 7 roadmap phases shipped on the Python/CI pipeline. See
[`ROADMAP.md`](ROADMAP.md) for the per-phase status table and
[`CHANGELOG.md`](CHANGELOG.md) for implementation summaries +
verification commands.

The truly-runtime follow-ups deferred to the nightly K8s + Gazebo +
Playwright lane have a single contract: [`docs/nightly_lane.md`](docs/nightly_lane.md).
Their CI-deliverable Python equivalents already ship.

## 2) Phase 8 — ML/Computer Vision Pipeline

Python/CI side closed; heavy-ML side deferred to a future branch
(needs PyTorch / Ultralytics / ONNX / TensorRT / Jetson / W&B /
Gazebo / CVAT). See [`docs/nightly_lane.md`](docs/nightly_lane.md)
for the deferred contract.

**Instructions:** [`todo/ml_*.md`](todo/) — six files (overview,
training pipeline, model zoo, edge deployment, sim-to-real,
continuous improvement).

**Reference docs:**
- [`docs/ml_pipeline.md`](docs/ml_pipeline.md) — per-module API
  reference for `simulation/ml/` (data flow, schemas, KPI tables,
  promotion logic, deferred-backend matrix).
- [`docs/ml_tutorial.md`](docs/ml_tutorial.md) — end-to-end
  developer walkthrough: dataset → augment → infer → mine →
  register → promote → wire to drone.

### 2.1) Synthetic data generation
- [x] SAR target catalogue — `simulation/ml/sar_targets.py` (21 classes).
- [x] COCO-format annotation pipeline — `simulation/ml/coco_annotator.py`.
- [x] Domain randomisation augmentations — `simulation/ml/image_augment.py`.
- [ ] Gazebo SAR training world (deferred — needs Gazebo).
- [ ] Automated RGB-D capture during lawnmower flights (deferred — needs Gazebo).

### 2.2) Training pipeline
- [x] Acceptance KPI thresholds + verdict — `simulation/ml/kpi.py`.
- [ ] YOLOv8s fine-tune on SAR data (deferred — needs PyTorch).
- [ ] RT-DETR / ViT (deferred).
- [ ] W&B experiment tracking (deferred).
- [ ] ONNX export + validation (deferred — needs ONNX runtime).

### 2.3) Model zoo
- [x] `ModelZoo` unified API + `Detector` ABC + `MockDetector` — `simulation/ml/model_zoo.py`.
- [x] Stub registrations for YOLO / RT-DETR / DETR / FCOS / ONNX / TensorRT (raise `NotImplementedError` with pointer at `docs/nightly_lane.md`).
- [ ] Real backend implementations (deferred — needs each backend's heavy deps).
- [ ] `detector.py` uses `ModelZoo` with ROS 2 parameter-driven model swap (deferred — needs ROS 2 + ultralytics in CI).

### 2.4) Edge deployment
- [ ] TensorRT FP16/INT8 builds on Jetson Orin Nano (deferred — Jetson hardware).
- [ ] INT8 calibration dataset (deferred).
- [ ] Edge runtime + inference metrics (deferred).
- [ ] `Dockerfile.jetson` for JetPack 6.0 (deferred).
- [ ] Fleet deployment script (deferred).

### 2.5) Continuous improvement
- [x] Inference logger — `simulation/ml/inference_logger.py` (JSONL).
- [x] Hard example miner — `simulation/ml/hard_example_miner.py`.
- [x] Model registry with version lineage — `simulation/ml/model_registry.py`.
- [x] Promotion gate (`compare_models`) — `simulation/ml/kpi.py`.
- [ ] CVAT / Label Studio labelling integration (deferred — needs CVAT).
- [ ] Automated retraining pipeline (deferred — needs PyTorch + GPU).

## 3) Documentation — per-maintenance gates

Continuous obligations (not one-shot tasks). Enforced on every
MAINTENANCE pass per [`AGENTS.md`](AGENTS.md):

- Keep `README.md` aligned with the scenario workflow.
- Keep `TESTING.md` test purposes + counts current.
- Append a `MAINTENANCE.log` entry per validation cycle.
- Keep `ROADMAP.md` phase status in sync with implementation progress.
