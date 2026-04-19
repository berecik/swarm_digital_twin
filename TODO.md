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

## 2) Phase 8 — ML/Computer Vision Pipeline (open)

The only remaining roadmap section. Deserves its own branch — the
sub-phase work is large enough that it should not interleave with
Phase 1–7 maintenance.

**Instructions:** see [`todo/ml_*.md`](todo/) (six files: overview,
training pipeline, model zoo, edge deployment, sim-to-real,
continuous improvement).

### 2.1) Synthetic data generation
- [ ] Gazebo SAR training world with 20+ human / vehicle models.
- [ ] Automated image capture during lawnmower flights.
- [ ] COCO-format annotation from Gazebo ground-truth poses.
- [ ] Domain randomisation augmentations (albumentations).

### 2.2) Training pipeline
- [ ] YOLOv8s fine-tune on SAR data (mAP@50 > 0.75).
- [ ] RT-DETR for high-accuracy pass.
- [ ] ViT scene classifier (damage, terrain type).
- [ ] Weights & Biases experiment tracking.
- [ ] ONNX export + validation.

### 2.3) Model zoo
- [ ] `ModelZoo` unified API replacing hard-coded YOLO in `detector.py`.
- [ ] Backends: YOLO, RT-DETR, DETR, FCOS, ONNX, TensorRT.
- [ ] ROS 2 parameter-driven model selection.

### 2.4) Edge deployment
- [ ] TensorRT FP16/INT8 builds on Jetson Orin Nano.
- [ ] INT8 calibration dataset (500+ images).
- [ ] `edge_runtime.py` with inference metrics.
- [ ] `Dockerfile.jetson` for JetPack 6.0.
- [ ] Fleet deployment script.

### 2.5) Continuous improvement
- [ ] Inference logger (1 frame/sec with metadata).
- [ ] Hard example miner (low confidence + missed detections).
- [ ] CVAT / Label Studio labelling integration.
- [ ] Automated retraining pipeline.
- [ ] Model registry with version lineage.
- [ ] CI gate: ONNX check + mAP threshold.

## 3) Documentation — per-maintenance gates

Continuous obligations (not one-shot tasks). Enforced on every
MAINTENANCE pass per [`AGENTS.md`](AGENTS.md):

- Keep `README.md` aligned with the scenario workflow.
- Keep `TESTING.md` test purposes + counts current.
- Append a `MAINTENANCE.log` entry per validation cycle.
- Keep `ROADMAP.md` phase status in sync with implementation progress.
