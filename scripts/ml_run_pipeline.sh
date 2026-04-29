#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# End-to-end SAR detection ML pipeline driver.
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
#
# Walks the synthetic detection pipeline that ships with CI:
#   1. Generate a 50-frame synthetic SAR dataset (COCO 2017 JSON).
#   2. Apply seeded Pillow augmentations to a few sample frames.
#   3. Run the MockDetector across the dataset and log every inference.
#   4. Mine hard examples (uncertainty band + missed targets).
#   5. Register the detector in the model registry and evaluate KPIs.
#
# All artefacts are written into reports/ml_pipeline/<timestamp>/. The
# heavy backends (YOLO/RT-DETR/ONNX/TensorRT) are stubbed today; see
# docs/nightly_lane.md for the deferred contract.
#
# Usage:
#   ./scripts/ml_run_pipeline.sh                  # default: 50 frames, mock detector
#   ./scripts/ml_run_pipeline.sh --frames 200     # bigger dataset
#   ./scripts/ml_run_pipeline.sh --backend mock   # explicit backend (only mock works today)
#   ./scripts/ml_run_pipeline.sh --out /tmp/run1  # custom output dir
# ──────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SIM_DIR="$ROOT_DIR/simulation"
VENV_DIR="$ROOT_DIR/.venv"

# ── Colors ───────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC}  $*" >&2; }
ok()   { echo -e "${GREEN}[OK]${NC}    $*" >&2; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*" >&2; }
fail() { echo -e "${RED}[FAIL]${NC}  $*" >&2; exit 1; }

# ── Args ─────────────────────────────────────────────────────────────────────
FRAMES=50
BACKEND="mock"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$ROOT_DIR/reports/ml_pipeline/$TIMESTAMP"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frames)  FRAMES="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --out)     OUT_DIR="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,21p' "$0" | sed 's/^# //; s/^#//'
      exit 0 ;;
    *) fail "Unknown flag: $1" ;;
  esac
done

[[ -d "$VENV_DIR" ]] || fail "venv not found at $VENV_DIR — run ./run_scenario.sh once to bootstrap."

info "Output directory: $OUT_DIR"
mkdir -p "$OUT_DIR"

# ── Pipeline ─────────────────────────────────────────────────────────────────
info "Running detection pipeline (frames=$FRAMES, backend=$BACKEND)…"
"$VENV_DIR/bin/python" - "$OUT_DIR" "$FRAMES" "$BACKEND" <<'PYEOF'
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, "simulation")

from ml.coco_annotator import (
    CameraIntrinsics, FrameCapture, Pose, TargetInstance, annotate, write,
)
from ml.hard_example_miner import mine, MinerConfig
from ml.image_augment import Augmenter, AugmentationSpec
from ml.inference_logger import InferenceLogger, read_log
from ml.kpi import compare_models, evaluate_kpis
from ml.model_registry import ModelEntry, ModelRegistry
from ml.model_zoo import ModelZoo
from ml.sar_targets import find

out_dir = Path(sys.argv[1])
n_frames = int(sys.argv[2])
backend = sys.argv[3]

# ── Step 1: synthetic dataset ────────────────────────────────────────────────
print(f"[1/5] Generating {n_frames}-frame synthetic SAR dataset…")
camera = CameraIntrinsics(fx=400.0, fy=400.0, cx=320.0, cy=240.0,
                           width=640, height=480)
rng = np.random.default_rng(42)
frames = []
for i in range(n_frames):
    drone_x = float(rng.uniform(-20.0, 20.0))
    drone_y = float(rng.uniform(-20.0, 20.0))
    drone_z = float(rng.uniform(20.0, 60.0))
    targets = [
        TargetInstance(target=find("casualty"),
                       pose=Pose(position=np.array([drone_x + rng.normal(0, 3),
                                                    drone_y + rng.normal(0, 3),
                                                    0.0]))),
        TargetInstance(target=find("car"),
                       pose=Pose(position=np.array([drone_x + rng.normal(0, 8),
                                                    drone_y + rng.normal(0, 8),
                                                    0.0]))),
    ]
    frames.append(FrameCapture(
        image_id=i,
        file_name=f"frame_{i:04d}.png",
        drone_pose=Pose(position=np.array([drone_x, drone_y, drone_z])),
        targets=targets,
    ))

dataset = annotate(frames, camera)
ds_path = write(dataset, out_dir / "coco_train.json")
print(f"    wrote {ds_path} — {len(dataset['images'])} images, "
      f"{len(dataset['annotations'])} annotations, "
      f"{len(dataset['categories'])} categories")

# ── Step 2: augmentation samples ────────────────────────────────────────────
print(f"[2/5] Applying seeded Pillow augmentations (5 samples)…")
aug_dir = out_dir / "augmented_samples"
aug_dir.mkdir(exist_ok=True)
aug = Augmenter(spec=AugmentationSpec(), seed=0)
for i in range(min(5, n_frames)):
    img = Image.new("RGB", (camera.width, camera.height), (135, 145, 155))
    aug.apply_random_chain(img, p=0.7).save(aug_dir / f"aug_{i:04d}.png")
print(f"    wrote {sum(1 for _ in aug_dir.iterdir())} augmented samples in {aug_dir}")

# ── Step 3: inference + logger ──────────────────────────────────────────────
print(f"[3/5] Running {backend} detector across {n_frames} frames + logging…")
detector = ModelZoo().load(backend)
log_path = out_dir / "inference_log.jsonl"
t0 = time.time()
with InferenceLogger(log_path, model_version=f"{backend}_v1") as log:
    for i in range(n_frames):
        img = np.zeros((camera.height, camera.width, 3), dtype=np.uint8)
        detections = detector.detect(img)
        log.log(detections, image_path=f"frame_{i:04d}.png",
                metadata={"weather": "clear", "expected_targets": 2})
elapsed_ms = (time.time() - t0) * 1000.0
per_frame_ms = elapsed_ms / max(n_frames, 1)
print(f"    {n_frames} inferences logged to {log_path} "
      f"({per_frame_ms:.2f} ms/frame)")

# ── Step 4: hard-example mining ─────────────────────────────────────────────
print(f"[4/5] Mining hard examples…")
records = list(read_log(log_path))
hard = mine(records, MinerConfig())
queue_path = out_dir / "relabel_queue.json"
queue_path.write_text(json.dumps([ex.to_dict() for ex in hard], indent=2),
                      encoding="utf-8")
n_uncertain = sum(1 for ex in hard if ex.reason == "uncertain")
n_missed = sum(1 for ex in hard if ex.reason == "missed")
print(f"    {len(hard)} hard examples surfaced "
      f"(uncertain={n_uncertain}, missed={n_missed}) → {queue_path}")

# ── Step 5: registry + KPI gate ────────────────────────────────────────────
print(f"[5/5] Registering model + evaluating KPIs…")
reg_path = out_dir / "registry.json"
reg = ModelRegistry(reg_path)
measured = {
    "mAP_50": 0.78,        # placeholder — real backend would compute
    "recall": 0.86,
    "inference_ms": per_frame_ms,
}
reg.register(ModelEntry(
    version=f"{backend}_v1", backend=backend,
    weights_path=f"(stub — {backend})",
    kpis=measured,
    notes="ml_run_pipeline.sh end-to-end synthetic run",
))
verdict = evaluate_kpis(measured)
print(f"    registered {backend}_v1 → verdict: {verdict.verdict}")
if verdict.failures:
    print(f"    failures: {verdict.failures}")

summary = {
    "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "backend": backend,
    "frames": n_frames,
    "annotations": len(dataset["annotations"]),
    "categories": len(dataset["categories"]),
    "inference_ms_per_frame": per_frame_ms,
    "hard_examples": len(hard),
    "kpi_verdict": verdict.verdict,
    "kpi_failures": verdict.failures,
}
(out_dir / "summary.json").write_text(json.dumps(summary, indent=2),
                                        encoding="utf-8")
print(f"\nSummary written to {out_dir / 'summary.json'}")
PYEOF

ok "Pipeline complete. Artefacts in: $OUT_DIR"
ls -la "$OUT_DIR" >&2
