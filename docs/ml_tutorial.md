# 🎓 ML Tutorial — Develop & Deploy SAR Detection on the Drones

This tutorial walks you through the full Swarm Digital Twin ML
workflow: from generating a synthetic dataset to running detections on
a flying drone in the live viewer. It uses **only the modules that
ship with CI** — the heavy backends (YOLOv8, ONNX, Jetson) follow the
same API once they land via the deferred branch (see
[`nightly_lane.md`](nightly_lane.md)).

If you want the design rationale or per-module reference, read
[`ml_pipeline.md`](ml_pipeline.md) first.

Polish version / Wersja polska: [`ml_tutorial.pl.md`](ml_tutorial.pl.md).

---

## What You'll Build

By the end of this tutorial you will have:

1. A 50-frame synthetic SAR dataset in COCO 2017 format.
2. An augmented variant for domain randomisation.
3. A mock detector run end-to-end through the inference logger.
4. A registered model with KPIs evaluated against the acceptance gate.
5. A list of "hard examples" the operator should re-label next.
6. A path to swap in a real PyTorch backend the day it lands.

Total runtime: **~10 seconds** on a laptop. No GPU required.

---

## Prerequisites

```bash
# Project root
cd swarm_digital_twin

# Python venv (already created by the repo)
source .venv/bin/activate

# Verify ML deps (numpy + Pillow ship with the venv)
python -c "import numpy, PIL; print('OK')"
```

If you're working inside the repo's tests:

```bash
.venv/bin/python -m pytest simulation/test_ml/ -q     # 150 tests
```

---

## Step 1 — Pick Your Targets

Open a Python REPL inside `simulation/`:

```bash
cd simulation
python
```

```python
from ml.sar_targets import find, total_count, coco_categories

print(total_count())                  # 21 — current catalogue size
person   = find("person")             # SARTarget(category_id=1, ...)
casualty = find("casualty")           # SARTarget(category_id=100, ...)
print(coco_categories()[:3])
```

The catalogue is the source of truth for *every* class your model can
detect. To add a new SAR target — say a "hi-vis vest" — append it to
`SAR_TARGETS` in `simulation/ml/sar_targets.py` with a free
`category_id` ≥ 200 to avoid clashing with COCO 2017 IDs.

---

## Step 2 — Generate a Synthetic Dataset

The COCO annotator turns drone-pose + target-pose tuples into ground-truth
labels via a pinhole camera model. In production those tuples come from
Gazebo SAR-world capture; for this tutorial we synthesise them in code.

```python
import numpy as np
from ml.coco_annotator import (
    CameraIntrinsics, FrameCapture, Pose, TargetInstance, annotate, write,
)
from ml.sar_targets import find
from pathlib import Path

# Down-looking camera matching an OAK-D Pro at 640x480.
camera = CameraIntrinsics(fx=400.0, fy=400.0,
                           cx=320.0, cy=240.0,
                           width=640, height=480)

frames = []
rng = np.random.default_rng(42)
for i in range(50):
    # Drone hovers between 20 m and 60 m AGL on a random NE bearing.
    drone_x = float(rng.uniform(-20.0, 20.0))
    drone_y = float(rng.uniform(-20.0, 20.0))
    drone_z = float(rng.uniform(20.0, 60.0))

    # Two targets on the ground beneath the drone.
    targets = [
        TargetInstance(
            target=find("casualty"),
            pose=Pose(position=np.array([drone_x + rng.normal(0, 3),
                                         drone_y + rng.normal(0, 3),
                                         0.0])),
        ),
        TargetInstance(
            target=find("car"),
            pose=Pose(position=np.array([drone_x + rng.normal(0, 8),
                                         drone_y + rng.normal(0, 8),
                                         0.0])),
        ),
    ]
    frames.append(FrameCapture(
        image_id=i,
        file_name=f"frame_{i:04d}.png",
        drone_pose=Pose(position=np.array([drone_x, drone_y, drone_z])),
        targets=targets,
    ))

dataset = annotate(frames, camera)
print(f"images:      {len(dataset['images'])}")
print(f"annotations: {len(dataset['annotations'])}")
print(f"categories:  {len(dataset['categories'])}")

write(dataset, Path("/tmp/sar/train.json"))
```

Expected output (numbers will vary slightly — most targets project
inside the FOV but a few projection-outside frames drop):

```
images:      50
annotations: 96    # ~2 per frame minus the few that fall outside
categories:  21
```

The output file at `/tmp/sar/train.json` is a fully-valid COCO 2017
dataset and can be loaded by `pycocotools`, Ultralytics' YOLO trainer,
detectron2, etc. without modification.

---

## Step 3 — Add Domain Randomisation

Real-world SAR images vary wildly in lighting, weather, motion blur,
and occlusion. The augmentation module simulates this so the model
doesn't overfit to perfect Gazebo renders.

```python
from PIL import Image
from ml.image_augment import Augmenter, AugmentationSpec

# Render a synthetic frame (in production this comes from Gazebo).
img = Image.new("RGB", (640, 480), (135, 145, 155))

aug = Augmenter(spec=AugmentationSpec(), seed=42)

bright_img   = aug.random_brightness(img)
contrast_img = aug.random_contrast(img)
blurred      = aug.random_blur(img)
cutout       = aug.random_cutout(img)
rainy        = aug.add_weather(img, kind="rain")
snowy        = aug.add_weather(img, kind="snow")

# Or chain them all with per-step probability:
chained = aug.apply_random_chain(img, p=0.7)
chained.save("/tmp/sar/aug_demo.png")
```

**Reproducibility:** The augmenter seeds its own `numpy.random.Generator`,
so two `Augmenter(seed=42)` instances produce byte-identical outputs.
This is the property the regression suite relies on — never rely on the
global `numpy.random` state inside augmentations.

**Dataset-side application:** A typical training script wraps the COCO
loader so the augmentation runs once per image per epoch:

```python
import json
from PIL import Image
from pathlib import Path

ds = json.loads(Path("/tmp/sar/train.json").read_text())
aug = Augmenter(seed=0)

for img_meta in ds["images"][:5]:
    # In production each image lives next to the COCO JSON.
    img = Image.new("RGB", (img_meta["width"], img_meta["height"]),
                    (135, 145, 155))
    aug.apply_random_chain(img, p=0.5).save(
        f"/tmp/sar/aug_{img_meta['id']:04d}.png")
```

Bounding boxes are **not** transformed by the current augmenter set
(none of the included augmentations move pixels — brightness, contrast,
blur, cutout, weather all preserve geometry). When the deferred
Albumentations branch lands, geometric transforms (rotate, crop, flip)
will arrive paired with bbox-aware updates.

---

## Step 4 — Pick a Model Backend

The `ModelZoo` is your single entry point. Today only `mock` is
runnable; the six other backends are registered as deferred stubs that
raise `NotImplementedError` with a clear pointer.

```python
from ml.model_zoo import ModelZoo, MockDetector

zoo = ModelZoo()
print(zoo.list())
# ['detr', 'fcos', 'mock', 'onnx', 'rtdetr', 'tensorrt', 'yolo']

detector = zoo.load("mock")          # works today
print(detector.name())               # "mock"
```

Trying a deferred backend is intentionally explicit:

```python
try:
    zoo.load("yolo", weights_path="yolov8n.pt")
except NotImplementedError as e:
    print(e)
# yolo backend requires the optional `ultralytics` dependency
# and is part of the deferred ML branch — see `docs/nightly_lane.md` ...
```

**Custom backend.** Once you're ready to ship a real PyTorch model,
register it once at startup:

```python
from ml.model_zoo import register, Detector, Detection

class YoloV8Backend(Detector):
    backend = "yolo"
    def __init__(self, weights_path: str, **kw):
        from ultralytics import YOLO
        self._model = YOLO(weights_path)
    def detect(self, image):
        results = self._model(image, verbose=False)[0]
        return [
            Detection(category_id=int(box.cls),
                      category_name=results.names[int(box.cls)],
                      bbox=tuple(box.xywh[0].tolist()),
                      confidence=float(box.conf))
            for box in results.boxes
        ]

# Register over the stub:
register("yolo", lambda weights_path=None, **kw:
         YoloV8Backend(weights_path=weights_path, **kw))
```

Calling code stays untouched: `zoo.load("yolo", weights_path="...")`.

---

## Step 5 — Log Inferences to JSONL

The inference logger captures every detection with frame metadata so
you can replay and analyse model behaviour without re-running the
model.

```python
from pathlib import Path
import numpy as np
from ml.inference_logger import InferenceLogger, read_log

log_path = Path("/tmp/sar/run_001.jsonl")
detector = ModelZoo().load("mock")

with InferenceLogger(log_path, model_version="mock_v1") as log:
    for i in range(10):
        # In production: img = camera.capture()
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        detections = detector.detect(img)
        log.log(detections,
                image_path=f"frame_{i:04d}.png",
                metadata={"weather": "clear", "expected_targets": 2})

print(f"logged {sum(1 for _ in read_log(log_path))} records")
```

Inspect the file:

```bash
head -1 /tmp/sar/run_001.jsonl | python -m json.tool
```

```json
{
  "frame_id": 0,
  "t_wall_s": 1745251200.123,
  "model_version": "mock_v1",
  "image_path": "frame_0000.png",
  "metadata": {"weather": "clear", "expected_targets": 2},
  "detections": [
    {"category_id": 1, "category_name": "person",
     "bbox": [50.0, 50.0, 30.0, 60.0], "confidence": 0.92},
    {"category_id": 100, "category_name": "casualty",
     "bbox": [200.0, 150.0, 90.0, 35.0], "confidence": 0.74}
  ]
}
```

**Crash recovery.** Open the same path again with a fresh logger and
it'll resume frame numbering from the last successfully written
record. If the previous run crashed mid-line, the partial record is
silently skipped.

```python
with InferenceLogger(log_path, model_version="mock_v1") as log:
    log.log([])     # this gets frame_id = 10 (continuing from before)
```

---

## Step 6 — Mine Hard Examples

Now that you have an inference log, surface the frames that need
operator attention. Two heuristics ship today:

```python
from ml.hard_example_miner import mine, MinerConfig
from ml.inference_logger import read_log

records = list(read_log(log_path))
hard = mine(records)
for ex in hard[:5]:
    print(ex.reason, ex.score, ex.image_path)
```

**`uncertain`** triggers when at least one detection has confidence in
the band `[0.30, 0.60]` — the model is on the fence. Score peaks at
1.0 when the detection is at the band centre (0.45) and decays toward
the edges.

**`missed`** triggers when the model returned zero detections but the
operator-supplied `metadata["expected_targets"]` is ≥ 1. Score equals
the expected count, so frames where the model missed multiple
casualties bubble to the top.

Tune the thresholds for your dataset:

```python
cfg = MinerConfig(low_conf=0.20, high_conf=0.70)   # widen the band
hard = mine(records, cfg)
```

The output is the input queue for your labelling tool (CVAT, Label
Studio, etc.). The CVAT push happens in the deferred branch — for now
write a JSON sidecar:

```python
import json
Path("/tmp/sar/relabel_queue.json").write_text(json.dumps(
    [ex.to_dict() for ex in hard], indent=2))
```

---

## Step 7 — Register & Promote a Model

After training, register the model so future runs can find it and
compare against it.

```python
from ml.model_registry import ModelEntry, ModelRegistry

reg = ModelRegistry(Path("/tmp/sar/registry.json"))

reg.register(ModelEntry(
    version="mock_v1",
    backend="mock",
    weights_path="(none — mock detector)",
    parent_version=None,
    kpis={"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
    notes="Tutorial seed model.",
))

print(reg.latest().version)         # 'mock_v1'
print(reg.best_by_kpi("mAP_50"))    # ModelEntry(version='mock_v1', ...)
```

When you train a new candidate, evaluate it through the gate before
registering:

```python
from ml.kpi import evaluate_kpis, compare_models

candidate = {"mAP_50": 0.83, "recall": 0.88, "inference_ms": 40.0}
verdict = evaluate_kpis(candidate)
print(verdict.verdict, verdict.failures)    # 'PASS', []

incumbent = reg.best_by_kpi("mAP_50").kpis
promote, reason = compare_models(candidate, incumbent)
print(promote, reason)
# True 'primary KPI improved by +0.0500'

if promote:
    reg.register(ModelEntry(
        version="mock_v2", backend="mock", weights_path="(mock)",
        parent_version="mock_v1", kpis=candidate,
        notes="Wider augmentation chain; +5 pp mAP.",
    ))
```

**Lineage walking** lets you trace any model back to its root:

```python
for entry in reg.lineage("mock_v2"):
    print(entry.version, "←", entry.parent_version)
# mock_v2 ← mock_v1
# mock_v1 ← None
```

Tampering with the JSON file to introduce a cycle (e.g. v1 ← v2 ← v1)
is detected and raises rather than spinning forever.

---

## Step 8 — Promotion-Gate Cheat Sheet

The acceptance gate is the **absolute floor**:

| Metric         | Threshold | Failure mode                         |
|:---------------|:---------:|:-------------------------------------|
| `mAP_50`       | ≥ 0.75    | "mAP_50 0.62 < target 0.75"          |
| `recall`       | ≥ 0.85    | "recall 0.40 < target 0.85"          |
| `inference_ms` | ≤ 50.0    | "inference_ms 120.0 > budget 50.0"   |

The promotion gate (`compare_models`) adds two **relative** rules on
top of the acceptance gate:

1. Latency may not regress by more than 0.5 ms.
2. Primary KPI (`mAP_50`) must improve by ≥ 0.005.

Recall is **not** gated on a delta — only on the absolute floor — so a
candidate at `mAP +0.01 / recall −0.04` will promote as long as recall
stays ≥ 0.85. This matches "primary KPI = mAP" but is worth knowing
when triaging promotion failures.

---

## Step 9 — Wire to the Drone

Once a model has been promoted, the runtime wiring is identical for
mock and real backends:

```python
from ml.model_zoo import ModelZoo
from ml.inference_logger import InferenceLogger
from ml.model_registry import ModelRegistry
from pathlib import Path

reg = ModelRegistry(Path("/var/lib/swarm/models/registry.json"))
active = reg.best_by_kpi("mAP_50")

zoo = ModelZoo()
detector = zoo.load(active.backend, weights_path=active.weights_path)

with InferenceLogger(
    Path(f"/var/lib/swarm/logs/{active.version}.jsonl"),
    model_version=active.version,
) as log:
    while flight_active():                        # your mission loop
        frame, metadata = camera.capture()
        detections = detector.detect(frame)
        log.log(detections, image_path=metadata["filename"],
                metadata={"weather": metadata.get("weather", "unknown"),
                          "expected_targets": metadata.get("expected", 0)})
        publish_to_zenoh(detections)              # downstream consumers
```

The Jetson runtime that ships with the deferred branch will use exactly
this code path. The only change is `zoo.load("yolo", ...)` instead of
`zoo.load("mock")` once a real backend is registered.

---

## Step 10 — Run the Tests

The project ships **150 ML tests across 12 files** (8 detection unit +
1 integration + 2 control). Run them anytime you touch `simulation/ml/`:

```bash
# Just the ML pipeline
.venv/bin/python -m pytest simulation/test_ml/ -q

# Just the integration flows (cross-module)
.venv/bin/python -m pytest simulation/test_ml/test_pipeline_integration.py -v

# Just the waypoint optimiser (control side)
.venv/bin/python -m pytest simulation/test_ml/test_waypoint_optimizer.py simulation/test_ml/test_waypoint_kpi.py -q

# Full project suite (603 incl. ML)
.venv/bin/python -m pytest simulation/ -q
```

The tests are organised so adding a new module to `simulation/ml/`
should be paired with a new `simulation/test_ml/test_<name>.py` plus
relevant edits to `test_pipeline_integration.py`.

---

## Common Pitfalls

| Symptom | Cause | Fix |
|:--|:--|:--|
| `KeyError: 'unknown SAR target'` | Misspelled class name | `find()` raises with the available list — copy the right name from the error message. |
| All projections return `None` | `dz <= 0` (camera below target) | Check drone_pose `position[2]` is greater than every target's `position[2]`. |
| Augmenter outputs differ on re-run | Used global `numpy.random` instead of the seeded generator | Always use `Augmenter(seed=...)` and don't touch `np.random.seed()` inside augmentations. |
| `NotImplementedError: yolo backend requires ...` | Tried to load a deferred backend | Either install the missing dep and `register()` your own loader, or use `mock` for development. |
| `ValueError: cycle detected in lineage` | Hand-edited registry.json with a parent_version cycle | Audit and fix the JSON. The check is intentional — never disable it. |
| `ValueError: candidate fails acceptance gate` | KPI below the absolute floor | Look at `evaluate_kpis(candidate).failures` for the per-metric reason. |

---

## Bonus — Waypoint-Achievement Optimisation (single drone)

The pipeline above optimises *what the drone sees*. The control side
(`simulation/ml/waypoint_optimizer.py` + `waypoint_kpi.py`) optimises
*how well it gets there*. Same registry, same KPI gate shape.

### Run it from the shell (3 commands)

```bash
# 1. Train a tuned PID policy on the patrol mission (5 trials, ~10 s).
./scripts/ml_train_waypoint.sh --trials 5 --mission patrol

# 2. Train a tougher version on patrol + lawnmower (16 trials, ~3 min).
./scripts/ml_train_waypoint.sh --trials 16 --missions patrol,lawnmower

# 3. Evaluate the latest registered policy on every mission.
./scripts/ml_evaluate_waypoint.sh \
    --missions patrol,lawnmower,escort,heavy_lift

# 4. Fly the trained policy in the live single-drone viewer.
./run_scenario.sh --physics-live --policy=pid_v1
```

Artefacts land in `reports/ml_waypoint/<timestamp>/`:

```
policy_registry.json    # ModelRegistry — one entry per promoted policy
pid_v1_gains.json       # 18-float gain vector
trial_history.json      # every trial's (gains, metrics, objective)
summary.json            # human-readable summary
```

### Or drive it from Python

```python
from ml.waypoint_optimizer import (
    PolicyGains, evaluate_policy, run_episode, random_search,
)
from ml.waypoint_kpi import compare_waypoint_policies, evaluate_waypoint_kpis

# Run the production-default controller through the patrol mission.
baseline = PolicyGains.from_baseline()
metrics = run_episode(baseline, mission_kind="patrol", max_time=60.0)
print(f"completion={metrics.completion_ratio:.2f} "
      f"rmse_settled={metrics.rmse_xyz_m:.3f} m")

# Tune via random search — first trial is always the baseline.
result = random_search(n_trials=8, seed=0,
                        mission_kinds_to_run=["patrol"],
                        max_time=60.0)
print(f"best objective: {result.best_objective:.3f}")
print(f"best gains: {result.best_gains}")

# Promotion gate against the production baseline.
baseline_metrics = evaluate_policy(baseline, max_time=60.0)
promote, reason = compare_waypoint_policies(
    result.best_metrics, baseline_metrics)
print(promote, reason)
```

### What metrics matter and why

| KPI | Meaning | Threshold |
|:--|:--|:--:|
| `completion_ratio` | Fraction of waypoints reached. | ≥ 0.95 |
| `rmse_xyz_m` | Settled tracking error — *only* counts samples while the drone is inside the capture radius. | ≤ 1.0 m |
| `time_to_first_wp_s` | Time before the first waypoint is captured + held. | ≤ 30 s |
| `max_overshoot_m` | Worst excursion past the waypoint while inside the capture zone (catches lazy braking without counting transit distance). | ≤ 6.0 m |
| `energy_proxy_j` | `∫ thrust dt` — surrogate for battery use. Tracked, not floored — used as a non-regression gate during promotion. | — |

**Promotion logic** (`compare_waypoint_policies`):

1. Candidate must clear the absolute floor (`evaluate_waypoint_kpis`).
2. Energy must not regress.
3. `completion_ratio` must improve by ≥ 0.01 *or* — if completion ties
   — RMSE must tighten by ≥ 0.05 m.

This means a policy that hits the same waypoints but holds them
tighter still earns a promotion.

### When to retrain per mission

The shipped acceptance gate is calibrated for `patrol` and `lawnmower`.
The other two missions (`escort`, `heavy_lift`) move the drone in
tighter circles around moving formations. If you tune *only* on
`patrol` and then evaluate on `lawnmower`, you'll often see the
overshoot threshold trip — the controller hasn't been exposed to
40 m-transit dynamics. Solution: pass the missions you'll actually fly
to `--missions` at training time, e.g.:

```bash
./scripts/ml_train_waypoint.sh \
    --trials 32 --missions patrol,lawnmower,escort
```

Trials count linearly: 32 trials × 3 missions × ~3 s/episode ≈ 5 min.

### Where the deferred branch fits

`PolicyGains` is the trained "weights" interface. Real RL trainers
(PPO, SAC), CMA-ES, Bayesian optimisation, and PX4-SITL in-the-loop
tuning all plug into the same `PolicyGains` shape — see
`docs/nightly_lane.md` for the contract. Today's `random_search` is
the CI-deliverable baseline that proves the surface works end-to-end.

### Flying the trained policy

The single-drone Python physics path
(`./run_scenario.sh --physics-live`) accepts the same trained policy
as a CLI flag — no code changes needed:

```bash
# Auto-pick the latest registered policy (best by completion).
./run_scenario.sh --physics-live --policy=pid_v1

# Pin the registry path explicitly when several reports exist.
./run_scenario.sh --physics-live \
    --policy=pid_v2 \
    --policy-registry=/path/to/policy_registry.json

# Loop the demo + use the trained policy.
./run_scenario.sh --physics-live --loop --policy=pid_v1
```

The flag is plumbed through `physics_live_replay`'s
`load_policy_gains()` into `drone_physics.run_simulation(policy_gains=…)`,
which seeds the cascaded `PositionController` before the loop starts.
Calling without `--policy` keeps the production-default PIDs — the
flag is opt-in. A startup line confirms which policy actually loaded:

```
Using trained PID policy pid_v1 from reports/ml_waypoint/<ts>/policy_registry.json
```

If the registry can't be found or the version doesn't exist, the
launcher exits with a clear error before booting the viewer — so
typo'd policy names fail fast rather than silently flying with the
baseline.

---

## What's Next

Once you've worked through the steps above, you have a complete mental
model of the ML pipeline as it ships in CI. The deferred branch
(`docs/nightly_lane.md`) plugs into the same surfaces:

- Replace `mock` in `model_zoo` with YOLO/RT-DETR/ONNX/TensorRT.
- Replace synthetic `FrameCapture` lists with Gazebo SAR-world capture.
- Replace Pillow augmentations with Albumentations (CPU/GPU).
- Push the hard-example queue into CVAT or Label Studio.
- Mirror the registry into Weights & Biases for cross-team visibility.

Each of those changes is a **swap behind an existing API** — none of
them require modifying calling code that already uses the modules
documented here.

For the per-module reference, see [`ml_pipeline.md`](ml_pipeline.md).
