# 🤖 ML Pipeline Reference — `simulation/ml/`

This is the module-by-module reference for the SAR detection ML pipeline.
For a hands-on walkthrough see [`ml_tutorial.md`](ml_tutorial.md). For
the deferred heavy components (real PyTorch training, ONNX/TensorRT
export, Jetson deployment, Gazebo SAR world capture) see
[`nightly_lane.md`](nightly_lane.md).

---

## Architecture at a Glance

The package ships **two parallel surfaces** with the same conventions
(registry, KPI gate, promotion logic):

* **Detection** — optimises the *perception* model.
* **Control** — optimises the *cascaded PID controller* gains for a
  single drone hitting waypoints.

```
DETECTION                         │   CONTROL
                                  │
┌──────────────────────┐          │   ┌────────────────────────┐
│  sar_targets.py      │          │   │  missions.py           │
│  21-class catalogue  │          │   │  4 mission kinds       │
└──────────┬───────────┘          │   └──────────┬─────────────┘
           │                      │              │
┌──────────▼─────────┐            │   ┌──────────▼─────────────┐
│ coco_annotator +   │            │   │ waypoint_optimizer.py  │
│ image_augment +    │            │   │ run_episode +          │
│ model_zoo (6 backs)│            │   │ random_search          │
└──────────┬─────────┘            │   │ over PolicyGains       │
           │                      │   └──────────┬─────────────┘
┌──────────▼─────────┐            │              │
│ inference_logger   │            │   ┌──────────▼─────────────┐
│ JSONL on disk      │            │   │ waypoint_kpi.py        │
└──────┬─────────────┘            │   │ evaluate +             │
       │                          │   │ compare_waypoint_      │
┌──────▼──────┐ ┌─────────────┐   │   │ policies               │
│ hard_example│ │ kpi +       │   │   └──────────┬─────────────┘
│ _miner      │ │ compare_    │   │              │
└─────────────┘ │ models      │   │              ▼
                └──────┬──────┘   │   ┌──────────────────────┐
                       │          │   │ model_registry       │
                       └──────────┴──►│ (shared — single     │
                                      │ JSON file, lineage)  │
                                      └──────────────────────┘
```

Every module is **pure Python** — no PyTorch, no Ultralytics, no OpenCV.
The contract with the deferred backends (`yolo`, `rtdetr`, `detr`,
`fcos`, `onnx`, `tensorrt`) is explicit registration with a clear
`NotImplementedError` pointing at the nightly lane.

---

## 1. `sar_targets.py` — Target Catalogue

The single source of truth for the 21 SAR detection classes. Each entry
ships its COCO `category_id`, name, supercategory, and physical
footprint `(length_m, width_m, height_m)` — which the COCO annotator
uses to size its synthetic bounding boxes.

### Schema

```python
@dataclass(frozen=True)
class SARTarget:
    category_id: int          # COCO ID (1-9 aligned with COCO 2017, 100+ SAR)
    name: str
    supercategory: str        # person | vehicle | equipment | hazard
    footprint_m: tuple        # (length, width, height) in metres
    canonical_yaw: float = 0.0
```

### Catalogue (21 classes)

| Range  | Class | Use |
|:--|:--|:--|
| 1-9    | person, bicycle, car, motorcycle, bus, truck, boat | Transfer-learning aligned with COCO 2017 |
| 100-104 | casualty, stretcher, tent, tarp, medkit | SAR primary targets |
| 105-109 | drone, fire, smoke_plume, debris, vehicle_wreck | Operational + hazard markers |
| 110-113 | raft, lifevest, flare, supply_drop | Maritime + air-drop SAR |

### API

```python
from ml.sar_targets import find, coco_categories, total_count, SAR_TARGETS

person = find("person")               # SARTarget(category_id=1, ...)
cats   = coco_categories()            # [{id, name, supercategory}, ...]
n      = total_count()                # 21 (current catalogue size)
```

### Invariants enforced by tests

- All 21 classes present (`>= 20` for catalogue acceptance).
- Category IDs are unique.
- Supercategories cover `{person, vehicle, equipment, hazard}`.
- Every footprint is a 3-tuple of strictly positive metres.
- `find()` raises `KeyError` with the available list on miss.
- `SARTarget` is frozen — entries cannot be mutated at runtime.

---

## 2. `coco_annotator.py` — Pinhole Projection → COCO JSON

Turns synthetic frame captures (image filename + drone pose + target
poses) into a COCO 2017 `dataset` dict ready for `pycocotools` or any
detection trainer that consumes COCO.

### Camera model

```python
@dataclass
class CameraIntrinsics:
    fx: float; fy: float        # focal lengths in pixels
    cx: float; cy: float        # principal point in pixels
    width: int; height: int     # image size in pixels
```

The projection assumes a **down-looking camera** (typical SAR rig). For
each target the offset `(dx, dy, dz)` between drone and target is
projected with similar triangles:

```
u  = cx + fx · dx / dz
v  = cy − fy · dy / dz       # negate y to match image coords
w_px = fx · footprint_w / dz
h_px = fy · footprint_l / dz
```

### Visibility filter

A target is dropped from `annotations` when:

- `dz <= 0` — camera below or level with target.
- The bbox after sizing falls entirely outside the image (`x + w_px <= 0`,
  `y + h_px <= 0`, `x >= width`, or `y >= height`).
- The clipped width or height collapses to ≤ 0.

A partially-visible target is **clipped** to image bounds (preserved
with reduced bbox), not dropped — the trainer still gets the partial
ground truth.

### Output schema (COCO 2017)

```json
{
  "info":        {"description": "...", "year": 2026, "version": "1.0"},
  "images":      [{"id", "file_name", "width", "height"}],
  "annotations": [{"id", "image_id", "category_id", "bbox": [x,y,w,h],
                   "area", "iscrowd": 0}],
  "categories":  [{"id", "name", "supercategory"}]
}
```

`annotations[].id` is monotonically increasing across all frames,
starting at 1.

### Writing to disk

```python
write(dataset, Path("datasets/sar/train.json"))
```

`write()` creates the parent directory tree if it doesn't exist and
emits indent-2 JSON with a trailing newline.

---

## 3. `image_augment.py` — Domain Randomisation (Pillow)

Pillow-based augmentation surface. Production will swap in Albumentations
once the heavy ML branch ships, but this module is what CI exercises.

### Spec

```python
@dataclass(frozen=True)
class AugmentationSpec:
    brightness_low: float = 0.6
    brightness_high: float = 1.4
    contrast_low: float = 0.7
    contrast_high: float = 1.3
    blur_radius_max: float = 1.5
    cutout_count_max: int = 3
    cutout_size_frac_max: float = 0.15
    weather_drop_density: float = 0.002    # rain/snow density
```

### `Augmenter`

Every method is **deterministic when seeded**:

```python
aug = Augmenter(spec=AugmentationSpec(), seed=42)
img = aug.random_brightness(img)
img = aug.random_contrast(img)
img = aug.random_blur(img)
img = aug.random_cutout(img)
img = aug.add_weather(img, kind="rain")    # or "snow"
img = aug.apply_random_chain(img, p=0.5)   # apply each step with prob p
```

`add_weather("rain")` paints 3-pixel vertical streaks; `"snow"` scatters
single white pixels. Any other `kind` raises `ValueError`.

`apply_random_chain` runs the five steps in fixed order, each gated by
the same probability `p`. Setting `p=0.0` is identity; `p=1.0` applies
every augmentation (used by integration tests).

---

## 4. `model_zoo.py` — Detector ABC + Registry

Replaces hard-coded `YOLO('yolov8n.pt')` calls with a single
`Detector` interface so callers don't depend on a specific backend.

### Detection record

```python
@dataclass(frozen=True)
class Detection:
    category_id: int
    category_name: str
    bbox: tuple              # (x, y, w, h) in pixels — COCO xywh
    confidence: float
    def to_dict(self) -> dict   # JSON-safe
```

### Detector ABC

```python
class Detector(ABC):
    backend: str = "abstract"
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]: ...
    def name(self) -> str: return self.backend
```

### Registry

| Backend     | Status   | Required dep      |
|:------------|:---------|:------------------|
| `mock`      | ✅ active | none              |
| `yolo`      | ⏸ stub   | `ultralytics`     |
| `rtdetr`    | ⏸ stub   | `ultralytics`     |
| `detr`      | ⏸ stub   | `transformers`    |
| `fcos`      | ⏸ stub   | `torchvision`     |
| `onnx`      | ⏸ stub   | `onnxruntime`     |
| `tensorrt`  | ⏸ stub   | `tensorrt`        |

Every stub raises `NotImplementedError` with the missing dep name and a
pointer to `docs/nightly_lane.md`. The ModelZoo API surface is
therefore complete *today* — calling code never needs to special-case
"backend not available", and switching from `mock` to `yolo` once the
deferred branch lands is a one-line change at the call site.

### Custom registration

External callers can register their own backends:

```python
from ml.model_zoo import register, ModelZoo
register("my_backend", lambda weights_path=None, **kw: MyDetector(weights_path))
det = ModelZoo().load("my_backend", weights_path="my.pt")
```

Loaders receive `weights_path` plus arbitrary `**kwargs` so they can
accept backend-specific tuning (confidence threshold, NMS IoU, half
precision, etc.).

### `MockDetector`

Returns either a caller-provided fixed list **or** a default
two-detection list (person + casualty). Passing `fixed=[]` explicitly
yields zero detections — the empty-list-vs-`None` distinction matters
for the hard-example miner integration tests.

---

## 5. `inference_logger.py` — JSONL Capture

Captures every inference outcome to a JSON Lines file so the hard-example
miner and the registry-promotion gate can replay model behaviour without
re-running the model.

### Record schema

```python
@dataclass
class InferenceRecord:
    frame_id: int
    t_wall_s: float
    model_version: str
    image_path: Optional[str] = None
    metadata: dict = field(default_factory=dict)
    detections: List[Detection] = field(default_factory=list)
```

### Logger lifecycle

```python
with InferenceLogger(Path("logs/run.jsonl"), model_version="v1") as log:
    rec = log.log(detections,
                  image_path="frame_001.png",
                  metadata={"weather": "rain", "expected_targets": 2})
```

- File is opened in **append mode**, so re-opening a partial log resumes
  numbering correctly.
- `_tail_max_frame_id()` scans the existing file and continues from
  `max(frame_id) + 1`. **Corrupted JSON lines are skipped** so a
  half-written record from a crashed run doesn't break the next run.
- `log()` flushes after every write — a `KeyboardInterrupt` mid-mission
  loses at most one record.

### Reading back

```python
from ml.inference_logger import read_log, replay
records = list(read_log(Path("logs/run.jsonl")))   # streaming
records = replay(read_log(...))                     # materialised list
```

`InferenceRecord.from_dict()` tolerates missing optional fields
(`model_version`, `image_path`, `metadata`, `detections`) — old logs
remain readable as the schema evolves.

---

## 6. `hard_example_miner.py` — Re-Labelling Triage

Reads inference logs, returns the records most worth re-labelling for
the next training epoch.

### Two heuristics

| Reason       | Trigger                                                  | Score                                       |
|:-------------|:---------------------------------------------------------|:--------------------------------------------|
| `uncertain`  | At least one detection in `[low_conf, high_conf]`        | `1.0` at band centre, decays toward 0 at edges |
| `missed`     | Zero detections AND `metadata["expected_targets"] >= 1`  | `expected_targets` (raw count)              |

```python
@dataclass(frozen=True)
class HardExample:
    frame_id: int
    image_path: Optional[str]
    reason: str           # "uncertain" | "missed"
    score: float          # higher = more important to re-label
    def to_dict(self) -> dict
```

### Config

```python
@dataclass
class MinerConfig:
    low_conf: float = 0.30
    high_conf: float = 0.60
    expected_targets_key: str = "expected_targets"
```

### API

```python
mine(records, cfg=MinerConfig())   # both heuristics, sorted by score desc
find_uncertain(records, cfg)       # "uncertain" only
find_missed(records, cfg)          # "missed" only
```

`mine()` calls both finders, then sorts by descending score so the
operator queue (CVAT, Label Studio) gets the highest-value frames first.

### Edge cases (from tests)

- Empty record list → empty result.
- A frame with detections never registers as "missed" even if
  `expected_targets > 0` — the model returned *something* and the
  uncertain heuristic handles the low-confidence case.
- Missing `expected_targets` metadata defaults to 0 → no missed signal.

---

## 7. `model_registry.py` — Lineage + Promotion State

Append-only JSON file tracking every registered model version and its
KPIs.

### File format

```json
{
  "models": [
    {
      "version": "yolov8s_sar_v1",
      "backend": "yolo",
      "weights_path": "models/yolov8s_sar_v1.pt",
      "trained_at_s": 1745251200.0,
      "parent_version": null,
      "kpis": {"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
      "notes": "First fine-tune on synthetic SAR data."
    }
  ]
}
```

### API

```python
reg = ModelRegistry(Path("models/registry.json"))
reg.register(ModelEntry(version="v1", backend="yolo",
                         weights_path="...", kpis={...}))
reg.all()                       # list[ModelEntry] (independent copy)
reg.get("v1")                   # KeyError if missing
reg.latest()                    # last registered, or None
reg.best_by_kpi("mAP_50")       # max by named KPI key, or None
reg.lineage("v3")               # walks parent_version pointers to root
```

### Safety guarantees

- `register()` rejects duplicate `version` strings (`ValueError`).
- `lineage()` defends against hand-edited cycles in the JSON file —
  uses a `seen` set and raises `ValueError("cycle detected")` rather
  than spinning forever.
- `lineage()` raises `KeyError` if the starting version doesn't exist.
- `register()` does **not** validate that `parent_version` points to an
  existing entry — broken pointers surface only when `lineage()` walks
  them. This is intentional: bulk-import flows can register children
  ahead of parents.
- `all()` returns a defensive copy so callers can't mutate internal
  state.
- `_flush()` rewrites the entire file on every `register()` — fine for
  the current scale (dozens of models). Production will replace this
  with an append-only line format if the registry grows past ~10⁴
  entries.

---

## 8. `kpi.py` — Acceptance Thresholds + Promotion Gate

The numbers a candidate model must clear before it can replace the
incumbent.

### Acceptance thresholds (absolute floor)

```python
ACCEPTANCE_THRESHOLDS = {
    "mAP_50":       0.75,    # primary detection metric
    "recall":       0.85,    # don't miss casualties
    "inference_ms": 50.0,    # per-frame budget on Jetson Orin Nano
}
```

Sourced from `todo/ml_training_pipeline.md`. A model failing any of
these is rejected outright, regardless of how it compares to the
incumbent.

### Promotion deltas (relative improvement)

```python
PROMOTION_MIN_DELTA = {
    "mAP_50":       0.005,    # candidate must beat by ≥ 0.005 mAP
    "recall":       0.005,    # tracked, not currently enforced (see below)
    "inference_ms": -1.0,     # latency: candidate must be ≥ 1 ms faster
                              # OR not regress more than 0.5 ms
}
```

### `evaluate_kpis(measured: dict) -> DetectionKPIs`

Applies the absolute thresholds and returns:

```python
@dataclass
class DetectionKPIs:
    mAP_50: float
    recall: float
    inference_ms: float
    extra: Dict[str, float]      # any additional metrics measured
    failures: List[str]          # human-readable failure descriptions
    verdict: str                 # "PASS" | "FAIL"
```

Missing keys are treated as 0.0 (mAP/recall) or `+inf` (latency) — an
empty dict therefore fails every threshold.

### `compare_models(candidate, incumbent) -> (promote: bool, reason: str)`

| Step | Behaviour |
|:--|:--|
| 1. Acceptance gate | Candidate must pass `evaluate_kpis()` first. |
| 2. Incumbent absent | If `incumbent is None`, the first acceptable model promotes. |
| 3. Latency regression | Candidate latency may not exceed incumbent by more than 0.5 ms. |
| 4. Primary KPI delta | Candidate's `mAP_50` must beat incumbent's by ≥ `PROMOTION_MIN_DELTA["mAP_50"]`. |

**Known limitation:** `compare_models` enforces the absolute recall floor
through `evaluate_kpis` but does not gate on a recall *delta*. A
candidate with `+0.01 mAP_50` and `−0.04 recall` will promote as long as
it still passes `recall >= 0.85`. This is intentional — primary KPI is
mAP — but the constant `PROMOTION_MIN_DELTA["recall"]` is reserved for
when downstream callers want to enforce a stricter rule.

---

## 9. `waypoint_optimizer.py` — Single-Drone PID Tuning

The control-side equivalent of `model_zoo` + `inference_logger` for one
drone. Optimises the cascaded **position + attitude PID gains** so a
single drone hits a mission's waypoints with high completion, tight
settled tracking, fast settling, and modest energy.

### Gain vector

```python
@dataclass(frozen=True)
class PolicyGains:
    pos_x_kp: float; pos_x_ki: float; pos_x_kd: float
    pos_y_kp: float; pos_y_ki: float; pos_y_kd: float
    pos_z_kp: float; pos_z_ki: float; pos_z_kd: float
    att_roll_kp:  float; att_roll_ki:  float; att_roll_kd:  float
    att_pitch_kp: float; att_pitch_ki: float; att_pitch_kd: float
    att_yaw_kp:   float; att_yaw_ki:   float; att_yaw_kd:   float

PolicyGains.from_baseline()                # production defaults
gains.to_dict() / PolicyGains.from_dict(d) # round-trip via JSON
gains.apply_to(controller)                 # mutate a PositionController in place
```

`from_baseline()` returns the exact six-PID seed that
`drone_physics.PositionController.__init__` uses, so a "trained" policy
starts from a known-good point.

### Episode runner

```python
metrics = run_episode(gains, mission_kind="patrol",
                      params=DroneParams(),
                      dt=0.01, waypoint_radius=0.5,
                      hover_time=1.0, max_time=60.0,
                      wind=None, terrain=None)
```

`mission_kind` is one of the four kinds from `simulation/missions.py`
(`patrol`, `lawnmower`, `escort`, `heavy_lift`); the runner picks
`build_mission(kind, n=1)[1]` so the drone follows the same waypoint
list a single-drone live scenario would. Returns:

```python
@dataclass
class EpisodeMetrics:
    waypoints_reached:    int
    waypoint_count:       int
    rmse_xyz_m:           float    # measured during settled (in-radius) phase only
    time_to_first_wp_s:   float    # +inf if never reached
    total_time_s:         float
    energy_proxy_j:       float    # ∫ thrust dt
    max_overshoot_m:      float    # peak excursion past the capture zone
    finite:               bool     # False if the gain vector diverged
    @property
    def completion_ratio(self) -> float
    def as_kpi_dict(self) -> Dict[str, float]
```

**Settled-RMSE** is the key design choice — averaging tracking error
over the full transit between distant waypoints would yield giant
numbers driven by mission geometry, not controller quality. Counting
only in-radius samples isolates "how tightly does the controller hold
the waypoint once it's there".

### Multi-mission evaluation

```python
agg = evaluate_policy(gains,
                      mission_kinds_to_run=["patrol", "lawnmower"],
                      n_episodes_per_mission=1,
                      seed=0,            # reserved (no stochastic wind today)
                      dt=0.01, max_time=60.0)
```

Returns the dict `waypoint_kpi.evaluate_waypoint_kpis()` consumes:
`completion_ratio`, `rmse_xyz_m`, `time_to_first_wp_s`,
`energy_proxy_j`, `max_overshoot_m`, plus `finite_ratio`
(fraction of episodes that didn't diverge).

### Random search trainer

```python
result = random_search(n_trials=16, seed=0,
                       baseline=PolicyGains.from_baseline(),
                       bounds=SearchBounds(pos_lo=0.5, pos_hi=1.5,
                                           att_lo=0.5, att_hi=1.5),
                       mission_kinds_to_run=["patrol"],
                       max_time=60.0)

result.best_gains      # PolicyGains
result.best_metrics    # dict
result.best_objective  # float (higher is better)
result.trials          # list of (gains, metrics, obj) — index 0 is the baseline
```

The objective is `100 * completion - rmse - 0.001 * energy`; divergent
trials return `-inf` and are always rejected. Trial 0 is *always* the
baseline, so the returned best is guaranteed to be at least as good as
the production defaults.

### What's deferred

Real RL trainers (PPO/SAC), CMA-ES, Bayesian optimisation, and PX4 SITL
in-the-loop tuning all plug into the same `PolicyGains` interface. See
[`nightly_lane.md`](nightly_lane.md). Today's `random_search` is the
CI-deliverable trainer that exists to validate the surface.

---

## 10. `waypoint_kpi.py` — Acceptance + Promotion for Policies

Same shape as `kpi.py` but for control quality.

### Acceptance thresholds (absolute floor)

```python
WAYPOINT_ACCEPTANCE_THRESHOLDS = {
    "completion_ratio":     0.95,    # at least 95% of waypoints reached
    "rmse_xyz_m":           1.0,     # settled tracking error ≤ 1 m
    "time_to_first_wp_s":   30.0,    # first waypoint within 30 s
    "max_overshoot_m":      6.0,     # excursion past capture zone ≤ 6 m
}
```

Calibrated so the production-default gains pass on `patrol` and
`lawnmower` (40 m-transit lawnmower has natural braking distance up to
~6 m).

### Promotion deltas

```python
WAYPOINT_PROMOTION_DELTA = {
    "completion_ratio":     0.01,    # +1 percentage-point gain
    "rmse_xyz_m":          -0.05,    # 5 cm tighter
    "time_to_first_wp_s":  -0.5,     # 0.5 s faster
    "energy_proxy_j":       0.0,     # do not regress
}
```

### Promotion logic (`compare_waypoint_policies`)

| Step | Behaviour |
|:--|:--|
| 1 | Candidate must clear `evaluate_waypoint_kpis` (acceptance gate). |
| 2 | If no incumbent, the first acceptable policy promotes. |
| 3 | Energy must not regress (`Δenergy ≤ 0`). |
| 4 | `completion_ratio` must improve by ≥ 0.01, OR — if completion ties — RMSE must tighten by ≥ 0.05 m. |

The completion-tied fallback is what lets a "same hits, tighter
hold" policy promote without inventing a fake completion delta.

---

## Test Coverage Summary

`simulation/test_ml/` ships **150 tests across 12 files**:

| File | Tests | Covers |
|:--|:--:|:--|
| `test_sar_targets.py`            | 10 | Catalogue invariants, frozen dataclass, COCO mapping |
| `test_coco_annotator.py`         | 10 | Projection visibility, multi-frame IDs, partial-bbox clipping, empty target lists, write side-effects |
| `test_image_augment.py`          | 13 | Seeded reproducibility, every augmentation, weather kinds, identity-at-p=0, default spec |
| `test_model_zoo.py`              | 17 | ABC compliance, registry, custom backends, kwargs forwarding, all 6 deferred stubs |
| `test_inference_logger.py`       |  9 | JSONL round-trip, append-mode resume, corrupted-line tolerance, missing-field defaults |
| `test_hard_example_miner.py`     | 11 | Both heuristics, score ordering, edge cases, `to_dict` |
| `test_model_registry.py`         | 13 | Persistence, duplicate rejection, lineage walking, cycle detection, defensive copies |
| `test_kpi.py`                    | 16 | Threshold constants, all failure paths, promotion gate, exact-delta acceptance |
| `test_pipeline_integration.py`   |  7 | Cross-module flows: annotate→write, augment+annotate, infer→mine, registry+kpi promotion |
| `test_waypoint_optimizer.py`     | 18 | PolicyGains round-trip + apply, episode determinism, baseline completes patrol/lawnmower, divergence handling, multi-mission averaging, search bounds, seeded reproducibility |
| `test_waypoint_kpi.py`           | 17 | Threshold constants, every failure path, promotion gate (energy regression, completion-tie rmse fallback, identical-metrics rejection) |

Run subset:

```bash
.venv/bin/python -m pytest simulation/test_ml/ -q
```

---

## Driver Scripts

Three shell entry points live under `scripts/`:

| Script | Purpose |
|:--|:--|
| `scripts/ml_run_pipeline.sh`        | End-to-end SAR detection demo: synthetic dataset → augment → log inferences → mine → register. Writes artefacts to `reports/ml_pipeline/<ts>/`. |
| `scripts/ml_train_waypoint.sh`      | Random search over PID gains for a single drone. Promotes through `compare_waypoint_policies` and writes `reports/ml_waypoint/<ts>/policy_registry.json`. |
| `scripts/ml_evaluate_waypoint.sh`   | Loads a registered policy and re-runs `evaluate_policy()` on one or more missions, prints per-mission breakdown + aggregated verdict. |

All three reuse the project venv at `.venv/` — bootstrap it once via
`./run_scenario.sh` if it doesn't exist yet.

### Flying a trained policy in `--physics-live`

Once a policy is registered, the single-drone Python live viewer can
load it directly:

```bash
./run_scenario.sh --physics-live --policy=pid_v1
./run_scenario.sh --physics-live --policy=pid_v2 \
    --policy-registry=/path/to/policy_registry.json
```

Plumbing:

* `run_scenario.sh` parses `--policy=...` and `--policy-registry=...`
  and forwards them as `--policy` / `--policy-registry` to
  `physics_live_replay`.
* `physics_live_replay.load_policy_gains(registry_path, version)`
  resolves the entry (by version, or best-by-completion when omitted)
  and rebuilds a `PolicyGains` from the entry's bundled gains —
  falling back to the JSON sidecar at `entry.weights_path` when the
  bundle is absent.
* Returns `None` for an empty registry; raises `KeyError` for a
  missing version; raises `ValueError` when neither bundled gains
  nor sidecar is recoverable. The launcher exits non-zero and the
  viewer never boots in the failure cases.
* `drone_physics.run_simulation(..., policy_gains=…)` calls
  `gains.apply_to(controller)` before the simulation loop starts —
  so the cascaded PID seed is the trained vector instead of the
  production defaults.

The flag is **opt-in**. Calling `--physics-live` without it keeps the
production-default PIDs, so existing workflows are unchanged.

## Where Things Go When the Heavy ML Branch Lands

| Module               | Today (CI)                         | Deferred (nightly lane)                  |
|:---------------------|:-----------------------------------|:-----------------------------------------|
| `sar_targets.py`     | ✅ shipped                          | unchanged                                |
| `coco_annotator.py`  | pinhole projection                  | Gazebo camera intrinsics + GT poses      |
| `image_augment.py`   | Pillow (CPU)                        | Albumentations (CPU/GPU)                 |
| `model_zoo.py`       | mock + 6 stubs                      | YOLO/RT-DETR/DETR/FCOS/ONNX/TensorRT     |
| `inference_logger.py`| JSONL on disk                       | unchanged (consumed by Jetson runtime)   |
| `hard_example_miner.py` | confidence band + missed         | unchanged (output → CVAT/Label Studio)   |
| `model_registry.py`  | flat JSON                           | optional W&B mirror                      |
| `kpi.py`             | thresholds only                     | + Ultralytics validator → fills `kpis`   |
| `waypoint_optimizer.py` | random search                    | PPO/SAC, CMA-ES, Bayesian opt, PX4-SITL in-the-loop |
| `waypoint_kpi.py`    | thresholds + completion/rmse gate   | unchanged (consumed by trainers above)   |

The contract is: **everything CI ships today is the consumer-facing
API.** When a heavy backend lands, it plugs into the existing surfaces
(`Detector` ABC, `InferenceLogger`, `ModelRegistry`) without breaking
calling code.
