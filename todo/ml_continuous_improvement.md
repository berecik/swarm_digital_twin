# ML Continuous Improvement — Test-Fix Loop for Perception

Detailed instructions for continuously improving the drone perception
stack through active learning, automated evaluation, and iterative
retraining.

Part of the [ML Vision Overview](ml_vision_overview.md).

---

## 1. Goal

Build a closed-loop system where:
1. Models are deployed to drones
2. Inference results are logged with input images
3. Hard examples (low confidence, false positives) are mined
4. New labels are added to the training set
5. Models are retrained and validated
6. Improved models are deployed — repeat

## 2. The ML Test-Fix Loop

```
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  ┌─────────┐    ┌──────────┐    ┌───────────┐           │
│  │  Deploy  │───►│  Infer   │───►│  Log      │           │
│  │  Model   │    │  (Edge)  │    │  Results  │           │
│  └─────────┘    └──────────┘    └─────┬─────┘           │
│       ▲                               │                  │
│       │                               ▼                  │
│  ┌─────────┐    ┌──────────┐    ┌───────────┐           │
│  │ Validate │◄───│ Retrain  │◄───│   Mine    │           │
│  │ (mAP)   │    │ (PyTorch)│    │  (Active  │           │
│  └─────────┘    └──────────┘    │  Learning)│           │
│                                  └───────────┘           │
└──────────────────────────────────────────────────────────┘
```

### Trigger Conditions

The loop is triggered when any of these occur:
- **Scheduled:** weekly retraining cycle
- **Performance drop:** mAP on validation set drops below threshold
- **Hard example threshold:** 100+ hard examples accumulated since last training
- **New domain:** drone deployed in a new environment (different terrain, weather)

## 3. Inference Logging

### On the Edge (Jetson)

Log every N-th frame (default: every 30th = 1 per second at 30 FPS):

```python
# perception/perception_core/inference_logger.py

import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List


@dataclass
class InferenceLog:
    timestamp: float
    image_path: str
    detections: List[dict]   # [{class_id, confidence, bbox}]
    inference_ms: float
    drone_id: int
    mission_id: str


class InferenceLogger:
    def __init__(self, log_dir: str = "/data/inference_logs",
                 sample_rate: int = 30):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.sample_rate = sample_rate
        self._frame_count = 0
        self._log_file = open(
            self.log_dir / "inference.jsonl", "a", encoding="utf-8")

    def log(self, image, detections, inference_ms, drone_id, mission_id):
        self._frame_count += 1
        if self._frame_count % self.sample_rate != 0:
            return

        # Save image
        import cv2
        ts = time.time()
        img_name = f"frame_{ts:.3f}.jpg"
        cv2.imwrite(str(self.log_dir / "images" / img_name), image)

        # Save metadata
        entry = InferenceLog(
            timestamp=ts,
            image_path=img_name,
            detections=[{
                "class_id": d.class_id,
                "confidence": d.confidence,
                "bbox": d.bbox_xyxy.tolist(),
            } for d in detections],
            inference_ms=inference_ms,
            drone_id=drone_id,
            mission_id=mission_id,
        )
        self._log_file.write(json.dumps(asdict(entry)) + "\n")
        self._log_file.flush()
```

### Collect from Fleet

After each mission, pull inference logs from all drones:

```bash
#!/bin/bash
# scripts/collect_inference_logs.sh
MISSION_ID=$(date +%Y%m%d_%H%M%S)
DRONES="drone1.local drone2.local drone3.local"

mkdir -p data/field_logs/$MISSION_ID

for host in $DRONES; do
    drone_id=$(echo $host | grep -o '[0-9]')
    scp -r "jetson@${host}:/data/inference_logs/" \
        "data/field_logs/${MISSION_ID}/drone_${drone_id}/"
done
```

## 4. Hard Example Mining

Automatically find images where the model is uncertain or wrong:

```python
# perception/training/hard_example_miner.py

import json
from pathlib import Path
from typing import List


def mine_hard_examples(log_dir: str,
                       low_conf_threshold: float = 0.3,
                       high_conf_threshold: float = 0.6) -> List[str]:
    """Find images with uncertain detections.

    Hard examples are frames where:
    - At least one detection has confidence in [low, high] range
    - OR no detections at all (potential false negatives)
    - OR many overlapping detections (confusion)
    """
    hard = []
    log_file = Path(log_dir) / "inference.jsonl"
    with open(log_file, encoding="utf-8") as f:
        for line in f:
            entry = json.loads(line)
            dets = entry["detections"]

            # No detections — possible false negative
            if not dets:
                hard.append(entry["image_path"])
                continue

            # Uncertain detections
            for d in dets:
                if low_conf_threshold < d["confidence"] < high_conf_threshold:
                    hard.append(entry["image_path"])
                    break

            # Too many overlapping detections
            if len(dets) > 10:
                hard.append(entry["image_path"])

    return hard
```

## 5. Labeling Workflow

Hard examples need manual annotation before they can be added to the
training set.

### Tools

| Tool | Type | Cost | Best for |
|:---|:---|:---|:---|
| **Label Studio** | Self-hosted | Free | Team annotation, quality control |
| **CVAT** | Self-hosted | Free | Bbox + segmentation, interpolation |
| **Roboflow** | Cloud | Free tier | Quick labeling, augmentation, export |

### Process

```bash
# 1. Export hard examples to labeling tool
python perception/training/hard_example_miner.py \
  --log-dir data/field_logs/20260419_143000/drone_1/ \
  --output data/to_label/batch_001/

# 2. Label in CVAT/Label Studio (manual step)
# ...

# 3. Export COCO annotations
# Import into data/real/annotations/

# 4. Merge with existing dataset
python perception/training/merge_datasets.py \
  --existing data/synthetic/annotations/train.json \
  --new data/real/annotations/batch_001.json \
  --output data/merged/annotations/train.json
```

## 6. Automated Retraining

```python
# perception/training/retrain.py
"""Automated retraining pipeline."""

from pathlib import Path
from train import train_yolo
from evaluate import evaluate


def retrain_cycle(data_config: str, baseline_map: float = 0.75):
    """Run one retraining cycle.

    1. Train on merged dataset
    2. Evaluate on validation set
    3. Compare against baseline
    4. Export if improved
    """
    # Train
    results = train_yolo(data_config, model="yolov8s.pt", epochs=50)

    # Evaluate
    best_model = Path(results.save_dir) / "weights" / "best.pt"
    metrics = evaluate(str(best_model), data_config)

    # Compare
    new_map = metrics.box.map50
    if new_map > baseline_map:
        print(f"IMPROVED: mAP {baseline_map:.3f} → {new_map:.3f}")
        # Export ONNX
        from export_onnx import export_and_validate
        export_and_validate(str(best_model))
        return True, new_map
    else:
        print(f"NO IMPROVEMENT: mAP {new_map:.3f} <= {baseline_map:.3f}")
        return False, new_map
```

## 7. Model Registry & Versioning

Track all model versions with their lineage:

```json
// perception/models/model_registry.json
{
  "versions": [
    {
      "version": "v1.0",
      "model": "yolov8s_sar_v1.pt",
      "onnx": "yolov8s_sar_v1.onnx",
      "dataset": "synthetic_5k",
      "map50": 0.72,
      "recall": 0.80,
      "date": "2026-04-19",
      "deployed": false
    },
    {
      "version": "v1.1",
      "model": "yolov8s_sar_v1.1.pt",
      "onnx": "yolov8s_sar_v1.1.onnx",
      "dataset": "synthetic_5k + real_200 + hard_150",
      "map50": 0.81,
      "recall": 0.88,
      "date": "2026-04-26",
      "deployed": true,
      "parent": "v1.0"
    }
  ]
}
```

## 8. CI Integration

Add ML validation to the CI pipeline:

```yaml
# .github/workflows/ml-validate.yml
name: ML Model Validation
on:
  push:
    paths: ['perception/models/**', 'perception/training/**']
jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Validate ONNX model
        run: |
          pip install onnx onnxruntime
          python perception/training/export_onnx.py --validate-only
      - name: Check model registry
        run: python -c "
          import json
          reg = json.load(open('perception/models/model_registry.json'))
          deployed = [v for v in reg['versions'] if v.get('deployed')]
          assert len(deployed) >= 1, 'No deployed model version'
          for v in deployed:
            assert v['map50'] >= 0.70, f'Deployed model mAP too low: {v[\"map50\"]}'
        "
```

## 9. Virtualisation for ML Development

### Local Development (no GPU)

```bash
# CPU-only inference for development and testing
pip install onnxruntime  # CPU only
python -c "
from perception_core.model_zoo import ModelZoo
zoo = ModelZoo('yolov8n.pt', backend='yolo', device='cpu')
# Uses CPU for inference — slower but works anywhere
"
```

### Docker GPU Development

```bash
# Start a GPU-enabled dev container
docker run -it --gpus all \
  -v $(pwd)/perception:/workspace/perception \
  -v $(pwd)/data:/workspace/data \
  nvcr.io/nvidia/pytorch:24.03-py3 \
  bash

# Inside container:
pip install ultralytics transformers albumentations
python perception/training/train.py --model yolov8s --epochs 10
```

### K8s Training Job

```yaml
# k8s/training-job.yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: ml-training
  namespace: swarm-sim
spec:
  template:
    spec:
      containers:
        - name: trainer
          image: nvcr.io/nvidia/pytorch:24.03-py3
          command: ["python", "perception/training/train.py",
                    "--model", "yolov8s", "--epochs", "100"]
          resources:
            limits:
              nvidia.com/gpu: 1
          volumeMounts:
            - name: data
              mountPath: /workspace/data
      volumes:
        - name: data
          persistentVolumeClaim:
            claimName: training-data
      restartPolicy: Never
```

## 10. Acceptance Criteria

- [ ] Inference logging captures 1 frame/sec with detections + metadata
- [ ] Hard example miner identifies uncertain/missed detections
- [ ] Retraining pipeline runs end-to-end (train → evaluate → export)
- [ ] Model registry tracks versions with dataset lineage
- [ ] Each retraining cycle improves mAP (or is rejected)
- [ ] CI validates ONNX model and deployed version mAP threshold
- [ ] Complete cycle runs in < 4 hours (5k images, 50 epochs)
