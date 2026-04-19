# ML Training Pipeline

Detailed instructions for building a reproducible training pipeline
using PyTorch, Ultralytics, and Transformers.

Part of the [ML Vision Overview](ml_vision_overview.md).

---

## 1. Dependencies

```bash
# Training environment (workstation or cloud GPU)
pip install torch torchvision ultralytics transformers
pip install onnx onnxruntime wandb albumentations
pip install pycocotools supervision  # evaluation + annotation

# Edge validation
pip install onnxruntime-gpu tensorrt  # on Jetson: use NVIDIA's wheel
```

## 2. Dataset Format

Use COCO format as the canonical standard (compatible with all models):

```
data/
├── synthetic/
│   ├── images/
│   │   ├── train/
│   │   │   ├── gazebo_0001.jpg
│   │   │   └── ...
│   │   └── val/
│   ├── annotations/
│   │   ├── train.json          # COCO format
│   │   └── val.json
│   └── manifest.yaml           # dataset version, source, stats
├── real/
│   ├── images/
│   └── annotations/
└── splits/
    ├── train.txt
    ├── val.txt
    └── test.txt
```

### COCO annotation schema

```json
{
  "images": [
    {"id": 1, "file_name": "gazebo_0001.jpg", "width": 640, "height": 480}
  ],
  "annotations": [
    {"id": 1, "image_id": 1, "category_id": 1, "bbox": [x, y, w, h],
     "area": 1200.0, "iscrowd": 0}
  ],
  "categories": [
    {"id": 1, "name": "person"},
    {"id": 2, "name": "vehicle"},
    {"id": 3, "name": "equipment"},
    {"id": 4, "name": "casualty"}
  ]
}
```

### YOLO format (auto-converted from COCO)

For YOLOv8/v11, the ultralytics trainer expects a YAML config:

```yaml
# configs/sar_dataset.yaml
path: data/synthetic
train: images/train
val: images/val
names:
  0: person
  1: vehicle
  2: equipment
  3: casualty
```

## 3. Training Script

```python
# perception/training/train.py
"""Train a detection model on SAR domain data."""

import argparse
from pathlib import Path

import torch
import wandb


def train_yolo(config_path: str, model: str = "yolov8s.pt",
               epochs: int = 100, imgsz: int = 640, batch: int = 16):
    """Fine-tune YOLOv8/v11 on SAR data."""
    from ultralytics import YOLO

    wandb.init(project="swarm-perception", config={
        "model": model, "epochs": epochs, "imgsz": imgsz,
    })

    yolo = YOLO(model)
    results = yolo.train(
        data=config_path,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device="0",           # GPU 0
        workers=8,
        patience=20,          # early stopping
        save=True,
        project="runs/detect",
        name="sar_yolov8s",
    )
    # Export to ONNX
    yolo.export(format="onnx", imgsz=imgsz, simplify=True)
    return results


def train_rtdetr(config_path: str, epochs: int = 50):
    """Fine-tune RT-DETR on SAR data."""
    from ultralytics import RTDETR

    model = RTDETR("rtdetr-l.pt")
    results = model.train(
        data=config_path,
        epochs=epochs,
        imgsz=640,
        batch=8,
        device="0",
    )
    model.export(format="onnx", imgsz=640, simplify=True)
    return results


def train_vit_classifier(data_dir: str, num_classes: int = 4,
                         epochs: int = 30):
    """Fine-tune ViT for scene classification."""
    from transformers import (
        ViTForImageClassification,
        ViTImageProcessor,
        Trainer,
        TrainingArguments,
    )
    from torchvision.datasets import ImageFolder
    from torchvision import transforms

    processor = ViTImageProcessor.from_pretrained("google/vit-base-patch16-224")
    model = ViTForImageClassification.from_pretrained(
        "google/vit-base-patch16-224",
        num_labels=num_classes,
        ignore_mismatched_sizes=True,
    )

    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=processor.image_mean,
                             std=processor.image_std),
    ])

    train_ds = ImageFolder(f"{data_dir}/train", transform=transform)
    val_ds = ImageFolder(f"{data_dir}/val", transform=transform)

    args = TrainingArguments(
        output_dir="runs/classify/vit_scene",
        num_train_epochs=epochs,
        per_device_train_batch_size=32,
        evaluation_strategy="epoch",
        save_strategy="epoch",
        load_best_model_at_end=True,
        metric_for_best_model="eval_accuracy",
        report_to="wandb",
    )

    trainer = Trainer(model=model, args=args,
                      train_dataset=train_ds, eval_dataset=val_ds)
    trainer.train()
    return trainer


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="yolov8s",
                        choices=["yolov8s", "yolov8n", "yolov11", "rtdetr", "vit"])
    parser.add_argument("--data", default="configs/sar_dataset.yaml")
    parser.add_argument("--epochs", type=int, default=100)
    args = parser.parse_args()

    if args.model.startswith("yolo"):
        train_yolo(args.data, model=f"{args.model}.pt", epochs=args.epochs)
    elif args.model == "rtdetr":
        train_rtdetr(args.data, epochs=args.epochs)
    elif args.model == "vit":
        train_vit_classifier("data/scenes", epochs=args.epochs)
```

## 4. Evaluation

```python
# perception/training/evaluate.py
"""Evaluate trained models with COCO mAP metrics."""

from ultralytics import YOLO

def evaluate(model_path: str, data_config: str):
    model = YOLO(model_path)
    metrics = model.val(data=data_config, imgsz=640, batch=16)
    print(f"mAP@50:    {metrics.box.map50:.4f}")
    print(f"mAP@50-95: {metrics.box.map:.4f}")
    print(f"Precision: {metrics.box.mp:.4f}")
    print(f"Recall:    {metrics.box.mr:.4f}")
    return metrics
```

### Acceptance KPIs

| Metric | Target (SAR) | Notes |
|:---|:---|:---|
| mAP@50 | > 0.75 | Person detection in aerial views |
| mAP@50-95 | > 0.50 | Strict IoU range |
| Recall@0.5 | > 0.85 | Missing a casualty is critical |
| Precision@0.5 | > 0.70 | False positives waste mission time |
| Inference time | < 50 ms | Real-time at 20+ FPS on Jetson |

## 5. ONNX Export

```python
# perception/training/export_onnx.py
"""Export trained model to ONNX with validation."""

from ultralytics import YOLO
import onnx
import onnxruntime as ort
import numpy as np

def export_and_validate(model_path: str, imgsz: int = 640):
    model = YOLO(model_path)
    onnx_path = model.export(format="onnx", imgsz=imgsz, simplify=True)

    # Validate ONNX
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)

    # Test inference
    session = ort.InferenceSession(onnx_path)
    dummy = np.random.randn(1, 3, imgsz, imgsz).astype(np.float32)
    outputs = session.run(None, {session.get_inputs()[0].name: dummy})
    print(f"ONNX output shapes: {[o.shape for o in outputs]}")
    print(f"Exported: {onnx_path}")
    return onnx_path
```

## 6. Reproducibility

- All training runs logged to Weights & Biases
- Random seeds fixed: `torch.manual_seed(42)`
- Dataset split manifests version-controlled in `data/splits/`
- Model registry (`models/model_registry.json`) tracks:
  ```json
  {
    "yolov8s_sar_v1": {
      "checkpoint": "models/yolov8s_sar.pt",
      "onnx": "models/yolov8s_sar.onnx",
      "dataset": "synthetic_v3+real_v1",
      "map50": 0.82,
      "recall": 0.88,
      "trained": "2026-04-19",
      "epochs": 100,
      "notes": "Fine-tuned on 5k synthetic + 500 real images"
    }
  }
  ```
