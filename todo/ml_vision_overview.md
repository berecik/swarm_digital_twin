# Machine Learning & Computer Vision — Overview

This document is the master scenario for implementing ML/DL-based
computer vision in the Swarm Digital Twin. It covers the full pipeline
from simulation-generated training data through model training,
edge deployment, and continuous improvement.

Related detailed scenarios:
- [`ml_training_pipeline.md`](ml_training_pipeline.md) — dataset creation, training, evaluation
- [`ml_model_zoo.md`](ml_model_zoo.md) — model architectures (YOLO, ViT, DETR, FCOS)
- [`ml_edge_deployment.md`](ml_edge_deployment.md) — ONNX export, TensorRT, Jetson deployment
- [`ml_sim_to_real.md`](ml_sim_to_real.md) — sim-to-real transfer, domain randomisation
- [`ml_continuous_improvement.md`](ml_continuous_improvement.md) — test-fix loop, active learning

---

## Current State

The perception pipeline has:
- **YOLOv8 nano** (`yolov8n.pt`) for person detection (class 0 only)
- **Depth fusion** via pinhole camera model (RGB + Depth → 3D world position)
- **ROS 2 Humble** integration with TF2 transforms
- **K8s/Docker** deployment (CUDA 12.2 base, 1 perception container per drone)
- **13 unit tests** for detector, localizer, search planner

What's missing:
- Multi-class detection (vehicles, equipment, casualties)
- Model fine-tuning on domain-specific data
- Advanced architectures (ViT, DETR, FCOS)
- ONNX/TensorRT optimisation for edge deployment
- Simulation-generated training data pipeline
- Active learning / continuous improvement loop

---

## Architecture Vision

```
┌─────────────────────────────────────────────────────────────────┐
│                    TRAINING (Cloud/Workstation)                  │
│                                                                 │
│  Gazebo Sim ──► Synthetic Data ──► Augmentation ──► Training    │
│       │              │                                  │       │
│  Real Logs ──► Manual Labels ──────────────────────►    │       │
│                                                         ▼       │
│                                              PyTorch Model      │
│                                                    │            │
│                                              ONNX Export        │
│                                                    │            │
│                                              TensorRT / INT8    │
└─────────────────────────────────┬───────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                    EDGE (Jetson Orin Nano per drone)             │
│                                                                 │
│  Camera ──► Pre-process ──► TensorRT Engine ──► Post-process    │
│                                                      │          │
│                                              Detections         │
│                                                      │          │
│                                        Depth Fusion + TF2       │
│                                                      │          │
│                                        3D World Coordinates     │
│                                                      │          │
│                                        ROS 2 → Zenoh → GCS     │
└─────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CONTINUOUS IMPROVEMENT                        │
│                                                                 │
│  Edge Inference Logs ──► Hard Example Mining ──► Re-label       │
│                                                      │          │
│                                              Augment Dataset    │
│                                                      │          │
│                                              Re-train Model     │
│                                                      │          │
│                                              Validate + Deploy  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Model Selection Matrix

| Model | Task | Speed (Jetson) | Accuracy | Use Case |
|:---|:---|:---|:---|:---|
| **YOLOv8n** | Detection | ~30 FPS | Good | Real-time SAR person detection |
| **YOLOv8s** | Detection | ~20 FPS | Better | Improved accuracy when compute allows |
| **YOLOv11** | Detection | ~25 FPS | Best YOLO | Latest ultralytics, improved small objects |
| **ViT-B/16** | Classification | ~5 FPS | Excellent | Scene classification, damage assessment |
| **DETR** | Detection | ~3 FPS | Excellent | End-to-end detection, no NMS needed |
| **FCOS** | Detection | ~15 FPS | Good | Anchor-free, good for dense scenes |
| **RT-DETR** | Detection | ~20 FPS | Excellent | Real-time DETR variant, best accuracy/speed |
| **SAM** | Segmentation | ~2 FPS | Excellent | Instance segmentation for detailed analysis |

**Recommended starting point:** YOLOv8s for detection + RT-DETR for high-accuracy secondary pass.

---

## Implementation Phases

### Phase A — Simulation Data Generation (2-3 weeks)
Generate labeled training data from Gazebo simulation.
See [`ml_sim_to_real.md`](ml_sim_to_real.md).

### Phase B — Training Pipeline (2-3 weeks)
Build reproducible training with PyTorch + Weights & Biases.
See [`ml_training_pipeline.md`](ml_training_pipeline.md).

### Phase C — Model Zoo Integration (1-2 weeks)
Integrate multiple architectures with a unified inference API.
See [`ml_model_zoo.md`](ml_model_zoo.md).

### Phase D — Edge Deployment (2-3 weeks)
Export to ONNX, optimise with TensorRT, deploy to Jetson.
See [`ml_edge_deployment.md`](ml_edge_deployment.md).

### Phase E — Continuous Improvement (ongoing)
Active learning, hard example mining, automated re-training.
See [`ml_continuous_improvement.md`](ml_continuous_improvement.md).

---

## Directory Structure (target)

```
perception/
├── perception_core/
│   ├── detector.py              # ROS 2 detection node (existing)
│   ├── object_localizer.py      # 3D localization (existing)
│   ├── search_planner.py        # Search patterns (existing)
│   ├── model_zoo.py             # Unified inference API (new)
│   ├── edge_runtime.py          # TensorRT/ONNX runtime (new)
│   └── active_learner.py        # Hard example mining (new)
├── training/
│   ├── train.py                 # Training script (PyTorch)
│   ├── evaluate.py              # mAP/recall evaluation
│   ├── export_onnx.py           # ONNX export + validation
│   ├── configs/                 # Model/training configs
│   │   ├── yolov8s_sar.yaml
│   │   ├── rtdetr_sar.yaml
│   │   └── vit_scene.yaml
│   └── augmentations.py         # Domain-specific augmentations
├── data/
│   ├── synthetic/               # Gazebo-generated images + labels
│   ├── real/                    # Manually labeled field images
│   ├── splits/                  # train/val/test split manifests
│   └── generate_synthetic.py   # Gazebo camera capture script
├── models/
│   ├── yolov8s_sar.pt           # Trained PyTorch weights
│   ├── yolov8s_sar.onnx         # ONNX export
│   ├── yolov8s_sar.engine       # TensorRT engine (device-specific)
│   └── model_registry.json      # Model versions + metrics
├── test/
│   ├── test_detector.py         # Existing tests
│   ├── test_model_zoo.py        # Model zoo tests (new)
│   ├── test_edge_runtime.py     # Edge inference tests (new)
│   └── test_training.py         # Training pipeline tests (new)
└── Dockerfile.training          # Training container (GPU)
```
