# ML Edge Deployment — ONNX, TensorRT, Jetson

Detailed instructions for deploying trained models to edge devices
(NVIDIA Jetson Orin Nano) with maximum throughput.

Part of the [ML Vision Overview](ml_vision_overview.md).

---

## 1. Deployment Pipeline

```
PyTorch Model (.pt)
    │
    ▼
ONNX Export (.onnx)     ← platform-independent
    │
    ▼
ONNX Validation         ← onnx.checker + onnxruntime inference test
    │
    ▼
TensorRT Build (.engine) ← device-specific, INT8/FP16 quantised
    │
    ▼
Edge Runtime             ← perception_core/edge_runtime.py
```

## 2. ONNX Export

```bash
# From the training machine
python perception/training/export_onnx.py \
  --model models/yolov8s_sar.pt \
  --imgsz 640 \
  --simplify
```

Validation checks:
- `onnx.checker.check_model()` passes
- `onnxruntime` inference on random input produces valid output shapes
- mAP on validation set matches PyTorch model within 0.5%

## 3. TensorRT Optimisation

### On the Jetson (must build on target device):

```bash
# Convert ONNX → TensorRT engine with FP16
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov8s_sar.onnx \
  --saveEngine=models/yolov8s_sar_fp16.engine \
  --fp16 \
  --workspace=4096

# INT8 quantisation (needs calibration dataset)
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov8s_sar.onnx \
  --saveEngine=models/yolov8s_sar_int8.engine \
  --int8 \
  --calib=data/calibration/ \
  --workspace=4096
```

### INT8 Calibration Dataset

Create a representative subset of 500-1000 images:

```python
# perception/training/create_calibration_set.py
import random
import shutil
from pathlib import Path

images = list(Path("data/synthetic/images/val").glob("*.jpg"))
calibration = random.sample(images, min(500, len(images)))
cal_dir = Path("data/calibration")
cal_dir.mkdir(exist_ok=True)
for img in calibration:
    shutil.copy(img, cal_dir / img.name)
```

## 4. Edge Runtime Module

```python
# perception/perception_core/edge_runtime.py
"""Optimised inference for Jetson edge deployment."""

import time
from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class InferenceMetrics:
    """Per-frame inference timing."""
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float
    total_ms: float
    fps: float


class EdgeRuntime:
    """TensorRT or ONNX inference with performance monitoring."""

    def __init__(self, model_path: str, backend: str = "onnx",
                 input_size: int = 640, conf_threshold: float = 0.5):
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.backend = backend
        self._metrics_history: List[InferenceMetrics] = []

        if backend == "tensorrt":
            self._load_tensorrt(model_path)
        else:
            self._load_onnx(model_path)

    def _load_onnx(self, path: str):
        import onnxruntime as ort
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self._session = ort.InferenceSession(path, providers=providers)
        self._input_name = self._session.get_inputs()[0].name

    def _load_tensorrt(self, path: str):
        # TensorRT loading requires pycuda context
        # See NVIDIA TensorRT Python API docs
        pass  # TODO: implement with torch2trt or pycuda

    def infer(self, image: np.ndarray) -> tuple:
        """Run inference with timing. Returns (outputs, metrics)."""
        t0 = time.perf_counter()

        # Preprocess
        import cv2
        resized = cv2.resize(image, (self.input_size, self.input_size))
        blob = resized.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[np.newaxis]  # NCHW
        t1 = time.perf_counter()

        # Inference
        outputs = self._session.run(None, {self._input_name: blob})
        t2 = time.perf_counter()

        # Postprocess timing (actual NMS done by caller)
        t3 = time.perf_counter()

        metrics = InferenceMetrics(
            preprocess_ms=(t1 - t0) * 1000,
            inference_ms=(t2 - t1) * 1000,
            postprocess_ms=(t3 - t2) * 1000,
            total_ms=(t3 - t0) * 1000,
            fps=1000.0 / max((t3 - t0) * 1000, 0.1),
        )
        self._metrics_history.append(metrics)
        return outputs, metrics

    @property
    def avg_fps(self) -> float:
        if not self._metrics_history:
            return 0.0
        return sum(m.fps for m in self._metrics_history) / len(self._metrics_history)

    @property
    def avg_inference_ms(self) -> float:
        if not self._metrics_history:
            return 0.0
        return sum(m.inference_ms for m in self._metrics_history) / len(self._metrics_history)
```

## 5. Jetson Deployment Checklist

### Hardware Requirements

| Device | GPU | RAM | Storage | Target FPS |
|:---|:---|:---|:---|:---|
| Jetson Orin Nano 8GB | 1024 CUDA cores | 8 GB | 64 GB NVMe | 20-30 |
| Jetson AGX Orin 32GB | 2048 CUDA cores | 32 GB | 64 GB NVMe | 40-60 |

### Software Stack

```
JetPack 6.0 (L4T r36.x)
├── CUDA 12.2
├── cuDNN 8.9
├── TensorRT 8.6
├── OpenCV 4.8 (with CUDA)
├── Python 3.10
├── PyTorch 2.1 (Jetson wheel)
└── ROS 2 Humble
```

### Container for Jetson

```dockerfile
# Dockerfile.jetson
FROM nvcr.io/nvidia/l4t-pytorch:r36.2.0-pth2.1-py3

# ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-ros-base python3-colcon-common-extensions

# Perception deps
RUN pip3 install ultralytics onnxruntime-gpu opencv-python-headless

COPY perception/ /workspace/perception/
COPY models/ /workspace/models/

ENV PYTHONPATH=/workspace/perception:$PYTHONPATH
CMD ["python3", "-m", "perception_core.detector"]
```

### Deployment Script

```bash
#!/bin/bash
# deploy_model.sh — push a model to the Jetson fleet

MODEL=$1  # e.g., models/yolov8s_sar.onnx
DRONES="drone1.local drone2.local drone3.local"

for host in $DRONES; do
    echo "Deploying $MODEL to $host..."
    scp "$MODEL" "jetson@${host}:/workspace/models/"
    ssh "jetson@${host}" "systemctl restart perception"
done
```

## 6. Performance Benchmarks

Run benchmarks on the target device:

```bash
# On Jetson
python3 -c "
from perception_core.edge_runtime import EdgeRuntime
import numpy as np
rt = EdgeRuntime('models/yolov8s_sar.onnx', backend='onnx')
img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
for _ in range(100):  # warmup
    rt.infer(img)
for _ in range(500):
    _, m = rt.infer(img)
print(f'Avg FPS: {rt.avg_fps:.1f}')
print(f'Avg inference: {rt.avg_inference_ms:.1f} ms')
"
```

### Target Performance

| Model | Precision | Jetson Orin Nano | Jetson AGX Orin |
|:---|:---|:---|:---|
| YOLOv8n | FP16 | 35 FPS | 60 FPS |
| YOLOv8s | FP16 | 22 FPS | 45 FPS |
| YOLOv8s | INT8 | 30 FPS | 55 FPS |
| RT-DETR-l | FP16 | 8 FPS | 20 FPS |

## 7. Acceptance Criteria

- [ ] ONNX export passes validation (checker + inference test)
- [ ] TensorRT FP16 engine builds on Jetson without errors
- [ ] INT8 calibration dataset generated (500+ images)
- [ ] Edge runtime achieves > 20 FPS on YOLOv8s FP16 (Orin Nano)
- [ ] mAP drop from PyTorch to ONNX < 0.5%
- [ ] mAP drop from FP32 to INT8 < 2.0%
- [ ] Inference metrics are logged and available via ROS 2 topic
