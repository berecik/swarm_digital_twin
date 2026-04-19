# ML Model Zoo — Unified Inference API

Detailed instructions for integrating multiple detection/classification
architectures behind a single API.

Part of the [ML Vision Overview](ml_vision_overview.md).

---

## 1. Goal

Replace the hard-coded `YOLO("yolov8n.pt")` call in `detector.py` with a
`ModelZoo` that can load any supported architecture and expose a uniform
`detect(image) → List[Detection]` interface.

## 2. Supported Architectures

| Architecture | Framework | Task | Key Advantage |
|:---|:---|:---|:---|
| **YOLOv8/v11** | ultralytics | Detection | Fast, battle-tested, easy fine-tune |
| **RT-DETR** | ultralytics | Detection | Transformer-based, no NMS, high accuracy |
| **DETR** | transformers | Detection | End-to-end, set prediction (no anchors) |
| **FCOS** | torchvision | Detection | Anchor-free, per-pixel prediction |
| **ViT** | transformers | Classification | Scene-level understanding |
| **ONNX Runtime** | onnxruntime | Any | Cross-platform, optimised inference |
| **TensorRT** | tensorrt | Any | NVIDIA GPU maximum throughput |

## 3. Detection Data Model

```python
# perception/perception_core/model_zoo.py

@dataclass
class Detection:
    """One detected object in an image."""
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: np.ndarray   # [x1, y1, x2, y2] pixels
    mask: Optional[np.ndarray] = None  # instance segmentation

    @property
    def center(self) -> tuple:
        x1, y1, x2, y2 = self.bbox_xyxy
        return ((x1 + x2) / 2, (y1 + y2) / 2)
```

## 4. Unified Inference API

```python
class ModelZoo:
    """Load and run any supported model behind a uniform API."""

    def __init__(self, model_name: str, backend: str = "yolo",
                 device: str = "cuda:0", conf_threshold: float = 0.5,
                 classes: Optional[List[int]] = None):
        ...

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run inference on a BGR numpy image."""
        ...
```

**Backend dispatch** uses a method lookup table (not dynamic code):

```python
_LOADERS = {
    "yolo": "_load_yolo",
    "rtdetr": "_load_rtdetr",
    "detr": "_load_detr",
    "fcos": "_load_fcos",
    "onnx": "_load_onnx",
}
```

Each loader initialises the model; each detector method converts output
to `List[Detection]`. See `ml_vision_overview.md` for the full
implementation.

## 5. Per-Backend Details

### YOLOv8/v11 (ultralytics)

```python
from ultralytics import YOLO
model = YOLO("yolov8s.pt")
results = model(image, conf=0.5, classes=[0])
# results[0].boxes → xyxy, conf, cls
```

- **Fine-tune:** `model.train(data="configs/sar.yaml", epochs=100)`
- **Export:** `model.export(format="onnx", simplify=True)`

### RT-DETR (ultralytics)

```python
from ultralytics import RTDETR
model = RTDETR("rtdetr-l.pt")
results = model(image, conf=0.5)
```

- Transformer-based, no NMS post-processing needed
- Higher accuracy than YOLO for overlapping objects

### DETR (transformers)

```python
from transformers import DetrForObjectDetection, DetrImageProcessor
processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")
inputs = processor(images=pil_img, return_tensors="pt")
outputs = model(**inputs)
results = processor.post_process_object_detection(outputs, threshold=0.5)
```

### FCOS (torchvision)

```python
import torchvision
model = torchvision.models.detection.fcos_resnet50_fpn(weights="DEFAULT")
tensor = torchvision.transforms.functional.to_tensor(image)
outputs = model([tensor])[0]  # boxes, scores, labels
```

- Anchor-free: predicts directly per pixel
- Good for dense scenes with many small objects

### ONNX Runtime

```python
import onnxruntime as ort
session = ort.InferenceSession("model.onnx",
    providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
outputs = session.run(None, {input_name: preprocessed_image})
```

## 6. Integration with detector.py

Replace current code:

```python
# Before:
self.model = YOLO("yolov8n.pt")
results = self.model(cv_image, conf=0.5, classes=[0])

# After:
from perception_core.model_zoo import ModelZoo
self.model = ModelZoo(
    model_name=self.get_parameter("model_name").value,
    backend=self.get_parameter("model_backend").value,
    conf_threshold=self.get_parameter("confidence").value,
)
detections = self.model.detect(cv_image)
```

ROS 2 parameters make the model swappable at launch time.

## 7. Acceptance Criteria

- [ ] `ModelZoo` loads YOLO, RT-DETR, and ONNX backends
- [ ] `detect()` returns `List[Detection]` for all backends
- [ ] `detector.py` uses `ModelZoo` instead of direct YOLO import
- [ ] Model is configurable via ROS 2 parameters
- [ ] Unit tests cover each backend's load + detect path
