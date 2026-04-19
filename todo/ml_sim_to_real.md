# Sim-to-Real Transfer — Synthetic Data from Gazebo

Detailed instructions for generating labeled training data from
Gazebo simulation and bridging the domain gap to real-world imagery.

Part of the [ML Vision Overview](ml_vision_overview.md).

---

## 1. Goal

Generate thousands of labeled images from Gazebo SITL runs, augment
them to reduce the domain gap, and combine with a small set of real
labeled images for robust detection in the field.

## 2. Synthetic Data Generation

### Camera Sensor in Gazebo

Add an RGB-D camera to the drone model SDF:

```xml
<!-- gazebo/models/x500/model.sdf — camera sensor -->
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 deg -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip><near>0.1</near><far>100</far></clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros><namespace>/drone_1</namespace></ros>
    <camera_name>camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>

<sensor name="depth_camera" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image><width>640</width><height>480</height></image>
    <clip><near>0.1</near><far>100</far></clip>
  </camera>
  <plugin name="depth_controller" filename="libgazebo_ros_camera.so">
    <ros><namespace>/drone_1</namespace></ros>
    <camera_name>depth</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

### World with Target Objects

Add human models (walking, lying, standing) to the Gazebo world:

```xml
<!-- gazebo/worlds/sar_training.world -->
<include>
  <uri>model://person_standing</uri>
  <pose>10 5 0 0 0 0.5</pose>
</include>
<include>
  <uri>model://person_lying</uri>
  <pose>-5 15 0 0 0 1.2</pose>
</include>
<!-- Add 10-50 targets at varied positions -->
```

### Automated Capture Script

```python
# perception/data/generate_synthetic.py
"""Capture labeled images from Gazebo simulation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
from pathlib import Path


class SyntheticDataCapture(Node):
    def __init__(self):
        super().__init__("synthetic_capture")
        self.bridge = CvBridge()
        self.frame_count = 0
        self.output_dir = Path("data/synthetic/images/train")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.annotations = {"images": [], "annotations": [], "categories": [
            {"id": 1, "name": "person"},
            {"id": 2, "name": "vehicle"},
        ]}
        self.ann_id = 0

        self.sub = self.create_subscription(
            Image, "/drone_1/camera/image_raw", self.callback, 10)

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        filename = f"gazebo_{self.frame_count:06d}.jpg"
        cv2.imwrite(str(self.output_dir / filename), cv_img)

        # Get ground-truth bounding boxes from Gazebo model poses
        # (via /gazebo/model_states topic or Gazebo transport API)
        bboxes = self._project_models_to_image(cv_img.shape)

        img_id = self.frame_count
        self.annotations["images"].append({
            "id": img_id,
            "file_name": filename,
            "width": cv_img.shape[1],
            "height": cv_img.shape[0],
        })
        for cls_id, bbox in bboxes:
            self.ann_id += 1
            x, y, w, h = bbox
            self.annotations["annotations"].append({
                "id": self.ann_id,
                "image_id": img_id,
                "category_id": cls_id,
                "bbox": [x, y, w, h],
                "area": w * h,
                "iscrowd": 0,
            })

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f"Captured {self.frame_count} frames")

    def save_annotations(self):
        ann_path = Path("data/synthetic/annotations/train.json")
        ann_path.parent.mkdir(parents=True, exist_ok=True)
        with open(ann_path, "w") as f:
            json.dump(self.annotations, f, indent=2)

    def _project_models_to_image(self, img_shape):
        """Project 3D model positions to 2D bounding boxes.

        Uses the camera intrinsics and drone pose to compute the
        pixel coordinates of known target positions.
        """
        # TODO: implement using Gazebo transport API
        return []  # [(class_id, [x, y, w, h]), ...]
```

### Data Generation Workflow

```bash
# 1. Launch Gazebo with SAR training world
ros2 launch gazebo sar_training.launch.py

# 2. Fly a lawnmower pattern to capture varied viewpoints
ros2 run perception_core search_planner

# 3. Simultaneously capture images
ros2 run perception_core synthetic_capture

# 4. Generate annotations
python perception/data/generate_synthetic.py --save
```

## 3. Domain Randomisation

To bridge the sim-to-real gap, apply heavy augmentation during training:

```python
# perception/training/augmentations.py
import albumentations as A

sim_to_real_augmentation = A.Compose([
    # Lighting variation
    A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.8),
    A.RandomGamma(gamma_limit=(60, 150), p=0.5),
    A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=40,
                         val_shift_limit=30, p=0.7),

    # Weather simulation
    A.RandomFog(fog_coef_lower=0.1, fog_coef_upper=0.4, p=0.2),
    A.RandomRain(brightness_coefficient=0.8, p=0.15),
    A.RandomSunFlare(flare_roi=(0, 0, 1, 0.5), p=0.1),

    # Camera effects
    A.MotionBlur(blur_limit=7, p=0.3),
    A.GaussNoise(var_limit=(10, 50), p=0.4),
    A.ImageCompression(quality_lower=60, quality_upper=95, p=0.3),

    # Geometric (simulate different altitudes/angles)
    A.Affine(scale=(0.7, 1.3), rotate=(-15, 15), p=0.5),
    A.Perspective(scale=(0.02, 0.08), p=0.3),

    # Cutout (simulate occlusion)
    A.CoarseDropout(max_holes=5, max_height=50, max_width=50, p=0.3),
], bbox_params=A.BboxParams(format="coco", label_fields=["class_labels"]))
```

## 4. Mixed Dataset Strategy

| Source | Volume | Purpose |
|:---|:---|:---|
| Gazebo synthetic | 5000-10000 images | Bulk training, varied viewpoints |
| Real field data | 200-500 images | Domain anchor, real textures |
| Public datasets (VisDrone, DOTA) | 1000-2000 images | Additional aerial views |

**Split:** 80% train / 10% val / 10% test, stratified by source.

## 5. Validation Protocol

1. Train on synthetic-only → measure mAP on real val set (baseline)
2. Train on synthetic + augmentation → measure improvement
3. Train on synthetic + real → measure final accuracy
4. Compare all three to quantify domain gap

### Expected Results

| Training Data | mAP@50 (real val) |
|:---|:---|
| Synthetic only | 0.40-0.55 |
| Synthetic + augmentation | 0.55-0.65 |
| Synthetic + real (mixed) | 0.70-0.85 |

## 6. Acceptance Criteria

- [ ] Gazebo SAR training world with 20+ target models
- [ ] Camera sensor captures RGB + Depth at 30 FPS
- [ ] Automated annotation pipeline produces COCO-format JSON
- [ ] Domain randomisation augmentations configured
- [ ] Mixed dataset achieves mAP@50 > 0.70 on real validation set
- [ ] Data generation workflow is reproducible (fixed seeds, versioned assets)
