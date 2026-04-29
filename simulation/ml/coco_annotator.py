"""
COCO annotation builder for synthetic Gazebo SAR captures.

The Gazebo capture pipeline (deferred to the nightly lane) records
(image, drone_pose, [target_pose, …]) tuples. This module turns those
tuples into COCO-format JSON: an `images` block, an `annotations` block
with one entry per visible target, and the `categories` block from
`sar_targets`.

The projection math is a thin pinhole-camera model so it stays unit-
testable in CI without OpenCV. Real-flight captures will replace the
projection with Gazebo's actual camera intrinsics + ground-truth poses.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from ml.sar_targets import SARTarget, coco_categories


# ── Camera + scene primitives ────────────────────────────────────────────────


@dataclass
class CameraIntrinsics:
    """Pinhole-camera intrinsics."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Pose:
    """ENU position + yaw orientation."""
    position: np.ndarray             # shape (3,) — east, north, up [m]
    yaw_rad: float = 0.0


@dataclass
class TargetInstance:
    """One target placed in the world."""
    target: SARTarget
    pose: Pose


@dataclass
class FrameCapture:
    """A single rendered frame plus its ground-truth labels."""
    image_id: int
    file_name: str
    drone_pose: Pose
    targets: List[TargetInstance] = field(default_factory=list)


# ── Projection ───────────────────────────────────────────────────────────────


def _project_target(camera: CameraIntrinsics,
                    drone_pose: Pose,
                    target: TargetInstance) -> Optional[tuple]:
    """Project a target's footprint into image coords.

    Returns ``(x, y, w, h)`` (COCO-style xywh) when the target is in
    front of the camera and projects inside the image, else ``None``.
    The drone is assumed to look straight down (the typical SAR camera
    rig); a single ENU offset is enough for the projection.
    """
    # Camera centre = drone position; image plane is the ENU XY plane.
    dx = target.pose.position[0] - drone_pose.position[0]
    dy = target.pose.position[1] - drone_pose.position[1]
    dz = drone_pose.position[2] - target.pose.position[2]    # height above target
    if dz <= 0:
        return None  # camera below target — out of FOV

    # Pinhole: image = f * world / depth + principal point. Down-looking
    # camera: world east → image x, world north → image y (negated to
    # match standard image coordinates).
    u = camera.cx + camera.fx * dx / dz
    v = camera.cy - camera.fy * dy / dz

    # Footprint scaled by depth (similar triangles).
    w_ground, l_ground, _ = target.target.footprint_m
    w_px = camera.fx * w_ground / dz
    h_px = camera.fy * l_ground / dz

    x = u - w_px / 2.0
    y = v - h_px / 2.0
    if (x + w_px <= 0 or y + h_px <= 0
            or x >= camera.width or y >= camera.height):
        return None  # bbox fully outside the image
    # Clamp to image bounds.
    x_clamped = max(0.0, x)
    y_clamped = max(0.0, y)
    w_clamped = min(camera.width - x_clamped, x + w_px - x_clamped)
    h_clamped = min(camera.height - y_clamped, y + h_px - y_clamped)
    if w_clamped <= 0 or h_clamped <= 0:
        return None
    return (float(x_clamped), float(y_clamped),
            float(w_clamped), float(h_clamped))


# ── Annotator ────────────────────────────────────────────────────────────────


def annotate(frames: Sequence[FrameCapture],
             camera: CameraIntrinsics) -> dict:
    """Turn a list of FrameCaptures into a COCO 2017 dataset dict."""
    images = []
    annotations = []
    next_ann_id = 1
    for frame in frames:
        images.append({
            "id": frame.image_id,
            "file_name": frame.file_name,
            "width": camera.width,
            "height": camera.height,
        })
        for tgt in frame.targets:
            bbox = _project_target(camera, frame.drone_pose, tgt)
            if bbox is None:
                continue
            x, y, w, h = bbox
            annotations.append({
                "id": next_ann_id,
                "image_id": frame.image_id,
                "category_id": tgt.target.category_id,
                "bbox": [x, y, w, h],
                "area": float(w * h),
                "iscrowd": 0,
            })
            next_ann_id += 1
    return {
        "info": {
            "description": "Swarm Digital Twin synthetic SAR dataset",
            "year": 2026,
            "version": "1.0",
        },
        "images": images,
        "annotations": annotations,
        "categories": coco_categories(),
    }


def write(dataset: dict, path: Path) -> Path:
    """Write a COCO dict to disk as JSON."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dataset, indent=2) + "\n", encoding="utf-8")
    return out
