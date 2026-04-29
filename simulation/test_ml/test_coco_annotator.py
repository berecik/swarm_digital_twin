"""Tests for the COCO annotator."""

import json
from pathlib import Path

import numpy as np
import pytest

from ml.coco_annotator import (
    CameraIntrinsics, FrameCapture, Pose, TargetInstance, annotate, write,
)
from ml.sar_targets import find


class TestCocoAnnotator:
    def _camera(self):
        return CameraIntrinsics(
            fx=400.0, fy=400.0, cx=320.0, cy=240.0,
            width=640, height=480,
        )

    def _frame_with_targets(self):
        return FrameCapture(
            image_id=1,
            file_name="frame_0001.png",
            drone_pose=Pose(position=np.array([0.0, 0.0, 30.0])),
            targets=[
                TargetInstance(target=find("person"),
                               pose=Pose(position=np.array([0.0, 0.0, 0.0]))),
                TargetInstance(target=find("car"),
                               pose=Pose(position=np.array([5.0, 0.0, 0.0]))),
            ],
        )

    def test_dataset_shape(self):
        ds = annotate([self._frame_with_targets()], self._camera())
        assert {"info", "images", "annotations", "categories"} <= ds.keys()
        assert ds["images"][0]["file_name"] == "frame_0001.png"
        assert ds["images"][0]["width"] == 640

    def test_visible_targets_get_annotations(self):
        ds = annotate([self._frame_with_targets()], self._camera())
        # Both targets are directly under the drone → visible.
        assert len(ds["annotations"]) == 2
        cat_ids = {a["category_id"] for a in ds["annotations"]}
        assert cat_ids == {find("person").category_id, find("car").category_id}
        for ann in ds["annotations"]:
            assert ann["area"] > 0
            assert ann["iscrowd"] == 0

    def test_target_below_camera_skipped(self):
        # Camera below target → projection rejects.
        frame = FrameCapture(
            image_id=2, file_name="bad.png",
            drone_pose=Pose(position=np.array([0.0, 0.0, -5.0])),
            targets=[TargetInstance(target=find("person"),
                                    pose=Pose(position=np.array([0.0, 0.0, 0.0])))],
        )
        ds = annotate([frame], self._camera())
        assert ds["annotations"] == []

    def test_target_outside_image_skipped(self):
        # Place target very far east — projection lands outside image.
        frame = FrameCapture(
            image_id=3, file_name="far.png",
            drone_pose=Pose(position=np.array([0.0, 0.0, 30.0])),
            targets=[TargetInstance(target=find("person"),
                                    pose=Pose(position=np.array([1000.0, 0.0, 0.0])))],
        )
        ds = annotate([frame], self._camera())
        assert ds["annotations"] == []

    def test_write_round_trip(self, tmp_path):
        ds = annotate([self._frame_with_targets()], self._camera())
        path = write(ds, tmp_path / "out.json")
        loaded = json.loads(path.read_text(encoding="utf-8"))
        assert loaded["images"] == ds["images"]
        assert loaded["annotations"] == ds["annotations"]

    def test_write_creates_parent_dirs(self, tmp_path):
        ds = annotate([self._frame_with_targets()], self._camera())
        path = write(ds, tmp_path / "subdir" / "nested" / "out.json")
        assert path.exists()

    def test_multiple_frames_get_unique_annotation_ids(self):
        frames = [
            self._frame_with_targets(),
            FrameCapture(
                image_id=2, file_name="frame_0002.png",
                drone_pose=Pose(position=np.array([0.0, 0.0, 30.0])),
                targets=[TargetInstance(
                    target=find("person"),
                    pose=Pose(position=np.array([0.0, 0.0, 0.0])))],
            ),
        ]
        ds = annotate(frames, self._camera())
        ids = [a["id"] for a in ds["annotations"]]
        assert ids == list(range(1, len(ids) + 1))
        assert len(set(ids)) == len(ids)

    def test_empty_targets_list(self):
        frame = FrameCapture(
            image_id=42, file_name="empty.png",
            drone_pose=Pose(position=np.array([0.0, 0.0, 30.0])),
            targets=[],
        )
        ds = annotate([frame], self._camera())
        assert ds["images"][0]["id"] == 42
        assert ds["annotations"] == []

    def test_partial_bbox_clipped_to_image_bounds(self):
        # Place a large target near the image edge so its raw bbox would
        # spill beyond width/height; the clip should keep it positive
        # and entirely inside the frame.
        cam = self._camera()
        # Project: u = cx + fx*dx/dz; pick dx so u sits 1px inside the
        # right edge — the half-bbox-width then spills over.
        dz = 30.0
        target_pose_x = (cam.width - 1 - cam.cx) * dz / cam.fx
        frame = FrameCapture(
            image_id=99, file_name="edge.png",
            drone_pose=Pose(position=np.array([0.0, 0.0, dz])),
            targets=[TargetInstance(
                target=find("car"),
                pose=Pose(position=np.array([target_pose_x, 0.0, 0.0])))],
        )
        ds = annotate([frame], cam)
        assert len(ds["annotations"]) == 1
        x, y, w, h = ds["annotations"][0]["bbox"]
        assert 0 <= x <= cam.width
        assert 0 <= y <= cam.height
        assert x + w <= cam.width
        assert y + h <= cam.height
        assert w > 0 and h > 0

    def test_pose_default_yaw_is_zero(self):
        p = Pose(position=np.array([1.0, 2.0, 3.0]))
        assert p.yaw_rad == 0.0
