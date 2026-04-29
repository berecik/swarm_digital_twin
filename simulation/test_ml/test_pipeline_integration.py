"""End-to-end ML pipeline integration tests.

Each test wires multiple `simulation/ml/` modules together to verify
they compose correctly: synthetic capture → COCO annotation →
augmentation → mock inference → JSONL log → hard-example mining →
model registration → KPI promotion.

These tests are the closest CI gets to running the deferred PyTorch
training pipeline; the heavy backends are still mocked, but the
data + control flow between modules is exercised exactly as it will
be in production.
"""

from __future__ import annotations

import numpy as np
import pytest
from PIL import Image

from ml.coco_annotator import (
    CameraIntrinsics, FrameCapture, Pose, TargetInstance, annotate, write,
)
from ml.hard_example_miner import mine
from ml.image_augment import Augmenter
from ml.inference_logger import InferenceLogger, read_log
from ml.kpi import compare_models, evaluate_kpis
from ml.model_registry import ModelEntry, ModelRegistry
from ml.model_zoo import Detection, MockDetector, ModelZoo
from ml.sar_targets import find


def _camera() -> CameraIntrinsics:
    return CameraIntrinsics(fx=400.0, fy=400.0,
                            cx=320.0, cy=240.0,
                            width=640, height=480)


def _frame(image_id: int, target_name: str = "person") -> FrameCapture:
    return FrameCapture(
        image_id=image_id,
        file_name=f"frame_{image_id:04d}.png",
        drone_pose=Pose(position=np.array([0.0, 0.0, 30.0])),
        targets=[TargetInstance(target=find(target_name),
                                pose=Pose(position=np.array([0.0, 0.0, 0.0])))],
    )


class TestAnnotateThenWrite:
    def test_annotated_dataset_is_jsonable_on_disk(self, tmp_path):
        ds = annotate([_frame(1), _frame(2)], _camera())
        path = write(ds, tmp_path / "coco" / "train.json")
        assert path.exists()
        # Every annotation references a known image id.
        image_ids = {im["id"] for im in ds["images"]}
        for ann in ds["annotations"]:
            assert ann["image_id"] in image_ids


class TestAugmentDoesNotBreakAnnotations:
    def test_augmented_image_keeps_bbox_within_size(self, tmp_path):
        # Render a fake image at the camera's declared size, augment it,
        # confirm the COCO bbox math still falls inside the augmented
        # image's pixel bounds. This is the contract the training
        # dataloader relies on.
        cam = _camera()
        img = Image.new("RGB", (cam.width, cam.height), (120, 130, 140))
        out_img = Augmenter(seed=0).apply_random_chain(img, p=1.0)
        assert out_img.size == (cam.width, cam.height)
        ds = annotate([_frame(1)], cam)
        for ann in ds["annotations"]:
            x, y, w, h = ann["bbox"]
            assert 0 <= x and 0 <= y
            assert x + w <= cam.width and y + h <= cam.height


class TestInferThenMine:
    def test_mock_inference_log_produces_hard_examples(self, tmp_path):
        # Use a custom MockDetector so each "frame" returns an
        # uncertainty-band detection — the miner should surface them.
        det = MockDetector(fixed=[
            Detection(category_id=1, category_name="person",
                      bbox=(0.0, 0.0, 10.0, 10.0), confidence=0.45),
        ])
        log_path = tmp_path / "infer.jsonl"
        with InferenceLogger(log_path, model_version="mock_v1") as log:
            for i in range(3):
                detections = det.detect(np.zeros((1, 1, 3), dtype=np.uint8))
                log.log(detections, image_path=f"frame_{i}.png")
        records = list(read_log(log_path))
        assert len(records) == 3
        hard = mine(records)
        # Every uncertain frame should surface; default config band is
        # [0.30, 0.60] and 0.45 sits at the centre.
        assert len(hard) == 3
        assert all(e.reason == "uncertain" for e in hard)
        assert all(e.score == pytest.approx(1.0) for e in hard)

    def test_missed_targets_surface_when_metadata_says_so(self, tmp_path):
        log_path = tmp_path / "infer.jsonl"
        with InferenceLogger(log_path, model_version="mock_v1") as log:
            log.log([], image_path="empty1.png",
                    metadata={"expected_targets": 4})
            log.log([], image_path="empty2.png",
                    metadata={"expected_targets": 1})
        hard = mine(read_log(log_path))
        assert {e.reason for e in hard} == {"missed"}
        # Sorted by descending score → 4 first.
        assert hard[0].score == 4.0
        assert hard[1].score == 1.0


class TestRegistryAndKPI:
    def test_promotion_pipeline(self, tmp_path):
        reg = ModelRegistry(tmp_path / "registry.json")
        first = ModelEntry(
            version="v1", backend="mock", weights_path="v1.pt",
            kpis={"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
        )
        reg.register(first)
        # Candidate v2 — passes acceptance and beats v1's mAP by 0.04.
        cand = {"mAP_50": 0.82, "recall": 0.88, "inference_ms": 40.0}
        promote, reason = compare_models(
            cand, reg.best_by_kpi("mAP_50").kpis)
        assert promote is True
        if promote:
            reg.register(ModelEntry(
                version="v2", backend="mock", weights_path="v2.pt",
                parent_version="v1", kpis=cand))
        # Lineage should walk back to v1.
        chain = [e.version for e in reg.lineage("v2")]
        assert chain == ["v2", "v1"]
        # Best by mAP is now v2.
        assert reg.best_by_kpi("mAP_50").version == "v2"

    def test_failed_candidate_does_not_get_registered(self, tmp_path):
        reg = ModelRegistry(tmp_path / "registry.json")
        reg.register(ModelEntry(
            version="v1", backend="mock", weights_path="v1.pt",
            kpis={"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
        ))
        bad = {"mAP_50": 0.40, "recall": 0.50, "inference_ms": 80.0}
        promote, reason = compare_models(bad, reg.latest().kpis)
        assert promote is False
        # Calling code must guard with the verdict before persisting.
        if promote:  # pragma: no cover — we already asserted False
            reg.register(ModelEntry(
                version="v2", backend="mock", weights_path="v2.pt",
                parent_version="v1", kpis=bad))
        assert [e.version for e in reg.all()] == ["v1"]


class TestZooDriveRegistry:
    def test_load_mock_then_log_then_register(self, tmp_path):
        det = ModelZoo().load("mock")
        log_path = tmp_path / "infer.jsonl"
        with InferenceLogger(log_path, model_version="mock_v1") as log:
            log.log(det.detect(np.zeros((10, 10, 3), dtype=np.uint8)),
                    image_path="a.png")
        recs = list(read_log(log_path))
        # Mock detector returns 2 fixed detections.
        assert len(recs[0].detections) == 2
        # Register the model that produced these logs.
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(ModelEntry(
            version="mock_v1", backend=det.name(),
            weights_path="(mock — no weights)",
            kpis={"mAP_50": 0.80, "recall": 0.87, "inference_ms": 8.0},
        ))
        # Promotion gate accepts the entry.
        verdict = evaluate_kpis(reg.latest().kpis)
        assert verdict.verdict == "PASS"
