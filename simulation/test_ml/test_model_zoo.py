"""Tests for the ModelZoo API."""

import json

import numpy as np
import pytest

from ml.model_zoo import (
    Detection, Detector, MockDetector, ModelZoo, registered_backends,
)


class TestDetectionDataclass:
    def test_to_dict_round_trips(self):
        d = Detection(category_id=1, category_name="person",
                      bbox=(10.0, 20.0, 30.0, 40.0), confidence=0.9)
        out = d.to_dict()
        assert out == {
            "category_id": 1,
            "category_name": "person",
            "bbox": [10.0, 20.0, 30.0, 40.0],
            "confidence": 0.9,
        }
        # JSON-serialisable.
        json.dumps(out)


class TestMockDetector:
    def test_detect_returns_canned_results(self):
        det = MockDetector()
        out = det.detect(np.zeros((100, 100, 3), dtype=np.uint8))
        assert isinstance(out, list)
        assert all(isinstance(d, Detection) for d in out)
        assert len(out) == 2

    def test_custom_fixed_list_overrides_defaults(self):
        canned = [Detection(99, "x", (0.0, 0.0, 1.0, 1.0), 0.5)]
        det = MockDetector(fixed=canned)
        assert det.detect(np.zeros((10, 10, 3), dtype=np.uint8)) == canned

    def test_backend_name(self):
        assert MockDetector().name() == "mock"


class TestModelZoo:
    def test_lists_known_backends(self):
        zoo = ModelZoo()
        names = zoo.list()
        # Mock is always available; YOLO/RT-DETR/DETR/FCOS/ONNX/TensorRT
        # are registered as deferred stubs.
        for n in ("mock", "yolo", "rtdetr", "detr",
                  "fcos", "onnx", "tensorrt"):
            assert n in names

    def test_load_mock_returns_detector(self):
        det = ModelZoo().load("mock")
        assert isinstance(det, Detector)
        assert det.detect(np.zeros((1, 1, 3), dtype=np.uint8))

    def test_load_unknown_backend_raises(self):
        with pytest.raises(ValueError, match="unknown backend"):
            ModelZoo().load("does_not_exist")

    @pytest.mark.parametrize(
        "backend,dep",
        [("yolo", "ultralytics"), ("rtdetr", "ultralytics"),
         ("detr", "transformers"), ("fcos", "torchvision"),
         ("onnx", "onnxruntime"), ("tensorrt", "tensorrt")],
    )
    def test_deferred_backends_raise_clear_error(self, backend, dep):
        with pytest.raises(NotImplementedError) as exc:
            ModelZoo().load(backend)
        msg = str(exc.value)
        assert dep in msg
        assert "nightly_lane" in msg

    def test_register_custom_backend(self):
        from ml.model_zoo import register

        class _CountingDetector(Detector):
            backend = "counting"

            def __init__(self) -> None:
                self.calls = 0

            def detect(self, image):
                self.calls += 1
                return []

        register("counting_test", lambda weights_path=None, **kw: _CountingDetector())
        try:
            det = ModelZoo().load("counting_test")
            assert det.name() == "counting"
            det.detect(np.zeros((1, 1, 3), dtype=np.uint8))
            assert det.calls == 1
        finally:
            from ml.model_zoo import _BACKENDS
            _BACKENDS.pop("counting_test", None)

    def test_mock_with_empty_fixed_list(self):
        det = MockDetector(fixed=[])
        out = det.detect(np.zeros((10, 10, 3), dtype=np.uint8))
        assert out == []

    def test_registered_backends_sorted(self):
        names = registered_backends()
        assert names == sorted(names)

    def test_load_passes_kwargs_to_loader(self):
        from ml.model_zoo import register, _BACKENDS

        captured: dict = {}

        def loader(weights_path=None, **kw):
            captured["weights_path"] = weights_path
            captured["extra"] = kw
            return MockDetector()

        register("kw_capture", loader)
        try:
            ModelZoo().load("kw_capture", weights_path="x.pt", confidence=0.5)
            assert captured["weights_path"] == "x.pt"
            assert captured["extra"] == {"confidence": 0.5}
        finally:
            _BACKENDS.pop("kw_capture", None)
