"""
ModelZoo unified detection API.

Replaces hard-coded `YOLO('yolov8n.pt')` calls with a registry-backed
`Detector` interface. Real backends (YOLO, RT-DETR, DETR, FCOS,
ONNX Runtime, TensorRT) get registered once their heavy dependencies
land; the `MockDetector` ships today so CI can verify the API
without pulling in PyTorch.

Typical caller::

    zoo = ModelZoo()
    detector = zoo.load("mock", weights_path="…")
    detections = detector.detect(image_array)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class Detection:
    """One detection result. Bounding box is COCO xywh in pixels."""
    category_id: int
    category_name: str
    bbox: tuple                  # (x, y, w, h)
    confidence: float

    def to_dict(self) -> dict:
        return {
            "category_id": int(self.category_id),
            "category_name": self.category_name,
            "bbox": [float(v) for v in self.bbox],
            "confidence": float(self.confidence),
        }


# ── Detector ABC ─────────────────────────────────────────────────────────────


class Detector(ABC):
    """Backend interface every concrete detector implements."""

    backend: str = "abstract"

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run inference on a single HxWxC image (uint8, RGB)."""

    def name(self) -> str:
        return self.backend


# ── Mock backend (always available — no extra deps) ──────────────────────────


class MockDetector(Detector):
    """Returns a fixed list of detections regardless of input.

    Used by tests and by the CI smoke runners that exercise the
    ModelZoo API without paying for a real model. The fixed list is
    chosen so the tests can assert the round-trip semantics (category
    name lookup, confidence threshold, JSON serialisation).
    """

    backend = "mock"

    def __init__(self, fixed: Optional[List[Detection]] = None) -> None:
        from ml.sar_targets import SAR_TARGETS
        if fixed is not None:
            self._fixed = list(fixed)
        else:
            self._fixed = [
                Detection(
                    category_id=SAR_TARGETS["person"].category_id,
                    category_name="person",
                    bbox=(50.0, 50.0, 30.0, 60.0),
                    confidence=0.92,
                ),
                Detection(
                    category_id=SAR_TARGETS["casualty"].category_id,
                    category_name="casualty",
                    bbox=(200.0, 150.0, 90.0, 35.0),
                    confidence=0.74,
                ),
            ]

    def detect(self, image: np.ndarray) -> List[Detection]:
        return list(self._fixed)


# ── Registry ─────────────────────────────────────────────────────────────────


# Each registered backend is `(loader_fn, requires_dep)`. `requires_dep`
# is the import name we probe before calling the loader so the error
# message is helpful when the optional dep is missing.
_BACKENDS: Dict[str, tuple] = {}


def register(name: str,
             loader: Callable[..., Detector],
             requires: Optional[str] = None) -> None:
    """Register a backend. Loader signature: ``(weights_path, **kwargs)``."""
    _BACKENDS[name] = (loader, requires)


def registered_backends() -> List[str]:
    return sorted(_BACKENDS)


# Mock is always available.
register("mock", lambda weights_path=None, **kw: MockDetector(), requires=None)


# Stub registration for the deferred backends. They raise
# `NotImplementedError` with a clear pointer when called, so the
# ModelZoo API surface is complete even though the heavy deps are not
# yet installed in CI.
def _stub(backend: str, dep: str) -> Callable:
    def _loader(weights_path=None, **kw):
        raise NotImplementedError(
            f"{backend} backend requires the optional `{dep}` dependency "
            f"and is part of the deferred ML branch — see "
            f"`docs/nightly_lane.md` for the contract"
        )
    return _loader


for backend, dep in (
    ("yolo", "ultralytics"),
    ("rtdetr", "ultralytics"),
    ("detr", "transformers"),
    ("fcos", "torchvision"),
    ("onnx", "onnxruntime"),
    ("tensorrt", "tensorrt"),
):
    register(backend, _stub(backend, dep), requires=dep)


# ── Zoo facade ───────────────────────────────────────────────────────────────


class ModelZoo:
    """Caller-facing facade over the backend registry."""

    def list(self) -> List[str]:
        return registered_backends()

    def load(self, backend: str,
             weights_path: Optional[str] = None,
             **kwargs) -> Detector:
        """Instantiate a detector. Raises ValueError on unknown backend."""
        if backend not in _BACKENDS:
            raise ValueError(
                f"unknown backend '{backend}'; "
                f"available: {registered_backends()}"
            )
        loader, _ = _BACKENDS[backend]
        return loader(weights_path=weights_path, **kwargs)
