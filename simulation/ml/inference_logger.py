"""
Inference logger.

Captures one inference outcome per call into a JSONL file with frame
metadata, detection list, and a wall-clock timestamp. Designed so the
hard-example miner and the model-registry promotion gate consume the
same log without re-running the model.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator, List, Optional

from ml.model_zoo import Detection


@dataclass
class InferenceRecord:
    """One captured inference."""
    frame_id: int
    t_wall_s: float
    model_version: str
    image_path: Optional[str] = None
    metadata: dict = field(default_factory=dict)
    detections: List[Detection] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "frame_id": int(self.frame_id),
            "t_wall_s": float(self.t_wall_s),
            "model_version": self.model_version,
            "image_path": self.image_path,
            "metadata": dict(self.metadata),
            "detections": [d.to_dict() for d in self.detections],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "InferenceRecord":
        return cls(
            frame_id=int(d["frame_id"]),
            t_wall_s=float(d["t_wall_s"]),
            model_version=d.get("model_version", "unknown"),
            image_path=d.get("image_path"),
            metadata=dict(d.get("metadata") or {}),
            detections=[
                Detection(
                    category_id=int(x["category_id"]),
                    category_name=x["category_name"],
                    bbox=tuple(x["bbox"]),
                    confidence=float(x["confidence"]),
                )
                for x in d.get("detections") or []
            ],
        )


class InferenceLogger:
    """JSONL writer for `InferenceRecord` entries."""

    def __init__(self, path: Path, model_version: str = "unknown") -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.model_version = model_version
        self._fh = open(self.path, "a", encoding="utf-8")
        self._next_frame = self._tail_max_frame_id() + 1

    def _tail_max_frame_id(self) -> int:
        if not self.path.exists() or self.path.stat().st_size == 0:
            return -1
        max_id = -1
        with open(self.path, "r", encoding="utf-8") as fh:
            for line in fh:
                try:
                    fid = int(json.loads(line).get("frame_id", -1))
                except json.JSONDecodeError:
                    continue
                if fid > max_id:
                    max_id = fid
        return max_id

    def log(self, detections: List[Detection], *,
            image_path: Optional[str] = None,
            metadata: Optional[dict] = None,
            t_wall_s: Optional[float] = None) -> InferenceRecord:
        rec = InferenceRecord(
            frame_id=self._next_frame,
            t_wall_s=t_wall_s if t_wall_s is not None else time.time(),
            model_version=self.model_version,
            image_path=image_path,
            metadata=metadata or {},
            detections=list(detections),
        )
        self._fh.write(json.dumps(rec.to_dict()) + "\n")
        self._fh.flush()
        self._next_frame += 1
        return rec

    def close(self) -> None:
        self._fh.close()

    def __enter__(self) -> "InferenceLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def read_log(path: Path) -> Iterator[InferenceRecord]:
    """Stream every record from a log file."""
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            yield InferenceRecord.from_dict(json.loads(line))


def replay(records: Iterable[InferenceRecord]) -> List[InferenceRecord]:
    """Materialise a record stream as a list."""
    return list(records)
