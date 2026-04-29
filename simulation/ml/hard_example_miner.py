"""
Hard-example miner.

Reads inference logs and surfaces the records most worth re-labelling.
Two heuristics:

* **uncertain**: at least one detection with confidence in
  ``[low_conf, high_conf]`` (the model is on the fence — operator
  decision improves the next training epoch).
* **missed**: zero detections in a frame whose metadata says a target
  *should* have been visible (operator-supplied
  ``metadata["expected_targets"]`` count, or ground-truth COCO
  annotation count from `coco_annotator`).

Both heuristics return :class:`HardExample` records the
labelling-pipeline (CVAT/Label Studio integration, deferred) consumes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional

from ml.inference_logger import InferenceRecord


@dataclass(frozen=True)
class HardExample:
    """One inference outcome flagged for re-labelling."""
    frame_id: int
    image_path: Optional[str]
    reason: str                         # "uncertain" | "missed"
    score: float                        # higher = needs labelling more

    def to_dict(self) -> dict:
        return {
            "frame_id": int(self.frame_id),
            "image_path": self.image_path,
            "reason": self.reason,
            "score": float(self.score),
        }


@dataclass
class MinerConfig:
    """Tunable thresholds for the two heuristics."""
    low_conf: float = 0.30
    high_conf: float = 0.60
    expected_targets_key: str = "expected_targets"


def find_uncertain(records: Iterable[InferenceRecord],
                   cfg: Optional[MinerConfig] = None) -> List[HardExample]:
    """Records with at least one detection in the uncertainty band."""
    c = cfg or MinerConfig()
    out: List[HardExample] = []
    for r in records:
        scores = [d.confidence for d in r.detections
                  if c.low_conf <= d.confidence <= c.high_conf]
        if not scores:
            continue
        # Score = "most uncertain" detection's distance from the centre
        # of the band, inverted so close-to-50% scores rank highest.
        mid = (c.low_conf + c.high_conf) / 2.0
        worst = min(abs(s - mid) for s in scores)
        score = 1.0 - worst / (c.high_conf - c.low_conf)
        out.append(HardExample(
            frame_id=r.frame_id, image_path=r.image_path,
            reason="uncertain", score=float(score),
        ))
    return out


def find_missed(records: Iterable[InferenceRecord],
                cfg: Optional[MinerConfig] = None) -> List[HardExample]:
    """Records where the model returned zero detections but at least
    one target was expected (per the operator-supplied metadata)."""
    c = cfg or MinerConfig()
    out: List[HardExample] = []
    for r in records:
        if r.detections:
            continue
        expected = int(r.metadata.get(c.expected_targets_key, 0))
        if expected <= 0:
            continue
        out.append(HardExample(
            frame_id=r.frame_id, image_path=r.image_path,
            reason="missed", score=float(expected),
        ))
    return out


def mine(records: Iterable[InferenceRecord],
         cfg: Optional[MinerConfig] = None) -> List[HardExample]:
    """Both heuristics combined, sorted by descending score."""
    materialised = list(records)
    examples = find_uncertain(materialised, cfg) + find_missed(materialised, cfg)
    return sorted(examples, key=lambda e: e.score, reverse=True)
