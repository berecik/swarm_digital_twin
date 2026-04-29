"""
Model registry.

Tracks the lineage of trained models in a flat JSON file:

    {
      "models": [
        {
          "version": "yolov8s_sar_v1",
          "backend": "yolo",
          "weights_path": "models/yolov8s_sar_v1.pt",
          "trained_at_s": 1745251200.0,
          "parent_version": null,
          "kpis": {"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
          "notes": "First fine-tune on synthetic SAR data."
        },
        ...
      ]
    }

Promotion logic (a new model only becomes the active one if its KPIs
beat the current best) lives in `ml.kpi.compare_models`.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class ModelEntry:
    version: str
    backend: str
    weights_path: str
    trained_at_s: float = field(default_factory=time.time)
    parent_version: Optional[str] = None
    kpis: Dict[str, float] = field(default_factory=dict)
    notes: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


class ModelRegistry:
    """Append-only JSON registry of trained models."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._entries: List[ModelEntry] = []
        if self.path.is_file() and self.path.stat().st_size > 0:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            for d in data.get("models", []):
                self._entries.append(ModelEntry(**d))

    # ── Mutation ──

    def register(self, entry: ModelEntry) -> ModelEntry:
        if any(e.version == entry.version for e in self._entries):
            raise ValueError(f"version '{entry.version}' already registered")
        self._entries.append(entry)
        self._flush()
        return entry

    # ── Lookup ──

    def all(self) -> List[ModelEntry]:
        return list(self._entries)

    def get(self, version: str) -> ModelEntry:
        for e in self._entries:
            if e.version == version:
                return e
        raise KeyError(f"no model with version '{version}'")

    def latest(self) -> Optional[ModelEntry]:
        return self._entries[-1] if self._entries else None

    def best_by_kpi(self, key: str) -> Optional[ModelEntry]:
        """Return the entry with the highest value of *key* in its KPIs."""
        candidates = [e for e in self._entries if key in e.kpis]
        if not candidates:
            return None
        return max(candidates, key=lambda e: e.kpis[key])

    def lineage(self, version: str) -> List[ModelEntry]:
        """Walk parent_version pointers from *version* up to the root.

        Defensive against cycles introduced by hand-editing the JSON
        file: registers visited versions and raises ValueError on
        repeat. Cycles can't form via `register()` itself (parents
        must exist when the child registers, and `register` rejects
        duplicates) but the on-disk file is open to manual edits.
        """
        out = []
        seen = set()
        cursor: Optional[str] = version
        while cursor is not None:
            if cursor in seen:
                raise ValueError(
                    f"cycle detected in lineage: {cursor!r} reached twice"
                )
            seen.add(cursor)
            entry = self.get(cursor)
            out.append(entry)
            cursor = entry.parent_version
        return out

    # ── Persistence ──

    def _flush(self) -> None:
        data = {"models": [e.to_dict() for e in self._entries]}
        self.path.write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf-8")
