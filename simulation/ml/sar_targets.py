"""
SAR (Search-And-Rescue) target catalogue for the synthetic data pipeline.

Defines the targets the Gazebo training world will eventually host
(person, vehicle, equipment), with their COCO category IDs and
bounding-box footprints. The catalogue is data-only and pure Python so
the COCO annotator + augmentation pipeline can drive against it without
a Gazebo runtime.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass(frozen=True)
class SARTarget:
    """One detectable target class."""
    category_id: int                     # COCO `category_id` field
    name: str                            # COCO `name` (e.g., "person")
    supercategory: str                   # COCO `supercategory`
    footprint_m: tuple                   # (length_m, width_m, height_m)
    canonical_yaw: float = 0.0           # default orientation in world frame [rad]


# Catalogue. Category IDs match COCO 2017 where applicable, otherwise
# reserve IDs in the 100+ range for SAR-specific classes.
SAR_TARGETS: Dict[str, SARTarget] = {
    # COCO-aligned (matches public dataset for transfer learning)
    "person":       SARTarget(1,  "person",       "person",  (0.5, 0.5, 1.7)),
    "bicycle":      SARTarget(2,  "bicycle",      "vehicle", (1.7, 0.5, 1.0)),
    "car":          SARTarget(3,  "car",          "vehicle", (4.5, 1.8, 1.5)),
    "motorcycle":   SARTarget(4,  "motorcycle",   "vehicle", (2.2, 0.8, 1.2)),
    "bus":          SARTarget(6,  "bus",          "vehicle", (12.0, 2.5, 3.2)),
    "truck":        SARTarget(8,  "truck",        "vehicle", (6.5, 2.4, 2.8)),
    "boat":         SARTarget(9,  "boat",         "vehicle", (5.0, 2.0, 1.8)),

    # SAR-specific (reserved IDs starting at 100)
    "casualty":     SARTarget(100, "casualty",    "person",     (1.8, 0.5, 0.4)),
    "stretcher":    SARTarget(101, "stretcher",   "equipment",  (2.0, 0.6, 0.3)),
    "tent":         SARTarget(102, "tent",        "equipment",  (3.0, 3.0, 2.0)),
    "tarp":         SARTarget(103, "tarp",        "equipment",  (4.0, 4.0, 0.1)),
    "medkit":       SARTarget(104, "medkit",      "equipment",  (0.4, 0.3, 0.2)),
    "drone":        SARTarget(105, "drone",       "equipment",  (0.5, 0.5, 0.2)),
    "fire":         SARTarget(106, "fire",        "hazard",     (1.0, 1.0, 1.5)),
    "smoke_plume":  SARTarget(107, "smoke_plume", "hazard",     (5.0, 5.0, 8.0)),
    "debris":       SARTarget(108, "debris",      "hazard",     (2.0, 2.0, 1.0)),
    "vehicle_wreck":SARTarget(109, "vehicle_wreck","vehicle",   (4.5, 1.8, 1.5)),

    # Additional SAR mission-relevant
    "raft":         SARTarget(110, "raft",        "vehicle",    (3.0, 1.5, 0.5)),
    "lifevest":     SARTarget(111, "lifevest",    "equipment",  (0.5, 0.4, 0.1)),
    "flare":        SARTarget(112, "flare",       "equipment",  (0.3, 0.3, 1.5)),
    "supply_drop":  SARTarget(113, "supply_drop", "equipment",  (1.0, 1.0, 1.0)),
}


def coco_categories() -> List[dict]:
    """Return the catalogue as the COCO `categories` list."""
    return [
        {
            "id": tgt.category_id,
            "name": tgt.name,
            "supercategory": tgt.supercategory,
        }
        for tgt in sorted(SAR_TARGETS.values(), key=lambda t: t.category_id)
    ]


def find(name: str) -> SARTarget:
    """Look up a target by name; raises KeyError on miss."""
    if name not in SAR_TARGETS:
        raise KeyError(
            f"unknown SAR target '{name}'; available: "
            f"{sorted(SAR_TARGETS)}"
        )
    return SAR_TARGETS[name]


def total_count() -> int:
    """How many target classes the catalogue defines."""
    return len(SAR_TARGETS)
