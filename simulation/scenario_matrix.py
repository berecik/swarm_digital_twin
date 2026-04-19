"""
Scenario matrix for full-system K8s validation (Phase 6).

Single source of truth for the dimensions:
  drone_count × terrain × wind × mission × fault.

Full matrix: 4 × 3 × 4 × 4 × 5 = 960 scenarios.
CI subset:   1 representative per dimension = 20 scenarios.

Faults are declarative in CI mode — the Python runner records the
requested fault but does not actually inject it (that requires K8s).
A future K8s runtime runner consumes the same configs and adds the
fault-injection step.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import List


# ── Dimension values ─────────────────────────────────────────────────────────

DRONE_COUNTS: List[int] = [1, 3, 6, 12]
TERRAINS: List[str] = ["flat", "synthetic_rolling", "antisana"]
WINDS: List[str] = ["calm", "crosswind", "gusty", "storm"]
MISSIONS: List[str] = ["patrol", "lawnmower", "escort", "heavy_lift"]
FAULTS: List[str] = ["none", "pod_restart", "packet_loss",
                     "telemetry_delay", "sensor_dropout"]


@dataclass(frozen=True)
class ScenarioConfig:
    """One row of the scenario matrix."""
    drones: int
    terrain: str
    wind: str
    mission: str
    fault: str

    @property
    def scenario_id(self) -> str:
        """Filesystem-friendly identifier for the report tree."""
        return (
            f"{self.drones:02d}drone-"
            f"{self.terrain}-"
            f"{self.wind}-"
            f"{self.mission}-"
            f"{self.fault}"
        )


def full_matrix() -> List[ScenarioConfig]:
    """Return the full 4×3×4×4×5 = 960 scenario combinations."""
    return [
        ScenarioConfig(*combo)
        for combo in product(DRONE_COUNTS, TERRAINS, WINDS, MISSIONS, FAULTS)
    ]


def ci_subset() -> List[ScenarioConfig]:
    """Return the recommended 20-scenario CI subset.

    The four diagonal rows from todo/k8s_test_matrix.md exercise every
    value of every dimension at least once, plus 16 representative
    critical-path combinations covering the dimensions that the four
    diagonals don't cross.

    The first four rows verbatim:
        1-drone  + flat              + calm      + patrol     + none
        3-drone  + synthetic_rolling + crosswind + lawnmower  + pod_restart
        6-drone  + antisana          + gusty     + escort     + packet_loss
        12-drone + flat              + storm     + heavy_lift + sensor_dropout
    """
    diagonal = [
        ScenarioConfig(1,  "flat",              "calm",      "patrol",     "none"),
        ScenarioConfig(3,  "synthetic_rolling", "crosswind", "lawnmower",  "pod_restart"),
        ScenarioConfig(6,  "antisana",          "gusty",     "escort",     "packet_loss"),
        ScenarioConfig(12, "flat",              "storm",     "heavy_lift", "sensor_dropout"),
    ]
    # 16 critical-path combinations: pair each (drone_count, terrain) with
    # a wind/mission/fault triple chosen so every value of each dimension
    # appears in at least one row that the diagonal didn't already cover.
    critical = [
        ScenarioConfig(1,  "synthetic_rolling", "crosswind", "lawnmower",  "none"),
        ScenarioConfig(1,  "antisana",          "gusty",     "escort",     "packet_loss"),
        ScenarioConfig(1,  "flat",              "storm",     "heavy_lift", "telemetry_delay"),
        ScenarioConfig(3,  "flat",              "calm",      "escort",     "none"),
        ScenarioConfig(3,  "antisana",          "gusty",     "patrol",     "telemetry_delay"),
        ScenarioConfig(3,  "synthetic_rolling", "storm",     "heavy_lift", "sensor_dropout"),
        ScenarioConfig(6,  "flat",              "calm",      "lawnmower",  "telemetry_delay"),
        ScenarioConfig(6,  "synthetic_rolling", "crosswind", "patrol",     "none"),
        ScenarioConfig(6,  "antisana",          "storm",     "heavy_lift", "pod_restart"),
        ScenarioConfig(12, "synthetic_rolling", "calm",      "patrol",     "packet_loss"),
        ScenarioConfig(12, "antisana",          "crosswind", "lawnmower",  "telemetry_delay"),
        ScenarioConfig(12, "flat",              "gusty",     "escort",     "none"),
        ScenarioConfig(6,  "flat",              "crosswind", "heavy_lift", "sensor_dropout"),
        ScenarioConfig(12, "antisana",          "gusty",     "patrol",     "sensor_dropout"),
        ScenarioConfig(3,  "antisana",          "calm",      "heavy_lift", "packet_loss"),
        ScenarioConfig(6,  "synthetic_rolling", "storm",     "lawnmower",  "telemetry_delay"),
    ]
    return diagonal + critical


SUBSETS = {"ci": ci_subset, "full": full_matrix}


def select(subset: str) -> List[ScenarioConfig]:
    """Resolve a subset name to its scenario list."""
    if subset not in SUBSETS:
        raise ValueError(
            f"unknown subset '{subset}'; available: {sorted(SUBSETS)}"
        )
    return SUBSETS[subset]()
