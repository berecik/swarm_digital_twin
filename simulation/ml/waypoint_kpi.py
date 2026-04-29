"""
Waypoint-policy KPI thresholds + promotion gate.

Symmetric counterpart of `ml/kpi.py` for the *control* model: turns the
five-number aggregate dict that `waypoint_optimizer.evaluate_policy()`
returns into a PASS/FAIL verdict, and adds a promotion gate that
prefers higher completion + lower tracking error than the incumbent
without regressing on energy or settling time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# ── Acceptance thresholds (absolute floor for any registered policy) ─────────

WAYPOINT_ACCEPTANCE_THRESHOLDS: Dict[str, float] = {
    "completion_ratio":     0.95,    # at least 95% of waypoints reached
    "rmse_xyz_m":           1.0,     # settled tracking error ≤ 1 m (only counts in-radius samples)
    "time_to_first_wp_s":   30.0,    # first waypoint within 30 s
    "max_overshoot_m":      6.0,     # excursion past the capture zone ≤ 6 m
                                     # (calibrated so 40 m-transit lawnmower passes)
}


# Promotion deltas — relative improvement a candidate must show.
#  +X means "candidate − incumbent ≥ +X" (higher-is-better metrics).
#  -X means "incumbent − candidate ≥ X"  (lower-is-better metrics).
WAYPOINT_PROMOTION_DELTA: Dict[str, float] = {
    "completion_ratio":     0.01,    # +1 percentage-point gain
    "rmse_xyz_m":          -0.05,    # 5 cm tighter
    "time_to_first_wp_s":  -0.5,     # 0.5 s faster
    "energy_proxy_j":       0.0,     # do not regress
}


# ── Verdict record ───────────────────────────────────────────────────────────


@dataclass
class WaypointKPIs:
    completion_ratio:    float = 0.0
    rmse_xyz_m:          float = 0.0
    time_to_first_wp_s:  float = 0.0
    max_overshoot_m:     float = 0.0
    energy_proxy_j:      float = 0.0
    extra:               Dict[str, float] = field(default_factory=dict)
    failures:            List[str] = field(default_factory=list)
    verdict:             str = "PASS"

    def as_kpi_dict(self) -> Dict[str, float]:
        d: Dict[str, float] = {
            "completion_ratio":    float(self.completion_ratio),
            "rmse_xyz_m":          float(self.rmse_xyz_m),
            "time_to_first_wp_s":  float(self.time_to_first_wp_s),
            "max_overshoot_m":     float(self.max_overshoot_m),
            "energy_proxy_j":      float(self.energy_proxy_j),
        }
        d.update({k: float(v) for k, v in self.extra.items()})
        return d


_KEYS = ("completion_ratio", "rmse_xyz_m", "time_to_first_wp_s",
         "max_overshoot_m", "energy_proxy_j")


def evaluate_waypoint_kpis(measured: Dict[str, float]) -> WaypointKPIs:
    """Apply `WAYPOINT_ACCEPTANCE_THRESHOLDS`, return KPIs + verdict."""
    failures: List[str] = []

    completion = measured.get("completion_ratio", 0.0)
    if completion < WAYPOINT_ACCEPTANCE_THRESHOLDS["completion_ratio"]:
        failures.append(
            f"completion_ratio {completion:.3f} < "
            f"target {WAYPOINT_ACCEPTANCE_THRESHOLDS['completion_ratio']:.3f}"
        )

    rmse = measured.get("rmse_xyz_m", float("inf"))
    if rmse > WAYPOINT_ACCEPTANCE_THRESHOLDS["rmse_xyz_m"]:
        failures.append(
            f"rmse_xyz_m {rmse:.3f} > "
            f"target {WAYPOINT_ACCEPTANCE_THRESHOLDS['rmse_xyz_m']:.3f}"
        )

    t_first = measured.get("time_to_first_wp_s", float("inf"))
    if t_first > WAYPOINT_ACCEPTANCE_THRESHOLDS["time_to_first_wp_s"]:
        failures.append(
            f"time_to_first_wp_s {t_first:.2f} > "
            f"target {WAYPOINT_ACCEPTANCE_THRESHOLDS['time_to_first_wp_s']:.2f}"
        )

    overshoot = measured.get("max_overshoot_m", float("inf"))
    if overshoot > WAYPOINT_ACCEPTANCE_THRESHOLDS["max_overshoot_m"]:
        failures.append(
            f"max_overshoot_m {overshoot:.3f} > "
            f"target {WAYPOINT_ACCEPTANCE_THRESHOLDS['max_overshoot_m']:.3f}"
        )

    return WaypointKPIs(
        completion_ratio=completion,
        rmse_xyz_m=rmse,
        time_to_first_wp_s=t_first,
        max_overshoot_m=overshoot,
        energy_proxy_j=measured.get("energy_proxy_j", 0.0),
        extra={k: v for k, v in measured.items() if k not in _KEYS},
        failures=failures,
        verdict="PASS" if not failures else "FAIL",
    )


# ── Promotion gate ───────────────────────────────────────────────────────────


def compare_waypoint_policies(candidate: Dict[str, float],
                              incumbent: Optional[Dict[str, float]]
                              ) -> Tuple[bool, str]:
    """``(promote, reason)`` for a candidate vs the active policy.

    Order of checks:
    1. Candidate must clear the acceptance gate.
    2. If no incumbent, promote.
    3. Energy must not regress (candidate ≤ incumbent).
    4. Primary KPI (completion_ratio) must improve by at least
       `WAYPOINT_PROMOTION_DELTA['completion_ratio']`.
       If completion ties, RMSE must improve by ≥ |rmse delta|.
    """
    cand = evaluate_waypoint_kpis(candidate)
    if cand.verdict == "FAIL":
        return False, f"candidate fails acceptance gate: {cand.failures}"

    if incumbent is None:
        return True, "no incumbent — first acceptable policy promotes"

    delta_energy = (candidate.get("energy_proxy_j", 0.0)
                    - incumbent.get("energy_proxy_j", 0.0))
    if delta_energy > WAYPOINT_PROMOTION_DELTA["energy_proxy_j"]:
        return False, (
            f"energy regression: candidate {candidate.get('energy_proxy_j', 0.0):.1f} J "
            f"vs incumbent {incumbent.get('energy_proxy_j', 0.0):.1f} J"
        )

    delta_completion = (candidate.get("completion_ratio", 0.0)
                        - incumbent.get("completion_ratio", 0.0))
    if delta_completion >= WAYPOINT_PROMOTION_DELTA["completion_ratio"]:
        return True, f"completion improved by +{delta_completion:.4f}"

    if delta_completion >= 0.0:
        # Completion tied (or trivially better) — fall back to rmse.
        delta_rmse = (incumbent.get("rmse_xyz_m", float("inf"))
                      - candidate.get("rmse_xyz_m", float("inf")))
        required = -WAYPOINT_PROMOTION_DELTA["rmse_xyz_m"]
        if delta_rmse >= required:
            return True, (
                f"completion held; rmse tightened by {delta_rmse:.3f} m"
            )

    return False, (
        f"completion only +{delta_completion:.4f} (need ≥ "
        f"{WAYPOINT_PROMOTION_DELTA['completion_ratio']:.4f}) and "
        f"rmse not tightened enough"
    )
