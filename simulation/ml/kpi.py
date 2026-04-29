"""
ML KPI thresholds + acceptance gate.

The actual training loop (PyTorch + Ultralytics, deferred) computes
mAP / Recall / Inference latency. This module defines the thresholds
those numbers must clear and provides `compare_models` for the
registry's promotion gate. Keeping the gate in CI means the rest of
the pipeline can integrate against it the moment real models start
landing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ── Threshold constants (from todo/ml_training_pipeline.md) ──────────────────

ACCEPTANCE_THRESHOLDS: Dict[str, float] = {
    "mAP_50": 0.75,         # primary detection metric
    "recall": 0.85,         # don't miss casualties
    "inference_ms": 50.0,   # per-frame budget on Jetson Orin Nano
}

# Promotion: a candidate model only replaces the current best if it
# improves the primary KPI by at least this margin (avoids constant
# churn from noise).
PROMOTION_MIN_DELTA: Dict[str, float] = {
    "mAP_50": 0.005,
    "recall": 0.005,
    "inference_ms": -1.0,   # latency improvements: candidate must be ≥1 ms faster
}


# ── KPI record + verdict ─────────────────────────────────────────────────────


@dataclass
class DetectionKPIs:
    """One trained model's measured KPIs."""
    mAP_50: float = 0.0
    recall: float = 0.0
    inference_ms: float = 0.0
    extra: Dict[str, float] = field(default_factory=dict)
    failures: List[str] = field(default_factory=list)
    verdict: str = "PASS"

    def as_kpi_dict(self) -> Dict[str, float]:
        """Flatten into the dict shape stored on `ModelEntry.kpis`."""
        d: Dict[str, float] = {
            "mAP_50": float(self.mAP_50),
            "recall": float(self.recall),
            "inference_ms": float(self.inference_ms),
        }
        d.update({k: float(v) for k, v in self.extra.items()})
        return d


def evaluate_kpis(measured: Dict[str, float]) -> DetectionKPIs:
    """Apply `ACCEPTANCE_THRESHOLDS`, return KPIs + verdict."""
    failures: List[str] = []
    if measured.get("mAP_50", 0.0) < ACCEPTANCE_THRESHOLDS["mAP_50"]:
        failures.append(
            f"mAP_50 {measured.get('mAP_50', 0.0):.3f} < "
            f"target {ACCEPTANCE_THRESHOLDS['mAP_50']:.3f}"
        )
    if measured.get("recall", 0.0) < ACCEPTANCE_THRESHOLDS["recall"]:
        failures.append(
            f"recall {measured.get('recall', 0.0):.3f} < "
            f"target {ACCEPTANCE_THRESHOLDS['recall']:.3f}"
        )
    if measured.get("inference_ms",
                    float("inf")) > ACCEPTANCE_THRESHOLDS["inference_ms"]:
        failures.append(
            f"inference_ms {measured.get('inference_ms', float('inf')):.1f} "
            f"> budget {ACCEPTANCE_THRESHOLDS['inference_ms']:.1f}"
        )
    return DetectionKPIs(
        mAP_50=measured.get("mAP_50", 0.0),
        recall=measured.get("recall", 0.0),
        inference_ms=measured.get("inference_ms", 0.0),
        extra={k: v for k, v in measured.items()
               if k not in ("mAP_50", "recall", "inference_ms")},
        failures=failures,
        verdict="PASS" if not failures else "FAIL",
    )


# ── Model promotion ──────────────────────────────────────────────────────────


def compare_models(candidate: Dict[str, float],
                   incumbent: Optional[Dict[str, float]]) -> tuple:
    """Return ``(promote, reason)`` for a candidate vs. the active model.

    *incumbent* is ``None`` for the very first model — that one always
    promotes if its KPIs pass. Otherwise the candidate must beat the
    incumbent on the primary KPI by ``PROMOTION_MIN_DELTA`` and not
    regress on the others.
    """
    cand = evaluate_kpis(candidate)
    if cand.verdict == "FAIL":
        return False, f"candidate fails acceptance gate: {cand.failures}"
    if incumbent is None:
        return True, "no incumbent — first acceptable model promotes"

    inc = evaluate_kpis(incumbent)

    # Latency must not regress.
    delta_lat = candidate.get("inference_ms", 0.0) - incumbent.get("inference_ms", 0.0)
    if delta_lat > 0.5:
        return False, (
            f"latency regression: candidate {candidate.get('inference_ms')} ms "
            f"vs incumbent {incumbent.get('inference_ms')} ms "
            f"(+{delta_lat:.1f} ms)"
        )

    # Primary KPI must improve by at least PROMOTION_MIN_DELTA.
    delta_map = candidate.get("mAP_50", 0.0) - incumbent.get("mAP_50", 0.0)
    if delta_map < PROMOTION_MIN_DELTA["mAP_50"]:
        return False, (
            f"mAP_50 only +{delta_map:.4f} (need ≥ "
            f"{PROMOTION_MIN_DELTA['mAP_50']:.4f})"
        )
    return True, f"primary KPI improved by +{delta_map:.4f}"
