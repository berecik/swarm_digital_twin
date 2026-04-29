"""Tests for the ML KPI thresholds + promotion gate."""

import pytest

from ml.kpi import (
    ACCEPTANCE_THRESHOLDS, PROMOTION_MIN_DELTA,
    DetectionKPIs, compare_models, evaluate_kpis,
)


class TestAcceptanceThresholds:
    def test_documented_thresholds_present(self):
        # Sourced from todo/ml_training_pipeline.md.
        assert ACCEPTANCE_THRESHOLDS["mAP_50"] == 0.75
        assert ACCEPTANCE_THRESHOLDS["recall"] == 0.85
        assert ACCEPTANCE_THRESHOLDS["inference_ms"] == 50.0


class TestEvaluate:
    def test_passing_kpis(self):
        out = evaluate_kpis({"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0})
        assert out.verdict == "PASS"
        assert out.failures == []

    def test_low_map_fails(self):
        out = evaluate_kpis({"mAP_50": 0.50, "recall": 0.86, "inference_ms": 42.0})
        assert out.verdict == "FAIL"
        assert any("mAP_50" in f for f in out.failures)

    def test_low_recall_fails(self):
        out = evaluate_kpis({"mAP_50": 0.80, "recall": 0.40, "inference_ms": 42.0})
        assert out.verdict == "FAIL"
        assert any("recall" in f for f in out.failures)

    def test_high_inference_time_fails(self):
        out = evaluate_kpis({"mAP_50": 0.80, "recall": 0.90, "inference_ms": 120.0})
        assert out.verdict == "FAIL"
        assert any("inference_ms" in f for f in out.failures)


class TestPromotion:
    def test_first_model_with_passing_kpis_promotes(self):
        promote, _ = compare_models(
            candidate={"mAP_50": 0.80, "recall": 0.90, "inference_ms": 40.0},
            incumbent=None,
        )
        assert promote is True

    def test_first_model_with_failing_kpis_does_not_promote(self):
        promote, reason = compare_models(
            candidate={"mAP_50": 0.5, "recall": 0.90, "inference_ms": 40.0},
            incumbent=None,
        )
        assert promote is False
        assert "acceptance gate" in reason

    def test_better_candidate_promotes(self):
        inc = {"mAP_50": 0.80, "recall": 0.85, "inference_ms": 45.0}
        cand = {"mAP_50": 0.83, "recall": 0.88, "inference_ms": 40.0}
        promote, _ = compare_models(cand, inc)
        assert promote is True

    def test_marginal_candidate_does_not_promote(self):
        # +0.001 mAP_50 — below the 0.005 promotion delta.
        inc = {"mAP_50": 0.80, "recall": 0.85, "inference_ms": 45.0}
        cand = {"mAP_50": 0.801, "recall": 0.85, "inference_ms": 45.0}
        promote, reason = compare_models(cand, inc)
        assert promote is False
        assert "mAP_50" in reason

    def test_latency_regression_blocks_promotion(self):
        inc = {"mAP_50": 0.80, "recall": 0.85, "inference_ms": 30.0}
        cand = {"mAP_50": 0.83, "recall": 0.86, "inference_ms": 49.0}
        promote, reason = compare_models(cand, inc)
        assert promote is False
        assert "latency" in reason.lower()

    def test_detection_kpis_round_trip(self):
        d = DetectionKPIs(mAP_50=0.8, recall=0.9, inference_ms=42.0,
                          extra={"map_75": 0.62})
        out = d.as_kpi_dict()
        assert out == {"mAP_50": 0.8, "recall": 0.9,
                        "inference_ms": 42.0, "map_75": 0.62}

    def test_promotion_min_delta_constants_present(self):
        assert PROMOTION_MIN_DELTA["mAP_50"] == 0.005
        assert PROMOTION_MIN_DELTA["recall"] == 0.005
        # Negative latency delta = candidate must be at least 1 ms faster.
        assert PROMOTION_MIN_DELTA["inference_ms"] == -1.0

    def test_evaluate_with_missing_keys_uses_safe_defaults(self):
        # An empty dict represents a model that hasn't been measured —
        # every threshold should fail.
        out = evaluate_kpis({})
        assert out.verdict == "FAIL"
        assert any("mAP_50" in f for f in out.failures)
        assert any("recall" in f for f in out.failures)

    def test_identical_metrics_do_not_promote(self):
        # No improvement on the primary KPI → don't churn the registry.
        same = {"mAP_50": 0.80, "recall": 0.85, "inference_ms": 45.0}
        promote, reason = compare_models(same, same)
        assert promote is False
        assert "mAP_50" in reason

    def test_extra_kpis_preserved_in_as_dict(self):
        d = DetectionKPIs(mAP_50=0.8, recall=0.9, inference_ms=42.0)
        # No extras — only the three core keys appear.
        assert d.as_kpi_dict() == {"mAP_50": 0.8, "recall": 0.9,
                                    "inference_ms": 42.0}

    def test_compare_promotes_with_minimum_delta_exactly(self):
        # Exactly meeting the promotion delta should still promote.
        inc = {"mAP_50": 0.80, "recall": 0.85, "inference_ms": 40.0}
        cand = {"mAP_50": 0.80 + PROMOTION_MIN_DELTA["mAP_50"],
                "recall": 0.85, "inference_ms": 40.0}
        promote, _ = compare_models(cand, inc)
        assert promote is True
