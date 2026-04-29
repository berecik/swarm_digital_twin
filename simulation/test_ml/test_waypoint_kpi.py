"""Tests for the waypoint-policy KPI gates."""

from __future__ import annotations

import pytest

from ml.waypoint_kpi import (
    WAYPOINT_ACCEPTANCE_THRESHOLDS, WAYPOINT_PROMOTION_DELTA,
    WaypointKPIs, compare_waypoint_policies, evaluate_waypoint_kpis,
)


def _passing_metrics(**overrides) -> dict:
    """A measured-KPI dict that PASSes every default threshold."""
    base = {
        "completion_ratio":    1.0,
        "rmse_xyz_m":          0.5,
        "time_to_first_wp_s":  10.0,
        "max_overshoot_m":     2.0,
        "energy_proxy_j":      750.0,
    }
    base.update(overrides)
    return base


class TestAcceptanceThresholds:
    def test_documented_thresholds_present(self):
        assert WAYPOINT_ACCEPTANCE_THRESHOLDS["completion_ratio"] >= 0.9
        assert WAYPOINT_ACCEPTANCE_THRESHOLDS["rmse_xyz_m"] > 0
        assert WAYPOINT_ACCEPTANCE_THRESHOLDS["time_to_first_wp_s"] > 0
        assert WAYPOINT_ACCEPTANCE_THRESHOLDS["max_overshoot_m"] > 0

    def test_promotion_deltas_signs(self):
        # Higher-is-better → positive delta.
        assert WAYPOINT_PROMOTION_DELTA["completion_ratio"] > 0
        # Lower-is-better → negative delta (incumbent − candidate).
        assert WAYPOINT_PROMOTION_DELTA["rmse_xyz_m"] < 0
        assert WAYPOINT_PROMOTION_DELTA["time_to_first_wp_s"] < 0


class TestEvaluateWaypointKPIs:
    def test_passing_metrics(self):
        out = evaluate_waypoint_kpis(_passing_metrics())
        assert out.verdict == "PASS"
        assert out.failures == []

    def test_low_completion_fails(self):
        out = evaluate_waypoint_kpis(_passing_metrics(completion_ratio=0.5))
        assert out.verdict == "FAIL"
        assert any("completion_ratio" in f for f in out.failures)

    def test_high_rmse_fails(self):
        out = evaluate_waypoint_kpis(_passing_metrics(rmse_xyz_m=5.0))
        assert out.verdict == "FAIL"
        assert any("rmse_xyz_m" in f for f in out.failures)

    def test_late_first_waypoint_fails(self):
        out = evaluate_waypoint_kpis(
            _passing_metrics(time_to_first_wp_s=120.0))
        assert out.verdict == "FAIL"
        assert any("time_to_first_wp_s" in f for f in out.failures)

    def test_overshoot_fails(self):
        out = evaluate_waypoint_kpis(_passing_metrics(max_overshoot_m=99.0))
        assert out.verdict == "FAIL"
        assert any("max_overshoot_m" in f for f in out.failures)

    def test_missing_keys_default_to_failures(self):
        # An empty dict means "nothing measured" — every threshold fails.
        out = evaluate_waypoint_kpis({})
        assert out.verdict == "FAIL"
        # All four enforced floors should appear.
        for key in ("completion_ratio", "rmse_xyz_m",
                    "time_to_first_wp_s", "max_overshoot_m"):
            assert any(key in f for f in out.failures)

    def test_extra_metrics_preserved(self):
        out = evaluate_waypoint_kpis(_passing_metrics(custom_metric=42.0))
        assert out.extra == {"custom_metric": 42.0}

    def test_as_kpi_dict_round_trip(self):
        d = WaypointKPIs(
            completion_ratio=1.0, rmse_xyz_m=0.5,
            time_to_first_wp_s=8.0, max_overshoot_m=1.5,
            energy_proxy_j=600.0, extra={"finite_ratio": 1.0},
        ).as_kpi_dict()
        assert d["completion_ratio"] == 1.0
        assert d["finite_ratio"] == 1.0


class TestCompareWaypointPolicies:
    def test_first_passing_policy_promotes(self):
        promote, reason = compare_waypoint_policies(
            candidate=_passing_metrics(), incumbent=None)
        assert promote is True
        assert "first acceptable policy" in reason

    def test_failing_candidate_does_not_promote(self):
        promote, reason = compare_waypoint_policies(
            candidate=_passing_metrics(completion_ratio=0.5),
            incumbent=_passing_metrics())
        assert promote is False
        assert "acceptance gate" in reason

    def test_better_completion_promotes(self):
        inc = _passing_metrics(completion_ratio=0.96)
        cand = _passing_metrics(completion_ratio=0.99)
        promote, reason = compare_waypoint_policies(cand, inc)
        assert promote is True
        assert "completion improved" in reason

    def test_marginal_completion_does_not_promote(self):
        # +0.001 completion is below the +0.01 promotion delta and
        # rmse hasn't tightened, so reject.
        inc = _passing_metrics(completion_ratio=0.96, rmse_xyz_m=0.5)
        cand = _passing_metrics(completion_ratio=0.961, rmse_xyz_m=0.5)
        promote, reason = compare_waypoint_policies(cand, inc)
        assert promote is False

    def test_completion_tie_falls_back_to_rmse(self):
        # Same completion, candidate is 0.10 m tighter on RMSE → promote.
        inc = _passing_metrics(rmse_xyz_m=0.50)
        cand = _passing_metrics(rmse_xyz_m=0.40)
        promote, reason = compare_waypoint_policies(cand, inc)
        assert promote is True
        assert "rmse tightened" in reason

    def test_energy_regression_blocks_promotion(self):
        inc = _passing_metrics(energy_proxy_j=700.0)
        # Candidate's completion improves but burns 50 J more — reject.
        cand = _passing_metrics(completion_ratio=0.99, energy_proxy_j=750.0)
        promote, reason = compare_waypoint_policies(cand, inc)
        assert promote is False
        assert "energy" in reason.lower()

    def test_identical_metrics_do_not_promote(self):
        same = _passing_metrics()
        promote, _ = compare_waypoint_policies(same, same)
        assert promote is False
