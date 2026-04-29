"""Tests for the waypoint-achievement optimiser."""

from __future__ import annotations

import math

import numpy as np
import pytest

from drone_physics import PositionController, DroneParams
from ml.waypoint_optimizer import (
    EpisodeMetrics, PolicyGains, SearchBounds, SearchResult,
    evaluate_policy, random_search, run_episode,
)


class TestPolicyGains:
    def test_baseline_matches_position_controller_defaults(self):
        baseline = PolicyGains.from_baseline()
        params = DroneParams()
        ctrl = PositionController(params)
        # The PositionController seeds its PIDs with these exact values
        # (see drone_physics.PositionController.__init__).
        assert baseline.pos_x_kp == ctrl.pid_x.kp
        assert baseline.pos_z_kd == ctrl.pid_z.kd
        assert baseline.att_roll_kp == ctrl.pid_roll.kp

    def test_to_dict_round_trip(self):
        g = PolicyGains.from_baseline()
        d = g.to_dict()
        # 18 fields = six PIDs × kp/ki/kd.
        assert len(d) == 18
        g2 = PolicyGains.from_dict(d)
        assert g == g2

    def test_apply_overwrites_controller_pids(self):
        ctrl = PositionController(DroneParams())
        custom = PolicyGains(
            pos_x_kp=11.0, pos_x_ki=0.0, pos_x_kd=0.0,
            pos_y_kp=11.0, pos_y_ki=0.0, pos_y_kd=0.0,
            pos_z_kp=11.0, pos_z_ki=0.0, pos_z_kd=0.0,
            att_roll_kp=22.0,  att_roll_ki=0.0,  att_roll_kd=0.0,
            att_pitch_kp=22.0, att_pitch_ki=0.0, att_pitch_kd=0.0,
            att_yaw_kp=22.0,   att_yaw_ki=0.0,   att_yaw_kd=0.0,
        )
        custom.apply_to(ctrl)
        assert ctrl.pid_x.kp == 11.0
        assert ctrl.pid_z.ki == 0.0
        assert ctrl.pid_yaw.kp == 22.0

    def test_apply_resets_pid_state(self):
        ctrl = PositionController(DroneParams())
        ctrl.pid_x.integral = 99.0
        ctrl.pid_x.prev_error = 7.0
        PolicyGains.from_baseline().apply_to(ctrl)
        assert ctrl.pid_x.integral == 0.0
        assert ctrl.pid_x.prev_error == 0.0


class TestRunEpisode:
    def test_baseline_completes_patrol_mission(self):
        m = run_episode(PolicyGains.from_baseline(),
                        mission_kind="patrol", max_time=120.0)
        assert m.finite
        assert m.completion_ratio == 1.0
        # Settled tracking is a fraction of the capture radius.
        assert m.rmse_xyz_m < 1.0
        assert math.isfinite(m.time_to_first_wp_s)

    def test_baseline_completes_lawnmower_mission(self):
        m = run_episode(PolicyGains.from_baseline(),
                        mission_kind="lawnmower", max_time=120.0)
        assert m.finite
        assert m.completion_ratio == 1.0

    def test_unknown_mission_kind_raises(self):
        with pytest.raises(ValueError, match="unknown mission"):
            run_episode(PolicyGains.from_baseline(),
                        mission_kind="not_a_mission")

    def test_episode_is_deterministic(self):
        gains = PolicyGains.from_baseline()
        a = run_episode(gains, mission_kind="patrol", max_time=60.0)
        b = run_episode(gains, mission_kind="patrol", max_time=60.0)
        assert a.completion_ratio == b.completion_ratio
        assert a.rmse_xyz_m == pytest.approx(b.rmse_xyz_m)
        assert a.time_to_first_wp_s == pytest.approx(b.time_to_first_wp_s)
        assert a.energy_proxy_j == pytest.approx(b.energy_proxy_j)

    def test_pathological_gains_return_failed_metrics(self):
        # Negative-Kp on every axis sends the controller divergent.
        bad = PolicyGains(
            pos_x_kp=-50.0, pos_x_ki=0.0, pos_x_kd=0.0,
            pos_y_kp=-50.0, pos_y_ki=0.0, pos_y_kd=0.0,
            pos_z_kp=-50.0, pos_z_ki=0.0, pos_z_kd=0.0,
            att_roll_kp=-50.0,  att_roll_ki=0.0,  att_roll_kd=0.0,
            att_pitch_kp=-50.0, att_pitch_ki=0.0, att_pitch_kd=0.0,
            att_yaw_kp=-50.0,   att_yaw_ki=0.0,   att_yaw_kd=0.0,
        )
        m = run_episode(bad, mission_kind="patrol", max_time=20.0)
        assert m.completion_ratio == 0.0
        # Either marked non-finite, or finished with an enormous overshoot.
        assert (not m.finite) or m.max_overshoot_m > 10.0

    def test_metrics_serialise_to_kpi_dict(self):
        m = run_episode(PolicyGains.from_baseline(),
                        mission_kind="patrol", max_time=120.0)
        d = m.as_kpi_dict()
        for key in ("completion_ratio", "rmse_xyz_m",
                    "time_to_first_wp_s", "energy_proxy_j",
                    "max_overshoot_m", "waypoints_reached",
                    "waypoint_count", "total_time_s"):
            assert key in d


class TestEvaluatePolicy:
    def test_single_mission_aggregation(self):
        m = evaluate_policy(PolicyGains.from_baseline(),
                            mission_kinds_to_run=["patrol"],
                            max_time=120.0)
        for key in ("completion_ratio", "rmse_xyz_m",
                    "time_to_first_wp_s", "energy_proxy_j",
                    "max_overshoot_m", "finite_ratio"):
            assert key in m
        assert m["finite_ratio"] == 1.0
        assert m["completion_ratio"] == 1.0

    def test_multi_mission_aggregation_averages(self):
        gains = PolicyGains.from_baseline()
        single = evaluate_policy(gains, mission_kinds_to_run=["patrol"],
                                 max_time=120.0)
        both = evaluate_policy(gains,
                               mission_kinds_to_run=["patrol", "lawnmower"],
                               max_time=120.0)
        # The two-mission average should differ from the single-mission
        # value (lawnmower has its own settled error and energy).
        assert both["energy_proxy_j"] != pytest.approx(single["energy_proxy_j"])

    def test_finite_ratio_drops_when_diverges(self):
        bad = PolicyGains(
            pos_x_kp=-100.0, pos_x_ki=0.0, pos_x_kd=0.0,
            pos_y_kp=-100.0, pos_y_ki=0.0, pos_y_kd=0.0,
            pos_z_kp=-100.0, pos_z_ki=0.0, pos_z_kd=0.0,
            att_roll_kp=-100.0,  att_roll_ki=0.0,  att_roll_kd=0.0,
            att_pitch_kp=-100.0, att_pitch_ki=0.0, att_pitch_kd=0.0,
            att_yaw_kp=-100.0,   att_yaw_ki=0.0,   att_yaw_kd=0.0,
        )
        m = evaluate_policy(bad, max_time=10.0)
        assert m["finite_ratio"] < 1.0 or m["completion_ratio"] == 0.0


class TestRandomSearch:
    def test_baseline_always_in_trials(self):
        result = random_search(n_trials=2, seed=0, max_time=30.0)
        assert isinstance(result, SearchResult)
        # First trial is always the baseline.
        baseline = PolicyGains.from_baseline()
        assert result.trials[0][0] == baseline

    def test_best_is_at_least_as_good_as_baseline(self):
        result = random_search(n_trials=4, seed=42, max_time=30.0)
        baseline_obj = result.trials[0][2]
        assert result.best_objective >= baseline_obj

    def test_search_is_seeded(self):
        a = random_search(n_trials=3, seed=42, max_time=30.0)
        b = random_search(n_trials=3, seed=42, max_time=30.0)
        # Same seed → same sampled gains (by reference equality of fields).
        for (ga, _, _), (gb, _, _) in zip(a.trials, b.trials):
            assert ga == gb

    def test_bounds_respected_in_sampled_gains(self):
        bnds = SearchBounds(pos_lo=0.9, pos_hi=1.1,
                            att_lo=0.9, att_hi=1.1)
        result = random_search(n_trials=8, seed=7, max_time=20.0,
                               bounds=bnds)
        baseline = PolicyGains.from_baseline()
        for sample, _, _ in result.trials[1:]:  # skip baseline
            for field_name in baseline.to_dict():
                base_val = getattr(baseline, field_name)
                samp_val = getattr(sample, field_name)
                if base_val == 0:
                    assert samp_val == 0
                else:
                    ratio = samp_val / base_val
                    assert 0.9 - 1e-9 <= ratio <= 1.1 + 1e-9, (
                        f"{field_name} ratio {ratio} outside [0.9, 1.1]"
                    )

    def test_trial_count_equals_n_trials_plus_one(self):
        # Always +1 because the baseline is evaluated as trial 0.
        result = random_search(n_trials=3, seed=0, max_time=20.0)
        assert len(result.trials) == 4
