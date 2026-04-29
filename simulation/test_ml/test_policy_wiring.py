"""Tests for wiring the trained PID policy into the Python physics loop.

These cover the glue between the ML training side (`ml.waypoint_optimizer`
+ `ml.model_registry`) and the physics side (`drone_physics.run_simulation`,
`physics_live_replay`). Without this wiring the single-drone live
viewer can't actually benefit from a trained policy.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

import physics_live_replay
from drone_physics import DroneParams, run_simulation
from ml.model_registry import ModelEntry, ModelRegistry
from ml.waypoint_optimizer import PolicyGains


def _tiny_square_waypoints() -> list:
    z = 5.0
    return [
        np.array([0.0, 0.0, z]),
        np.array([5.0, 0.0, z]),
        np.array([5.0, 5.0, z]),
        np.array([0.0, 0.0, z]),
    ]


class TestRunSimulationAcceptsPolicyGains:
    def test_default_and_custom_gains_differ_in_trajectory(self):
        """A policy with distinct gains must produce a different trajectory
        than the baseline (controller behaviour is gain-dependent)."""
        wp = _tiny_square_waypoints()
        base = run_simulation(wp, dt=0.02, max_time=20.0)

        tuned = PolicyGains(
            pos_x_kp=6.0, pos_x_ki=0.7, pos_x_kd=4.0,
            pos_y_kp=6.0, pos_y_ki=0.7, pos_y_kd=4.0,
            pos_z_kp=8.0, pos_z_ki=1.3, pos_z_kd=5.0,
            att_roll_kp=10.0,  att_roll_ki=0.15, att_roll_kd=2.5,
            att_pitch_kp=10.0, att_pitch_ki=0.15, att_pitch_kd=2.5,
            att_yaw_kp=5.0,    att_yaw_ki=0.08,  att_yaw_kd=1.3,
        )
        custom = run_simulation(wp, dt=0.02, max_time=20.0,
                                policy_gains=tuned)

        n = min(len(base), len(custom))
        assert n > 20, "simulation should produce at least some steps"
        diff = np.linalg.norm(base[-1].position - custom[-1].position)
        assert diff > 1e-3, "custom gains produced identical trajectory"

    def test_default_none_keeps_baseline_behaviour(self):
        """Omitting policy_gains (or passing None explicitly) runs with
        the production defaults — no silent behaviour change."""
        wp = _tiny_square_waypoints()
        a = run_simulation(wp, dt=0.02, max_time=10.0)
        b = run_simulation(wp, dt=0.02, max_time=10.0, policy_gains=None)
        assert len(a) == len(b)
        assert np.allclose(a[-1].position, b[-1].position)

    def test_baseline_gains_match_default_behaviour(self):
        """Passing PolicyGains.from_baseline() must be observationally
        identical to passing None (the baseline IS the default)."""
        wp = _tiny_square_waypoints()
        a = run_simulation(wp, dt=0.02, max_time=10.0)
        b = run_simulation(wp, dt=0.02, max_time=10.0,
                           policy_gains=PolicyGains.from_baseline())
        assert len(a) == len(b)
        assert np.allclose(a[-1].position, b[-1].position)


class TestLoadPolicyGains:
    def _register(self, tmp_path: Path, *, bundle_gains: bool,
                   version: str = "pid_v1") -> Path:
        """Create a registry with one policy; optionally bundle gains
        inside kpis (modern layout) or only via the weights sidecar
        (legacy). Returns the registry path."""
        reg_path = tmp_path / "policy_registry.json"
        reg = ModelRegistry(reg_path)
        gains = PolicyGains.from_baseline()
        weights_path = tmp_path / f"{version}_gains.json"
        weights_path.write_text(json.dumps(gains.to_dict()),
                                encoding="utf-8")
        kpis = {
            "completion_ratio":   1.0,
            "rmse_xyz_m":         0.4,
            "time_to_first_wp_s": 12.0,
            "max_overshoot_m":    2.0,
            "energy_proxy_j":     700.0,
        }
        if bundle_gains:
            kpis.update(gains.to_dict())
        reg.register(ModelEntry(
            version=version, backend="pid",
            weights_path=str(weights_path),
            kpis=kpis,
            notes="fixture",
        ))
        return reg_path

    def test_load_with_explicit_version_from_bundled_gains(self, tmp_path):
        reg = self._register(tmp_path, bundle_gains=True)
        gains = physics_live_replay.load_policy_gains(reg, version="pid_v1")
        assert isinstance(gains, PolicyGains)
        assert gains == PolicyGains.from_baseline()

    def test_load_falls_back_to_sidecar_when_kpis_lack_gains(self, tmp_path):
        reg = self._register(tmp_path, bundle_gains=False)
        gains = physics_live_replay.load_policy_gains(reg, version="pid_v1")
        assert gains == PolicyGains.from_baseline()

    def test_load_without_version_picks_best_by_completion(self, tmp_path):
        # Seed two policies; the better-by-completion should win.
        reg_path = tmp_path / "policy_registry.json"
        reg = ModelRegistry(reg_path)
        gains_a = PolicyGains.from_baseline()
        gains_b = PolicyGains(
            pos_x_kp=99.0, pos_x_ki=0.0, pos_x_kd=0.0,
            pos_y_kp=99.0, pos_y_ki=0.0, pos_y_kd=0.0,
            pos_z_kp=99.0, pos_z_ki=0.0, pos_z_kd=0.0,
            att_roll_kp=99.0, att_roll_ki=0.0, att_roll_kd=0.0,
            att_pitch_kp=99.0, att_pitch_ki=0.0, att_pitch_kd=0.0,
            att_yaw_kp=99.0, att_yaw_ki=0.0, att_yaw_kd=0.0,
        )
        reg.register(ModelEntry(
            version="pid_v1", backend="pid", weights_path="ignored",
            kpis={**gains_a.to_dict(), "completion_ratio": 0.8}))
        reg.register(ModelEntry(
            version="pid_v2", backend="pid", weights_path="ignored",
            kpis={**gains_b.to_dict(), "completion_ratio": 0.99}))
        loaded = physics_live_replay.load_policy_gains(reg_path)
        assert loaded == gains_b

    def test_load_empty_registry_returns_none(self, tmp_path):
        reg_path = tmp_path / "policy_registry.json"
        ModelRegistry(reg_path)  # create empty
        assert physics_live_replay.load_policy_gains(reg_path) is None

    def test_load_missing_version_raises(self, tmp_path):
        reg = self._register(tmp_path, bundle_gains=True)
        with pytest.raises(KeyError, match="no model with version"):
            physics_live_replay.load_policy_gains(reg, version="ghost")

    def test_load_missing_sidecar_without_bundled_gains_raises(self, tmp_path):
        reg_path = tmp_path / "policy_registry.json"
        reg = ModelRegistry(reg_path)
        # Neither bundled gains in kpis nor a readable weights file.
        reg.register(ModelEntry(
            version="pid_v1", backend="pid",
            weights_path=str(tmp_path / "nonexistent.json"),
            kpis={"completion_ratio": 1.0}))
        with pytest.raises(ValueError, match="neither full gain dict"):
            physics_live_replay.load_policy_gains(reg_path, version="pid_v1")
