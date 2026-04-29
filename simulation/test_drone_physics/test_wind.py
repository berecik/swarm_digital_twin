"""
Test Wind

Auto-split from `test_drone_physics.py` into a focused per-domain test file.
Shared parametrize-time helpers live in `simulation/_test_common.py`.
"""

import numpy as np
import pytest
import warnings
from urllib.error import HTTPError
from pathlib import Path

from drone_physics import (
    DroneParams, DroneState, DroneCommand,
    PositionController, PIDController,
    physics_step, euler_to_rotation, rotation_to_euler,
    euler_rates_from_body_rates,
    run_simulation, run_trajectory_tracking, GRAVITY,
    AeroCoefficients, Atmosphere, _compute_quadratic_drag,
    _compute_lift, compute_aoa,
    FixedWingAero, QuadrotorAero, MotorModel, BatteryModel,
    make_generic_quad, make_holybro_x500, make_fixed_wing,
    make_valencia_fixed_wing, make_irs4_quadrotor,
    run_swarm_simulation, calculate_flocking_vector, FlockingParams,
)
from wind_model import WindField
from terrain import TerrainMap
from validation import (
    BENCHMARK_PROFILES,
    ValidationResult,
    ValidationEnvelope,
    assert_validation_pass,
    get_benchmark_profile,
    get_real_log_mission,
    assert_real_log_validation_pass,
    compute_rmse,
    compare_sim_real,
    auto_tune_wind_force_scale,
    ensure_real_log_logs,
)
from drone_scenario import run_benchmark, run_irs4_benchmark, replay_mission
from flight_log import FlightLog
from swarm_scenario import run_swarm_benchmark, SWARM_BENCHMARK_PROFILES, get_swarm_benchmark_profile
from sensor_models import GPSNoise, IMUNoise, BaroNoise

from _test_common import (
    SIM_DIR,
    PROJECT_ROOT,
    PARITY_MAX_DELTA_M,
    PARITY_RNG_SEED,
    PARITY_SAMPLE_COUNT,
    PROFILE_BASE_SPEED_MS,
    CRUISE_AGL_FLOOR_M,
    CRUISE_ALTITUDE_M,
    CLIMB_TIMEOUT_S,
    live_js_source,
    parity_entry_names,
    ramp_terrain,
    regression_mission,
    stress_mission,
    wind_profile_names_safe,
)

# Aliases preserve the underscore-prefixed names the existing test
# bodies still reference. New tests should use the public names from
# `_test_common` directly.
_LIVE_JS = SIM_DIR / "runtime_view" / "web" / "live.js"
_live_js_source = live_js_source
_parity_entry_names = parity_entry_names
_ramp_terrain = ramp_terrain
_regression_mission = regression_mission
_wind_profile_names = wind_profile_names_safe
_stress_mission = stress_mission
_PROFILE_BASE_SPEED_MS = PROFILE_BASE_SPEED_MS


class TestWind:
    def test_no_wind_unchanged(self):
        """Wind with type='none' produces zero force."""
        wind = WindField(turbulence_type="none")
        force = wind.get_force(0.0, np.array([0.0, 0.0, 10.0]), None, 1.225)
        np.testing.assert_allclose(force, np.zeros(3))

    def test_constant_wind_deflects_hover(self):
        """Drone hovering in constant wind should drift downwind."""
        wind = WindField(wind_speed=5.0,
                         wind_direction=np.array([1.0, 0.0, 0.0]),
                         turbulence_type="constant")
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):  # 10 seconds
            state = physics_step(state, cmd, params, dt=0.005, wind=wind, t=0.0)

        # Should have drifted in +x direction
        assert state.position[0] > 1.0

    def test_stronger_wind_more_force(self):
        """Force should scale with V^2."""
        aero = AeroCoefficients(reference_area=0.04, C_D=1.0)
        w1 = WindField(wind_speed=5.0,
                       wind_direction=np.array([1.0, 0.0, 0.0]),
                       turbulence_type="constant")
        w2 = WindField(wind_speed=10.0,
                       wind_direction=np.array([1.0, 0.0, 0.0]),
                       turbulence_type="constant")

        pos = np.array([0.0, 0.0, 10.0])
        F1 = np.linalg.norm(w1.get_force(0.0, pos, aero, 1.225))
        F2 = np.linalg.norm(w2.get_force(0.0, pos, aero, 1.225))

        # F2/F1 should be (10/5)^2 = 4
        np.testing.assert_allclose(F2 / F1, 4.0, rtol=1e-6)

    def test_wind_from_log_matches_data(self):
        """Replayed log should produce expected force profile."""
        profile = np.array([
            [0.0, 0.0],
            [5.0, 5.0],
            [10.0, 10.0],
        ])
        wind = WindField(
            wind_direction=np.array([0.0, 1.0, 0.0]),
            turbulence_type="from_log",
            altitude_profile=profile,
        )

        # At t=0, wind speed = 0 → zero force
        v0 = wind.get_wind_velocity(0.0, np.zeros(3))
        np.testing.assert_allclose(np.linalg.norm(v0), 0.0, atol=1e-8)

        # At t=5, wind speed = 5 m/s in +y
        v5 = wind.get_wind_velocity(5.0, np.zeros(3))
        np.testing.assert_allclose(v5, [0.0, 5.0, 0.0], atol=1e-8)

        # At t=7.5 (interpolated), wind speed = 7.5 m/s
        v75 = wind.get_wind_velocity(7.5, np.zeros(3))
        np.testing.assert_allclose(np.linalg.norm(v75), 7.5, atol=1e-8)


# ── Sensor Noise Models (GPS / IMU / Barometer) ─────────────────────────────


class TestWindEstimation3D:
    def test_still_trajectory_gives_zero_wind(self):
        """Stationary drone should produce near-zero wind estimate."""
        from flight_log import FlightLog
        n = 100
        log = FlightLog(
            timestamps=np.linspace(0, 10, n),
            positions=np.tile([0.0, 0.0, -50.0], (n, 1)),
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape[1] == 4  # [t, wx, wy, wz]
        # Stationary -> zero velocity deviations -> zero wind
        np.testing.assert_allclose(profile[:, 1:], 0.0, atol=1e-10)

    def test_constant_drift_detected(self):
        """Constant lateral drift should produce non-zero X/Y wind estimate."""
        from flight_log import FlightLog
        n = 200
        t = np.linspace(0, 20, n)
        # Straight-line flight along X, but with constant Y drift (wind pushing in Y)
        positions = np.column_stack([
            t * 5.0,           # X: steady forward flight
            t * 0.0 + np.sin(t * 0.5) * 2.0,  # Y: sinusoidal deviation
            np.full(n, -50.0),  # Z: constant altitude
        ])
        log = FlightLog(
            timestamps=t,
            positions=positions,
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape[0] == n - 1
        # Should detect non-zero Y-component wind
        y_wind_rms = np.sqrt(np.mean(profile[:, 2] ** 2))
        assert y_wind_rms > 0.01, f"Expected non-zero Y wind, got RMS={y_wind_rms}"

    def test_3d_profile_has_correct_shape(self):
        """3D wind profile should have Nx4 shape [t, wx, wy, wz]."""
        from flight_log import FlightLog
        n = 50
        log = FlightLog(
            timestamps=np.linspace(0, 5, n),
            positions=np.random.randn(n, 3).cumsum(axis=0),
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape == (n - 1, 4)
        # Timestamps should match input (minus last)
        np.testing.assert_allclose(profile[:, 0], log.timestamps[:-1], atol=1e-10)

    def test_3d_profile_too_short_returns_zero(self):
        """Trajectory with < 3 points should return zero wind."""
        from flight_log import FlightLog
        log = FlightLog(
            timestamps=np.array([0.0, 1.0]),
            positions=np.array([[0, 0, 0], [1, 0, 0]], dtype=float),
            attitudes=np.zeros((2, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape == (1, 4)
        np.testing.assert_allclose(profile[0, 1:], 0.0, atol=1e-10)

    def test_from_log_3d_wind_replay(self):
        """WindField with from_log_3d should replay 3D wind profile."""
        profile = np.array([
            [0.0, 1.0, 2.0, -0.5],
            [5.0, 1.5, 2.5, -0.3],
            [10.0, 2.0, 3.0, -0.1],
        ])
        wf = WindField(
            turbulence_type="from_log_3d",
            wind_profile_3d=profile,
        )
        # At t=0
        v0 = wf.get_wind_velocity(0.0, np.zeros(3))
        np.testing.assert_allclose(v0, [1.0, 2.0, -0.5], atol=1e-10)

        # At t=5 (midpoint)
        v5 = wf.get_wind_velocity(5.0, np.zeros(3))
        np.testing.assert_allclose(v5, [1.5, 2.5, -0.3], atol=1e-10)

        # At t=2.5 (interpolated)
        v25 = wf.get_wind_velocity(2.5, np.zeros(3))
        np.testing.assert_allclose(v25, [1.25, 2.25, -0.4], atol=1e-10)


# ── CI Pipeline ──────────────────────────────────────────────────────────


class TestWindAutoTuning:
    def test_auto_tuning_converges_and_recovers_scale(self):
        t = np.linspace(0.0, 60.0, 601)
        base = np.sin(t * 0.15) + 0.25 * np.sin(t * 0.03)
        true_scale = 1.7
        ref_z = true_scale * base
        ref_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), ref_z])

        def simulate(scale: float):
            sim_z = scale * base
            sim_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), sim_z])
            return t, sim_positions

        result = auto_tune_wind_force_scale(
            ref_times=t,
            ref_positions=ref_positions,
            simulate_with_scale=simulate,
            initial_scale=0.3,
            initial_step=0.7,
            max_iterations=30,
            convergence_tol=0.01,
        )

        assert result.converged
        assert result.best_rmse_z < 0.01
        assert abs(result.best_scale - true_scale) < 0.05


class TestPositionAwareWind:
    """Tests for wind node position subscription (altitude-dependent density)."""

    def test_wind_node_has_pose_subscription(self):
        """wind_node.py subscribes to /mavros/local_position/pose."""
        import os
        path = os.path.join(str(PROJECT_ROOT), 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '/mavros/local_position/pose' in content
        assert 'PoseStamped' in content

    def test_wind_node_has_pose_callback(self):
        """wind_node.py has _pose_callback method."""
        import os
        path = os.path.join(str(PROJECT_ROOT), 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '_pose_callback' in content

    def test_wind_node_uses_drone_pos(self):
        """wind_node.py uses drone_pos instead of hardcoded zeros."""
        import os
        path = os.path.join(str(PROJECT_ROOT), 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        # Old hardcoded line in publish_wind should be gone
        assert 'pos = np.zeros(3)  # TODO' not in content
        assert 'self.drone_pos' in content

    def test_wind_node_altitude_density(self):
        """wind_node.py computes altitude-dependent density."""
        import os
        path = os.path.join(str(PROJECT_ROOT), 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '_get_altitude_density' in content
        assert 'base_altitude_msl' in content

    def test_isa_density_at_altitude(self):
        """ISA density at 4500m differs from sea level."""
        rho_sea = Atmosphere(altitude_msl=0.0).rho
        rho_4500 = Atmosphere(altitude_msl=4500.0).rho
        ratio = rho_4500 / rho_sea
        # At 4500m, density should be ~0.60-0.65 of sea level
        assert 0.55 < ratio < 0.70, f"Density ratio {ratio:.3f} unexpected"


# ── Euler Rate Kinematics (Eq. 2) ────────────────────────────────────────────


class TestWindProfileManifest:
    """Manifest loader for wind profiles."""

    def test_unknown_profile_name_raises(self):
        from wind_model import load_wind_profile
        with pytest.raises(ValueError, match="unknown wind profile"):
            load_wind_profile("does_not_exist")

    def test_missing_manifest_file(self, tmp_path):
        from wind_model import load_wind_profile
        with pytest.raises(FileNotFoundError, match="wind manifest not found"):
            load_wind_profile("calm", manifest_path=tmp_path / "nope.toml")

    def test_missing_profile_field(self, tmp_path):
        from wind_model import load_wind_profile
        manifest = tmp_path / "wind.toml"
        manifest.write_text('[oops]\nspeed_ms = 1.0\n', encoding="utf-8")
        with pytest.raises(ValueError,
                           match=r"missing required field 'profile'"):
            load_wind_profile("oops", manifest_path=manifest)

    def test_unknown_profile_kind(self, tmp_path):
        from wind_model import load_wind_profile
        manifest = tmp_path / "wind.toml"
        manifest.write_text('[oops]\nprofile = "alien"\n', encoding="utf-8")
        with pytest.raises(ValueError, match=r"unknown profile 'alien'"):
            load_wind_profile("oops", manifest_path=manifest)

    def test_constant_missing_required(self, tmp_path):
        from wind_model import load_wind_profile
        manifest = tmp_path / "wind.toml"
        manifest.write_text(
            '[bad]\nprofile = "constant"\ndirection = [1.0, 0.0, 0.0]\n',
            encoding="utf-8",
        )
        with pytest.raises(ValueError,
                           match=r"missing required field 'speed_ms'"):
            load_wind_profile("bad", manifest_path=manifest)

    @pytest.mark.parametrize("name", _wind_profile_names())
    def test_profile_loads_with_expected_base_speed(self, name):
        """Each manifest profile produces wind near the documented base speed."""
        from wind_model import load_wind_profile
        wind = load_wind_profile(name)
        # For Dryden the steady-state base equals wind_speed; for constant it
        # equals wind_speed times the unit direction. For 'none' both are 0.
        v = wind.get_wind_velocity(0.0, np.array([0.0, 0.0, 10.0]))
        mag = float(np.linalg.norm(v))
        expected = _PROFILE_BASE_SPEED_MS.get(name)
        if expected is None:  # pragma: no cover
            pytest.skip(f"no expected base-speed entry for '{name}'")
        # Dryden adds turbulence: tolerate ±50% of base on the first sample
        # (the per-instance RNG keeps this deterministic anyway).
        assert mag == pytest.approx(expected, rel=0.5, abs=0.5), (
            f"{name}: base wind {mag:.2f} m/s vs expected ~{expected:.2f}"
        )


class TestWindDeterminism:
    """Seeded Dryden produces bit-identical timeseries."""

    def test_seeded_dryden_is_reproducible(self):
        from wind_model import WindField

        def sample(seed):
            w = WindField(
                wind_speed=4.0,
                wind_direction=np.array([1.0, 0.0, 0.0]),
                gust_intensity=0.3,
                turbulence_type="dryden",
                seed=seed,
            )
            return np.array([
                w.get_wind_velocity(0.005 * i, np.array([0.0, 0.0, 10.0]))
                for i in range(200)
            ])

        a = sample(42)
        b = sample(42)
        c = sample(43)
        assert np.array_equal(a, b), \
            "same seed must yield identical Dryden samples"
        assert not np.array_equal(a, c), \
            "different seeds must yield different Dryden samples"

    def test_unseeded_dryden_uses_fresh_rng(self):
        """Two unseeded WindFields must not share state across instances."""
        from wind_model import WindField
        a = WindField(wind_speed=2.0, gust_intensity=0.2,
                      turbulence_type="dryden")
        b = WindField(wind_speed=2.0, gust_intensity=0.2,
                      turbulence_type="dryden")
        # They use different seeds (None → entropy), so almost certainly
        # produce different samples; but more importantly, sampling from a
        # must not change b's state.
        a.get_wind_velocity(0.0, np.array([0.0, 0.0, 10.0]))
        before = b._dryden_state.copy()
        a.get_wind_velocity(0.005, np.array([0.0, 0.0, 10.0]))
        np.testing.assert_array_equal(b._dryden_state, before)


class TestWindSpatialGradient:
    """Spatially varying wind for multi-drone operations."""

    def test_constant_with_east_gradient(self):
        from wind_model import WindField
        g = np.zeros((3, 3))
        g[0, 0] = 0.1  # +0.1 m/s of east-wind per metre east
        w = WindField(wind_speed=5.0,
                      wind_direction=np.array([1.0, 0.0, 0.0]),
                      turbulence_type="constant",
                      spatial_gradient=g)
        v0 = w.get_wind_velocity(0.0, np.array([0.0, 0.0, 10.0]))
        v1 = w.get_wind_velocity(0.0, np.array([100.0, 0.0, 10.0]))
        assert v0[0] == pytest.approx(5.0)
        assert v1[0] == pytest.approx(15.0)

    def test_no_gradient_unchanged(self):
        """Without spatial_gradient the result equals the base wind."""
        from wind_model import WindField
        w = WindField(wind_speed=3.0,
                      wind_direction=np.array([0.0, 1.0, 0.0]),
                      turbulence_type="constant")
        v = w.get_wind_velocity(0.0, np.array([1000.0, 0.0, 50.0]))
        np.testing.assert_allclose(v, [0.0, 3.0, 0.0])


class TestWindStressEnvelopes:
    """Each manifest profile completes a default mission and
    leaves a populated wind log in the SimRecord stream.

    These are intentionally soft envelopes: we verify the wind layer
    does what the manifest promises (correct base speed, deterministic
    Dryden, gradient applies) and that the existing controller still
    flies through it. Tighter aborts/RTL behavior needs PX4
    integration; Gazebo-runtime parity is a nightly-lane item.
    """

    @pytest.mark.parametrize("name", _wind_profile_names())
    def test_mission_runs_with_wind_in_loop(self, name):
        from drone_physics import run_simulation, DroneParams
        from wind_model import load_wind_profile

        wind = load_wind_profile(name)
        records = run_simulation(
            waypoints=_stress_mission(z=10.0),
            params=DroneParams(),
            dt=0.005,
            waypoint_radius=1.0,
            hover_time=0.5,
            max_time=40.0,
            wind=wind,
        )
        assert records, f"{name}: no records produced"
        # Wind logging: every record carries the ENU wind seen.
        assert all(r.wind_velocity is not None for r in records), (
            f"{name}: SimRecord.wind_velocity must be populated when wind is set"
        )
        # Cruise-window mean wind magnitude is near the expected base.
        cruise = [r for r in records if r.t >= 5.0]
        assert cruise, f"{name}: simulation ended before climb-up window"
        mean_mag = float(np.mean([np.linalg.norm(r.wind_velocity) for r in cruise]))
        expected = _PROFILE_BASE_SPEED_MS[name]
        assert mean_mag == pytest.approx(expected, rel=0.5, abs=0.5), (
            f"{name}: mean cruise wind {mean_mag:.2f} m/s vs expected "
            f"~{expected:.2f}"
        )

    def test_no_wind_records_have_none_wind(self):
        """Without a wind argument SimRecord.wind_velocity stays None."""
        from drone_physics import run_simulation, DroneParams
        records = run_simulation(
            waypoints=_stress_mission(z=10.0),
            params=DroneParams(), dt=0.02,
            waypoint_radius=1.0, hover_time=0.3, max_time=20.0,
        )
        assert records and all(r.wind_velocity is None for r in records)


# ──────────────────────────────────────────────────────────────────────────────
# Scenario matrix + acceptance report.
# ──────────────────────────────────────────────────────────────────────────────
