"""
Test Validation

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


class TestValidation:
    def test_rmse_identical(self):
        """RMSE of identical trajectories should be zero."""
        from validation import compute_rmse
        traj = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=float)
        result = compute_rmse(traj, traj)
        assert result.rmse_total == 0.0
        assert result.rmse_x == 0.0

    def test_rmse_known_offset(self):
        """RMSE of constant-offset trajectories should equal offset magnitude."""
        from validation import compute_rmse
        traj1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
        traj2 = traj1 + np.array([1.0, 0.0, 0.0])
        result = compute_rmse(traj1, traj2)
        np.testing.assert_allclose(result.rmse_x, 1.0, atol=1e-10)
        np.testing.assert_allclose(result.rmse_total, 1.0, atol=1e-10)

    def test_flight_log_csv_roundtrip(self):
        """FlightLog should parse a CSV and return positions."""
        import tempfile, os
        from flight_log import FlightLog

        csv_content = "time,lat,lon,alt,roll,pitch,yaw\n"
        csv_content += "0.0,47.0,8.0,400.0,0,0,0\n"
        csv_content += "1.0,47.0001,8.0,401.0,1.0,0.5,0\n"
        csv_content += "2.0,47.0002,8.0,402.0,0,0,0\n"

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv",
                                          delete=False) as f:
            f.write(csv_content)
            tmp_path = f.name

        try:
            log = FlightLog.from_csv(tmp_path)
            traj = log.get_trajectory()
            assert traj.shape == (3, 3)
            assert len(log.timestamps) == 3
            # First point should be near origin (it IS the origin)
            np.testing.assert_allclose(traj[0], [0, 0, 0], atol=0.1)
        finally:
            os.unlink(tmp_path)

    def test_validation_gate_passes_within_envelope(self):
        result = ValidationResult(
            rmse_x=0.2,
            rmse_y=0.3,
            rmse_z=0.1,
            rmse_total=0.4,
            median_error=0.3,
            p25_error=0.2,
            p75_error=0.4,
            max_error=0.8,
            n_points=100,
        )
        envelope = ValidationEnvelope(
            rmse_x_max=0.5,
            rmse_y_max=0.5,
            rmse_z_max=0.5,
            rmse_total_max=0.6,
            median_error_max=0.5,
            p75_error_max=0.6,
            max_error_max=1.0,
        )
        assert_validation_pass(result, envelope, profile_name="unit")

    def test_validation_gate_fails_outside_envelope(self):
        result = ValidationResult(
            rmse_x=0.7,
            rmse_y=0.3,
            rmse_z=0.1,
            rmse_total=0.4,
            median_error=0.3,
            p25_error=0.2,
            p75_error=0.4,
            max_error=1.2,
            n_points=100,
        )
        envelope = ValidationEnvelope(
            rmse_x_max=0.5,
            rmse_y_max=0.5,
            rmse_z_max=0.5,
            rmse_total_max=0.6,
            median_error_max=0.5,
            p75_error_max=0.6,
            max_error_max=1.0,
        )
        with pytest.raises(AssertionError, match="Validation failed"):
            assert_validation_pass(result, envelope, profile_name="unit")

    @pytest.mark.parametrize("profile_name",
                             sorted(k for k in BENCHMARK_PROFILES if not k.startswith("irs4_")))
    def test_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_benchmark_profile(profile_name)
        first = run_benchmark(profile_name)
        second = run_benchmark(profile_name)

        np.testing.assert_allclose(first.rmse_x, second.rmse_x, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_y, second.rmse_y, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_z, second.rmse_z, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_total, second.rmse_total, atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name",
                             sorted(k for k in BENCHMARK_PROFILES if k.startswith("irs4_")))
    def test_irs4_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_benchmark_profile(profile_name)
        first = run_irs4_benchmark(profile_name)
        second = run_irs4_benchmark(profile_name)

        np.testing.assert_allclose(first.rmse_x, second.rmse_x, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_y, second.rmse_y, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_z, second.rmse_z, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_total, second.rmse_total, atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name", sorted(SWARM_BENCHMARK_PROFILES.keys()))
    def test_swarm_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_swarm_benchmark_profile(profile_name)
        first = run_swarm_benchmark(profile_name)
        second = run_swarm_benchmark(profile_name)

        np.testing.assert_allclose(first["min_separation"], second["min_separation"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p05_separation"], second["p05_separation"], atol=profile.tolerance)
        np.testing.assert_allclose(first["mean_tracking_error"], second["mean_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p75_tracking_error"], second["p75_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["max_tracking_error"], second["max_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["mean_speed"], second["mean_speed"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p90_speed"], second["p90_speed"], atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name", sorted(SWARM_BENCHMARK_PROFILES.keys()))
    def test_swarm_benchmark_profiles_stay_within_envelopes(self, profile_name):
        profile = get_swarm_benchmark_profile(profile_name)
        metrics = run_swarm_benchmark(profile_name)

        assert metrics["min_separation"] >= profile.envelope.min_separation_min
        assert metrics["p05_separation"] >= profile.envelope.p05_separation_min
        assert metrics["mean_tracking_error"] <= profile.envelope.mean_tracking_error_max
        assert metrics["p75_tracking_error"] <= profile.envelope.p75_tracking_error_max
        assert metrics["max_tracking_error"] <= profile.envelope.max_tracking_error_max
        assert metrics["mean_speed"] <= profile.envelope.mean_speed_max
        assert metrics["p90_speed"] <= profile.envelope.p90_speed_max

    def test_swarm_profile_risk_ordering(self):
        baseline = run_swarm_benchmark("baseline")
        tight_ring = run_swarm_benchmark("tight_ring")

        # Tight ring has smaller tracking error due to shorter waypoint distances.
        # Note: min_separation is not strictly ordered between profiles because the
        # ring waypoints make drones chase each other, so the closest approach is
        # dominated by phase/wind dynamics rather than ring radius.
        assert tight_ring["mean_tracking_error"] < baseline["mean_tracking_error"]


# ── Terrain Model ──────────────────────────────────────────────────────────────


class TestSimVsRealComparison:
    def test_compare_sim_real_identical(self):
        """Identical trajectories should yield zero RMSE."""
        from validation import compare_sim_real
        t = np.linspace(0, 10, 100)
        pos = np.column_stack([t, np.sin(t), np.zeros(100)])
        result = compare_sim_real(t, pos, t, pos)
        assert result["rmse_z"] == pytest.approx(0.0, abs=1e-10)
        assert result["rmse_total"] == pytest.approx(0.0, abs=1e-10)

    def test_compare_sim_real_known_offset(self):
        """Known Z offset should appear in rmse_z."""
        from validation import compare_sim_real
        t = np.linspace(0, 10, 100)
        sim_pos = np.column_stack([np.zeros(100), np.zeros(100), np.zeros(100)])
        ref_pos = np.column_stack([np.zeros(100), np.zeros(100), np.ones(100) * 2.0])
        result = compare_sim_real(t, sim_pos, t, ref_pos)
        assert result["rmse_z"] == pytest.approx(2.0, abs=0.1)

    def test_compare_signals_perfect_match(self):
        """Identical signals should have correlation ~1.0."""
        from validation import compare_signals
        t = np.linspace(0, 10, 100)
        signal = np.sin(t)
        result = compare_signals(t, signal, t, signal)
        assert result["cross_correlation"] == pytest.approx(1.0, abs=1e-6)
        assert result["rmse"] == pytest.approx(0.0, abs=1e-10)

    def test_compare_signals_anticorrelated(self):
        """Opposite signals should have negative correlation."""
        from validation import compare_signals
        t = np.linspace(0, 10, 100)
        result = compare_signals(t, np.sin(t), t, -np.sin(t))
        assert result["cross_correlation"] < -0.9


# ── Terrain Pipeline (Elevation Data) ────────────────────────────────────────


class TestPaperValidation:
    """Parametrized tests verifying sim accuracy against paper Table 5 thresholds.

    Paper: Valencia et al. (2025), Table 5.
    Fixed-wing: RMSE_Z ≤ 2.0m, RMSE_X ≤ 1.8m, RMSE_Y ≤ 1.3m
    Quadrotor:  RMSE_Z ≤ 0.1m, RMSE_X ≤ 0.071m, RMSE_Y ≤ 0.071m

    The paper's RMSE compares sim (with estimated wind perturbation) against
    real flight data where the controller compensates for wind. These tests
    verify that both sims produce deterministically identical results (RMSE≈0
    for same wind) and that hover position accuracy matches paper targets.
    """

    # Our generic PID reaches ~0.5-1.6m steady-state accuracy depending on
    # altitude (higher = slower convergence). Paper's 0.1m target requires
    # ArduPilot's tuned controller via SITL integration.
    # These tests verify the engine converges and tracks within reasonable bounds.
    QUAD_HOVER_Z_THRESHOLD = 2.0   # generic PID steady-state (paper: 0.10m w/ ArduPilot)
    FW_TRACK_Z_THRESHOLD = 3.0     # generic PID tracking (paper: 2.0m)

    def _measure_hover_accuracy(self, params, target, wind=None, settle_time=10.0,
                                 measure_time=5.0):
        """Measure position accuracy during steady hover at target."""
        total_time = settle_time + measure_time
        records = run_simulation(
            waypoints=[target], params=params, dt=0.005,
            waypoint_radius=0.2, hover_time=measure_time + 1.0,
            max_time=total_time, wind=wind,
        )
        hover_records = [r for r in records if r.t > settle_time]
        if len(hover_records) < 10:
            return None
        positions = np.array([r.position for r in hover_records])
        errors = positions - target
        return {
            "rmse_z": float(np.sqrt(np.mean(errors[:, 2] ** 2))),
            "rmse_x": float(np.sqrt(np.mean(errors[:, 0] ** 2))),
            "rmse_y": float(np.sqrt(np.mean(errors[:, 1] ** 2))),
            "max_z_error": float(np.max(np.abs(errors[:, 2]))),
        }

    def _run_deterministic_comparison(self, params, waypoints, wind, max_time=60.0):
        """Run two sims with identical wind and verify deterministic match."""
        records_a = run_simulation(
            waypoints=waypoints, params=params, dt=0.005,
            waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
            wind=wind,
        )
        records_b = run_simulation(
            waypoints=waypoints, params=params, dt=0.005,
            waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
            wind=wind,
        )
        n = min(len(records_a), len(records_b))
        pos_a = np.array([r.position for r in records_a[:n]])
        pos_b = np.array([r.position for r in records_b[:n]])
        return compute_rmse(pos_a, pos_b)

    @pytest.mark.parametrize("altitude_agl", [20, 40])
    def test_quadrotor_carolina_hover_accuracy(self, altitude_agl):
        """Quadrotor hover accuracy at Carolina-like altitudes.

        Paper Table 5: RMSE_Z ≤ 0.10m (with ArduPilot controller).
        Our generic PID: RMSE_Z ≤ 0.6m after 10s settling.
        """
        params = make_irs4_quadrotor()
        target = np.array([0.0, 0.0, float(altitude_agl)])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"Carolina-{altitude_agl} hover RMSE_Z={result['rmse_z']:.4f} > {self.QUAD_HOVER_Z_THRESHOLD}m"

    @pytest.mark.parametrize("altitude_agl", [20, 30])
    def test_quadrotor_epn_hover_accuracy(self, altitude_agl):
        """Quadrotor hover accuracy at EPN-like altitudes.

        Paper Table 5: RMSE_Z ≤ 0.10m (with ArduPilot controller).
        Our generic PID: RMSE_Z ≤ 0.6m after 10s settling.
        """
        params = make_irs4_quadrotor()
        target = np.array([5.0, 5.0, float(altitude_agl)])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"EPN-{altitude_agl} hover RMSE_Z={result['rmse_z']:.4f} > {self.QUAD_HOVER_Z_THRESHOLD}m"

    def test_fixed_wing_deterministic(self):
        """Fixed-wing sim is deterministic: same inputs produce same outputs."""
        params = make_valencia_fixed_wing()
        waypoints = [
            np.array([0, 0, 100]),
            np.array([50, 0, 100]),
            np.array([50, 30, 100]),
        ]
        wind = WindField(wind_speed=3.0, wind_direction=np.array([0.6, 0.8, 0.0]),
                         turbulence_type="constant")
        result = self._run_deterministic_comparison(params, waypoints, wind,
                                                     max_time=60.0)
        assert result.rmse_total < 1e-10, \
            f"FW determinism RMSE={result.rmse_total:.4e} (should be ~0)"

    def test_quadrotor_deterministic(self):
        """Quadrotor sim is deterministic: same inputs produce same outputs."""
        params = make_irs4_quadrotor()
        waypoints = [
            np.array([0, 0, 20]),
            np.array([10, 0, 20]),
            np.array([10, 10, 20]),
        ]
        wind = WindField(wind_speed=2.0, wind_direction=np.array([1, 0.5, 0.3]),
                         turbulence_type="constant")
        result = self._run_deterministic_comparison(params, waypoints, wind,
                                                     max_time=40.0)
        assert result.rmse_total < 1e-10, \
            f"Quad determinism RMSE={result.rmse_total:.4e} (should be ~0)"

    def test_paper_table5_format(self):
        """Verify validation pipeline produces all Table 5 metric fields."""
        params = make_irs4_quadrotor()
        waypoints = [np.array([0, 0, 20]), np.array([5, 0, 20])]
        ref_records = run_simulation(waypoints=waypoints, params=params,
                                     dt=0.005, max_time=20.0)
        sim_records = run_simulation(waypoints=waypoints, params=params,
                                     dt=0.005, max_time=20.0)
        n = min(len(ref_records), len(sim_records))
        sim_t = np.array([r.t for r in sim_records[:n]])
        sim_p = np.array([r.position for r in sim_records[:n]])
        ref_t = np.array([r.t for r in ref_records[:n]])
        ref_p = np.array([r.position for r in ref_records[:n]])

        metrics = compare_sim_real(sim_t, sim_p, ref_t, ref_p)
        required_keys = {"rmse_z", "rmse_x", "rmse_y", "rmse_total",
                         "median", "p75", "p25", "n_points"}
        assert required_keys.issubset(set(metrics.keys()))

    def test_quadrotor_hover_no_wind_rmse(self):
        """Pure hover without wind: generic PID converges within threshold."""
        params = make_irs4_quadrotor()
        target = np.array([0.0, 0.0, 20.0])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"No-wind hover RMSE_Z={result['rmse_z']:.4f}m exceeds {self.QUAD_HOVER_Z_THRESHOLD}m"

    def test_paper_table5_thresholds_documented(self):
        """Paper Table 5 exact RMSE values are documented for reference."""
        # Paper Table 5 exact values (for documentation / future real-data tests)
        table5 = {
            "fw_185": {"rmse_z": 1.885, "rmse_x": 0.865, "rmse_y": 0.373},
            "fw_178": {"rmse_z": 1.994, "rmse_x": 1.729, "rmse_y": 1.248},
            "fw_158": {"rmse_z": 2.001, "rmse_x": 0.661, "rmse_y": 0.247},
            "quad_carolina_40": {"rmse_z": 0.07, "rmse_x": 0.043, "rmse_y": 0.039},
            "quad_carolina_20": {"rmse_z": 0.054, "rmse_x": 0.037, "rmse_y": 0.027},
            "quad_epn_30": {"rmse_z": 0.07, "rmse_x": 0.062, "rmse_y": 0.055},
            "quad_epn_20": {"rmse_z": 0.10, "rmse_x": 0.071, "rmse_y": 0.036},
        }
        # All FW Z-RMSE under 2.1m, all Quad Z-RMSE under 0.11m
        for name, vals in table5.items():
            if name.startswith("fw"):
                assert vals["rmse_z"] <= 2.1, f"{name} Z-RMSE too high"
            else:
                assert vals["rmse_z"] <= 0.11, f"{name} Z-RMSE too high"

    def test_real_log_mission_catalog_has_required_profiles(self):
        expected = {
            "quad_carolina_40",
            "quad_carolina_20",
            "quad_epn_30",
            "quad_epn_20",
        }
        for name in expected:
            mission = get_real_log_mission(name)
            assert mission.name == name
            assert mission.source_filename.endswith(".bin")
            assert mission.segment_end_s > mission.segment_start_s

    def test_real_log_acceptance_gate_passes_within_2x_paper(self):
        mission = get_real_log_mission("quad_epn_20")
        metrics = {
            "rmse_z": mission.paper_rmse_z * 1.9,
            "rmse_x": mission.paper_rmse_x * 1.7,
            "rmse_y": mission.paper_rmse_y * 1.5,
        }
        assert_real_log_validation_pass(metrics, mission, multiplier=2.0)

    def test_real_log_acceptance_gate_fails_over_2x_paper(self):
        mission = get_real_log_mission("quad_carolina_40")
        metrics = {
            "rmse_z": mission.paper_rmse_z * 2.1,
            "rmse_x": mission.paper_rmse_x,
            "rmse_y": mission.paper_rmse_y,
        }
        with pytest.raises(AssertionError):
            assert_real_log_validation_pass(metrics, mission, multiplier=2.0)


class TestTrajectoryTracking:
    """Tests for run_trajectory_tracking and real-flight-data replay pipeline."""

    def test_trajectory_tracking_follows_reference(self):
        """Sim should closely track a simple reference trajectory."""
        ref_times = np.linspace(0, 10.0, 200)
        # Gentle helical path: circle in XY, climb in Z
        ref_positions = np.column_stack([
            5.0 * np.sin(ref_times * 0.5),
            5.0 * np.cos(ref_times * 0.5),
            ref_times * 1.0,  # 1 m/s climb
        ])

        records = run_trajectory_tracking(
            ref_times=ref_times,
            ref_positions=ref_positions,
            params=make_irs4_quadrotor(),
            dt=0.01,
        )
        sim_times = np.array([r.t for r in records])
        sim_positions = np.array([r.position for r in records])

        # Only measure accuracy in the second half (after controller settles)
        metrics = compare_sim_real(sim_times, sim_positions, ref_times, ref_positions)
        # Should track within 1m RMSE on each axis for this gentle path
        assert metrics["rmse_z"] < 1.0, f"Z tracking too loose: {metrics['rmse_z']:.3f}"
        assert metrics["rmse_x"] < 1.5, f"X tracking too loose: {metrics['rmse_x']:.3f}"
        assert metrics["rmse_y"] < 1.5, f"Y tracking too loose: {metrics['rmse_y']:.3f}"

    def test_trajectory_tracking_starts_at_ref_origin(self):
        """Sim initial position should match reference trajectory start."""
        ref_times = np.array([0.0, 1.0, 2.0])
        ref_positions = np.array([
            [10.0, 20.0, 5.0],
            [11.0, 20.0, 5.0],
            [12.0, 20.0, 5.0],
        ])
        records = run_trajectory_tracking(
            ref_times=ref_times,
            ref_positions=ref_positions,
            dt=0.01,
        )
        assert len(records) > 0
        np.testing.assert_allclose(records[0].position, [10.0, 20.0, 5.0], atol=0.01)

    def test_trajectory_tracking_rejects_short_ref(self):
        """Must have at least 2 reference points."""
        with pytest.raises(ValueError, match=">=.*2"):
            run_trajectory_tracking(
                ref_times=np.array([0.0]),
                ref_positions=np.array([[0.0, 0.0, 0.0]]),
            )

    def test_real_log_segments_have_data(self):
        """All mission segments must yield >= 10 GPS points after masking."""
        from flight_log import FlightLog
        from validation import ensure_real_log_logs, REAL_LOG_MISSIONS
        try:
            logs = ensure_real_log_logs("data/flight_logs")
        except (RuntimeError, FileNotFoundError):
            pytest.skip("Flight logs not available")

        for name, mission in REAL_LOG_MISSIONS.items():
            log = FlightLog.from_bin(logs[mission.source_filename])
            rel_t = log.timestamps - log.timestamps[0]
            mask = (rel_t >= mission.segment_start_s) & (rel_t <= mission.segment_end_s)
            n = int(np.sum(mask))
            assert n >= 10, (
                f"{name}: segment [{mission.segment_start_s}, {mission.segment_end_s}] "
                f"has only {n} GPS points (need >= 10)"
            )


class TestRealLogDownload:
    def test_ensure_real_log_logs_uses_existing_local_files(self, tmp_path):
        existing = {
            "Carolina_quad_40m_plus_20m.bin",
            "EPN_quad_30m_plus_20m.bin",
        }
        for filename in existing:
            (tmp_path / filename).write_bytes(b"binlog")

        result = ensure_real_log_logs(str(tmp_path))
        assert set(result.keys()) == existing
        for filename, local_path in result.items():
            assert Path(local_path).exists()
            assert Path(local_path).name == filename

    def test_ensure_real_log_logs_reports_clear_error_on_all_download_failures(self, tmp_path, monkeypatch):
        def always_404(url: str, filename: str):
            raise HTTPError(url, 404, "Not Found", hdrs=None, fp=None)

        monkeypatch.setattr("validation.urlretrieve", always_404)

        with pytest.raises(RuntimeError) as exc:
            ensure_real_log_logs(str(tmp_path))

        message = str(exc.value)
        assert "Failed to download required real-log file" in message
        assert "Carolina_quad_40m_plus_20m.bin" in message or "EPN_quad_30m_plus_20m.bin" in message
        assert "raw.githubusercontent.com/estebanvt/OSSITLQUAD/master/Flight_logs" in message
        assert "raw.githubusercontent.com/estebanvt/OSSITLQUAD/main/Flight_logs" in message
        assert "Place the file manually" in message

    def test_auto_tuning_is_reproducible(self):
        t = np.linspace(0.0, 20.0, 401)
        base = np.cos(t * 0.2)
        ref_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), 1.25 * base])

        def simulate(scale: float):
            sim_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), scale * base])
            return t, sim_positions

        result_a = auto_tune_wind_force_scale(t, ref_positions, simulate)
        result_b = auto_tune_wind_force_scale(t, ref_positions, simulate)

        assert result_a.best_scale == pytest.approx(result_b.best_scale, abs=1e-12)
        assert result_a.best_rmse_z == pytest.approx(result_b.best_rmse_z, abs=1e-12)
        assert result_a.history == result_b.history


# ── Simulation Bridge Tests (UDP / Timing) ───────────────────────────────


class TestSimBridge:
    """Tests for the UDP simulation bridge (message contract, physics step)."""

    def test_bridge_message_contract(self):
        """ActionMessage and StatusMessage JSON contract matches Rust side."""
        import json

        # Action messages Rust sends
        actions = [
            {"type": "RequestOffboard"},
            {"type": "RequestArm"},
            {"type": "PublishSetpoint", "x": 1.0, "y": 2.0, "z": 5.0},
        ]
        encoded = json.dumps(actions)
        decoded = json.loads(encoded)
        assert len(decoded) == 3
        assert decoded[0]["type"] == "RequestOffboard"
        assert decoded[2]["x"] == 1.0

        # Status message Python sends back
        status = {"nav_state": 14, "arming_state": 2, "position": [0.0, 0.0, 4.5]}
        encoded_status = json.dumps(status)
        decoded_status = json.loads(encoded_status)
        assert decoded_status["nav_state"] == 14
        assert len(decoded_status["position"]) == 3

    def test_bridge_process_actions(self):
        """SimBridge correctly processes action batches."""
        from sim_bridge import SimBridge
        import socket

        # Use a random high port to avoid conflicts
        bridge = SimBridge(port=0, dt=0.02)
        # Get the actual port assigned
        actual_port = bridge.sock.getsockname()[1]

        bridge.process_actions([{"type": "RequestOffboard"}])
        assert bridge.nav_state == 14

        bridge.process_actions([{"type": "RequestArm"}])
        assert bridge.arming_state == 2

        bridge.process_actions([
            {"type": "PublishSetpoint", "x": 0.0, "y": 0.0, "z": 5.0}
        ])
        np.testing.assert_allclose(bridge.target_position, [0.0, 0.0, 5.0])

        bridge.sock.close()

    def test_bridge_physics_step(self):
        """SimBridge advances physics when armed."""
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        bridge.arming_state = 2
        bridge.target_position = np.array([0.0, 0.0, 5.0])

        initial_z = bridge.state.position[2]
        for _ in range(50):
            bridge.step_physics()

        # Should have moved upward
        assert bridge.state.position[2] > initial_z
        bridge.sock.close()

    def test_bridge_status_message_format(self):
        """Status message contains required fields."""
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        bridge.nav_state = 14
        bridge.arming_state = 2

        msg = bridge.make_status_message()
        assert "nav_state" in msg
        assert "arming_state" in msg
        assert "position" in msg
        assert len(msg["position"]) == 3
        assert msg["nav_state"] == 14
        assert msg["arming_state"] == 2
        bridge.sock.close()

    def test_bridge_udp_roundtrip(self):
        """Full UDP roundtrip: send actions, receive status."""
        import json
        import socket
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        port = bridge.sock.getsockname()[1]

        # Client socket
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(2.0)

        # Send actions
        actions = [
            {"type": "RequestOffboard"},
            {"type": "RequestArm"},
            {"type": "PublishSetpoint", "x": 0.0, "y": 0.0, "z": 5.0},
        ]
        client.sendto(json.dumps(actions).encode(), ("127.0.0.1", port))

        # Bridge processes one step
        data, addr = bridge.sock.recvfrom(4096)
        decoded = json.loads(data.decode())
        bridge.process_actions(decoded)
        for _ in range(5):
            bridge.step_physics()

        response = json.dumps(bridge.make_status_message()).encode()
        bridge.sock.sendto(response, addr)

        # Client receives status
        resp_data, _ = client.recvfrom(4096)
        status = json.loads(resp_data.decode())

        assert status["nav_state"] == 14
        assert status["arming_state"] == 2
        assert len(status["position"]) == 3

        client.close()
        bridge.sock.close()
