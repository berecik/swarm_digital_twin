"""
Tests: Acceptance Matrix

Split from `test_drone_physics.py` so the per-domain test surface is
navigable. Helpers live in `_test_common.py`.
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

from _test_common import (
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


# Aliases: keep the underscore-prefixed call sites that the test bodies
# carry over from the original `test_drone_physics.py`.
_LIVE_JS = (Path(__file__).resolve().parent
            / "runtime_view" / "web" / "live.js")
_live_js_source = live_js_source
_parity_entry_names = parity_entry_names
_ramp_terrain = ramp_terrain
_regression_mission = regression_mission
_wind_profile_names = wind_profile_names_safe
_stress_mission = stress_mission
_PROFILE_BASE_SPEED_MS = PROFILE_BASE_SPEED_MS


class TestPhysicsParity:
    """Tests for physics parity contract."""

    def test_sdf_parameter_match(self):
        """X500 SDF model parameters must match DroneParams exactly."""
        from physics_parity import ParityContract
        sdf_path = str(Path(__file__).resolve().parents[1]
                       / "gazebo" / "models" / "x500" / "model.sdf")
        if not Path(sdf_path).is_file():
            pytest.skip("gazebo/models/x500/model.sdf not found")
        contract = ParityContract.from_sdf(sdf_path)
        assert contract.passed, contract.summary()

    def test_trajectory_comparison_identical(self):
        """Identical trajectories must produce RMSE=0 and pass."""
        from physics_parity import compare_trajectories
        pos = np.array([[0, 0, 5], [1, 0, 5], [2, 0, 5]], dtype=float)
        result = compare_trajectories(pos, pos)
        assert result.passed
        assert result.position_rmse_xy == pytest.approx(0.0, abs=1e-9)
        assert result.position_rmse_z == pytest.approx(0.0, abs=1e-9)

    def test_trajectory_comparison_within_threshold(self):
        """Small deviations within threshold must pass."""
        from physics_parity import compare_trajectories
        pos_a = np.array([[0, 0, 5], [10, 0, 5], [20, 0, 5]], dtype=float)
        pos_b = pos_a + np.array([0.5, 0.3, 0.2])  # small offset
        result = compare_trajectories(pos_a, pos_b)
        assert result.passed
        assert result.position_rmse_xy < 2.0
        assert result.position_rmse_z < 1.0

    def test_trajectory_comparison_exceeds_threshold(self):
        """Large deviations must fail."""
        from physics_parity import compare_trajectories
        pos_a = np.array([[0, 0, 5], [10, 0, 5]], dtype=float)
        pos_b = np.array([[0, 0, 5], [10, 0, 10]], dtype=float)  # 5m Z error
        result = compare_trajectories(pos_a, pos_b)
        assert not result.passed

    def test_timing_determinism_stable(self):
        """Evenly spaced timestamps must pass jitter check."""
        from physics_parity import check_timing_determinism
        dt = 0.02
        t = np.arange(0, 5.0, dt)
        result = check_timing_determinism(t, expected_dt=dt)
        assert result.passed
        assert result.max_jitter_ms < 0.01

    def test_timing_determinism_jittery(self):
        """Irregular timestamps must fail jitter check."""
        from physics_parity import check_timing_determinism
        t = np.cumsum(np.random.uniform(0.01, 0.05, 100))
        result = check_timing_determinism(t, expected_dt=0.02, max_jitter_ms=1.0)
        assert not result.passed

    def test_truth_record_extraction(self):
        """extract_truth_from_records produces valid truth records."""
        from physics_parity import extract_truth_from_records
        from physics_live_replay import run_physics_simulation
        records = run_physics_simulation(max_time=1.0)
        truth = extract_truth_from_records(records, source="standalone")
        assert len(truth) == len(records)
        assert truth[0].source == "standalone"
        assert truth[0].position.shape == (3,)

    def test_truth_csv_roundtrip(self, tmp_path):
        """save_truth_csv writes a valid CSV file."""
        import csv
        from physics_parity import extract_truth_from_records, save_truth_csv
        from physics_live_replay import run_physics_simulation
        records = run_physics_simulation(max_time=0.5)
        truth = extract_truth_from_records(records)
        csv_path = str(tmp_path / "truth.csv")
        save_truth_csv(truth, csv_path)
        with open(csv_path, encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
        assert "t" in header
        assert "source" in header
        assert len(rows) == len(records)

    def test_parity_contract_summary(self):
        """ParityContract.summary() is printable."""
        from physics_parity import ParityContract
        sdf_path = str(Path(__file__).resolve().parents[1]
                       / "gazebo" / "models" / "x500" / "model.sdf")
        if not Path(sdf_path).is_file():
            pytest.skip("SDF not found")
        contract = ParityContract.from_sdf(sdf_path)
        s = contract.summary()
        assert "ParityContract" in s
        assert len(s) > 10

    @pytest.mark.timeout(15)
    def test_k8s_ref_coordinates_roundtrip(self):
        """GPS→ENU with matching ref produces ENU near [0,0,0] for the ref point.

        This verifies that when SITL_REF_LAT/LNG/ALT match the SITL
        origin, the GPS→ENU conversion produces coordinates near zero
        (not thousands of km away as with mismatched refs).
        """
        import time
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue
        from mavlink_bridge import MAVLinkBridge, SimState

        # Simulate K8s SITL at Quito coordinates
        quito_lat, quito_lon, quito_alt = -0.508333, -78.141667, 4500.0

        q = TelemetryQueue(maxlen=128)
        src = MAVLinkLiveSource(
            listen_ip='127.0.0.1', listen_port=0, queue=q,
            ref_lat=quito_lat, ref_lon=quito_lon, ref_alt_msl=quito_alt,
        )
        src.start()
        port = src._sock.getsockname()[1]

        # Bridge sends from ENU [5, 10, 20] with matching Quito ref
        bridge = MAVLinkBridge(
            target_ip='127.0.0.1', target_port=port, listen_port=0,
        )
        bridge.start()
        try:
            state = SimState(
                time_s=1.0,
                position=np.array([5.0, 10.0, 20.0]),
                ref_lat=quito_lat, ref_lon=quito_lon, ref_alt_msl=quito_alt,
            )
            bridge.send_state(state)
            time.sleep(0.5)

            assert len(q) >= 1, "No samples received"
            sample = q.latest()
            # With matching refs, pos_enu should be near [5, 10, 20]
            np.testing.assert_allclose(
                sample.pos_enu, [5.0, 10.0, 20.0], atol=0.5,
                err_msg="GPS roundtrip with matching refs should preserve ENU")
        finally:
            bridge.stop()
            src.stop()

    @pytest.mark.timeout(15)
    def test_mismatched_ref_produces_large_offset(self):
        """GPS→ENU with MISMATCHED refs produces enormous offsets.

        This is the bug that the ref-coordinate fix prevents.
        """
        import time
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue
        from mavlink_bridge import MAVLinkBridge, SimState

        q = TelemetryQueue(maxlen=128)
        # Receiver uses Zurich ref (default)
        src = MAVLinkLiveSource(
            listen_ip='127.0.0.1', listen_port=0, queue=q,
            ref_lat=47.3769, ref_lon=8.5417, ref_alt_msl=408.0,
        )
        src.start()
        port = src._sock.getsockname()[1]

        # Bridge sends from ENU [0, 0, 5] but with QUITO ref
        bridge = MAVLinkBridge(
            target_ip='127.0.0.1', target_port=port, listen_port=0,
        )
        bridge.start()
        try:
            state = SimState(
                time_s=1.0,
                position=np.array([0.0, 0.0, 5.0]),
                ref_lat=-0.508333, ref_lon=-78.141667, ref_alt_msl=4500.0,
            )
            bridge.send_state(state)
            time.sleep(0.5)

            assert len(q) >= 1
            sample = q.latest()
            # With mismatched refs (Quito sender, Zurich receiver), the
            # ENU position should be thousands of km away — not near [0,0,5]
            offset = np.linalg.norm(sample.pos_enu)
            assert offset > 1000.0, (
                f"Expected large offset with mismatched refs, got {offset:.1f} m")
        finally:
            bridge.stop()
            src.stop()

    def test_run_scenario_exports_sitl_ref_for_k8s(self):
        """run_single_mission_live must export SITL_REF_LAT/LNG/ALT."""
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')
        assert 'SITL_REF_LAT' in body
        assert 'SITL_REF_LNG' in body
        assert 'SITL_REF_ALT' in body

    def test_server_cli_accepts_ref_args(self):
        """runtime_view.server --help must list --ref-lat/--ref-lon/--ref-alt."""
        import subprocess, sys as _sys
        sim_dir = str(Path(__file__).resolve().parent)
        result = subprocess.run(
            [_sys.executable, '-m', 'runtime_view.server', '--help'],
            capture_output=True, text=True, timeout=10, cwd=sim_dir,
        )
        assert result.returncode == 0
        for flag in ('--ref-lat', '--ref-lon', '--ref-alt'):
            assert flag in result.stdout, f"{flag} missing from --help"


# ──────────────────────────────────────────────────────────────────────────────
# Terrain manifest + export-roundtrip parity gate.
#
# Contract: TerrainMap.get_elevation(x, y) must agree with itself when the
# same source is round-tripped through the STL pipeline that ships to
# Gazebo. Catches the realistic failure mode of "we trained the controller
# on heights X but Gazebo flies over heights Y because the export lost
# detail."
# ──────────────────────────────────────────────────────────────────────────────

PARITY_MAX_DELTA_M = 0.5
PARITY_SAMPLE_COUNT = 100
PARITY_RNG_SEED = 42


def _parity_entry_names() -> list:
    """Manifest entries to exercise; safe to call at pytest collection time."""
    from terrain import manifest_entries
    try:
        return manifest_entries()
    except FileNotFoundError:  # pragma: no cover - manifest is in-tree
        return []


class TestScenarioMatrix:
    """Matrix generator + CI subset selector."""

    def test_full_matrix_count(self):
        from scenario_matrix import full_matrix
        # 4 drones × 3 terrains × 4 winds × 4 missions × 5 faults = 960.
        assert len(full_matrix()) == 960

    def test_ci_subset_count(self):
        from scenario_matrix import ci_subset
        # The 4 diagonal rows + 16 critical-path rows = 20.
        assert len(ci_subset()) == 20

    def test_ci_subset_covers_every_dimension_value(self):
        from scenario_matrix import (
            DRONE_COUNTS, FAULTS, MISSIONS, TERRAINS, WINDS, ci_subset,
        )
        rows = ci_subset()
        assert {r.drones for r in rows} == set(DRONE_COUNTS)
        assert {r.terrain for r in rows} == set(TERRAINS)
        assert {r.wind for r in rows} == set(WINDS)
        assert {r.mission for r in rows} == set(MISSIONS)
        assert {r.fault for r in rows} == set(FAULTS)

    def test_scenario_id_is_filesystem_friendly(self):
        from scenario_matrix import ScenarioConfig
        c = ScenarioConfig(6, "synthetic_rolling", "gusty", "patrol", "none")
        assert c.scenario_id == "06drone-synthetic_rolling-gusty-patrol-none"
        # No characters that need escaping in filesystem paths.
        assert "/" not in c.scenario_id and " " not in c.scenario_id

    def test_select_unknown_subset_raises(self):
        from scenario_matrix import select
        with pytest.raises(ValueError, match="unknown subset"):
            select("does_not_exist")


class TestMissionFactory:
    """Mission kind → per-drone waypoint dict."""

    @pytest.mark.parametrize("kind", ["patrol", "lawnmower", "escort", "heavy_lift"])
    @pytest.mark.parametrize("n", [1, 3, 6, 12])
    def test_each_drone_gets_a_waypoint_list(self, kind, n):
        from missions import build_mission
        wps = build_mission(kind, n)
        assert len(wps) == n
        for did, lst in wps.items():
            assert lst, f"{kind}/{n}: drone {did} has no waypoints"
            for p in lst:
                assert p.shape == (3,)

    def test_unknown_mission_raises(self):
        from missions import build_mission
        with pytest.raises(ValueError, match="unknown mission"):
            build_mission("does_not_exist", 1)

    @pytest.mark.parametrize("kind", ["patrol", "escort", "heavy_lift"])
    def test_initial_separation_above_floor(self, kind):
        """At t=0, every pair of drones must be above the 1.5 m floor."""
        from missions import build_mission
        wps = build_mission(kind, 12)
        starts = np.array([wps[i + 1][0] for i in range(12)])
        for i in range(12):
            for j in range(i + 1, 12):
                d = float(np.linalg.norm(starts[i] - starts[j]))
                assert d >= 1.5, (
                    f"{kind}: drones {i+1}/{j+1} start {d:.2f} m apart"
                )


class TestAcceptanceReport:
    """KPI computation, verdict, report writer."""

    def test_single_drone_calm_passes(self, tmp_path):
        """The lightest scenario must pass the Python-pipeline gates."""
        from acceptance_report import run_scenario, write_report
        from scenario_matrix import ScenarioConfig
        cfg = ScenarioConfig(1, "flat", "calm", "patrol", "none")
        kpis = run_scenario(cfg, max_time=90.0)
        assert kpis.verdict == "PASS", kpis.failures
        assert kpis.mission_completion_rate == pytest.approx(1.0)

        out = write_report(cfg, kpis, tmp_path)
        assert (out / "kpis.json").is_file()
        assert (out / "summary.md").is_file()
        assert (out / "config.toml").is_file()

    def test_kpis_json_matches_documented_schema(self, tmp_path):
        """`kpis.json` must contain every documented field."""
        import json
        from acceptance_report import run_scenario, write_report
        from scenario_matrix import ScenarioConfig
        cfg = ScenarioConfig(1, "flat", "calm", "patrol", "none")
        kpis = run_scenario(cfg, max_time=90.0)
        out = write_report(cfg, kpis, tmp_path)
        data = json.loads((out / "kpis.json").read_text(encoding="utf-8"))
        for required in (
            "scenario_id", "git_revision", "verdict",
            "mission_completion_rate",
            "mean_separation_m", "min_separation_m",
            "collision_count", "near_miss_count",
            "trajectory_rmse_xy_m", "trajectory_rmse_z_m",
            "agl_violation_count", "clearance_violation_count",
            "min_agl_m", "mean_agl_m",
            "max_roll_deg", "max_pitch_deg",
            "failover_recovery_s", "control_jitter_ms",
        ):
            assert required in data, f"missing field: {required}"

    def test_separation_min_and_mean_are_not_aliased(self):
        from acceptance_report import compute_kpis
        from drone_physics import SimRecord
        from scenario_matrix import ScenarioConfig
        from safety import SeparationMonitor, TerrainMonitor
        cfg = ScenarioConfig(2, "flat", "calm", "patrol", "none")
        sep = SeparationMonitor(min_separation=1.5)
        sep.check({1: np.array([0.0, 0.0, 0.0]), 2: np.array([2.0, 0.0, 0.0])}, 0.0)
        sep.check({1: np.array([0.0, 0.0, 0.0]), 2: np.array([4.0, 0.0, 0.0])}, 1.0)
        ter = TerrainMonitor(TerrainMap.flat(), min_agl=2.0)
        recs = {
            1: [SimRecord(0.0, np.array([0.0, 0.0, 10.0]), np.zeros(3), (0.0, 0.0, 0.0), 0.0, np.zeros(3))],
            2: [SimRecord(0.0, np.array([4.0, 0.0, 10.0]), np.zeros(3), (0.0, 0.0, 0.0), 0.0, np.zeros(3))],
        }
        wps = {1: [np.array([0.0, 0.0, 10.0])], 2: [np.array([4.0, 0.0, 10.0])]}
        kpis = compute_kpis(cfg, recs, wps, sep, ter)
        assert kpis.min_separation_m == pytest.approx(2.0)
        assert kpis.mean_separation_m == pytest.approx(3.0)

    def test_storm_attitude_hard_gate_fails(self):
        from acceptance_report import compute_kpis
        from drone_physics import SimRecord
        from scenario_matrix import ScenarioConfig
        from safety import SeparationMonitor, TerrainMonitor
        cfg = ScenarioConfig(1, "flat", "storm", "patrol", "none")
        sep = SeparationMonitor(min_separation=1.5)
        ter = TerrainMonitor(TerrainMap.flat(), min_agl=2.0)
        recs = {
            1: [
                SimRecord(0.0, np.array([0.0, 0.0, 10.0]), np.zeros(3), (np.radians(50.0), 0.0, 0.0), 0.0, np.zeros(3)),
                SimRecord(6.0, np.array([0.0, 0.0, 10.0]), np.zeros(3), (0.0, 0.0, 0.0), 0.0, np.zeros(3)),
            ]
        }
        wps = {1: [np.array([0.0, 0.0, 10.0])]}
        kpis = compute_kpis(cfg, recs, wps, sep, ter)
        assert kpis.max_roll_deg > 40.0
        assert kpis.max_pitch_deg == pytest.approx(0.0)

    def test_verdict_fails_on_completion_below_target(self, tmp_path):
        """Force a failure by giving the drone too little time.

        Uses lawnmower (last waypoint != first) so the drone can't satisfy
        the completion check just by being near the start.
        """
        from acceptance_report import run_scenario
        from scenario_matrix import ScenarioConfig
        cfg = ScenarioConfig(1, "flat", "calm", "lawnmower", "none")
        kpis = run_scenario(cfg, max_time=2.0)  # not enough to complete
        assert kpis.verdict == "FAIL"
        assert any("completion" in f for f in kpis.failures)


class TestPhase6Smoke:
    """End-to-end: run the 1-drone diagonals from the CI subset."""

    def test_one_drone_subset_passes(self, tmp_path):
        from acceptance_report import run_scenario, write_report
        from scenario_matrix import ci_subset

        single_drone = [c for c in ci_subset() if c.drones == 1]
        assert single_drone, "no 1-drone scenarios in the CI subset"
        for cfg in single_drone:
            kpis = run_scenario(cfg, max_time=90.0)
            write_report(cfg, kpis, tmp_path)
            assert kpis.verdict == "PASS", (
                f"{cfg.scenario_id} failed: {kpis.failures}"
            )


# ──────────────────────────────────────────────────────────────────────────────
# Live view & replay backlog.
# ──────────────────────────────────────────────────────────────────────────────


_LIVE_JS = (Path(__file__).resolve().parent
            / "runtime_view" / "web" / "live.js")


def _live_js_source() -> str:
    return _LIVE_JS.read_text(encoding="utf-8")


class TestGazeboWindPluginEmulator:
    """File-level parity vs Gazebo's wind-plugin algorithm."""

    @pytest.mark.parametrize("name", ["calm", "crosswind"])
    def test_constant_profile_matches_gazebo_emulator(self, name):
        from gz_wind_plugin_emulator import parity_samples
        from wind_model import load_wind_profile
        wind = load_wind_profile(name)
        _, max_delta, rmse = parity_samples(wind, n=100)
        assert max_delta < 1e-9, (
            f"{name}: max |Δv| {max_delta:.6f} m/s > 0 vs Gazebo emulator "
            f"(rmse={rmse:.6f} m/s)"
        )

    def test_dryden_parity_helper_rejects_dryden(self):
        from gz_wind_plugin_emulator import parity_samples
        from wind_model import load_wind_profile
        with pytest.raises(ValueError, match="constant \\+ gradient profile only"):
            parity_samples(load_wind_profile("gusty"), n=10)


class TestCruiseAttitudeGate:
    """Hard gate on cruise-window median attitude."""

    def test_cruise_p50_replaces_max_for_verdict(self):
        """The verdict must use cruise_p50_*, not the noisy cruise_max_*."""
        from acceptance_report import (
            CRUISE_WINDOW_START_S, compute_kpis,
        )
        from drone_physics import SimRecord
        from scenario_matrix import ScenarioConfig
        from safety import SeparationMonitor, TerrainMonitor
        cfg = ScenarioConfig(1, "flat", "calm", "patrol", "none")
        sep = SeparationMonitor(min_separation=1.5)
        ter = TerrainMonitor(TerrainMap.flat(), min_agl=2.0)
        recs = {
            1: [
                SimRecord(
                    t=CRUISE_WINDOW_START_S + i,
                    position=np.array([0.0, 0.0, 10.0]),
                    velocity=np.zeros(3),
                    euler=(np.radians(5.0), 0.0, 0.0),
                    thrust=0.0,
                    angular_velocity=np.zeros(3),
                )
                for i in range(10)
            ] + [
                SimRecord(
                    t=CRUISE_WINDOW_START_S + 11,
                    position=np.array([0.0, 0.0, 10.0]),
                    velocity=np.zeros(3),
                    euler=(np.radians(80.0), 0.0, 0.0),
                    thrust=0.0,
                    angular_velocity=np.zeros(3),
                )
            ],
        }
        wps = {1: [np.array([0.0, 0.0, 10.0])]}
        kpis = compute_kpis(cfg, recs, wps, sep, ter)
        assert kpis.cruise_p50_roll_deg == pytest.approx(5.0, abs=0.1)
        assert kpis.cruise_max_roll_deg == pytest.approx(80.0, abs=0.1)
        # 5° < 15° calm gate → no attitude failure on the verdict
        assert not any("cruise_p50_attitude" in f for f in kpis.failures), \
            kpis.failures

    def test_attitude_envelope_constants_present(self):
        """Both Python and K8s threshold tables must cover all wind classes."""
        from acceptance_report import (
            WIND_ATTITUDE_LIMITS_DEG, WIND_ATTITUDE_LIMITS_DEG_K8S,
        )
        for name in ("calm", "crosswind", "gusty", "storm"):
            assert name in WIND_ATTITUDE_LIMITS_DEG
            assert name in WIND_ATTITUDE_LIMITS_DEG_K8S
            # K8s must always be at least as strict as the Python pipeline.
            assert (WIND_ATTITUDE_LIMITS_DEG_K8S[name]
                    <= WIND_ATTITUDE_LIMITS_DEG[name]), name


class TestFaultInjection:
    """In-process versions of the K8s fault classes."""

    def _records(self) -> list:
        from drone_physics import SimRecord
        return [
            SimRecord(
                t=0.02 * i,
                position=np.array([0.0, 0.0, 10.0]),
                velocity=np.array([1.0, 0.0, 0.0]),
                euler=(0.0, 0.0, 0.0),
                thrust=0.0,
                angular_velocity=np.zeros(3),
            )
            for i in range(800)  # 16 s of telemetry
        ]

    def test_unknown_fault_kind_raises(self):
        from acceptance_report import apply_fault
        with pytest.raises(ValueError, match="unknown fault"):
            apply_fault("does_not_exist", self._records())

    def test_pod_restart_drops_records_in_window(self):
        from acceptance_report import (
            FAULT_INJECT_AT_S, apply_fault,
        )
        recs = self._records()
        out, injected_at, detected_at = apply_fault("pod_restart", recs)
        assert injected_at == FAULT_INJECT_AT_S
        for r in out:
            assert not (FAULT_INJECT_AT_S <= r.t < FAULT_INJECT_AT_S + 5.0)
        assert detected_at == pytest.approx(FAULT_INJECT_AT_S, abs=0.05)

    def test_packet_loss_drops_about_10_percent(self):
        from acceptance_report import apply_fault
        recs = self._records()
        out, injected_at, _ = apply_fault("packet_loss", recs, seed=42)
        post_in = sum(1 for r in recs if r.t >= injected_at)
        post_out = sum(1 for r in out if r.t >= injected_at)
        ratio = (post_in - post_out) / post_in
        assert 0.05 <= ratio <= 0.20, ratio

    def test_telemetry_delay_shifts_timestamps(self):
        from acceptance_report import apply_fault
        recs = self._records()
        out, _, _ = apply_fault("telemetry_delay", recs)
        for r_in, r_out in zip(recs, out):
            if r_in.t < 8.0:
                assert r_out.t == pytest.approx(r_in.t)
            else:
                assert r_out.t == pytest.approx(r_in.t + 0.2)

    def test_sensor_dropout_zeros_velocity(self):
        from acceptance_report import apply_fault
        recs = self._records()
        out, _, _ = apply_fault("sensor_dropout", recs)
        for r_in, r_out in zip(recs, out):
            if r_in.t < 8.0:
                np.testing.assert_array_equal(r_out.velocity, r_in.velocity)
            else:
                np.testing.assert_array_equal(r_out.velocity, np.zeros(3))


class TestScalabilityTiming:
    """setup_time_s + sim_wall_time_s in kpis.json."""

    def test_kpis_carry_timing_fields(self, tmp_path):
        import json
        from acceptance_report import run_scenario, write_report
        from scenario_matrix import ScenarioConfig
        cfg = ScenarioConfig(1, "flat", "calm", "patrol", "none")
        kpis = run_scenario(cfg, max_time=60.0)
        out = write_report(cfg, kpis, tmp_path)
        data = json.loads((out / "kpis.json").read_text(encoding="utf-8"))
        for k in ("setup_time_s", "sim_wall_time_s", "records_per_drone"):
            assert k in data, f"missing field: {k}"
        assert data["setup_time_s"] >= 0
        assert data["sim_wall_time_s"] >= 0
        assert data["records_per_drone"] > 0

    def test_swarm_setup_takes_longer_than_single(self):
        """A 3-drone setup must measure > a 1-drone setup (sanity check)."""
        from acceptance_report import run_scenario
        from scenario_matrix import ScenarioConfig
        single = run_scenario(
            ScenarioConfig(1, "flat", "calm", "patrol", "none"), max_time=15.0)
        swarm = run_scenario(
            ScenarioConfig(3, "flat", "calm", "patrol", "none"), max_time=15.0)
        assert swarm.sim_wall_time_s > single.sim_wall_time_s


# ──────────────────────────────────────────────────────────────────────────────
# Safety response state machine.
# ──────────────────────────────────────────────────────────────────────────────
