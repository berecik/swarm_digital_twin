"""
Test Replay

Auto-split from `test_runtime_view.py` into a focused per-domain test file.
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

from test_runtime_view._helpers import (
    _runtime_view_start_uvicorn,
    _runtime_view_stop_uvicorn,
)
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


class TestPostFlightReplay:
    """Tests for post-flight replay in the web viewer (roadmap item 3):
    --replay-live shell mode, /api/files, /api/load endpoints."""

    def test_run_scenario_replay_live_in_help(self):
        """--replay-live must appear in run_scenario.sh --help."""
        import subprocess, sys as _sys
        script = str(PROJECT_ROOT / 'run_scenario.sh')
        result = subprocess.run(
            ['bash', script, '--help'],
            capture_output=True, text=True, timeout=10,
        )
        assert '--replay-live' in result.stdout

    def test_run_scenario_replay_live_branch_exists(self):
        """run_scenario.sh must have a --replay-live case branch."""
        import re
        script = PROJECT_ROOT / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')
        m = re.search(r'\n\s*--replay-live\)\s*\n(?P<body>.*?);;\s*\n',
                       body, re.DOTALL)
        assert m is not None, '--replay-live branch not found'
        branch = m.group('body')
        assert 'run_physics_live' in branch
        assert '--replay' in branch

    def test_api_files_lists_npz(self):
        """GET /api/files must return at least one .npz file."""
        import urllib.request, json
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/files'
            with urllib.request.urlopen(url, timeout=5) as resp:
                data = json.loads(resp.read())
            assert isinstance(data, list)
            npz_files = [f for f in data if f['type'] == 'npz']
            assert len(npz_files) >= 1, (
                f'Expected at least one .npz file, got: {data}')
            assert 'scenario_data.npz' in [f['name'] for f in npz_files]
        finally:
            _runtime_view_stop_uvicorn(server, thread)

    @pytest.mark.timeout(30)
    def test_api_load_starts_replay(self):
        """POST /api/load with a valid .npz starts a replay."""
        import urllib.request, urllib.parse, json, time
        from live_telemetry import TelemetryQueue, MAVLinkLiveSource

        q = TelemetryQueue(maxlen=512)
        server, thread, base = _runtime_view_start_uvicorn(q)

        import runtime_view.server as srv
        src = MAVLinkLiveSource(listen_ip='127.0.0.1', listen_port=0, queue=q)
        src.start()
        srv.live_source = src

        try:
            npz_path = str(SIM_DIR / 'scenario_data.npz')
            url = f'{base}/api/load?path={urllib.parse.quote(npz_path)}'
            req = urllib.request.Request(url, method='POST')
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read())

            assert data['status'] == 'playing'
            assert data['records'] > 0

            deadline = time.time() + 5.0
            while len(q) < 3 and time.time() < deadline:
                time.sleep(0.1)
            assert len(q) >= 1, f'No samples after /api/load, got {len(q)}'
        finally:
            if srv._replay_bridge is not None:
                srv._replay_bridge.stop()
                srv._replay_bridge = None
            try:
                src.stop()
            except Exception:
                pass
            srv.live_source = None
            _runtime_view_stop_uvicorn(server, thread)

    def test_api_load_rejects_missing_file(self):
        """POST /api/load with a nonexistent path returns 404."""
        import urllib.request, urllib.error, urllib.parse
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/load?path={urllib.parse.quote("/nonexistent/file.npz")}'
            req = urllib.request.Request(url, method='POST')
            try:
                urllib.request.urlopen(req, timeout=5)
                assert False, 'Expected HTTP error'
            except urllib.error.HTTPError as e:
                assert e.code in (404, 422), f'Expected 404, got {e.code}'
        finally:
            _runtime_view_stop_uvicorn(server, thread)


class TestBinReplay:
    """POST /api/load accepts ArduPilot DataFlash .BIN logs."""

    def _write_synthetic_bin(self, path: Path) -> None:
        """Write a tiny .BIN log with ATT + 5 GPS samples."""
        import math
        from dataflash_recorder import DataFlashRecorder
        rec = DataFlashRecorder(str(path))
        # Recorder takes ATT angles in degrees; the parser converts back to
        # radians. Use a non-trivial roll so the parity check is meaningful.
        roll_deg = math.degrees(0.05)
        rec.write_att(0.005, roll_deg, 0.0, 0.0)
        for i in range(5):
            rec.write_gps(
                t_wall=0.01 + i * 0.05,
                lat=47.3769 + 1e-5 * i,
                lon=8.5417 + 1e-5 * i,
                alt=408.0 + i,
                speed=1.5,
                course=45.0,
                vz=-0.5,
                n_sats=12,
            )
        rec.close()

    def test_load_bin_records_parses_synthetic_log(self, tmp_path):
        from physics_live_replay import load_bin_records
        bin_path = tmp_path / "syn.bin"
        self._write_synthetic_bin(bin_path)
        records = load_bin_records(str(bin_path))
        assert len(records) == 5
        # ENU position is relative to the first sample's GPS (origin).
        np.testing.assert_allclose(records[0].position, [0.0, 0.0, 0.0],
                                   atol=1e-6)
        # Attitude carries over from the preceding ATT message (radians).
        for r in records:
            assert r.euler[0] == pytest.approx(0.05)
        # Velocity decomposed from speed + course (45° → ~equal east/north).
        expected = 1.5 / np.sqrt(2)
        np.testing.assert_allclose(records[0].velocity[:2],
                                   [expected, expected], atol=1e-3)
        assert records[0].velocity[2] == pytest.approx(-0.5)

    def test_api_load_accepts_bin_file(self, tmp_path, monkeypatch):
        """POST /api/load with a .BIN path starts a replay and returns 200."""
        from fastapi.testclient import TestClient
        import runtime_view.server as srv

        # Stub out the bridge + telemetry so the test doesn't bind sockets.
        srv._stop_replay()
        monkeypatch.setattr(srv, "_start_records_replay",
                            lambda recs, p, ft: srv.JSONResponse({
                                "status": "playing",
                                "file": str(p),
                                "type": ft,
                                "records": len(recs),
                            }))

        bin_path = tmp_path / "sample.bin"
        self._write_synthetic_bin(bin_path)
        client = TestClient(srv.app)
        r = client.post(f"/api/load?path={bin_path}")
        assert r.status_code == 200, r.text
        body = r.json()
        assert body["type"] == "bin"
        assert body["records"] == 5

    def test_api_load_rejects_unknown_extension(self, tmp_path):
        from fastapi.testclient import TestClient
        import runtime_view.server as srv
        bad = tmp_path / "ignored.txt"
        bad.write_text("nothing", encoding="utf-8")
        client = TestClient(srv.app)
        r = client.post(f"/api/load?path={bad}")
        assert r.status_code == 400
        assert ".npz and .bin" in r.json()["detail"]


# ──────────────────────────────────────────────────────────────────────────────
# Runtime items closed in CI via Python equivalents
# (Gazebo emulators, in-process fault injection, scalability timing,
# cruise-attitude hard gate).
# ──────────────────────────────────────────────────────────────────────────────
