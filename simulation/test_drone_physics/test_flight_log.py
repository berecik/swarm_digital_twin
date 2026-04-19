"""
Test Flight Log

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


class TestFlightLogBin:
    def _create_minimal_dataflash(self, tmpdir):
        """Create a minimal synthetic DataFlash .bin file for testing."""
        import struct
        buf = bytearray()

        # FMT message for FMT itself (msg_id=128)
        def write_fmt(buf, type_id, length, name, fmt_str, labels):
            buf += bytes([0xA3, 0x95, 128])
            buf += bytes([type_id])
            buf += bytes([length])
            buf += name.encode().ljust(4, b'\x00')[:4]
            buf += fmt_str.encode().ljust(16, b'\x00')[:16]
            buf += labels.encode().ljust(64, b'\x00')[:64]

        # FMT for GPS message (type=130)
        write_fmt(buf, 130, 35, "GPS", "QBIHBcLLefffB",
                  "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,U")

        # FMT for ATT message (type=131)
        write_fmt(buf, 131, 19, "ATT", "QccccCC",
                  "TimeUS,Roll,Pitch,Yaw,ErrRP,ErrYaw,APTS")

        # Write a GPS data message
        buf += bytes([0xA3, 0x95, 130])
        # TimeUS(Q), Status(B), GMS(I), GWk(H), NSats(B), HDop(c=h*0.01),
        # Lat(L=i*1e-7), Lng(L=i*1e-7), Alt(e=i*0.01), Spd(e), GCrs(e), VZ(e), U(B)
        buf += struct.pack('<Q', 1000000)  # 1.0 sec
        buf += struct.pack('B', 3)  # Status
        buf += struct.pack('<I', 0)  # GMS
        buf += struct.pack('<H', 2200)  # GWk
        buf += struct.pack('B', 12)  # NSats
        buf += struct.pack('<h', 100)  # HDop (1.0)
        buf += struct.pack('<i', -5083330)  # Lat (-0.508333 * 1e7)
        buf += struct.pack('<i', -781416670)  # Lng (-78.141667 * 1e7)
        buf += struct.pack('<i', 450000)  # Alt (4500.0 * 100)
        buf += struct.pack('<i', 1200)  # Spd (12.0 * 100)
        buf += struct.pack('<i', 9000)  # GCrs (90.0 * 100)
        buf += struct.pack('<i', 0)  # VZ
        buf += struct.pack('B', 1)  # U

        # Second GPS point
        buf += bytes([0xA3, 0x95, 130])
        buf += struct.pack('<Q', 2000000)
        buf += struct.pack('B', 3)
        buf += struct.pack('<I', 0)
        buf += struct.pack('<H', 2200)
        buf += struct.pack('B', 12)
        buf += struct.pack('<h', 100)
        buf += struct.pack('<i', -5082330)  # slightly different lat
        buf += struct.pack('<i', -781416670)
        buf += struct.pack('<i', 451000)  # 4510m
        buf += struct.pack('<i', 1200)
        buf += struct.pack('<i', 9000)
        buf += struct.pack('<i', 0)
        buf += struct.pack('B', 1)

        path = str(tmpdir / "test_log.bin")
        with open(path, 'wb') as f:
            f.write(bytes(buf))
        return path

    def test_bin_parser_creates_flight_log(self, tmp_path):
        """DataFlash parser should produce a FlightLog with GPS data."""
        from flight_log import FlightLog
        path = self._create_minimal_dataflash(tmp_path)
        log = FlightLog.from_bin(path)
        assert len(log.timestamps) >= 1
        assert log.positions.shape[1] == 3

    def test_bin_parser_file_not_found(self):
        """Non-existent .bin file should raise FileNotFoundError."""
        from flight_log import FlightLog
        with pytest.raises(FileNotFoundError):
            FlightLog.from_bin("/nonexistent/log.bin")

    def test_get_elevator_returns_pitch_rate(self):
        """get_elevator should return pitch rate derivative."""
        from flight_log import FlightLog
        log = FlightLog(
            timestamps=np.array([0.0, 1.0, 2.0, 3.0]),
            positions=np.zeros((4, 3)),
            attitudes=np.array([
                [0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0],
                [0.0, 0.3, 0.0],
                [0.0, 0.3, 0.0],
            ]),
        )
        elevator = log.get_elevator()
        assert len(elevator) == 4
        assert elevator[0] == pytest.approx(0.1, abs=1e-6)
        assert elevator[1] == pytest.approx(0.2, abs=1e-6)

    def test_extract_waypoints_from_dwell(self):
        """extract_waypoints should detect dwell points (low speed)."""
        from flight_log import FlightLog
        # Simulate: fly fast, then hover, then fly fast, then hover
        t = np.linspace(0, 20, 200)
        pos = np.zeros((200, 3))
        pos[:, 0] = np.where(t < 5, t * 2, np.where(t < 10, 10.0,
                    np.where(t < 15, 10.0 + (t - 10) * 2, 20.0)))
        log = FlightLog(
            timestamps=t,
            positions=pos,
            attitudes=np.zeros((200, 3)),
        )
        wps = log.extract_waypoints(speed_threshold=0.5, min_dwell=2.0)
        assert len(wps) >= 1  # Should detect at least the hover at t=5-10


class TestMissionReplay:
    """Verify replay_mission() pipeline operates correctly."""

    def _make_synthetic_log(self, airframe_type="quad"):
        """Create a synthetic flight log for testing replay."""
        n_points = 500
        dt = 0.1
        timestamps = np.arange(n_points) * dt

        if airframe_type == "quad":
            # Quadrotor: short square path at 20m
            phase = n_points // 4
            positions = np.zeros((n_points, 3))
            for i in range(n_points):
                t_frac = i / n_points
                if t_frac < 0.25:
                    positions[i] = [t_frac * 40, 0, 20]
                elif t_frac < 0.5:
                    positions[i] = [10, (t_frac - 0.25) * 40, 20]
                elif t_frac < 0.75:
                    positions[i] = [10 - (t_frac - 0.5) * 40, 10, 20]
                else:
                    positions[i] = [0, 10 - (t_frac - 0.75) * 40, 20]
            # Add small perturbation to simulate wind effects
            positions[:, 2] += 0.05 * np.sin(timestamps * 0.5)
        else:
            # Fixed-wing: longer path at 80m
            positions = np.zeros((n_points, 3))
            positions[:, 0] = np.linspace(0, 100, n_points)
            positions[:, 1] = 20 * np.sin(timestamps * 0.1)
            positions[:, 2] = 80 + 2.0 * np.sin(timestamps * 0.3)

        attitudes = np.zeros((n_points, 3))
        log = FlightLog(
            timestamps=timestamps,
            positions=positions,
            attitudes=attitudes,
            airspeeds=np.ones(n_points) * 5.0,
            throttle=np.ones(n_points) * 0.5,
        )
        return log

    def test_replay_returns_metrics(self):
        """replay_mission() should return dict with standard metric keys."""
        log = self._make_synthetic_log("quad")
        params = make_irs4_quadrotor()
        metrics = replay_mission(log, params, wind_source="none")
        required_keys = {"rmse_z", "rmse_x", "rmse_y", "rmse_total", "n_points"}
        assert required_keys.issubset(set(metrics.keys()))

    def test_replay_quad_produces_valid_rmse(self):
        """Quadrotor replay should produce finite, positive RMSE values."""
        log = self._make_synthetic_log("quad")
        params = make_irs4_quadrotor()
        metrics = replay_mission(log, params, wind_source="none")
        assert 0 < metrics["rmse_z"] < 50
        assert 0 < metrics["rmse_total"] < 100
        assert metrics["n_points"] > 10

    def test_replay_fixed_wing_produces_valid_rmse(self):
        """Fixed-wing replay should produce finite, positive RMSE values."""
        log = self._make_synthetic_log("fw")
        params = make_valencia_fixed_wing()
        metrics = replay_mission(log, params, wind_source="none")
        assert 0 < metrics["rmse_z"] < 100
        assert metrics["n_points"] > 10

    def test_replay_with_wind_from_log(self):
        """Replay with from_log wind should complete without error."""
        log = self._make_synthetic_log("quad")
        params = make_irs4_quadrotor()
        metrics = replay_mission(log, params, wind_source="from_log")
        assert metrics["n_points"] > 10

    def test_replay_rejects_short_log(self):
        """Should reject flight logs with fewer than 10 points."""
        log = FlightLog(
            timestamps=np.array([0, 1, 2]),
            positions=np.array([[0, 0, 10], [1, 0, 10], [2, 0, 10]]),
        )
        with pytest.raises(ValueError, match="too short"):
            replay_mission(log, make_irs4_quadrotor(), wind_source="none")


# ── Paper Table 5 Acceptance Tests ───────────────────────────────────────
