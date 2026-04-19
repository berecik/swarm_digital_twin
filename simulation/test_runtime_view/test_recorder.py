"""
Test Recorder

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


class TestDataFlashRecorder:
    """Tests for the DataFlash .BIN recorder (roadmap item 4)."""

    def test_write_and_read_header_magic(self, tmp_path):
        """Written .BIN file must start with 0xA3 0x95 header."""
        from dataflash_recorder import DataFlashRecorder, HEAD1, HEAD2
        path = str(tmp_path / 'test.BIN')
        with DataFlashRecorder(path) as rec:
            rec.write_att(1000.0, 5.0, -2.0, 45.0)
        with open(path, 'rb') as f:
            data = f.read(3)
        assert data[0] == HEAD1
        assert data[1] == HEAD2

    def test_fmt_records_written(self, tmp_path):
        """File must contain FMT records for ATT, GPS, BAT, MODE."""
        from dataflash_recorder import DataFlashRecorder
        path = str(tmp_path / 'test.BIN')
        with DataFlashRecorder(path) as rec:
            pass  # Just headers
        with open(path, 'rb') as f:
            data = f.read()
        # FMT records contain the type name as ASCII
        assert b'ATT\x00' in data
        assert b'GPS\x00' in data
        assert b'BAT\x00' in data
        assert b'MODE' in data

    def test_record_sample_writes_data(self, tmp_path):
        """record_sample() must increase the file size."""
        import os
        from dataflash_recorder import DataFlashRecorder
        from live_telemetry import LiveTelemetrySample
        path = str(tmp_path / 'test.BIN')
        with DataFlashRecorder(path) as rec:
            header_size = os.path.getsize(path)
            sample = LiveTelemetrySample(
                t_wall=1000.0,
                euler=(0.1, -0.05, 1.2),
                lat_deg=47.3769,
                lon_deg=8.5417,
                alt_msl=408.0,
                battery_voltage_v=12.6,
                battery_current_a=1.5,
                battery_remaining_pct=85.0,
            )
            rec.record_sample(sample)
        final_size = os.path.getsize(path)
        assert final_size > header_size, (
            f'File should grow after record_sample '
            f'(header={header_size}, final={final_size})')

    def test_multiple_samples_roundtrip(self, tmp_path):
        """Multiple samples produce a file proportional to sample count."""
        import os
        from dataflash_recorder import DataFlashRecorder
        from live_telemetry import LiveTelemetrySample
        path = str(tmp_path / 'test.BIN')
        with DataFlashRecorder(path) as rec:
            for i in range(10):
                sample = LiveTelemetrySample(
                    t_wall=1000.0 + i * 0.02,
                    pos_enu=np.array([float(i), 0.0, 5.0]),
                    euler=(0.01 * i, 0.0, 0.0),
                    lat_deg=47.3769 + i * 0.0001,
                    lon_deg=8.5417,
                    alt_msl=408.0 + i,
                )
                rec.record_sample(sample)
        size = os.path.getsize(path)
        # Each sample writes ATT + GPS + BAT = 3 records.
        # File should be significantly larger than just headers.
        assert size > 500, f'Expected >500 bytes for 10 samples, got {size}'

    def test_cli_record_bin_flag(self):
        """physics_live_replay --help must list --record-bin."""
        import subprocess, sys as _sys
        sim_dir = str(SIM_DIR)
        result = subprocess.run(
            [_sys.executable, '-m', 'physics_live_replay', '--help'],
            capture_output=True, text=True, timeout=10,
            cwd=sim_dir,
        )
        assert '--record-bin' in result.stdout
