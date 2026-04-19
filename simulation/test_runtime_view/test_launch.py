"""
Test Launch

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


class TestBrowserLaunch:
    """Tests for browser-driven command execution (roadmap item 2)."""

    def test_api_launch_rejects_unknown_id(self):
        """POST /api/launch?id=nonexistent must return 404."""
        import urllib.request, urllib.error, urllib.parse
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/launch?id=nonexistent'
            req = urllib.request.Request(url, method='POST')
            try:
                urllib.request.urlopen(req, timeout=5)
                assert False, 'Expected 404'
            except urllib.error.HTTPError as e:
                assert e.code == 404
        finally:
            _runtime_view_stop_uvicorn(server, thread)

    def test_api_launch_rejects_disabled_mission(self):
        """POST /api/launch?id=swarm-3 (disabled) must return 403."""
        import urllib.request, urllib.error
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/launch?id=swarm-3'
            req = urllib.request.Request(url, method='POST')
            try:
                urllib.request.urlopen(req, timeout=5)
                assert False, 'Expected 403'
            except urllib.error.HTTPError as e:
                assert e.code == 403
        finally:
            _runtime_view_stop_uvicorn(server, thread)

    def test_api_launch_status_not_running(self):
        """GET /api/launch/status with no launched process returns running=false."""
        import urllib.request, json
        from live_telemetry import TelemetryQueue
        import runtime_view.server as srv
        srv._launch_proc = None
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/launch/status'
            with urllib.request.urlopen(url, timeout=5) as resp:
                data = json.loads(resp.read())
            assert data['running'] is False
        finally:
            _runtime_view_stop_uvicorn(server, thread)
