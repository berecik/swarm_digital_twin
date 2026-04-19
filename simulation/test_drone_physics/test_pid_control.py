"""
Test Pid Control

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


class TestPID:
    def test_converges_to_zero(self):
        pid = PIDController(kp=1.0, ki=0.2, kd=0.5)
        val = 10.0
        dt = 0.01
        for _ in range(5000):
            u = pid.update(val, dt=dt)
            val -= u * dt
        assert abs(val) < 0.1

    def test_limit(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, limit=5.0)
        out = pid.update(1000.0, dt=0.01)
        assert abs(out) <= 5.0


# ── Position Controller ─────────────────────────────────────────────────────


class TestPositionController:
    def test_reach_target(self):
        """Controller should fly drone to target position."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 0.0]))
        controller = PositionController(params)
        target = np.array([0.0, 0.0, 5.0])
        dt = 0.005

        for _ in range(4000):  # 20 seconds
            cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)
            state = physics_step(state, cmd, params, dt)

        np.testing.assert_allclose(state.position, target, atol=0.5)

    def test_horizontal_flight(self):
        """Controller should handle horizontal + vertical targets."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 0.0]))
        controller = PositionController(params)
        target = np.array([10.0, 5.0, 8.0])
        dt = 0.005

        for _ in range(8000):  # 40 seconds
            cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)
            state = physics_step(state, cmd, params, dt)

        dist = np.linalg.norm(state.position - target)
        assert dist < 1.0


# ── Full simulation run ─────────────────────────────────────────────────────
