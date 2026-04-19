"""
Test Swarm

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


class TestSwarmStandaloneTwin:
    def test_flocking_vector_returns_zero_without_neighbors(self):
        """No neighbors should yield a zero steering vector."""
        got = calculate_flocking_vector(
            me_position=np.array([1.0, 2.0, 3.0]),
            me_velocity=np.array([0.5, -0.2, 0.1]),
            neighbor_positions=[],
            neighbor_velocities=[],
            params=FlockingParams(),
        )
        np.testing.assert_allclose(got, np.zeros(3), atol=1e-12)

    def test_flocking_vector_excludes_neighbor_at_radius_boundary(self):
        """Neighbor exactly at radius boundary should be excluded like boids.rs (<, not <=)."""
        params = FlockingParams(neighbor_radius=10.0)
        got = calculate_flocking_vector(
            me_position=np.array([0.0, 0.0, 0.0]),
            me_velocity=np.array([0.3, -0.1, 0.0]),
            neighbor_positions=[np.array([10.0, 0.0, 0.0])],
            neighbor_velocities=[np.array([1.0, 1.0, 0.0])],
            params=params,
        )
        np.testing.assert_allclose(got, -np.array([0.3, -0.1, 0.0]), atol=1e-12)

    def test_flocking_vector_matches_rust_reference_case(self):
        """Python boids helper should match boids.rs equation for a fixed case."""
        params = FlockingParams(
            neighbor_radius=10.0,
            separation_radius=2.0,
            separation_weight=2.0,
            alignment_weight=1.0,
            cohesion_weight=1.0,
        )
        me_position = np.array([0.0, 0.0, 0.0])
        me_velocity = np.array([1.0, 0.0, 0.0])
        neighbor_positions = [
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 3.0, 0.0]),
        ]
        neighbor_velocities = [
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, 2.0, 0.0]),
        ]

        got = calculate_flocking_vector(
            me_position,
            me_velocity,
            neighbor_positions,
            neighbor_velocities,
            params,
        )

        # Hand-computed from boids.rs formula:
        # separation=[-1,0,0], alignment=[0,1.5,0], cohesion=[0.5,1.5,0]
        # result = sep*2 + (align-me_vel)*1 + coh*1 = [-2.5, 3.0, 0]
        expected = np.array([-2.5, 3.0, 0.0])
        np.testing.assert_allclose(got, expected, atol=1e-9)

    def test_six_agent_run_maintains_min_separation(self):
        """Swarm scenario should sustain 6 agents without collisions."""
        base_alt = 8.0
        drone_waypoints = {
            f"drone_{i}": [
                np.array([
                    10.0 * np.cos(i * np.pi / 3.0),
                    10.0 * np.sin(i * np.pi / 3.0),
                    base_alt,
                ]),
                np.array([
                    10.0 * np.cos(i * np.pi / 3.0 + np.pi / 6.0),
                    10.0 * np.sin(i * np.pi / 3.0 + np.pi / 6.0),
                    base_alt,
                ]),
            ]
            for i in range(6)
        }

        records = run_swarm_simulation(
            drone_waypoints,
            params=make_generic_quad(),
            dt=0.01,
            hover_time=0.4,
            max_time=6.0,
            min_separation=1.5,
        )

        assert len(records) > 0
        min_dist = float("inf")
        for rec in records:
            positions = rec.positions
            for i in range(len(positions)):
                for j in range(i + 1, len(positions)):
                    d = np.linalg.norm(positions[i] - positions[j])
                    min_dist = min(min_dist, d)

        assert min_dist >= 1.0


# ── Flight Log & Validation Pipeline ─────────────────────────────────────────
