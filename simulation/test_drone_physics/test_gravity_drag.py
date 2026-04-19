"""
Test Gravity Drag

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


class TestGravity:
    def test_freefall_no_thrust(self):
        """Drone with zero thrust should fall under gravity."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 100.0]))
        cmd = DroneCommand(thrust=0.0)

        for _ in range(200):  # 1 second at dt=0.005
            state = physics_step(state, cmd, params, dt=0.005)

        # After 1s of freefall: z ≈ 100 - 0.5*g*t² = 95.1 (with some drag)
        assert state.position[2] < 100.0
        assert state.velocity[2] < 0.0  # falling

    def test_freefall_analytical(self):
        """Without drag, freefall should match kinematics."""
        params = DroneParams(drag_coeff=0.0)
        state = DroneState(position=np.array([0.0, 0.0, 100.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.001
        t_total = 1.0

        for _ in range(int(t_total / dt)):
            state = physics_step(state, cmd, params, dt)

        expected_z = 100.0 - 0.5 * GRAVITY * t_total**2
        expected_vz = -GRAVITY * t_total

        np.testing.assert_allclose(state.position[2], expected_z, atol=0.05)
        np.testing.assert_allclose(state.velocity[2], expected_vz, atol=0.05)

    def test_ground_constraint(self):
        """Drone should not fall below z=0."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 1.0]))
        cmd = DroneCommand(thrust=0.0)

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005)

        assert state.position[2] >= 0.0
        assert state.velocity[2] >= 0.0


# ── Hover equilibrium ────────────────────────────────────────────────────────


class TestHover:
    def test_hover_thrust(self):
        """Thrust = mg should maintain hover (no drag at zero velocity)."""
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):  # 10 seconds
            state = physics_step(state, cmd, params, dt=0.005)

        np.testing.assert_allclose(state.position[2], 10.0, atol=0.01)
        np.testing.assert_allclose(state.velocity[2], 0.0, atol=0.01)

    def test_hover_xy_stable(self):
        """Hover should not drift in XY."""
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([5.0, 3.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005)

        np.testing.assert_allclose(state.position[0], 5.0, atol=0.001)
        np.testing.assert_allclose(state.position[1], 3.0, atol=0.001)


# ── Drag ─────────────────────────────────────────────────────────────────────


class TestDrag:
    def test_drag_slows_horizontal(self):
        """Horizontal velocity should decay due to drag during hover."""
        params = DroneParams(drag_coeff=0.5)
        hover_thrust = params.mass * GRAVITY
        state = DroneState(
            position=np.array([0.0, 0.0, 10.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
        )
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(4000):  # 20 seconds
            state = physics_step(state, cmd, params, dt=0.005)

        assert abs(state.velocity[0]) < 0.1  # should have decayed

    def test_more_drag_slows_faster(self):
        """Higher drag coefficient should result in less displacement."""
        def fly(drag):
            params = DroneParams(drag_coeff=drag)
            hover_thrust = params.mass * GRAVITY
            state = DroneState(
                position=np.array([0.0, 0.0, 10.0]),
                velocity=np.array([10.0, 0.0, 0.0]),
            )
            cmd = DroneCommand(thrust=hover_thrust)
            for _ in range(1000):
                state = physics_step(state, cmd, params, dt=0.005)
            return state.position[0]

        x_low_drag = fly(0.05)
        x_high_drag = fly(1.0)
        assert x_high_drag < x_low_drag


# ── PID Controller ───────────────────────────────────────────────────────────


class TestQuadraticDrag:
    def test_quadratic_drag_scales_with_v_squared(self):
        """Drag force should quadruple when velocity doubles."""
        aero = AeroCoefficients(reference_area=0.04, C_D=1.0)
        rho = 1.225
        V1 = np.array([5.0, 0.0, 0.0])
        V2 = np.array([10.0, 0.0, 0.0])

        F1 = np.linalg.norm(_compute_quadratic_drag(V1, aero, rho))
        F2 = np.linalg.norm(_compute_quadratic_drag(V2, aero, rho))

        # F2/F1 should be (10/5)^2 = 4
        np.testing.assert_allclose(F2 / F1, 4.0, rtol=1e-10)

    def test_high_altitude_less_drag(self):
        """Same velocity at 4500m ASL should produce less drag than sea level."""
        aero = AeroCoefficients(reference_area=0.04, C_D=1.0)
        atmo_sea = Atmosphere(altitude_msl=0.0)
        atmo_high = Atmosphere(altitude_msl=4500.0)

        V = np.array([10.0, 0.0, 0.0])
        F_sea = np.linalg.norm(_compute_quadratic_drag(V, aero, atmo_sea.rho))
        F_high = np.linalg.norm(_compute_quadratic_drag(V, aero, atmo_high.rho))

        assert F_high < F_sea
        # At 4500m, rho ~ 0.77, so drag ~ 0.77/1.225 = 0.63x
        ratio = F_high / F_sea
        assert 0.5 < ratio < 0.8

    def test_terminal_velocity(self):
        """Freefall with quadratic drag should converge to finite terminal velocity."""
        aero = AeroCoefficients(reference_area=0.04, C_D=1.0)
        atmo = Atmosphere()
        params = DroneParams(aero=aero, atmo=atmo, drag_coeff=0.0)
        state = DroneState(position=np.array([0.0, 0.0, 5000.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.005

        speeds = []
        for i in range(10000):  # 50 seconds
            state = physics_step(state, cmd, params, dt)
            if i % 200 == 0:
                speeds.append(np.linalg.norm(state.velocity))

        # Speed should stabilize (last few readings close together)
        assert speeds[-1] > 10.0  # should be falling fast
        # Terminal velocity reached: variation in last readings < 5%
        recent = speeds[-5:]
        variation = (max(recent) - min(recent)) / np.mean(recent)
        assert variation < 0.05


class TestQuadrotorAeroArea:
    def test_effective_area_increases_with_tilt(self):
        """Quadrotor effective drag area should increase with tilt angle."""
        aero = QuadrotorAero(reference_area=0.05, C_D=1.0)
        area_level = aero.get_effective_area(tilt_angle=0.0, thrust_ratio=0.5)
        area_tilted = aero.get_effective_area(tilt_angle=np.deg2rad(35.0), thrust_ratio=0.5)
        assert area_tilted > area_level

    def test_effective_area_increases_with_prop_wash(self):
        """Prop-wash should increase effective drag area at higher thrust."""
        aero = QuadrotorAero(reference_area=0.05, C_D=1.0)
        area_idle = aero.get_effective_area(tilt_angle=np.deg2rad(15.0), thrust_ratio=0.0)
        area_high_thrust = aero.get_effective_area(tilt_angle=np.deg2rad(15.0), thrust_ratio=0.9)
        assert area_high_thrust > area_idle

    def test_drag_force_responds_to_tilt_and_prop_wash(self):
        """Quadratic drag should grow when tilt/prop-wash increase effective area."""
        aero = QuadrotorAero(reference_area=0.05, C_D=1.0)
        rho = 1.0
        V = np.array([8.0, 0.0, 0.0])

        drag_level_low = np.linalg.norm(
            _compute_quadratic_drag(V, aero, rho, tilt_angle=0.0, thrust_ratio=0.0)
        )
        drag_tilted_low = np.linalg.norm(
            _compute_quadratic_drag(V, aero, rho, tilt_angle=np.deg2rad(35.0), thrust_ratio=0.0)
        )
        drag_tilted_high = np.linalg.norm(
            _compute_quadratic_drag(V, aero, rho, tilt_angle=np.deg2rad(35.0), thrust_ratio=0.9)
        )

        assert drag_tilted_low > drag_level_low
        assert drag_tilted_high > drag_tilted_low
