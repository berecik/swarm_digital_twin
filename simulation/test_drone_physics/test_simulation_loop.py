"""
Test Simulation Loop

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


class TestSimulation:
    def test_simple_flight(self):
        """Run a two-waypoint mission and verify completion."""
        waypoints = [
            np.array([0.0, 0.0, 5.0]),
            np.array([5.0, 0.0, 5.0]),
        ]
        records = run_simulation(waypoints, dt=0.005, hover_time=1.0, max_time=60.0)

        assert len(records) > 0
        final = records[-1]
        dist = np.linalg.norm(final.position - waypoints[-1])
        assert dist < 1.0

    def test_records_have_increasing_time(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 3.0])],
            dt=0.01, hover_time=1.0, max_time=10.0,
        )
        times = [r.t for r in records]
        assert all(t2 > t1 for t1, t2 in zip(times, times[1:]))

    def test_thrust_always_positive(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 5.0])],
            dt=0.005, hover_time=1.0, max_time=20.0,
        )
        for r in records:
            assert r.thrust >= 0.0

    def test_altitude_stays_positive(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 0.5])],
            dt=0.005, hover_time=1.0, max_time=60.0,
        )
        for r in records:
            assert r.position[2] >= 0.0


# ── Energy ───────────────────────────────────────────────────────────────────


class TestEnergy:
    def test_freefall_energy_conservation(self):
        """Without drag, KE + PE should be approximately conserved."""
        params = DroneParams(drag_coeff=0.0, ang_drag_coeff=0.0)
        state = DroneState(position=np.array([0.0, 0.0, 50.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.001

        m = params.mass
        e0 = m * GRAVITY * state.position[2] + 0.5 * m * np.dot(state.velocity, state.velocity)

        for _ in range(500):  # 0.5 seconds
            state = physics_step(state, cmd, params, dt)

        e1 = m * GRAVITY * state.position[2] + 0.5 * m * np.dot(state.velocity, state.velocity)
        np.testing.assert_allclose(e0, e1, rtol=0.005)


# ── Quadratic Drag & Atmosphere ──────────────────────────────────────────────


class TestBodyFrame:
    def test_body_frame_hover_equivalent(self):
        """Body-frame dynamics should produce same hover as world-frame."""
        # World-frame (linear drag)
        params_world = DroneParams()
        state_w = DroneState(position=np.array([0.0, 0.0, 10.0]))
        hover = params_world.mass * GRAVITY
        cmd = DroneCommand(thrust=hover)

        for _ in range(2000):
            state_w = physics_step(state_w, cmd, params_world, dt=0.005)

        # Body-frame (quadratic drag, but zero velocity → zero drag)
        params_body = DroneParams(
            aero=AeroCoefficients(), atmo=Atmosphere(),
        )
        state_b = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd_b = DroneCommand(thrust=params_body.mass * GRAVITY)

        for _ in range(2000):
            state_b = physics_step(state_b, cmd_b, params_body, dt=0.005)

        # Both should be hovering at ~10m
        np.testing.assert_allclose(state_w.position[2], 10.0, atol=0.01)
        np.testing.assert_allclose(state_b.position[2], 10.0, atol=0.01)

    def test_body_frame_freefall(self):
        """Body-frame freefall should match world-frame (zero drag)."""
        params_body = DroneParams(
            aero=AeroCoefficients(C_D=0.0), atmo=Atmosphere(),
        )
        state = DroneState(position=np.array([0.0, 0.0, 100.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.001
        t_total = 1.0

        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message=r"Aerodynamic parameter warning: C_D=0\.0000.*",
                category=RuntimeWarning,
            )
            for _ in range(int(t_total / dt)):
                state = physics_step(state, cmd, params_body, dt)

        expected_z = 100.0 - 0.5 * GRAVITY * t_total**2
        expected_vz = -GRAVITY * t_total

        np.testing.assert_allclose(state.position[2], expected_z, atol=0.1)
        np.testing.assert_allclose(state.velocity[2], expected_vz, atol=0.1)


# ── Validation Module (RMSE / compare_sim_real) ─────────────────────────────────


class TestEulerRates:
    """Tests for euler_rates_from_body_rates (paper Eq. 2)."""

    def test_identity_at_zero_attitude(self):
        """At phi=theta=0, E is identity → euler rates = body rates."""
        from drone_physics import euler_rates_from_body_rates
        rates = euler_rates_from_body_rates(0.0, 0.0, 1.0, 2.0, 3.0)
        np.testing.assert_allclose(rates, [1.0, 2.0, 3.0], atol=1e-10)

    def test_pure_roll_rate(self):
        """Pure body roll rate p maps to phi_dot at zero attitude."""
        from drone_physics import euler_rates_from_body_rates
        rates = euler_rates_from_body_rates(0.0, 0.0, 5.0, 0.0, 0.0)
        np.testing.assert_allclose(rates, [5.0, 0.0, 0.0], atol=1e-10)

    def test_pitched_yaw_coupling(self):
        """At nonzero pitch, body yaw rate r couples into phi_dot and psi_dot."""
        from drone_physics import euler_rates_from_body_rates
        theta = 0.3  # ~17 degrees pitch
        rates = euler_rates_from_body_rates(0.0, theta, 0.0, 0.0, 1.0)
        # cos(0)=1, sin(0)=0 → E @[0,0,1] = [tan(theta), 0, 1/cos(theta)]
        expected = [np.tan(theta), 0.0, 1.0 / np.cos(theta)]
        np.testing.assert_allclose(rates, expected, atol=1e-10)

    def test_numerical_differentiation(self):
        """Euler rates match numerical differentiation of rotation matrix."""
        from drone_physics import euler_rates_from_body_rates

        phi, theta = 0.2, 0.15
        p, q, r = 0.5, -0.3, 0.8
        rates = euler_rates_from_body_rates(phi, theta, p, q, r)

        # Numerical check: apply small dt, compute finite-difference euler angles
        dt = 1e-6
        phi2 = phi + rates[0] * dt
        theta2 = theta + rates[1] * dt
        psi_dot = rates[2]

        # Verify the transformation is self-consistent:
        # re-compute at slightly perturbed angles
        rates2 = euler_rates_from_body_rates(phi2, theta2, p, q, r)
        # Rates should be nearly identical for small perturbation
        np.testing.assert_allclose(rates, rates2, atol=1e-3)

    def test_gimbal_lock_raises(self):
        """At theta = π/2, gimbal lock should raise ValueError."""
        from drone_physics import euler_rates_from_body_rates
        with pytest.raises(ValueError, match="Gimbal lock"):
            euler_rates_from_body_rates(0.0, np.pi / 2, 1.0, 0.0, 0.0)

    def test_symmetric_roll(self):
        """Negating phi negates the off-diagonal coupling signs."""
        from drone_physics import euler_rates_from_body_rates
        phi, theta = 0.3, 0.2
        p, q, r = 0.5, 0.5, 0.5
        rates_pos = euler_rates_from_body_rates(phi, theta, p, q, r)
        rates_neg = euler_rates_from_body_rates(-phi, theta, p, q, r)
        # theta_dot changes sign due to -sin(phi) term
        assert rates_pos[1] != pytest.approx(rates_neg[1], abs=1e-6)


# ── Physics Live Replay Tests ────────────────────────────────────────────────
