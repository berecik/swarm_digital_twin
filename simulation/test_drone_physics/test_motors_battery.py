"""
Test Motors Battery

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


class TestMotorDynamics:
    def test_motor_step_response_reaches_63pct_near_tau(self):
        tau = 0.05
        model = MotorModel(k_T=9.2e-6, k_D=0.0, tau_motor=tau, num_motors=4)
        params = DroneParams(
            mass=1.8,
            max_thrust=35.0,
            motor_model=model,
            motor_dynamics_enabled=True,
            drag_coeff=0.0,
            ang_drag_coeff=0.0,
            aero=AeroCoefficients(reference_area=0.05, C_D=1.0, C_L=0.0),
        )
        cmd = DroneCommand(thrust=24.0, torque=np.zeros(3))
        dt = 0.001

        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        thrust_samples = []
        for _ in range(300):
            state = physics_step(state, cmd, params, dt=dt)
            thrust_samples.append(float(np.sum(model.thrust_from_omega(state.motor_omega))))

        steady = np.mean(thrust_samples[-50:])
        target_63 = 0.632 * steady
        idx = int(np.argmax(np.array(thrust_samples) >= target_63))
        t63 = idx * dt
        assert 0.03 <= t63 <= 0.08

    def test_motor_steady_state_matches_kt_omega_squared(self):
        model = MotorModel(k_T=9.2e-6, k_D=0.0, tau_motor=0.05, num_motors=4)
        params = DroneParams(
            mass=1.8,
            max_thrust=35.0,
            motor_model=model,
            motor_dynamics_enabled=True,
            drag_coeff=0.0,
            ang_drag_coeff=0.0,
            aero=AeroCoefficients(reference_area=0.05, C_D=1.0, C_L=0.0),
        )
        cmd = DroneCommand(thrust=20.0, torque=np.zeros(3))
        dt = 0.005

        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        for _ in range(500):
            state = physics_step(state, cmd, params, dt=dt)

        omega = state.motor_omega
        thrust_from_map = np.sum(model.k_T * omega**2)
        assert thrust_from_map > 0.0
        np.testing.assert_allclose(thrust_from_map, cmd.thrust, rtol=0.05)

    def test_default_physics_step_without_motor_model_stays_backward_compatible(self):
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


class TestBatteryModel:
    def test_lipo_voltage_curve_is_soc_dependent(self):
        model = BatteryModel(cells=6, capacity_mah=6000.0)
        v_full = model.open_circuit_voltage(1.0)
        v_half = model.open_circuit_voltage(0.5)
        v_empty = model.open_circuit_voltage(0.0)
        assert v_full > v_half > v_empty
        np.testing.assert_allclose(v_full, 25.2, atol=1e-6)
        np.testing.assert_allclose(v_empty, 19.8, atol=1e-6)

    def test_motor_power_draw_reduces_soc(self):
        model = MotorModel(k_T=9.2e-6, k_Q=1.5e-7, tau_motor=0.01, num_motors=4)
        params = DroneParams(
            mass=1.8,
            max_thrust=35.0,
            motor_model=model,
            motor_dynamics_enabled=True,
            battery_model=BatteryModel(cells=6, capacity_mah=6000.0),
            drag_coeff=0.0,
            ang_drag_coeff=0.0,
            aero=AeroCoefficients(reference_area=0.05, C_D=1.0, C_L=0.0),
        )
        state = DroneState(position=np.array([0.0, 0.0, 5.0]), battery_soc=1.0)
        cmd = DroneCommand(thrust=20.0, torque=np.zeros(3))

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.01)

        assert state.battery_power_w > 0.0
        assert state.battery_current_a > 0.0
        assert 0.0 < state.battery_soc < 1.0
        assert state.battery_voltage_v > 0.0
        assert state.battery_remaining_min < float("inf")

    def test_fixed_wing_autonomy_estimate_matches_table2(self):
        params = make_valencia_fixed_wing()
        assert params.battery_model is not None
        endurance_min = params.battery_model.estimate_endurance_min(power_w=152.0)
        assert endurance_min == pytest.approx(85.0, abs=6.0)


# ── Inertia & Airframe Presets ───────────────────────────────────────────────


class TestInertiaPresets:
    def test_off_diagonal_inertia_coupling(self):
        """Off-diagonal inertia should cause cross-axis coupling."""
        # Symmetric quad (diagonal) — torque around x only affects roll
        diag_params = DroneParams(
            inertia=np.diag([0.02, 0.02, 0.04]),
        )
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(
            thrust=diag_params.mass * GRAVITY,
            torque=np.array([1.0, 0.0, 0.0]),
        )
        state_diag = physics_step(state, cmd, diag_params, dt=0.01)

        # With off-diagonal terms — cross-coupling should appear
        coupled_params = DroneParams(
            inertia=np.array([
                [0.02,  0.005, 0.0  ],
                [0.005, 0.02,  0.0  ],
                [0.0,   0.0,   0.04 ],
            ]),
        )
        state_coupled = physics_step(state, cmd, coupled_params, dt=0.01)

        # With coupling, a pure x-torque should also affect y angular velocity
        assert abs(state_coupled.angular_velocity[1]) > abs(state_diag.angular_velocity[1])

    def test_preset_loading(self):
        """Airframe presets should load with correct parameters."""
        generic = make_generic_quad()
        assert generic.mass == 1.5
        assert generic.aero is None

        x500 = make_holybro_x500()
        assert x500.mass == 2.0
        assert x500.aero is not None
        assert x500.aero.reference_area == 0.06
        assert x500.atmo is not None


# ── Body-Frame Dynamics (Eq. 3) ────────────────────────────────────────────────
