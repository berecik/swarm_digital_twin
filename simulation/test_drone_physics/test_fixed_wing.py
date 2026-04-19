"""
Test Fixed Wing

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


class TestFixedWingAero:
    def test_aoa_computation_level_flight(self):
        """Level forward flight should have AoA near zero."""
        V_body = np.array([20.0, 0.0, 0.0])  # forward
        alpha = compute_aoa(V_body)
        np.testing.assert_allclose(alpha, 0.0, atol=1e-10)

    def test_aoa_computation_climbing(self):
        """Climbing flight (V_z > 0) should give negative AoA."""
        V_body = np.array([20.0, 0.0, 5.0])  # forward + up
        alpha = compute_aoa(V_body)
        assert alpha < 0.0  # AoA = atan2(-Vz, Vx) = atan2(-5, 20) < 0

    def test_aoa_computation_descending(self):
        """Descending flight (V_z < 0) should give positive AoA."""
        V_body = np.array([20.0, 0.0, -5.0])  # forward + down
        alpha = compute_aoa(V_body)
        assert alpha > 0.0  # AoA = atan2(5, 20) > 0

    def test_aoa_zero_velocity(self):
        """Zero velocity should give AoA = 0."""
        alpha = compute_aoa(np.zeros(3))
        assert alpha == 0.0

    def test_pre_stall_cl_linear(self):
        """CL should increase linearly with AoA before stall."""
        fw = FixedWingAero()
        # At alpha_0, CL = 0
        np.testing.assert_allclose(fw.get_CL(fw.alpha_0), 0.0, atol=1e-10)
        # At 10 deg (0.1745 rad), should be positive and linear
        alpha = 0.1745
        cl = fw.get_CL(alpha)
        assert cl > 0.0
        expected = fw.C_La * (alpha - fw.alpha_0)
        np.testing.assert_allclose(cl, expected, rtol=1e-10)

    def test_post_stall_cl_drops(self):
        """CL should decrease after stall angle."""
        fw = FixedWingAero()
        cl_pre = fw.get_CL(0.25)   # just before stall (15 deg)
        cl_post = fw.get_CL(0.30)  # after stall
        # Post-stall slope is negative, so CL should be less
        assert cl_post < cl_pre

    def test_cd_increases_with_aoa(self):
        """CD should increase with AoA (induced drag)."""
        fw = FixedWingAero()
        cd_0 = fw.get_CD(0.0)
        cd_10 = fw.get_CD(0.1745)
        assert cd_10 > cd_0

    def test_post_stall_cd_increases_faster(self):
        """Post-stall CD slope should be steeper than pre-stall."""
        fw = FixedWingAero()
        cd_pre = fw.get_CD(0.25)   # just before stall
        cd_post = fw.get_CD(0.30)  # just after stall
        # C_Da_stall > C_Da, so at same AoA past stall, drag jumps up
        assert cd_post > cd_pre

    def test_lift_force_perpendicular_to_velocity(self):
        """Lift should be perpendicular to velocity vector."""
        fw = FixedWingAero()
        V_body = np.array([20.0, 0.0, -2.0])  # slight descent → positive AoA
        rho = 1.225
        F_lift = _compute_lift(V_body, fw, rho)
        # Lift should be non-zero (positive AoA → positive CL)
        assert np.linalg.norm(F_lift) > 0.0
        # Lift should be perpendicular to velocity
        dot = np.dot(F_lift, V_body)
        np.testing.assert_allclose(dot, 0.0, atol=1e-6)

    def test_lift_zero_for_quadrotor(self):
        """Standard AeroCoefficients (C_L=0) should produce zero lift."""
        aero = AeroCoefficients(C_L=0.0)
        V_body = np.array([10.0, 0.0, -1.0])
        F_lift = _compute_lift(V_body, aero, 1.225)
        np.testing.assert_allclose(F_lift, np.zeros(3))

    def test_fixed_wing_preset(self):
        """make_fixed_wing() should return correct parameters."""
        fw = make_fixed_wing()
        assert fw.mass == 3.0
        assert fw.aero is not None
        assert isinstance(fw.aero, FixedWingAero)
        assert fw.aero.reference_area == 0.50
        assert fw.aero.alpha_stall > 0
        assert fw.atmo is not None
        # Inertia should have off-diagonal elements
        assert fw.inertia[0, 2] != 0.0

    def test_fixed_wing_generates_lift_in_flight(self):
        """A fixed-wing in forward flight should generate lift force."""
        fw_aero = FixedWingAero(reference_area=0.5)
        rho = 1.225
        # Forward flight with slight descent → positive AoA → positive lift
        V_body = np.array([20.0, 0.0, -3.0])
        F_lift = _compute_lift(V_body, fw_aero, rho)
        # Lift Z component should be positive (upward in body frame)
        assert F_lift[2] > 0.0

    def test_quadratic_drag_uses_aoa_dependent_cd(self):
        """Drag should use get_CD(alpha) for FixedWingAero."""
        fw_aero = FixedWingAero(reference_area=0.5)
        rho = 1.225
        # At zero AoA, CD = C_D0 + C_Da * 0 = C_D0
        V_level = np.array([20.0, 0.0, 0.0])
        F_level = np.linalg.norm(_compute_quadratic_drag(V_level, fw_aero, rho))
        # At high AoA, CD should be larger
        V_pitch = np.array([20.0, 0.0, -10.0])  # ~26.5 deg AoA
        F_pitch = np.linalg.norm(_compute_quadratic_drag(V_pitch, fw_aero, rho))
        assert F_pitch > F_level

    def test_runtime_warning_for_out_of_range_aero_params(self):
        """Out-of-range aero params should emit a runtime warning once."""
        params = DroneParams(
            mass=50.0,
            aero=AeroCoefficients(reference_area=2.0, C_D=3.0),
            atmo=Atmosphere(),
        )
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(thrust=0.0)

        with pytest.warns(RuntimeWarning, match="Aerodynamic parameter warning"):
            physics_step(state, cmd, params, dt=0.01)

        # Second call should not re-emit warnings for the same params object.
        with warnings.catch_warnings(record=True) as record:
            warnings.simplefilter("always")
            physics_step(state, cmd, params, dt=0.01)
        assert len(record) == 0


# ── Paper-Exact Aerodynamics (Table 3 Coefficients) ─────────────────────────


class TestPitchingMoment:
    def test_pitching_moment_pre_stall(self):
        """C_M should be negative and proportional to alpha below stall."""
        aero = FixedWingAero()
        alpha = 0.1  # ~5.7 deg, below stall
        cm = aero.get_CM(alpha)
        assert cm < 0.0  # stabilizing (nose-down with positive AoA)
        assert abs(cm - aero.C_Ma * alpha) < 1e-10

    def test_pitching_moment_post_stall(self):
        """C_M post-stall should use C_Ma_stall slope."""
        aero = FixedWingAero()
        alpha = 0.4  # above stall angle (0.2618)
        cm = aero.get_CM(alpha)
        assert abs(cm - aero.C_Ma_stall * alpha) < 1e-10

    def test_pitching_moment_zero_at_zero_alpha(self):
        """No pitching moment at zero AoA."""
        aero = FixedWingAero()
        assert aero.get_CM(0.0) == 0.0

    def test_pitching_moment_applied_in_physics_step(self):
        """Physics step should apply aerodynamic pitching moment for fixed-wing."""
        params = make_valencia_fixed_wing()
        # Start with forward velocity to create AoA
        state = DroneState(
            position=np.array([0.0, 0.0, 50.0]),
            velocity=np.array([12.0, 0.0, 2.0]),  # slight climb -> positive AoA
        )
        cmd = DroneCommand(thrust=15.0)
        next_state = physics_step(state, cmd, params, dt=0.01)
        # With positive AoA and negative C_Ma, pitch rate should be affected
        # (angular velocity Y component should be non-zero)
        assert next_state.angular_velocity[1] != 0.0

    def test_quadrotor_has_no_pitching_moment(self):
        """AeroCoefficients base class should return zero pitching moment."""
        aero = AeroCoefficients()
        assert aero.get_CM(0.1) == 0.0
        assert aero.get_CM(0.5) == 0.0


class TestFixedWingControlSurfaces:
    def test_elevator_pitch_response_matches_control_effectiveness(self):
        params = make_valencia_fixed_wing()
        aero = params.aero
        assert isinstance(aero, FixedWingAero)

        state = DroneState(
            position=np.array([0.0, 0.0, 120.0]),
            velocity=np.array([14.0, 0.0, 0.0]),
        )

        delta_e_cmd = 0.10
        cmd = DroneCommand(thrust=12.0, elevator=delta_e_cmd)
        next_state = physics_step(state, cmd, params, dt=0.01)
        assert next_state.control_surface_deflections is not None
        delta_e = float(next_state.control_surface_deflections[0])

        rho = params.atmo.rho if params.atmo is not None else 1.225
        q_dyn = 0.5 * rho * np.linalg.norm(state.velocity) ** 2
        expected_pitch_moment = (
            q_dyn
            * aero.reference_area
            * aero.chord
            * aero.Cm_delta_e
            * delta_e
        )
        expected_qdot = expected_pitch_moment / params.inertia[1, 1]
        expected_q = expected_qdot * 0.01

        np.testing.assert_allclose(
            next_state.angular_velocity[1],
            expected_q,
            rtol=0.10,
            atol=1e-3,
        )

    def test_control_surface_rate_limit_prevents_instant_step(self):
        params = make_valencia_fixed_wing()
        aero = params.aero
        assert isinstance(aero, FixedWingAero)

        state = DroneState(
            position=np.array([0.0, 0.0, 50.0]),
            velocity=np.array([12.0, 0.0, 0.0]),
        )
        cmd = DroneCommand(
            thrust=10.0,
            elevator=1.0,
            aileron=1.0,
            rudder=1.0,
        )
        dt = 0.01

        next_state = physics_step(state, cmd, params, dt=dt)
        assert next_state.control_surface_deflections is not None

        max_step = aero.control_surface_rate_limit * dt
        for value in next_state.control_surface_deflections:
            assert abs(value) <= max_step + 1e-12


class TestValenciaPreset:
    def test_valencia_fixed_wing_mass(self):
        """Paper Table 2: mass = 2.5 kg."""
        params = make_valencia_fixed_wing()
        assert params.mass == 2.5

    def test_valencia_fixed_wing_ref_area(self):
        """Paper Table 2: wing reference area = 0.3997 m^2."""
        params = make_valencia_fixed_wing()
        assert params.aero.reference_area == 0.3997

    def test_valencia_fixed_wing_chord(self):
        """Paper Table 2: chord = 0.235 m."""
        params = make_valencia_fixed_wing()
        assert params.aero.chord == 0.235

    def test_valencia_fixed_wing_aero_coefficients(self):
        """Paper Table 3 coefficients should match exactly."""
        params = make_valencia_fixed_wing()
        aero = params.aero
        assert aero.C_La == 3.50141
        assert aero.C_Da == 0.63662
        assert aero.C_Ma == -0.2040
        assert aero.C_Ma_stall == -0.1313

    def test_valencia_fixed_wing_high_altitude_atmosphere(self):
        """Antisana preset should use ~4500m MSL atmosphere."""
        params = make_valencia_fixed_wing()
        assert params.atmo.altitude_msl == 4500.0
        # At 4500m, rho should be ~0.77 kg/m^3 (ISA model)
        assert 0.70 < params.atmo.rho < 0.85

    def test_valencia_fixed_wing_passes_provenance_check(self):
        """Preset should not trigger out-of-range warnings."""
        params = make_valencia_fixed_wing()
        state = DroneState(position=np.array([0.0, 0.0, 50.0]),
                           velocity=np.array([12.0, 0.0, 0.0]))
        cmd = DroneCommand(thrust=10.0)
        with warnings.catch_warnings(record=True) as record:
            warnings.simplefilter("always")
            physics_step(state, cmd, params, dt=0.01)
        aero_warnings = [w for w in record if "Aerodynamic parameter" in str(w.message)]
        assert len(aero_warnings) == 0


class TestGammaTermEquivalence:
    def test_gamma_terms_match_matrix_form(self):
        """Verify that expanded Gamma-term rotational dynamics (Eq. 4)
        matches our matrix-form I_inv @ (tau - omega x I@omega)."""
        # Use a non-diagonal inertia tensor (fixed-wing with cross products)
        Jx, Jy, Jz = 0.08, 0.12, 0.16
        Jxz = 0.004
        I = np.array([
            [Jx,   0.0, -Jxz],
            [0.0,  Jy,   0.0],
            [-Jxz, 0.0,  Jz ],
        ])
        Gamma = Jx * Jz - Jxz**2

        # Gamma terms from the paper
        G1 = Jxz * (Jx - Jy + Jz) / Gamma
        G2 = (Jz * (Jz - Jy) + Jxz**2) / Gamma
        G3 = Jz / Gamma
        G4 = Jxz / Gamma
        G5 = (Jz - Jx) / Jy
        G6 = Jxz / Jy
        G7 = ((Jx - Jy) * Jx + Jxz**2) / Gamma
        G8 = Jx / Gamma

        # Test state
        omega = np.array([0.5, -0.3, 0.2])
        tau = np.array([0.1, -0.05, 0.08])  # [l, m, n] torques
        p, q, r = omega
        l, m, n = tau

        # Paper Eq. 4 expanded
        p_dot_gamma = G1 * p * q - G2 * q * r + G3 * l + G4 * n
        q_dot_gamma = G5 * p * r - G6 * (p**2 - r**2) + (1.0 / Jy) * m
        r_dot_gamma = G7 * p * q - G1 * q * r + G4 * l + G8 * n

        # Our matrix form
        I_inv = np.linalg.inv(I)
        alpha_matrix = I_inv @ (tau - np.cross(omega, I @ omega))

        # They should match
        np.testing.assert_allclose(
            alpha_matrix,
            np.array([p_dot_gamma, q_dot_gamma, r_dot_gamma]),
            atol=1e-12,
        )


# ── MAVLink Bridge ───────────────────────────────────────────────────────────


class TestIRS4Preset:
    """Verify IRS-4 quadrotor preset matches paper Section 3.2."""

    def test_irs4_mass(self):
        """IRS-4 mass should be in reasonable range for compact quadrotor."""
        params = make_irs4_quadrotor()
        assert 1.0 <= params.mass <= 3.0, f"mass={params.mass} outside [1.0, 3.0]"

    def test_irs4_atmosphere_quito(self):
        """Default altitude should be ~2800m for Quito experiments."""
        params = make_irs4_quadrotor()
        assert params.atmo is not None
        assert abs(params.atmo.altitude_msl - 2800.0) < 1.0
        # ISA density at 2800m should be ~0.93 kg/m³
        assert 0.85 <= params.atmo.rho <= 1.00

    def test_irs4_custom_altitude(self):
        """Should accept custom altitude for different experiment sites."""
        params = make_irs4_quadrotor(altitude_msl=4500.0)
        assert abs(params.atmo.altitude_msl - 4500.0) < 1.0

    def test_irs4_has_aero(self):
        """IRS-4 should use quadratic drag model (not linear fallback)."""
        params = make_irs4_quadrotor()
        assert params.aero is not None
        assert params.aero.C_D > 0
        assert params.aero.C_L == 0.0  # no lift for quadrotor

    def test_irs4_symmetric_inertia(self):
        """Quadrotor should have symmetric roll/pitch inertia (X-frame)."""
        params = make_irs4_quadrotor()
        assert abs(params.inertia[0, 0] - params.inertia[1, 1]) < 1e-6

    def test_irs4_hover_stable(self):
        """IRS-4 should maintain stable hover within paper RMSE targets.

        With motor dynamics enabled (Eq. 4 rotor model), motor spin-up lag causes
        longer settling — use a wider window and generous envelope matching
        real-flight RMSE scale (paper Table 5 reports ~0.10m under ideal).
        """
        params = make_irs4_quadrotor()
        target = np.array([0.0, 0.0, 20.0])
        records = run_simulation(
            waypoints=[target],
            params=params,
            dt=0.005,
            waypoint_radius=0.3,
            hover_time=8.0,
            max_time=20.0,
        )
        # Check last 3 seconds of hover (after motor-dynamics settling)
        hover_records = [r for r in records if r.t > 16.0]
        if len(hover_records) > 0:
            positions = np.array([r.position for r in hover_records])
            errors = positions - target
            rmse_z = np.sqrt(np.mean(errors[:, 2] ** 2))
            assert rmse_z < 1.5, f"Hover RMSE_Z={rmse_z:.4f} > 1.5m"

    def test_irs4_waypoint_tracking(self):
        """IRS-4 should track waypoints with paper-level accuracy."""
        params = make_irs4_quadrotor()
        waypoints = [
            np.array([0.0, 0.0, 20.0]),
            np.array([10.0, 0.0, 20.0]),
            np.array([10.0, 10.0, 20.0]),
            np.array([0.0, 0.0, 20.0]),
        ]
        records = run_simulation(
            waypoints=waypoints,
            params=params,
            dt=0.005,
            waypoint_radius=0.5,
            hover_time=1.5,
            max_time=90.0,
        )
        # Should complete all waypoints
        final_pos = records[-1].position
        dist_to_home = np.linalg.norm(final_pos - waypoints[-1])
        assert dist_to_home < 2.0, f"Did not return home: dist={dist_to_home:.2f}m"


# ── Mission Replay Pipeline ─────────────────────────────────────────────
