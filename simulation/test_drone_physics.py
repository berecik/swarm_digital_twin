"""
Drone Physics Tests - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app
"""

import numpy as np
import pytest
import warnings
from drone_physics import (
    DroneParams, DroneState, DroneCommand,
    PositionController, PIDController,
    physics_step, euler_to_rotation, rotation_to_euler,
    run_simulation, GRAVITY,
    AeroCoefficients, Atmosphere, _compute_quadratic_drag,
    _compute_lift, compute_aoa,
    FixedWingAero, make_generic_quad, make_holybro_x500, make_fixed_wing,
    make_valencia_fixed_wing,
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
)
from drone_scenario import run_benchmark
from swarm_scenario import run_swarm_benchmark, SWARM_BENCHMARK_PROFILES, get_swarm_benchmark_profile


# ── Rotation helpers ─────────────────────────────────────────────────────────

class TestRotation:
    def test_identity(self):
        R = euler_to_rotation(0, 0, 0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_roundtrip(self):
        for _ in range(50):
            r, p, y = np.random.uniform(-0.5, 0.5, 3)
            R = euler_to_rotation(r, p, y)
            r2, p2, y2 = rotation_to_euler(R)
            R2 = euler_to_rotation(r2, p2, y2)
            np.testing.assert_allclose(R, R2, atol=1e-10)

    def test_orthogonality(self):
        R = euler_to_rotation(0.3, -0.2, 1.1)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


# ── Gravity free-fall ────────────────────────────────────────────────────────

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


# ── Phase 1: Quadratic Drag & Atmosphere ────────────────────────────────────

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


class TestAtmosphere:
    def test_sea_level_density(self):
        """Sea level should have standard ISA density."""
        atmo = Atmosphere()
        np.testing.assert_allclose(atmo.rho, 1.225, rtol=1e-6)

    def test_high_altitude_density(self):
        """4500m should have significantly lower density."""
        atmo = Atmosphere(altitude_msl=4500.0)
        assert atmo.rho < 1.0
        # ISA at 4500m: ~0.77 kg/m³
        np.testing.assert_allclose(atmo.rho, 0.7695, rtol=0.02)


# ── Phase 1: Wind Model ────────────────────────────────────────────────────

class TestWind:
    def test_no_wind_unchanged(self):
        """Wind with type='none' produces zero force."""
        wind = WindField(turbulence_type="none")
        force = wind.get_force(0.0, np.array([0.0, 0.0, 10.0]), None, 1.225)
        np.testing.assert_allclose(force, np.zeros(3))

    def test_constant_wind_deflects_hover(self):
        """Drone hovering in constant wind should drift downwind."""
        wind = WindField(wind_speed=5.0,
                         wind_direction=np.array([1.0, 0.0, 0.0]),
                         turbulence_type="constant")
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):  # 10 seconds
            state = physics_step(state, cmd, params, dt=0.005, wind=wind, t=0.0)

        # Should have drifted in +x direction
        assert state.position[0] > 1.0

    def test_stronger_wind_more_force(self):
        """Force should scale with V^2."""
        aero = AeroCoefficients(reference_area=0.04, C_D=1.0)
        w1 = WindField(wind_speed=5.0,
                       wind_direction=np.array([1.0, 0.0, 0.0]),
                       turbulence_type="constant")
        w2 = WindField(wind_speed=10.0,
                       wind_direction=np.array([1.0, 0.0, 0.0]),
                       turbulence_type="constant")

        pos = np.array([0.0, 0.0, 10.0])
        F1 = np.linalg.norm(w1.get_force(0.0, pos, aero, 1.225))
        F2 = np.linalg.norm(w2.get_force(0.0, pos, aero, 1.225))

        # F2/F1 should be (10/5)^2 = 4
        np.testing.assert_allclose(F2 / F1, 4.0, rtol=1e-6)

    def test_wind_from_log_matches_data(self):
        """Replayed log should produce expected force profile."""
        profile = np.array([
            [0.0, 0.0],
            [5.0, 5.0],
            [10.0, 10.0],
        ])
        wind = WindField(
            wind_direction=np.array([0.0, 1.0, 0.0]),
            turbulence_type="from_log",
            altitude_profile=profile,
        )

        # At t=0, wind speed = 0 → zero force
        v0 = wind.get_wind_velocity(0.0, np.zeros(3))
        np.testing.assert_allclose(np.linalg.norm(v0), 0.0, atol=1e-8)

        # At t=5, wind speed = 5 m/s in +y
        v5 = wind.get_wind_velocity(5.0, np.zeros(3))
        np.testing.assert_allclose(v5, [0.0, 5.0, 0.0], atol=1e-8)

        # At t=7.5 (interpolated), wind speed = 7.5 m/s
        v75 = wind.get_wind_velocity(7.5, np.zeros(3))
        np.testing.assert_allclose(np.linalg.norm(v75), 7.5, atol=1e-8)


# ── Phase 1: Inertia & Presets ──────────────────────────────────────────────

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


# ── Phase 1: Body-Frame Dynamics ────────────────────────────────────────────

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

        for _ in range(int(t_total / dt)):
            state = physics_step(state, cmd, params_body, dt)

        expected_z = 100.0 - 0.5 * GRAVITY * t_total**2
        expected_vz = -GRAVITY * t_total

        np.testing.assert_allclose(state.position[2], expected_z, atol=0.1)
        np.testing.assert_allclose(state.velocity[2], expected_vz, atol=0.1)


# ── Phase 1: Validation Module ──────────────────────────────────────────────

class TestValidation:
    def test_rmse_identical(self):
        """RMSE of identical trajectories should be zero."""
        from validation import compute_rmse
        traj = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=float)
        result = compute_rmse(traj, traj)
        assert result.rmse_total == 0.0
        assert result.rmse_x == 0.0

    def test_rmse_known_offset(self):
        """RMSE of constant-offset trajectories should equal offset magnitude."""
        from validation import compute_rmse
        traj1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
        traj2 = traj1 + np.array([1.0, 0.0, 0.0])
        result = compute_rmse(traj1, traj2)
        np.testing.assert_allclose(result.rmse_x, 1.0, atol=1e-10)
        np.testing.assert_allclose(result.rmse_total, 1.0, atol=1e-10)

    def test_flight_log_csv_roundtrip(self):
        """FlightLog should parse a CSV and return positions."""
        import tempfile, os
        from flight_log import FlightLog

        csv_content = "time,lat,lon,alt,roll,pitch,yaw\n"
        csv_content += "0.0,47.0,8.0,400.0,0,0,0\n"
        csv_content += "1.0,47.0001,8.0,401.0,1.0,0.5,0\n"
        csv_content += "2.0,47.0002,8.0,402.0,0,0,0\n"

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv",
                                          delete=False) as f:
            f.write(csv_content)
            tmp_path = f.name

        try:
            log = FlightLog.from_csv(tmp_path)
            traj = log.get_trajectory()
            assert traj.shape == (3, 3)
            assert len(log.timestamps) == 3
            # First point should be near origin (it IS the origin)
            np.testing.assert_allclose(traj[0], [0, 0, 0], atol=0.1)
        finally:
            os.unlink(tmp_path)

    def test_validation_gate_passes_within_envelope(self):
        result = ValidationResult(
            rmse_x=0.2,
            rmse_y=0.3,
            rmse_z=0.1,
            rmse_total=0.4,
            median_error=0.3,
            p25_error=0.2,
            p75_error=0.4,
            max_error=0.8,
            n_points=100,
        )
        envelope = ValidationEnvelope(
            rmse_x_max=0.5,
            rmse_y_max=0.5,
            rmse_z_max=0.5,
            rmse_total_max=0.6,
            median_error_max=0.5,
            p75_error_max=0.6,
            max_error_max=1.0,
        )
        assert_validation_pass(result, envelope, profile_name="unit")

    def test_validation_gate_fails_outside_envelope(self):
        result = ValidationResult(
            rmse_x=0.7,
            rmse_y=0.3,
            rmse_z=0.1,
            rmse_total=0.4,
            median_error=0.3,
            p25_error=0.2,
            p75_error=0.4,
            max_error=1.2,
            n_points=100,
        )
        envelope = ValidationEnvelope(
            rmse_x_max=0.5,
            rmse_y_max=0.5,
            rmse_z_max=0.5,
            rmse_total_max=0.6,
            median_error_max=0.5,
            p75_error_max=0.6,
            max_error_max=1.0,
        )
        with pytest.raises(AssertionError, match="Validation failed"):
            assert_validation_pass(result, envelope, profile_name="unit")

    @pytest.mark.parametrize("profile_name", sorted(BENCHMARK_PROFILES.keys()))
    def test_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_benchmark_profile(profile_name)
        first = run_benchmark(profile_name)
        second = run_benchmark(profile_name)

        np.testing.assert_allclose(first.rmse_x, second.rmse_x, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_y, second.rmse_y, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_z, second.rmse_z, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_total, second.rmse_total, atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name", sorted(SWARM_BENCHMARK_PROFILES.keys()))
    def test_swarm_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_swarm_benchmark_profile(profile_name)
        first = run_swarm_benchmark(profile_name)
        second = run_swarm_benchmark(profile_name)

        np.testing.assert_allclose(first["min_separation"], second["min_separation"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p05_separation"], second["p05_separation"], atol=profile.tolerance)
        np.testing.assert_allclose(first["mean_tracking_error"], second["mean_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p75_tracking_error"], second["p75_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["max_tracking_error"], second["max_tracking_error"], atol=profile.tolerance)
        np.testing.assert_allclose(first["mean_speed"], second["mean_speed"], atol=profile.tolerance)
        np.testing.assert_allclose(first["p90_speed"], second["p90_speed"], atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name", sorted(SWARM_BENCHMARK_PROFILES.keys()))
    def test_swarm_benchmark_profiles_stay_within_envelopes(self, profile_name):
        profile = get_swarm_benchmark_profile(profile_name)
        metrics = run_swarm_benchmark(profile_name)

        assert metrics["min_separation"] >= profile.envelope.min_separation_min
        assert metrics["p05_separation"] >= profile.envelope.p05_separation_min
        assert metrics["mean_tracking_error"] <= profile.envelope.mean_tracking_error_max
        assert metrics["p75_tracking_error"] <= profile.envelope.p75_tracking_error_max
        assert metrics["max_tracking_error"] <= profile.envelope.max_tracking_error_max
        assert metrics["mean_speed"] <= profile.envelope.mean_speed_max
        assert metrics["p90_speed"] <= profile.envelope.p90_speed_max

    def test_swarm_profile_risk_ordering(self):
        baseline = run_swarm_benchmark("baseline")
        tight_ring = run_swarm_benchmark("tight_ring")

        # Tight ring formation has tighter separation (more collision risk)
        assert tight_ring["min_separation"] < baseline["min_separation"]
        # Tight ring has smaller tracking error due to shorter waypoint distances
        assert tight_ring["mean_tracking_error"] < baseline["mean_tracking_error"]


# ── Phase 2: Terrain ────────────────────────────────────────────────────────

class TestTerrain:
    def test_flat_terrain_elevation(self):
        """Flat terrain should return constant elevation everywhere."""
        t = TerrainMap.flat(elevation=50.0, size=200.0, resolution=10.0)
        np.testing.assert_allclose(t.get_elevation(0, 0), 50.0)
        np.testing.assert_allclose(t.get_elevation(99, -50), 50.0)
        np.testing.assert_allclose(t.get_elevation(-100, 100), 50.0)

    def test_from_array_elevation_query(self):
        """Elevation query should interpolate grid values correctly."""
        # 3x3 grid: center cell is a hill
        grid = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0],
        ])
        t = TerrainMap.from_array(grid, origin=(0.0, 0.0), resolution=1.0)
        # Exact grid point (1,1) → 10.0
        np.testing.assert_allclose(t.get_elevation(1.0, 1.0), 10.0)
        # Corner (0,0) → 0.0
        np.testing.assert_allclose(t.get_elevation(0.0, 0.0), 0.0)
        # Midpoint (0.5, 1.0) → bilinear interp of 0 and 10 → 5.0
        np.testing.assert_allclose(t.get_elevation(0.5, 1.0), 5.0, atol=0.01)

    def test_terrain_collision_detection(self):
        """check_collision should detect when position is below terrain."""
        grid = np.array([
            [5.0, 5.0],
            [5.0, 5.0],
        ])
        t = TerrainMap.from_array(grid, origin=(0.0, 0.0), resolution=10.0)
        assert t.check_collision(np.array([5.0, 5.0, 4.0])) == True
        assert t.check_collision(np.array([5.0, 5.0, 5.0])) == True
        assert t.check_collision(np.array([5.0, 5.0, 6.0])) == False

    def test_terrain_in_physics_step(self):
        """Drone should not fall below terrain surface."""
        # Hill at 20m elevation
        grid = np.full((10, 10), 20.0)
        t = TerrainMap.from_array(grid, origin=(-50.0, -50.0), resolution=10.0)

        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 25.0]))
        cmd = DroneCommand(thrust=0.0)  # no thrust, will fall

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005, terrain=t)

        # Should have landed on terrain at z=20, not z=0
        assert state.position[2] >= 20.0
        np.testing.assert_allclose(state.position[2], 20.0, atol=0.1)

    def test_from_function_terrain(self):
        """Terrain from analytical function should give correct elevations."""
        # Simple slope: z = 0.1 * x
        t = TerrainMap.from_function(
            lambda x, y: 0.1 * x,
            x_range=(0, 100), y_range=(0, 100), resolution=1.0,
        )
        np.testing.assert_allclose(t.get_elevation(50.0, 50.0), 5.0, atol=0.1)
        np.testing.assert_allclose(t.get_elevation(0.0, 0.0), 0.0, atol=0.1)
        np.testing.assert_allclose(t.get_elevation(100.0, 0.0), 10.0, atol=0.1)

    def test_stl_file_not_found(self):
        """Loading a nonexistent STL should raise FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            TerrainMap.from_stl("/nonexistent/terrain.stl")


# ── Phase 3: Fixed-Wing Aerodynamics ──────────────────────────────────────

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


# ── Phase D: Paper-exact aerodynamics ──────────────────────────────────────

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


# ── Phase 3: MAVLink Bridge ───────────────────────────────────────────────

class TestMAVLink:
    def test_crc_computation(self):
        """MAVLink CRC should produce consistent results."""
        from mavlink_bridge import mavlink_crc
        data = b'\x01\x02\x03\x04'
        crc1 = mavlink_crc(data, crc_extra=50)
        crc2 = mavlink_crc(data, crc_extra=50)
        assert crc1 == crc2
        assert isinstance(crc1, int)
        assert 0 <= crc1 <= 0xFFFF

    def test_heartbeat_encoding(self):
        """Heartbeat message should have correct MAVLink v2 header."""
        from mavlink_bridge import build_heartbeat, MAVLINK_STX_V2
        msg = build_heartbeat(system_id=1, component_id=1, armed=True)
        assert msg[0] == MAVLINK_STX_V2
        assert len(msg) > 10  # header + payload + CRC

    def test_heartbeat_decode_roundtrip(self):
        """Encoded heartbeat should decode successfully."""
        from mavlink_bridge import (build_heartbeat, decode_mavlink_v2,
                                     MAVLINK_MSG_ID_HEARTBEAT)
        msg = build_heartbeat(system_id=1, component_id=1)
        result = decode_mavlink_v2(msg)
        assert result is not None
        msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_HEARTBEAT
        assert len(payload) > 0

    def test_attitude_encode_decode(self):
        """Attitude message should roundtrip correctly."""
        from mavlink_bridge import (build_attitude, decode_mavlink_v2,
                                     MAVLINK_MSG_ID_ATTITUDE)
        import struct
        msg = build_attitude(roll=0.1, pitch=-0.2, yaw=1.5,
                             time_boot_ms=5000)
        result = decode_mavlink_v2(msg)
        assert result is not None
        msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_ATTITUDE
        fields = struct.unpack_from('<Iffffff', payload)
        assert fields[0] == 5000  # time_boot_ms
        np.testing.assert_allclose(fields[1], 0.1, atol=1e-6)   # roll
        np.testing.assert_allclose(fields[2], -0.2, atol=1e-6)  # pitch
        np.testing.assert_allclose(fields[3], 1.5, atol=1e-6)   # yaw

    def test_global_position_encode_decode(self):
        """GPS position message should encode lat/lon correctly."""
        from mavlink_bridge import (build_global_position_int,
                                     decode_mavlink_v2,
                                     MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        import struct
        msg = build_global_position_int(
            lat_deg=47.3769, lon_deg=8.5417,
            alt_msl_m=500.0, alt_rel_m=92.0,
            time_boot_ms=10000,
        )
        result = decode_mavlink_v2(msg)
        assert result is not None
        msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT
        fields = struct.unpack_from('<IiiiihhhH', payload)
        # lat in 1e-7 degrees
        np.testing.assert_allclose(fields[1] / 1e7, 47.3769, atol=1e-4)
        np.testing.assert_allclose(fields[2] / 1e7, 8.5417, atol=1e-4)

    def test_vfr_hud_encode_decode(self):
        """VFR HUD message should roundtrip."""
        from mavlink_bridge import (build_vfr_hud, decode_mavlink_v2,
                                     MAVLINK_MSG_ID_VFR_HUD)
        import struct
        msg = build_vfr_hud(
            airspeed=15.5, groundspeed=14.2,
            heading_deg=90, throttle_pct=65,
            alt_msl=500.0, climb_rate=2.5,
        )
        result = decode_mavlink_v2(msg)
        assert result is not None
        msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_VFR_HUD
        fields = struct.unpack_from('<ffhHff', payload)
        np.testing.assert_allclose(fields[0], 15.5, atol=1e-4)
        np.testing.assert_allclose(fields[1], 14.2, atol=1e-4)

    def test_command_long_parse(self):
        """COMMAND_LONG parsing should extract command ID and params."""
        from mavlink_bridge import (parse_mavlink_payload,
                                     MAVLINK_MSG_ID_COMMAND_LONG,
                                     MAV_CMD_COMPONENT_ARM_DISARM)
        import struct
        # Build a COMMAND_LONG payload: 7 params(f32) + command(u16) +
        # target_system(u8) + target_component(u8) + confirmation(u8)
        payload = struct.pack('<fffffffHBBB',
                              1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # params
                              MAV_CMD_COMPONENT_ARM_DISARM,          # command
                              1, 1, 0)                               # target, confirm
        result = parse_mavlink_payload(MAVLINK_MSG_ID_COMMAND_LONG, payload)
        assert result is not None
        assert result.command_id == MAV_CMD_COMPONENT_ARM_DISARM
        np.testing.assert_allclose(result.param1, 1.0)

    def test_enu_to_gps_conversion(self):
        """ENU→GPS conversion should produce correct lat/lon offsets."""
        from mavlink_bridge import _enu_to_gps
        # Zero offset should return reference point
        lat, lon, alt = _enu_to_gps(np.array([0.0, 0.0, 0.0]),
                                     ref_lat=47.3769, ref_lon=8.5417,
                                     ref_alt=408.0)
        np.testing.assert_allclose(lat, 47.3769, atol=1e-6)
        np.testing.assert_allclose(lon, 8.5417, atol=1e-6)
        np.testing.assert_allclose(alt, 408.0, atol=0.1)

        # 100m North should increase latitude
        lat2, _, _ = _enu_to_gps(np.array([0.0, 100.0, 0.0]),
                                  ref_lat=47.3769, ref_lon=8.5417,
                                  ref_alt=408.0)
        assert lat2 > lat

    def test_sim_state_from_record(self):
        """SimState should be correctly populated from a SimRecord."""
        from mavlink_bridge import sim_state_from_record
        from drone_physics import SimRecord
        record = SimRecord(
            t=5.0,
            position=np.array([10.0, 20.0, 30.0]),
            velocity=np.array([1.0, 2.0, 3.0]),
            euler=(0.1, -0.2, 1.5),
            thrust=15.0,
            angular_velocity=np.array([0.01, -0.02, 0.03]),
        )
        state = sim_state_from_record(record, max_thrust=25.0)
        assert state.time_s == 5.0
        np.testing.assert_allclose(state.position, [10.0, 20.0, 30.0])
        np.testing.assert_allclose(state.roll, 0.1)
        np.testing.assert_allclose(state.thrust_pct, 60.0)  # 15/25 * 100

    def test_invalid_message_returns_none(self):
        """Decoding garbage data should return None."""
        from mavlink_bridge import decode_mavlink_v2
        assert decode_mavlink_v2(b'\x00\x01\x02') is None
        assert decode_mavlink_v2(b'') is None
        assert decode_mavlink_v2(b'\xFD' + b'\x00' * 20) is None


# ── Phase C: Swarm-ready standalone twin ───────────────────────────────────

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
            dt=0.02,
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


# ── Phase E: Flight Log & Validation Pipeline ──────────────────────────────

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


class TestSimVsRealComparison:
    def test_compare_sim_real_identical(self):
        """Identical trajectories should yield zero RMSE."""
        from validation import compare_sim_real
        t = np.linspace(0, 10, 100)
        pos = np.column_stack([t, np.sin(t), np.zeros(100)])
        result = compare_sim_real(t, pos, t, pos)
        assert result["rmse_z"] == pytest.approx(0.0, abs=1e-10)
        assert result["rmse_total"] == pytest.approx(0.0, abs=1e-10)

    def test_compare_sim_real_known_offset(self):
        """Known Z offset should appear in rmse_z."""
        from validation import compare_sim_real
        t = np.linspace(0, 10, 100)
        sim_pos = np.column_stack([np.zeros(100), np.zeros(100), np.zeros(100)])
        ref_pos = np.column_stack([np.zeros(100), np.zeros(100), np.ones(100) * 2.0])
        result = compare_sim_real(t, sim_pos, t, ref_pos)
        assert result["rmse_z"] == pytest.approx(2.0, abs=0.1)

    def test_compare_signals_perfect_match(self):
        """Identical signals should have correlation ~1.0."""
        from validation import compare_signals
        t = np.linspace(0, 10, 100)
        signal = np.sin(t)
        result = compare_signals(t, signal, t, signal)
        assert result["cross_correlation"] == pytest.approx(1.0, abs=1e-6)
        assert result["rmse"] == pytest.approx(0.0, abs=1e-10)

    def test_compare_signals_anticorrelated(self):
        """Opposite signals should have negative correlation."""
        from validation import compare_signals
        t = np.linspace(0, 10, 100)
        result = compare_signals(t, np.sin(t), t, -np.sin(t))
        assert result["cross_correlation"] < -0.9


# ── Phase F: Terrain Pipeline ──────────────────────────────────────────────

class TestTerrainSRTM:
    def test_from_hgt_synthetic(self, tmp_path):
        """from_hgt should load a synthetic .hgt file and return terrain."""
        # Create synthetic SRTM3 tile (1201x1201, big-endian int16)
        size = 1201
        grid = np.full((size, size), 4500, dtype='>i2')
        hgt_path = str(tmp_path / "S01W079.hgt")
        with open(hgt_path, 'wb') as f:
            f.write(grid.tobytes())

        terrain = TerrainMap.from_hgt(hgt_path, lat=-0.5, lon=-78.5, size_km=2.0)
        assert terrain.origin_gps is not None
        assert terrain.elevations.shape[0] > 1
        assert terrain.elevations.shape[1] > 1
        # Elevation should be ~4500m
        center_elev = terrain.get_elevation(0, 0)
        assert 4000 < center_elev < 5000

    def test_gps_elevation_query(self):
        """get_elevation_gps should convert lat/lon to local coords."""
        # Create terrain with known GPS origin
        grid = np.full((10, 10), 100.0)
        terrain = TerrainMap(
            elevations=grid,
            origin=(-500, -500),
            resolution=100.0,
            origin_gps=(-0.5, -78.0, 100.0),
        )
        # Query at origin should return ~100m
        elev = terrain.get_elevation_gps(-0.5, -78.0)
        assert 90 < elev < 110

    def test_gps_query_without_origin_raises(self):
        """get_elevation_gps without origin_gps should raise ValueError."""
        terrain = TerrainMap.flat(100.0)
        with pytest.raises(ValueError, match="origin_gps not set"):
            terrain.get_elevation_gps(-0.5, -78.0)

    def test_hgt_parser_file_not_found(self):
        """Missing .hgt file should raise FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            TerrainMap.from_hgt("/nonexistent/S01W079.hgt", lat=-0.5, lon=-78.5)


# ── Phase G: Gazebo Model Validation ───────────────────────────────────────

class TestGazeboModels:
    def test_sdf_model_exists(self):
        """Valencia fixed-wing SDF model file should exist."""
        import os
        sdf_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        assert os.path.exists(sdf_path), f"SDF not found: {sdf_path}"

    def test_sdf_model_config_exists(self):
        """Model config file should exist."""
        import os
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "models", "valencia_fixed_wing", "model.config"
        )
        assert os.path.exists(config_path)

    def test_sdf_contains_liftdrag_plugin(self):
        """SDF should contain LiftDrag plugin with Table 3 coefficients."""
        import os
        sdf_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        content = open(sdf_path).read()
        assert "LiftDragPlugin" in content
        assert "3.50141" in content  # C_La
        assert "0.63662" in content  # C_Da
        assert "-0.2040" in content  # C_Ma

    def test_sdf_mass_matches_paper(self):
        """SDF model mass should match paper Table 2 (2.5 kg)."""
        import os
        sdf_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        content = open(sdf_path).read()
        assert "<mass>2.5</mass>" in content

    def test_antisana_world_exists(self):
        """Antisana Gazebo world file should exist."""
        import os
        world_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "worlds", "antisana.world"
        )
        assert os.path.exists(world_path)

    def test_antisana_world_gps_origin(self):
        """Antisana world should have correct GPS coordinates."""
        import os
        world_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "worlds", "antisana.world"
        )
        content = open(world_path).read()
        assert "-0.508333" in content  # Antisana latitude
        assert "-78.141667" in content  # Antisana longitude
        assert "4500" in content  # Elevation

    def test_parm_file_exists(self):
        """ArduPilot parameter file should exist."""
        import os
        parm_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "gazebo", "models", "valencia_fixed_wing", "valencia_fw.parm"
        )
        assert os.path.exists(parm_path)


# ── Phase H: SITL Lifecycle Script ─────────────────────────────────────────

class TestSITLLifecycle:
    def test_sitl_script_exists(self):
        """SITL mission lifecycle script should exist and be executable."""
        import os, stat
        script_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "scripts", "run_sitl_mission.sh"
        )
        assert os.path.exists(script_path)
        mode = os.stat(script_path).st_mode
        assert mode & stat.S_IXUSR  # executable

    def test_sitl_script_has_required_steps(self):
        """Script should contain all 6 lifecycle steps."""
        import os
        script_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "scripts", "run_sitl_mission.sh"
        )
        content = open(script_path).read()
        assert "[1/6]" in content  # Start stack
        assert "[2/6]" in content  # Health check
        assert "[3/6]" in content  # Upload mission
        assert "[4/6]" in content  # Arm and fly
        assert "[5/6]" in content  # Capture logs
        assert "[6/6]" in content  # Validate


# ── Phase H2: 3D Wind Estimation ─────────────────────────────────────────

class TestWindEstimation3D:
    def test_still_trajectory_gives_zero_wind(self):
        """Stationary drone should produce near-zero wind estimate."""
        from flight_log import FlightLog
        n = 100
        log = FlightLog(
            timestamps=np.linspace(0, 10, n),
            positions=np.tile([0.0, 0.0, -50.0], (n, 1)),
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape[1] == 4  # [t, wx, wy, wz]
        # Stationary -> zero velocity deviations -> zero wind
        np.testing.assert_allclose(profile[:, 1:], 0.0, atol=1e-10)

    def test_constant_drift_detected(self):
        """Constant lateral drift should produce non-zero X/Y wind estimate."""
        from flight_log import FlightLog
        n = 200
        t = np.linspace(0, 20, n)
        # Straight-line flight along X, but with constant Y drift (wind pushing in Y)
        positions = np.column_stack([
            t * 5.0,           # X: steady forward flight
            t * 0.0 + np.sin(t * 0.5) * 2.0,  # Y: sinusoidal deviation
            np.full(n, -50.0),  # Z: constant altitude
        ])
        log = FlightLog(
            timestamps=t,
            positions=positions,
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape[0] == n - 1
        # Should detect non-zero Y-component wind
        y_wind_rms = np.sqrt(np.mean(profile[:, 2] ** 2))
        assert y_wind_rms > 0.01, f"Expected non-zero Y wind, got RMS={y_wind_rms}"

    def test_3d_profile_has_correct_shape(self):
        """3D wind profile should have Nx4 shape [t, wx, wy, wz]."""
        from flight_log import FlightLog
        n = 50
        log = FlightLog(
            timestamps=np.linspace(0, 5, n),
            positions=np.random.randn(n, 3).cumsum(axis=0),
            attitudes=np.zeros((n, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape == (n - 1, 4)
        # Timestamps should match input (minus last)
        np.testing.assert_allclose(profile[:, 0], log.timestamps[:-1], atol=1e-10)

    def test_3d_profile_too_short_returns_zero(self):
        """Trajectory with < 3 points should return zero wind."""
        from flight_log import FlightLog
        log = FlightLog(
            timestamps=np.array([0.0, 1.0]),
            positions=np.array([[0, 0, 0], [1, 0, 0]], dtype=float),
            attitudes=np.zeros((2, 3)),
        )
        profile = log.get_wind_profile_3d()
        assert profile.shape == (1, 4)
        np.testing.assert_allclose(profile[0, 1:], 0.0, atol=1e-10)

    def test_from_log_3d_wind_replay(self):
        """WindField with from_log_3d should replay 3D wind profile."""
        profile = np.array([
            [0.0, 1.0, 2.0, -0.5],
            [5.0, 1.5, 2.5, -0.3],
            [10.0, 2.0, 3.0, -0.1],
        ])
        wf = WindField(
            turbulence_type="from_log_3d",
            wind_profile_3d=profile,
        )
        # At t=0
        v0 = wf.get_wind_velocity(0.0, np.zeros(3))
        np.testing.assert_allclose(v0, [1.0, 2.0, -0.5], atol=1e-10)

        # At t=5 (midpoint)
        v5 = wf.get_wind_velocity(5.0, np.zeros(3))
        np.testing.assert_allclose(v5, [1.5, 2.5, -0.3], atol=1e-10)

        # At t=2.5 (interpolated)
        v25 = wf.get_wind_velocity(2.5, np.zeros(3))
        np.testing.assert_allclose(v25, [1.25, 2.25, -0.4], atol=1e-10)


# ── Phase I1: CI Pipeline ────────────────────────────────────────────────

class TestCIPipeline:
    def test_ci_workflow_exists(self):
        """GitHub Actions CI workflow file should exist."""
        import os
        ci_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            ".github", "workflows", "ci.yml"
        )
        assert os.path.exists(ci_path)

    def test_ci_workflow_has_required_jobs(self):
        """CI workflow should run tests and benchmarks."""
        import os
        ci_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            ".github", "workflows", "ci.yml"
        )
        content = open(ci_path).read()
        assert "pytest" in content
        assert "benchmark" in content.lower()
        assert "upload-artifact" in content

    def test_ci_workflow_triggers_on_push(self):
        """CI should trigger on push to master/main."""
        import os
        ci_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            ".github", "workflows", "ci.yml"
        )
        content = open(ci_path).read()
        assert "push" in content
        assert "master" in content or "main" in content
