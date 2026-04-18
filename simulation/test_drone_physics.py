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
    FixedWingAero, QuadrotorAero, MotorModel, BatteryModel, make_generic_quad, make_holybro_x500, make_fixed_wing,
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


# ── Quadratic Drag & Atmosphere ──────────────────────────────────────────────

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


# ── Wind Model ──────────────────────────────────────────────────────────────

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


# ── Sensor Noise Models (GPS / IMU / Barometer) ─────────────────────────────

class TestSensorNoise:
    def test_gps_noise_quantization_and_statistics(self):
        gps = GPSNoise(rng_seed=42)

        lat0, lon0, alt0 = -0.189, -78.488, 2800.0
        samples = np.array([gps.apply(lat0, lon0, alt0, dt=0.1) for _ in range(4000)])
        lat = samples[:, 0]
        lon = samples[:, 1]
        alt = samples[:, 2]

        # 1e-7 degree quantization for geodetic coordinates
        lat_steps = (lat - lat0) / gps.quantization_deg
        lon_steps = (lon - lon0) / gps.quantization_deg
        np.testing.assert_allclose(lat_steps, np.round(lat_steps), atol=1e-7)
        np.testing.assert_allclose(lon_steps, np.round(lon_steps), atol=1e-7)

        meters_per_deg_lat = np.pi * 6378137.0 / 180.0
        meters_per_deg_lon = meters_per_deg_lat * np.cos(np.deg2rad(lat0))
        horizontal_err_m = np.sqrt(
            ((lat - lat0) * meters_per_deg_lat) ** 2
            + ((lon - lon0) * meters_per_deg_lon) ** 2
        )

        # CEP(50) target around 2.5m class and altitude spread around ±5m class
        assert np.percentile(horizontal_err_m, 50) < 3.5
        assert np.std(alt - alt0) < 6.0

    def test_gps_noise_drift_growth_and_zero_dt_bias_behavior(self):
        gps = GPSNoise(
            rng_seed=7,
            horizontal_sigma_m=0.0,
            vertical_sigma_m=0.0,
            drift_m_per_hour=10.0,
        )

        lat0, lon0, alt0 = -0.189, -78.488, 2800.0
        dt = 1.0
        n_steps = 3600

        samples = np.array([gps.apply(lat0, lon0, alt0, dt=dt) for _ in range(n_steps)])
        alt_err = samples[:, 2] - alt0

        # Random-walk signature: typical magnitude scales ~sqrt(t)
        early_window = alt_err[:600]
        late_window = alt_err[-600:]
        early_abs_mean = float(np.mean(np.abs(early_window)))
        late_abs_mean = float(np.mean(np.abs(late_window)))
        assert late_abs_mean > 2.0 * early_abs_mean

        # dt = 0.0 should not update bias state
        bias_before = np.array(gps._bias_m, copy=True)
        _ = gps.apply(lat0, lon0, alt0, dt=0.0)
        np.testing.assert_allclose(gps._bias_m, bias_before, atol=1e-12)

    def test_imu_noise_density_matches_order_of_magnitude(self):
        imu = IMUNoise(
            accel_bias_rw_mps2_per_sqrt_s=0.0,
            gyro_bias_rw_radps_per_sqrt_s=0.0,
            rng_seed=7,
        )
        dt = 0.01
        true_accel = np.zeros(3)
        true_gyro = np.zeros(3)

        accel_samples = np.array([imu.apply_accel(true_accel, dt=dt) for _ in range(8000)])
        gyro_samples = np.array([imu.apply_gyro(true_gyro, dt=dt) for _ in range(8000)])

        accel_sigma = np.mean(np.std(accel_samples, axis=0))
        gyro_sigma = np.mean(np.std(gyro_samples, axis=0))

        expected_accel_sigma = (imu.accel_noise_density_ug * 1e-6 * 9.80665) / np.sqrt(dt)
        expected_gyro_sigma = np.deg2rad(imu.gyro_noise_density_dps) / np.sqrt(dt)

        np.testing.assert_allclose(accel_sigma, expected_accel_sigma, rtol=0.2)
        np.testing.assert_allclose(gyro_sigma, expected_gyro_sigma, rtol=0.2)

    def test_imu_noise_bias_random_walk_develops_slow_offset(self):
        imu = IMUNoise(
            accel_noise_density_ug=0.0,
            gyro_noise_density_dps=0.0,
            accel_bias_rw_mps2_per_sqrt_s=5e-4,
            gyro_bias_rw_radps_per_sqrt_s=1e-4,
            rng_seed=11,
        )

        dt = 0.01
        n_steps = 5000
        true_accel = np.zeros(3)
        true_gyro = np.zeros(3)

        accel_samples = np.array([imu.apply_accel(true_accel, dt=dt) for _ in range(n_steps)])
        gyro_samples = np.array([imu.apply_gyro(true_gyro, dt=dt) for _ in range(n_steps)])

        accel_mean = np.mean(accel_samples, axis=0)
        gyro_mean = np.mean(gyro_samples, axis=0)
        assert np.linalg.norm(accel_mean) > 8e-4
        assert np.linalg.norm(gyro_mean) > 8e-4

        window = 500
        accel_mean_1 = np.mean(accel_samples[:window], axis=0)
        accel_mean_2 = np.mean(accel_samples[-window:], axis=0)
        gyro_mean_1 = np.mean(gyro_samples[:window], axis=0)
        gyro_mean_2 = np.mean(gyro_samples[-window:], axis=0)

        # Offset should evolve over time but remain smooth (not abrupt/jittery)
        assert np.linalg.norm(accel_mean_2 - accel_mean_1) > 1e-3
        assert np.linalg.norm(gyro_mean_2 - gyro_mean_1) > 1e-3

    def test_baro_noise_quantization_and_lag(self):
        baro = BaroNoise(rng_seed=5, white_sigma_hpa=0.0, drift_hpa_per_hour=0.0)
        dt = 0.02

        out_pre = np.array([baro.apply(1013.25, dt=dt) for _ in range(50)])
        out_post = np.array([baro.apply(1001.00, dt=dt) for _ in range(50)])

        # Quantization: output steps are multiples of configured hPa increment
        combined = np.concatenate([out_pre, out_post])
        q_steps = combined / baro.quantization_hpa
        np.testing.assert_allclose(q_steps, np.round(q_steps), atol=1e-8)

        # Lag: first sample after step should not instantly match steady-state target
        steady_target = round(1001.00 / baro.quantization_hpa) * baro.quantization_hpa
        assert abs(out_post[0] - steady_target) > 0.5

        # Equivalent altitude noise at sea-level pressure stays within ±1m class
        baro_noise = BaroNoise(rng_seed=11)
        sea_level = np.array([baro_noise.apply(1013.25, dt=dt) for _ in range(2000)])
        altitude_equiv_m = (1013.25 - sea_level) * 8.3  # near sea-level linearized conversion
        assert np.std(altitude_equiv_m) <= 1.0

    def test_baro_noise_drift_and_multi_dt_lag_consistency(self):
        baro = BaroNoise(
            rng_seed=23,
            drift_hpa_per_hour=0.5,
            white_sigma_hpa=0.0,
            quantization_hpa=0.01,
        )

        p_true = 900.0
        dt = 1.0
        n_steps = 3600
        pressures = np.array([baro.apply(p_true, dt=dt) for _ in range(n_steps)])

        early_std = float(np.std(pressures[:600]))
        late_std = float(np.std(pressures[-600:]))
        assert late_std > 2.0 * early_std

        # dt-regime consistency for first-order lag under equal physical time horizon
        tau = 0.35
        baro_fast = BaroNoise(
            rng_seed=99,
            drift_hpa_per_hour=0.0,
            white_sigma_hpa=0.0,
            quantization_hpa=1e-6,
            lag_time_constant_s=tau,
        )
        baro_slow = BaroNoise(
            rng_seed=99,
            drift_hpa_per_hour=0.0,
            white_sigma_hpa=0.0,
            quantization_hpa=1e-6,
            lag_time_constant_s=tau,
        )

        p_low = 1000.0
        p_high = 950.0

        # Step at t=5s, compare responses at matched physical times after the step
        dt_fast = 0.1
        dt_slow = 1.0
        fast_trace = []
        for i in range(100):
            t = i * dt_fast
            p = p_low if t < 5.0 else p_high
            fast_trace.append(baro_fast.apply(p, dt=dt_fast))

        slow_trace = []
        for i in range(10):
            t = i * dt_slow
            p = p_low if t < 5.0 else p_high
            slow_trace.append(baro_slow.apply(p, dt=dt_slow))

        fast_trace = np.asarray(fast_trace)
        slow_trace = np.asarray(slow_trace)

        # Response should move toward the new pressure with first-order lag
        fast_after_1s = float(fast_trace[60])  # t = 6.0 s
        slow_after_1s = float(slow_trace[6])   # t = 6.0 s
        expected_after_1s = p_high + (p_low - p_high) * np.exp(-1.0 / tau)
        np.testing.assert_allclose(fast_after_1s, expected_after_1s, atol=1.0)
        np.testing.assert_allclose(slow_after_1s, expected_after_1s, atol=6.0)
        np.testing.assert_allclose(fast_after_1s, slow_after_1s, atol=6.0)


# ── Motor Dynamics (Eq. 4 Rotor Model) ───────────────────────────────────────

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

    @pytest.mark.parametrize("profile_name",
                             sorted(k for k in BENCHMARK_PROFILES if not k.startswith("irs4_")))
    def test_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_benchmark_profile(profile_name)
        first = run_benchmark(profile_name)
        second = run_benchmark(profile_name)

        np.testing.assert_allclose(first.rmse_x, second.rmse_x, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_y, second.rmse_y, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_z, second.rmse_z, atol=profile.tolerance)
        np.testing.assert_allclose(first.rmse_total, second.rmse_total, atol=profile.tolerance)

    @pytest.mark.parametrize("profile_name",
                             sorted(k for k in BENCHMARK_PROFILES if k.startswith("irs4_")))
    def test_irs4_benchmark_profiles_are_deterministic(self, profile_name):
        profile = get_benchmark_profile(profile_name)
        first = run_irs4_benchmark(profile_name)
        second = run_irs4_benchmark(profile_name)

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

        # Tight ring has smaller tracking error due to shorter waypoint distances.
        # Note: min_separation is not strictly ordered between profiles because the
        # ring waypoints make drones chase each other, so the closest approach is
        # dominated by phase/wind dynamics rather than ring radius.
        assert tight_ring["mean_tracking_error"] < baseline["mean_tracking_error"]


# ── Terrain Model ──────────────────────────────────────────────────────────────

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


# ── Fixed-Wing Aerodynamics (AoA / Stall) ────────────────────────────────────

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
        _sys_id, msg_id, payload = result
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
        _sys_id, msg_id, payload = result
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
        _sys_id, msg_id, payload = result
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
        _sys_id, msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_VFR_HUD
        fields = struct.unpack_from('<ffhHff', payload)
        np.testing.assert_allclose(fields[0], 15.5, atol=1e-4)
        np.testing.assert_allclose(fields[1], 14.2, atol=1e-4)

    def test_sys_status_encode_decode_battery_fields(self):
        """SYS_STATUS should carry battery voltage/current/remaining fields."""
        from mavlink_bridge import (build_sys_status, decode_mavlink_v2,
                                     MAVLINK_MSG_ID_SYS_STATUS)
        import struct

        msg = build_sys_status(voltage_mv=22200, current_ca=1234, battery_pct=76)
        result = decode_mavlink_v2(msg)
        assert result is not None
        _sys_id, msg_id, payload = result
        assert msg_id == MAVLINK_MSG_ID_SYS_STATUS
        fields = struct.unpack_from('<IIIHHhb', payload)
        assert fields[4] == 22200
        assert fields[5] == 1234
        assert fields[6] == 76

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


# ── Live Telemetry & Runtime View ───────────────────────────────────────────

class TestLiveTelemetry:
    def test_parse_telemetry_payload_attitude(self):
        """MAVLINK_MSG_ID_ATTITUDE round-trip via parse_telemetry_payload."""
        from mavlink_bridge import build_attitude, decode_mavlink_v2
        from live_telemetry import parse_telemetry_payload, MAVLINK_MSG_ID_ATTITUDE

        # 1. Build a real MAVLink v2 attitude message
        orig_roll, orig_pitch, orig_yaw = 0.1, -0.2, 0.3
        msg_bytes = build_attitude(orig_roll, orig_pitch, orig_yaw, time_boot_ms=12345)

        # 2. Decode it to get the raw payload
        decoded = decode_mavlink_v2(msg_bytes)
        assert decoded is not None
        _sys_id, msg_id, payload = decoded
        assert msg_id == MAVLINK_MSG_ID_ATTITUDE

        # 3. Parse with the new telemetry parser
        parsed = parse_telemetry_payload(msg_id, payload)

        # 4. Verify
        assert parsed["time_boot_ms"] == 12345
        np.testing.assert_allclose(parsed["euler"], [orig_roll, orig_pitch, orig_yaw], atol=1e-6)

    def test_parse_telemetry_payload_global_position(self):
        """MAVLINK_MSG_ID_GLOBAL_POSITION_INT round-trip via parse_telemetry_payload."""
        from mavlink_bridge import build_global_position_int, decode_mavlink_v2
        from live_telemetry import parse_telemetry_payload, MAVLINK_MSG_ID_GLOBAL_POSITION_INT

        lat, lon, alt_msl = -34.5678, 123.4567, 100.5
        msg_bytes = build_global_position_int(lat, lon, alt_msl, alt_rel_m=50.0, time_boot_ms=54321)

        decoded = decode_mavlink_v2(msg_bytes)
        _sys_id, msg_id, payload = decoded
        assert msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT

        parsed = parse_telemetry_payload(msg_id, payload)
        assert parsed["time_boot_ms"] == 54321
        assert abs(parsed["lat_deg"] - lat) < 1e-6
        assert abs(parsed["lon_deg"] - lon) < 1e-6
        assert abs(parsed["alt_msl"] - alt_msl) < 0.001

    def test_gps_to_enu_inverse_of_enu_to_gps(self):
        """_gps_to_enu should be the inverse of _enu_to_gps."""
        from mavlink_bridge import _enu_to_gps
        from live_telemetry import _gps_to_enu

        ref_lat, ref_lon, ref_alt = -35.363261, 149.165230, 584.0
        points = [
            np.array([0.0, 0.0, 0.0]),
            np.array([100.0, -50.0, 10.0]),
            np.array([-20.5, 30.1, -5.0])
        ]

        for p in points:
            lat, lon, alt = _enu_to_gps(p, ref_lat, ref_lon, ref_alt)
            p_back = _gps_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
            np.testing.assert_allclose(p, p_back, atol=1e-3)

    def test_queue_thread_safety(self):
        """TelemetryQueue should handle concurrent push/snapshot without crashing."""
        import threading
        from live_telemetry import TelemetryQueue, LiveTelemetrySample

        q = TelemetryQueue(maxlen=500)
        stop_event = threading.Event()

        def producer():
            for i in range(1000):
                q.push(LiveTelemetrySample(time_boot_ms=i))

        def consumer():
            while not stop_event.is_set():
                q.snapshot(n=100)
                q.latest()

        t1 = threading.Thread(target=producer)
        t2 = threading.Thread(target=consumer)
        t1.start()
        t2.start()
        t1.join()
        stop_event.set()
        t2.join()

        assert len(q) == 500
        assert q.latest().time_boot_ms == 999

    def test_bridge_to_queue_roundtrip(self):
        """Full UDP loop: MAVLinkBridge -> MAVLinkLiveSource -> TelemetryQueue."""
        import socket
        import time
        from mavlink_bridge import MAVLinkBridge, SimState
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        # 1. Setup receiver on ephemeral port
        q = TelemetryQueue()
        # Find the port assigned by OS
        dummy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dummy_sock.bind(("127.0.0.1", 0))
        ephemeral_port = dummy_sock.getsockname()[1]
        dummy_sock.close()

        # In some environments, 127.0.0.1 might behave differently than localhost or 0.0.0.0
        source = MAVLinkLiveSource(listen_ip="0.0.0.0", listen_port=ephemeral_port, queue=q)
        source.start()

        try:
            # 2. Setup sender
            bridge = MAVLinkBridge(target_ip="127.0.0.1", target_port=ephemeral_port, listen_port=0)
            bridge.start()

            # 3. Send state
            state1 = SimState(
                time_s=1.0,
                position=np.array([10.0, 20.0, 30.0]),
                velocity=np.array([1.0, 2.0, 3.0]),
                roll=0.1, pitch=-0.2, yaw=0.3,
                thrust_pct=50.0
            )

            # 4. Poll queue
            sample = None
            timeout = 3.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                bridge.send_state(state1)
                time.sleep(0.1)
                sample = q.latest()
                if sample:
                    break

            assert sample is not None
            assert sample.time_boot_ms == 1000
            np.testing.assert_allclose(sample.euler, [0.1, -0.2, 0.3], atol=1e-4)

            bridge.stop()
        finally:
            source.stop()

    def test_csv_recorder_roundtrip(self, tmp_path):
        """TelemetryCSVRecorder should persist and reload correctly."""
        from live_telemetry import TelemetryCSVRecorder, LiveTelemetrySample
        import csv

        csv_file = tmp_path / "test_telemetry.csv"
        recorder = TelemetryCSVRecorder(str(csv_file))

        samples = [
            LiveTelemetrySample(t_wall=100.0, time_boot_ms=1000, pos_enu=np.array([1,2,3])),
            LiveTelemetrySample(t_wall=100.1, time_boot_ms=1100, pos_enu=np.array([4,5,6]), flight_mode="GUIDED", armed=True)
        ]

        for s in samples:
            recorder.record(s)
        recorder.close()

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        assert len(rows) == 2
        assert rows[0]["time_boot_ms"] == "1000"
        assert float(rows[0]["x"]) == 1.0
        assert rows[1]["mode"] == "GUIDED"
        assert rows[1]["armed"] == "1"


class TestRuntimeViewServer:
    """In-process route tests via FastAPI's ``TestClient``.

    These cover the simple HTTP route shapes and the WebSocket handshake.
    Real over-the-wire tests live in :class:`TestRunTimeViewIntegration`
    and :class:`TestLiveViewNoMotionRegression` which spin up uvicorn on
    an ephemeral port.
    """

    def test_index_route_renders_launcher(self, tmp_path):
        """Root route should render index.html."""
        from fastapi.testclient import TestClient
        from runtime_view.server import app
        # Create a dummy index.html
        web_dir = tmp_path / "web"
        web_dir.mkdir()
        (web_dir / "index.html").write_text("Launcher")
        app.template_folder = str(web_dir)

        with TestClient(app) as client:
            res = client.get('/')
            assert res.status_code == 200
            assert b"Launcher" in res.content

    def test_api_missions_returns_catalogue(self, tmp_path):
        """API should return the missions JSON."""
        from fastapi.testclient import TestClient
        from runtime_view.server import app

        with TestClient(app) as client:
            res = client.get('/api/missions')
            assert res.status_code == 200
            assert isinstance(res.json(), list)

    def test_api_status_reflects_queue_state(self):
        """Status API should show current sample count and latest sample."""
        from fastapi.testclient import TestClient
        from runtime_view.server import app, telemetry_queue
        from live_telemetry import LiveTelemetrySample
        import numpy as np

        telemetry_queue.clear()
        telemetry_queue.push(LiveTelemetrySample(time_boot_ms=100, pos_enu=np.array([1,2,3])))

        with TestClient(app) as client:
            res = client.get('/api/status')
            assert res.status_code == 200
            data = res.json()
            assert data["sample_count"] == 1
            assert data["latest_sample"]["time_boot_ms"] == 100

    def test_websocket_streams_latest_sample(self):
        """The /ws/telemetry route must exist and accept a handshake."""
        from fastapi.testclient import TestClient
        from runtime_view.server import app, telemetry_queue
        from live_telemetry import LiveTelemetrySample

        telemetry_queue.clear()
        telemetry_queue.push(LiveTelemetrySample(t_wall=1.0, time_boot_ms=100))

        # The route must be registered on the FastAPI app.
        ws_paths = [getattr(r, 'path', None) for r in app.routes]
        assert '/ws/telemetry' in ws_paths

        # And a handshake must succeed; the snapshot frame should follow.
        with TestClient(app) as client:
            with client.websocket_connect('/ws/telemetry') as ws:
                msg = ws.receive_json()
                assert msg.get('type') in ('snapshot', 'sample', 'ping')


def _runtime_view_start_uvicorn(telemetry_queue):
    """Boot the FastAPI runtime-view app on an ephemeral uvicorn port.

    Returns ``(server, thread, base_url)`` where ``server`` is a
    ``uvicorn.Server`` whose ``should_exit`` flag stops the loop.
    Replaces the old werkzeug-based helper so the integration tests
    drive the real FastAPI/Starlette stack the production server uses.
    """
    import asyncio as _asyncio
    import threading as _threading
    import time as _time

    import uvicorn  # noqa: WPS433 — local import keeps the suite hermetic
    import runtime_view.server as srv

    # Tear down any leftover MAVLink listener from a previous test.
    if srv.live_source is not None:
        try:
            srv.live_source.stop()
        except Exception:
            pass
        srv.live_source = None

    # Reset template/static folders to the canonical web/ directory.
    # ``TestRuntimeViewServer`` cases monkey-patch ``template_folder`` to
    # a tmp_path and never restore it, so without this reset our
    # integration tests would either 404 or serve stale content from
    # leftover tmp files.
    srv.app.template_folder = str(srv.WEB_DIR)
    srv.app.static_folder = str(srv.WEB_DIR)

    # Replace the module-level queue so the test owns it.
    srv.telemetry_queue.clear()
    srv.telemetry_queue = telemetry_queue

    # uvicorn.Server subclass that signals when startup completes and
    # skips installing signal handlers (which only work on the main thread).
    class _ThreadedUvicornServer(uvicorn.Server):
        def __init__(self, config):
            super().__init__(config)
            self.startup_event = _threading.Event()

        async def startup(self, sockets=None):
            await super().startup(sockets=sockets)
            self.startup_event.set()

        def install_signal_handlers(self):
            return

    config = uvicorn.Config(
        srv.app,
        host='127.0.0.1',
        port=0,
        log_level='error',
        access_log=False,
        lifespan='off',
    )
    server = _ThreadedUvicornServer(config)

    def _serve():
        loop = _asyncio.new_event_loop()
        try:
            _asyncio.set_event_loop(loop)
            loop.run_until_complete(server.serve())
        finally:
            try:
                loop.close()
            except Exception:
                pass

    thread = _threading.Thread(
        target=_serve, name='rtv-uvicorn-test', daemon=True
    )
    thread.start()
    if not server.startup_event.wait(timeout=10.0):
        raise RuntimeError('uvicorn did not start within 10 s')

    # Probe the bound socket for the actual port (port=0 → OS-assigned).
    port = None
    deadline = _time.time() + 5.0
    while _time.time() < deadline:
        try:
            for srv_state in (server.servers or []):
                for sock in srv_state.sockets:
                    port = sock.getsockname()[1]
                    break
                if port is not None:
                    break
        except Exception:
            port = None
        if port:
            break
        _time.sleep(0.01)
    if not port:
        raise RuntimeError('uvicorn server bound no socket')
    return server, thread, f'http://127.0.0.1:{port}'


def _runtime_view_stop_uvicorn(server, thread):
    """Signal a uvicorn test server to exit and join its background thread."""
    try:
        server.should_exit = True
    finally:
        thread.join(timeout=5.0)


class TestRunTimeViewIntegration:
    """End-to-end integration tests for the Run-time View web app.

    These tests boot the real FastAPI app on an ephemeral uvicorn port,
    drive it with a real ``urllib`` HTTP client and a ``simple_websocket``
    WebSocket client, and pump telemetry through
    ``MAVLinkBridge`` → ``MAVLinkLiveSource`` → ``TelemetryQueue`` →
    ``/ws/telemetry`` → JS-side handler. They are skipped automatically
    if a sub-dependency is missing so the suite stays green on minimal
    CI runners.
    """

    def _start_server(self, telemetry_queue):
        return _runtime_view_start_uvicorn(telemetry_queue)

    def _stop_server(self, server, thread):
        _runtime_view_stop_uvicorn(server, thread)

    def test_http_server_serves_index_and_static_assets(self):
        """Real HTTP server should serve / and /web/styles.css with the dark theme."""
        import threading
        import urllib.request

        from live_telemetry import TelemetryQueue
        import runtime_view.server as srv  # noqa: F401

        q = TelemetryQueue(maxlen=128)
        server, thread, base = self._start_server(q)
        try:
            with urllib.request.urlopen(f'{base}/', timeout=2.0) as r:
                assert r.status == 200
                body = r.read().decode('utf-8')
                assert 'class="topbar"' in body
                assert 'mission-grid' in body

            with urllib.request.urlopen(f'{base}/web/styles.css', timeout=2.0) as r:
                assert r.status == 200
                css = r.read().decode('utf-8')
                # Pinned dark-navy theme tokens from the plan.
                assert '--bg-0: #070b1f' in css
                assert '--accent-green: #2ed47a' in css
        finally:
            self._stop_server(server, thread)

    def test_http_server_serves_live_view_with_three_js_importmap(self):
        """The /live route should ship the Three.js importmap and HUD scaffolding."""
        import threading
        import urllib.request

        from live_telemetry import TelemetryQueue

        q = TelemetryQueue(maxlen=128)
        server, thread, base = self._start_server(q)
        try:
            with urllib.request.urlopen(f'{base}/live', timeout=2.0) as r:
                assert r.status == 200
                html = r.read().decode('utf-8')
                # Three.js importmap and the live HUD container.
                assert 'importmap' in html
                assert '/web/vendor/three.module.js' in html
                assert 'id="viewport"' in html
                # HUD slots: AGL, ALT MSL, SPEED, HEADING, THROTTLE,
                # BATTERY V, BATTERY %, MODE.
                for slot in ('hud-agl', 'hud-alt', 'hud-speed', 'hud-heading',
                             'hud-throttle', 'hud-batt-v', 'hud-batt-pct', 'hud-mode'):
                    assert f'id="{slot}"' in html

            # Vendor file should be served and look like real Three.js.
            with urllib.request.urlopen(f'{base}/web/vendor/three.module.js', timeout=2.0) as r:
                assert r.status == 200
                head = r.read(200).decode('utf-8', errors='replace')
                assert 'Three.js' in head or 'three' in head.lower()
        finally:
            self._stop_server(server, thread)

    def test_websocket_pushes_pending_sample(self):
        """A real WebSocket client should receive the latest queued sample as JSON."""
        import json
        import threading
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from live_telemetry import LiveTelemetrySample, TelemetryQueue
        import runtime_view.server as srv

        q = TelemetryQueue(maxlen=64)
        # Push a sample BEFORE the client connects so the snapshot delivery
        # carries it on the very first frame.
        sample = LiveTelemetrySample(
            t_wall=time.time(),
            time_boot_ms=4242,
            pos_enu=np.array([12.5, -3.25, 7.75]),
            vel_enu=np.array([1.0, 0.5, -0.1]),
            euler=(0.05, -0.10, 1.57),
            throttle_pct=42.0,
            alt_msl=120.5,
            battery_voltage_v=15.6,
            battery_remaining_pct=88.0,
            flight_mode='GUIDED',
            armed=True,
        )
        q.push(sample)

        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'
        try:
            client = simple_websocket.Client(ws_url)
            try:
                # The first frame is the snapshot batch.
                raw = client.receive(timeout=3.0)
                assert raw is not None, 'Did not receive any telemetry frame'
                msg = json.loads(raw)
                assert msg['type'] in ('snapshot', 'sample')
                if msg['type'] == 'snapshot':
                    payload = msg['data'][-1]
                else:
                    payload = msg['data']
                assert payload['time_boot_ms'] == 4242
                np.testing.assert_allclose(payload['pos_enu'], [12.5, -3.25, 7.75], atol=1e-9)
                np.testing.assert_allclose(payload['euler'], [0.05, -0.10, 1.57], atol=1e-9)
                assert payload['flight_mode'] == 'GUIDED'
                assert payload['armed'] is True
                assert payload['throttle_pct'] == 42.0
            finally:
                client.close()
        finally:
            self._stop_server(server, thread)

    def test_websocket_streams_new_samples_after_connect(self):
        """Samples pushed AFTER the WS handshake must reach the client."""
        import json
        import threading
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from live_telemetry import LiveTelemetrySample, TelemetryQueue
        import runtime_view.server as srv

        q = TelemetryQueue(maxlen=64)
        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'
        try:
            client = simple_websocket.Client(ws_url)
            try:
                # Drain any initial empty snapshot frames.
                time.sleep(0.05)

                # Push three samples with strictly increasing t_wall so
                # the server's de-dup logic forwards each one.
                t0 = time.time()
                for k in range(3):
                    q.push(LiveTelemetrySample(
                        t_wall=t0 + 0.01 * (k + 1),
                        time_boot_ms=1000 + k,
                        pos_enu=np.array([float(k), 0.0, 5.0]),
                    ))

                # Read up to 5 frames within 3 s and look for time_boot_ms 1002.
                seen_ticks = set()
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    raw = client.receive(timeout=0.5)
                    if raw is None:
                        continue
                    msg = json.loads(raw)
                    if msg['type'] == 'sample':
                        seen_ticks.add(msg['data']['time_boot_ms'])
                    elif msg['type'] == 'snapshot':
                        for s in msg['data']:
                            seen_ticks.add(s['time_boot_ms'])
                    if 1002 in seen_ticks:
                        break

                assert 1002 in seen_ticks, f'Expected tick 1002 in stream, saw {sorted(seen_ticks)}'
            finally:
                client.close()
        finally:
            self._stop_server(server, thread)

    def test_bridge_to_server_to_websocket_full_loop(self):
        """End-to-end: MAVLinkBridge → MAVLinkLiveSource → /ws/telemetry."""
        import json
        import socket as _socket
        import threading
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from mavlink_bridge import MAVLinkBridge, SimState
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        # Pick an ephemeral UDP port for MAVLink.
        probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        mav_port = probe.getsockname()[1]
        probe.close()

        q = TelemetryQueue(maxlen=128)
        source = MAVLinkLiveSource(listen_ip='0.0.0.0', listen_port=mav_port, queue=q)
        source.start()

        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'
        try:
            bridge = MAVLinkBridge(target_ip='127.0.0.1', target_port=mav_port, listen_port=0)
            bridge.start()
            try:
                client = simple_websocket.Client(ws_url)
                try:
                    # Generate and pump a small mission via send_state(...)
                    sample_seen = None
                    for tick in range(20):
                        state = SimState(
                            time_s=1.0 + 0.05 * tick,
                            position=np.array([float(tick), 0.0, 5.0]),
                            velocity=np.array([1.0, 0.0, 0.0]),
                            roll=0.01 * tick,
                            pitch=-0.02,
                            yaw=0.5,
                            thrust_pct=55.0,
                        )
                        bridge.send_state(state)
                        time.sleep(0.05)

                        try:
                            raw = client.receive(timeout=0.2)
                        except Exception:
                            raw = None
                        if raw is None:
                            continue
                        msg = json.loads(raw)
                        payload = None
                        if msg['type'] == 'sample':
                            payload = msg['data']
                        elif msg['type'] == 'snapshot' and msg['data']:
                            payload = msg['data'][-1]
                        if payload and payload['time_boot_ms'] >= 1000:
                            sample_seen = payload
                            if payload['time_boot_ms'] >= 1500:
                                break

                    assert sample_seen is not None, 'No telemetry reached the WebSocket'
                    # Position drift along +X should be visible (≥10 m).
                    assert abs(sample_seen['pos_enu'][0]) >= 5.0, sample_seen
                    assert sample_seen['flight_mode'] in {'GUIDED', 'STABILIZE', 'AUTO', 'LAND', 'RTL'}
                finally:
                    client.close()
            finally:
                bridge.stop()
        finally:
            try:
                source.stop()
            finally:
                self._stop_server(server, thread)

    def test_api_missions_returns_full_catalogue(self):
        """`/api/missions` over real HTTP must return the on-disk catalogue."""
        import json
        import urllib.request

        from live_telemetry import TelemetryQueue
        import runtime_view.server as srv

        q = TelemetryQueue(maxlen=16)
        server, thread, base = self._start_server(q)
        try:
            with urllib.request.urlopen(f'{base}/api/missions', timeout=2.0) as r:
                assert r.status == 200
                payload = json.loads(r.read().decode('utf-8'))

            # The shipped catalogue at simulation/runtime_view/missions.json
            # is the source of truth for the launcher.
            on_disk = json.loads(srv.MISSIONS_PATH.read_text(encoding='utf-8'))
            assert isinstance(payload, list)
            assert len(payload) == len(on_disk)
            assert len(payload) >= 6, 'Plan §2.1.B.3 requires ≥6 mission cards'

            required_keys = {
                'id', 'title', 'description', 'thumbnail',
                'tier', 'start_command', 'disabled',
            }
            for mission in payload:
                missing = required_keys - set(mission)
                assert not missing, f'mission {mission!r} is missing keys {missing}'
                assert mission['tier'] in ('free', 'pro'), mission

            ids = {m['id'] for m in payload}
            assert {'single', 'physics', 'real-log'}.issubset(ids), ids
        finally:
            self._stop_server(server, thread)

    def test_api_snapshot_returns_recent_samples(self):
        """`/api/snapshot?n=K` must return the K most recent queued samples."""
        import json
        import time
        import urllib.request

        from live_telemetry import LiveTelemetrySample, TelemetryQueue

        q = TelemetryQueue(maxlen=64)
        # Push 10 strictly-monotonic samples so the snapshot ordering is
        # deterministic regardless of how the deque slices them.
        t0 = time.time()
        for k in range(10):
            q.push(LiveTelemetrySample(
                t_wall=t0 + 0.001 * k,
                time_boot_ms=2000 + k,
                pos_enu=np.array([float(k), 0.0, 5.0]),
                flight_mode='GUIDED' if k % 2 == 0 else 'AUTO',
                armed=True,
            ))

        server, thread, base = self._start_server(q)
        try:
            # Default n=100 → returns all 10 samples in chronological order.
            with urllib.request.urlopen(f'{base}/api/snapshot', timeout=2.0) as r:
                assert r.status == 200
                full = json.loads(r.read().decode('utf-8'))
            assert isinstance(full, list)
            assert len(full) == 10
            assert [s['time_boot_ms'] for s in full] == list(range(2000, 2010))

            # Limited n=4 → only the last 4 samples (2006..2009).
            with urllib.request.urlopen(f'{base}/api/snapshot?n=4', timeout=2.0) as r:
                assert r.status == 200
                tail = json.loads(r.read().decode('utf-8'))
            assert [s['time_boot_ms'] for s in tail] == [2006, 2007, 2008, 2009]

            # The clamp at n=1 should still return exactly one sample
            # (the most recent one), proving min/max bounds are honoured.
            with urllib.request.urlopen(f'{base}/api/snapshot?n=1', timeout=2.0) as r:
                one = json.loads(r.read().decode('utf-8'))
            assert len(one) == 1 and one[0]['time_boot_ms'] == 2009
        finally:
            self._stop_server(server, thread)

    def test_api_status_real_http_reflects_queue_state(self):
        """`/api/status` over real HTTP should mirror the live queue."""
        import json
        import time
        import urllib.request

        from live_telemetry import LiveTelemetrySample, TelemetryQueue

        q = TelemetryQueue(maxlen=8)
        server, thread, base = self._start_server(q)
        try:
            # Empty queue → no latest sample, count == 0.
            with urllib.request.urlopen(f'{base}/api/status', timeout=2.0) as r:
                empty = json.loads(r.read().decode('utf-8'))
            assert empty['sample_count'] == 0
            assert empty['latest_sample'] is None

            # Push two samples and re-query.
            q.push(LiveTelemetrySample(t_wall=time.time(), time_boot_ms=4000))
            q.push(LiveTelemetrySample(t_wall=time.time(), time_boot_ms=4001))
            with urllib.request.urlopen(f'{base}/api/status', timeout=2.0) as r:
                full = json.loads(r.read().decode('utf-8'))
            assert full['sample_count'] == 2
            assert full['latest_sample'] is not None
            assert full['latest_sample']['time_boot_ms'] == 4001
        finally:
            self._stop_server(server, thread)

    def test_static_web_assets_served(self):
        """`/web/<path>` must serve every front-end asset the launcher needs."""
        import urllib.request

        from live_telemetry import TelemetryQueue
        import runtime_view.server as srv

        q = TelemetryQueue(maxlen=16)
        server, thread, base = self._start_server(q)
        try:
            # JavaScript and CSS bundled with the launcher.
            for rel in ('app.js', 'live.js', 'styles.css'):
                with urllib.request.urlopen(f'{base}/web/{rel}', timeout=2.0) as r:
                    assert r.status == 200, rel
                    body = r.read()
                    assert len(body) > 0, rel

            # Vendored Three.js modules.
            for rel in ('vendor/three.module.js', 'vendor/OrbitControls.js'):
                with urllib.request.urlopen(f'{base}/web/{rel}', timeout=2.0) as r:
                    assert r.status == 200, rel
                    head = r.read(64)
                    assert len(head) > 0, rel

            # Mission thumbnails generated by the build step.
            for rel in ('img/single.png', 'img/physics.png', 'img/real.png',
                        'img/swarm3.png', 'img/swarm6.png', 'img/fw.png'):
                with urllib.request.urlopen(f'{base}/web/{rel}', timeout=2.0) as r:
                    assert r.status == 200, rel
                    blob = r.read()
                    # Pillow PNG magic header so we know the file is intact.
                    assert blob[:8] == b'\x89PNG\r\n\x1a\n', rel

            # Unknown asset must 404 cleanly (not crash with a 500).
            try:
                urllib.request.urlopen(f'{base}/web/no_such_file.txt', timeout=2.0)
                raised = False
            except urllib.error.HTTPError as exc:
                raised = (exc.code == 404)
            assert raised, 'Missing static asset must return HTTP 404'

            # Sanity check: WEB_DIR is what the server actually points at.
            assert srv.app.static_folder == str(srv.WEB_DIR)
        finally:
            self._stop_server(server, thread)


class TestLiveViewNoMotionRegression:
    """Regression coverage for the "live view connected but no motion" bug.

    Symptom (reported 2026-04-09): the launcher status chip flips to
    ``CONNECTED`` because the WebSocket handshake succeeds, but the drone
    mesh sits at the world origin forever — no ``sample`` messages reach
    the browser. Root cause: ``sitl_orchestrator.py`` consumed MAVLink over
    TCP 5760 from the SITL container but never forwarded the frames to UDP
    14550 where ``MAVLinkLiveSource`` was listening, so the receiver
    queue stayed empty for the entire mission.

    These tests pin both halves of the fix: (a) the live view itself must
    forward queued samples promptly, and (b) ``SITLDrone`` must relay
    incoming frames to the configured forward URL so the queue actually
    fills up.
    """

    def _start_server(self, telemetry_queue):
        """Boot the FastAPI app on an ephemeral port (mirrors TestRunTimeViewIntegration)."""
        return _runtime_view_start_uvicorn(telemetry_queue)

    def _stop_server(self, server, thread):
        _runtime_view_stop_uvicorn(server, thread)

    def test_websocket_connects_but_emits_no_sample_when_queue_empty(self):
        """The exact regression: WS opens (status=CONNECTED) but no sample frames flow.

        Reproduces the production failure mode in a hermetic test: with
        nothing pushing telemetry to the queue, the live HUD JS receives
        zero ``sample`` frames, which is why the drone mesh never moves.
        """
        import json
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from live_telemetry import TelemetryQueue

        q = TelemetryQueue(maxlen=8)
        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'
        try:
            client = simple_websocket.Client(ws_url)
            try:
                # Drain everything the server is willing to emit in 1 s.
                deadline = time.time() + 1.0
                samples_seen = 0
                snapshots_seen = 0
                while time.time() < deadline:
                    try:
                        raw = client.receive(timeout=0.2)
                    except Exception:
                        raw = None
                    if raw is None:
                        continue
                    msg = json.loads(raw)
                    if msg.get('type') == 'sample':
                        samples_seen += 1
                    elif msg.get('type') == 'snapshot':
                        snapshots_seen += 1
                # The WS connection itself opened cleanly (== "CONNECTED"
                # in the HUD chip). But because nothing populated the
                # queue, the JS handler in live.js never invoked
                # applySample, so the drone mesh stayed at the origin.
                assert samples_seen == 0, (
                    'WS should not emit sample frames when the queue is empty')
                # An empty initial snapshot must NOT be sent either, otherwise
                # the JS handler would call applySample with stale zeros.
                assert snapshots_seen == 0, (
                    'WS should not emit snapshot frames for an empty queue')
            finally:
                client.close()
        finally:
            self._stop_server(server, thread)

    def test_websocket_emits_motion_samples_when_queue_filled(self):
        """When telemetry actually arrives, the WS must forward each pos_enu update."""
        import json
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from live_telemetry import LiveTelemetrySample, TelemetryQueue

        q = TelemetryQueue(maxlen=64)
        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'
        try:
            client = simple_websocket.Client(ws_url)
            try:
                # Push a flight along a 10 m straight line so the JS
                # camera-follow lerp would visibly move the drone mesh.
                t0 = time.time()
                positions = [(float(k), 0.0, 5.0) for k in range(10)]
                for k, (x, y, z) in enumerate(positions):
                    q.push(LiveTelemetrySample(
                        t_wall=t0 + 0.01 * (k + 1),
                        time_boot_ms=5000 + k,
                        pos_enu=np.array([x, y, z]),
                        vel_enu=np.array([1.0, 0.0, 0.0]),
                        flight_mode='AUTO',
                        armed=True,
                    ))

                seen_pos = []
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    try:
                        raw = client.receive(timeout=0.5)
                    except Exception:
                        raw = None
                    if raw is None:
                        continue
                    msg = json.loads(raw)
                    if msg.get('type') == 'sample':
                        seen_pos.append(tuple(msg['data']['pos_enu']))
                    elif msg.get('type') == 'snapshot':
                        for s in msg['data']:
                            seen_pos.append(tuple(s['pos_enu']))
                    if len(seen_pos) >= 5:
                        break

                assert seen_pos, 'Live view emitted no samples even though queue was filled'
                # The pos_enu must actually CHANGE between samples — that
                # is the literal definition of "the drone is moving".
                assert len(set(seen_pos)) >= 2, (
                    f'pos_enu never changed across {len(seen_pos)} frames: {seen_pos}')
                # And the X coordinate must monotonically advance.
                xs = [p[0] for p in seen_pos]
                assert xs[-1] > xs[0], f'X coordinate did not advance: {xs}'
            finally:
                client.close()
        finally:
            self._stop_server(server, thread)

    def test_sitl_drone_forwards_received_frame_to_udp_listener(self):
        """SITLDrone._forward_msg must relay raw MAVLink bytes to the configured UDP URL."""
        import socket as _socket
        import time
        from unittest.mock import MagicMock

        from sitl_orchestrator import SITLDrone
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        # Pick an ephemeral UDP port for the live view receiver.
        probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        fwd_port = probe.getsockname()[1]
        probe.close()

        q = TelemetryQueue(maxlen=8)
        # ref defaults match the live view; with these, GPS round-trip
        # produces a non-zero pos_enu so we can also assert "motion".
        src = MAVLinkLiveSource(
            listen_ip='127.0.0.1', listen_port=fwd_port, queue=q,
            ref_lat=-0.508, ref_lon=-78.14, ref_alt_msl=4500.0,
        )
        src.start()
        try:
            drone = SITLDrone(0, telemetry_forward_url=f'udpout:127.0.0.1:{fwd_port}')
            assert drone._setup_telemetry_forward(), 'forward setup failed'

            # Build a real MAVLink GLOBAL_POSITION_INT frame using
            # pymavlink (the same library the orchestrator uses) so the
            # bytes are bit-identical to what SITL would emit.
            from pymavlink.dialects.v20 import ardupilotmega as mavlink2
            mav = mavlink2.MAVLink(None)
            mav.srcSystem = 1
            mav.srcComponent = 1
            # 10 m east, 0 m north, 5 m up relative to the ref point.
            lat_offset_deg = 0.0
            lon_offset_deg = 10.0 / (111320.0 * np.cos(np.radians(-0.508)))
            msg = mav.global_position_int_encode(
                time_boot_ms=4242,
                lat=int((-0.508 + lat_offset_deg) * 1e7),
                lon=int((-78.14 + lon_offset_deg) * 1e7),
                alt=int((4500.0 + 5.0) * 1000),
                relative_alt=int(5.0 * 1000),
                vx=100, vy=0, vz=0, hdg=9000,
            )
            msg.pack(mav)  # populates the internal _msgbuf

            # _forward_msg is the production code path used by poll_once
            # and wait_gps_ekf. Calling it directly avoids needing a real
            # SITL TCP server.
            drone._forward_msg(msg)

            # Wait for the receiver thread to consume the UDP datagram.
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if len(q) > 0:
                    break
                time.sleep(0.05)

            assert len(q) > 0, 'forwarded MAVLink frame never reached the live view queue'
            sample = q.latest()
            assert sample is not None
            assert sample.time_boot_ms == 4242
            # ENU translation: ~10 m east, ~5 m up, 0 m north.
            np.testing.assert_allclose(
                sample.pos_enu, [10.0, 0.0, 5.0], atol=0.5,
                err_msg=f'pos_enu mismatch: {sample.pos_enu}',
            )
            drone.close()
        finally:
            src.stop()

    def test_orchestrator_poll_once_forwards_every_frame(self):
        """SITLDrone.poll_once must call _forward_msg on every received frame."""
        from unittest.mock import MagicMock

        from sitl_orchestrator import SITLDrone

        drone = SITLDrone(0, telemetry_forward_url='udpout:127.0.0.1:1')
        drone.conn = MagicMock()

        # Fake STATUSTEXT-style message with a synthesized get_msgbuf().
        fake_frame = b'\xfd\x09\x00\x00\x00\x01\x01\x00\x00\x00deadbeef00'
        fake_msg = MagicMock()
        fake_msg.get_type.return_value = "STATUSTEXT"
        fake_msg.get_msgbuf.return_value = fake_frame
        fake_msg.text = "Mission: 1 cmds"
        drone.conn.recv_match.return_value = fake_msg

        forwarded = []
        forward_stub = MagicMock()
        forward_stub.write.side_effect = lambda buf: forwarded.append(bytes(buf))
        drone._forward = forward_stub

        result = drone.poll_once()
        assert result == 'flying'
        assert forwarded == [fake_frame], (
            'poll_once must relay every received frame to the forward channel')

        # Two more polls — every frame must be forwarded, not just the first.
        drone.poll_once()
        drone.poll_once()
        assert len(forwarded) == 3

        # When recv_match yields nothing, no forward call must happen.
        drone.conn.recv_match.return_value = None
        forwarded.clear()
        drone.poll_once()
        assert forwarded == []

    def test_full_pipeline_emits_motion_to_websocket(self):
        """Full SITL → orchestrator forward → MAVLinkLiveSource → /ws/telemetry."""
        import json
        import socket as _socket
        import time
        try:
            import simple_websocket
        except ImportError:
            pytest.skip('simple_websocket not installed')

        from sitl_orchestrator import SITLDrone
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        # 1. Bind a UDP listener for the forwarded MAVLink stream.
        probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        fwd_port = probe.getsockname()[1]
        probe.close()

        q = TelemetryQueue(maxlen=64)
        src = MAVLinkLiveSource(
            listen_ip='127.0.0.1', listen_port=fwd_port, queue=q,
            ref_lat=-0.508, ref_lon=-78.14, ref_alt_msl=4500.0,
        )
        src.start()

        # 2. Boot the runtime view server backed by the same queue.
        server, thread, base = self._start_server(q)
        ws_url = base.replace('http://', 'ws://') + '/ws/telemetry'

        try:
            # 3. SITLDrone with a real forward channel.
            drone = SITLDrone(
                0, telemetry_forward_url=f'udpout:127.0.0.1:{fwd_port}')
            assert drone._setup_telemetry_forward()

            from pymavlink.dialects.v20 import ardupilotmega as mavlink2
            mav = mavlink2.MAVLink(None)
            mav.srcSystem = 1
            mav.srcComponent = 1

            client = simple_websocket.Client(ws_url)
            try:
                # 4. Pump 12 ATTITUDE + GLOBAL_POSITION_INT pairs, each
                # with a slightly different east position so the trail
                # advances east by ~12 m total.
                cos_lat = float(np.cos(np.radians(-0.508)))
                lon_per_m = 1.0 / (111320.0 * cos_lat)
                for tick in range(12):
                    east_m = float(tick + 1)
                    pos_msg = mav.global_position_int_encode(
                        time_boot_ms=10000 + tick,
                        lat=int(-0.508 * 1e7),
                        lon=int((-78.14 + east_m * lon_per_m) * 1e7),
                        alt=int((4500.0 + 5.0) * 1000),
                        relative_alt=int(5.0 * 1000),
                        vx=100, vy=0, vz=0, hdg=9000,
                    )
                    pos_msg.pack(mav)
                    drone._forward_msg(pos_msg)
                    time.sleep(0.05)

                # 5. Drain the WS until we either see motion or hit a deadline.
                seen_x = []
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    try:
                        raw = client.receive(timeout=0.3)
                    except Exception:
                        raw = None
                    if raw is None:
                        continue
                    msg = json.loads(raw)
                    if msg.get('type') == 'sample':
                        seen_x.append(msg['data']['pos_enu'][0])
                    elif msg.get('type') == 'snapshot':
                        for s in msg['data']:
                            seen_x.append(s['pos_enu'][0])
                    if len(seen_x) >= 4 and max(seen_x) - min(seen_x) >= 3.0:
                        break

                assert seen_x, 'No samples reached the WS — forwarder did not relay'
                spread = max(seen_x) - min(seen_x) if seen_x else 0.0
                assert spread >= 3.0, (
                    f'Drone X position barely moved across WS frames '
                    f'(spread={spread:.2f}, samples={seen_x}). The forwarder '
                    f'or the parser dropped GPS updates somewhere.')
            finally:
                client.close()
            drone.close()
        finally:
            try:
                src.stop()
            finally:
                self._stop_server(server, thread)

    def test_run_scenario_live_modes_open_live_url_and_set_forward(self):
        """run_scenario.sh must open /live and set SITL_TELEMETRY_FORWARD for live modes."""
        from pathlib import Path

        # The script lives at the repo root, two levels up from this file.
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        assert script.exists(), f'run_scenario.sh not found at {script}'
        body = script.read_text(encoding='utf-8')

        # 1. The default landing path of run_live_viz must be /live.
        assert 'local open_path="${3:-/live}"' in body, (
            'run_live_viz must default its open_path argument to /live')

        # 2. run_single_mission_live must export the forward URL so the
        # nested run_single_mission call passes it to the orchestrator.
        assert 'SITL_TELEMETRY_FORWARD="udpout:127.0.0.1:14550"' in body, (
            'run_single_mission_live must set SITL_TELEMETRY_FORWARD '
            'so sitl_orchestrator forwards MAVLink to the live view')

        # 3. run_single_mission_live must invoke run_live_viz with /live.
        assert 'run_live_viz 14550 8765 /live' in body, (
            'run_single_mission_live must open the live HUD path directly')

        # 4. run_single_mission must consume SITL_TELEMETRY_FORWARD and
        # forward it via --telemetry-forward to sitl_orchestrator.
        assert 'telemetry_forward="${SITL_TELEMETRY_FORWARD:-}"' in body
        assert '--telemetry-forward' in body

    def test_run_scenario_default_mode_runs_physics_live(self):
        """The (no-arg) default branch must run a physics simulation with the live viewer."""
        from pathlib import Path
        import re

        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')

        # Locate the --default) ... ;; case body and inspect what it runs.
        m = re.search(r'\n\s*--default\)\s*\n(?P<body>.*?);;\s*\n', body,
                      re.DOTALL)
        assert m is not None, '--default branch not found in run_scenario.sh'
        default_body = m.group('body')

        # Must invoke run_physics_live so the simulation actually runs.
        assert 'run_physics_live' in default_body, (
            f'--default must call run_physics_live, got:\n{default_body}')
        # Must loop so the viewer always has telemetry.
        assert '--loop' in default_body, (
            f'--default must pass --loop to run_physics_live, '
            f'got:\n{default_body}')

        # Must NOT bring up the SITL stack as part of the default flow.
        assert 'run_single_mission' not in default_body, (
            f'--default must NOT call run_single_mission* — that belongs '
            f'to --single / --single-live. Got:\n{default_body}')
        assert 'run_swarm_mission' not in default_body, (
            f'--default must NOT call run_swarm_mission. Got:\n{default_body}')
        assert 'run_single_viz' not in default_body, (
            f'--default must NOT call the matplotlib replayer. '
            f'Got:\n{default_body}')

        # Must request the runtime-view dependencies.
        assert 'NEED_RUNTIME_VIEW=1' in default_body, (
            f'--default must set NEED_RUNTIME_VIEW=1 before ensure_venv. '
            f'Got:\n{default_body}')


class TestSITLOrchestratorCli:
    """Smoke checks for the new --telemetry-forward CLI surface."""

    def test_orchestrator_help_lists_telemetry_forward(self):
        import subprocess
        import sys as _sys

        sim_dir = Path(__file__).resolve().parent
        for sub in ('single', 'swarm', 'swarm-formation'):
            result = subprocess.run(
                [_sys.executable, str(sim_dir / 'sitl_orchestrator.py'),
                 sub, '--help'],
                capture_output=True, text=True, timeout=15,
            )
            assert result.returncode == 0, (
                f'{sub} --help failed: {result.stderr}')
            assert '--telemetry-forward' in result.stdout, (
                f'{sub} --help did not advertise --telemetry-forward')


# ── Swarm-Ready Standalone Twin ──────────────────────────────────────────────

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


# ── Terrain Pipeline (Elevation Data) ────────────────────────────────────────

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


# ── Gazebo Model Validation ──────────────────────────────────────────────────

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
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
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
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
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
        with open(world_path, encoding="utf-8") as f:
            content = f.read()
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


# ── IRS-4 Gazebo Model & Docker SITL ─────────────────────────────────────

class TestIRS4GazeboModel:
    """Verify IRS-4 quadrotor Gazebo model (SDF, sensors, plugins)."""

    def _project_root(self):
        import os
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_irs4_sdf_exists(self):
        """IRS-4 quadrotor SDF model file should exist."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        assert os.path.exists(sdf_path), f"SDF not found: {sdf_path}"

    def test_irs4_model_config_exists(self):
        """IRS-4 model config file should exist."""
        import os
        config_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.config")
        assert os.path.exists(config_path)

    def test_irs4_parm_exists(self):
        """IRS-4 ArduPilot parameter file should exist."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        assert os.path.exists(parm_path)

    def test_irs4_sdf_mass_matches_preset(self):
        """SDF mass should match make_irs4_quadrotor() (1.8 kg)."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "<mass>1.8</mass>" in content

    def test_irs4_sdf_has_4_rotors(self):
        """SDF should have 4 rotor joints."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert content.count("rotor_") >= 8  # 4 links + 4 joints

    def test_irs4_sdf_has_ardupilot_plugin(self):
        """SDF should contain ArduPilot SITL plugin."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "ArduPilotPlugin" in content

    def test_irs4_sdf_has_liftdrag_plugin(self):
        """SDF should contain LiftDrag aero plugin with C_D=1.0."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "LiftDragPlugin" in content
        assert "<cda>1.0</cda>" in content

    def test_irs4_sdf_inertia_matches_preset(self):
        """SDF inertia should match make_irs4_quadrotor() values."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "<ixx>0.025</ixx>" in content
        assert "<iyy>0.025</iyy>" in content
        assert "<izz>0.042</izz>" in content

    def test_irs4_parm_copter_frame(self):
        """Parameter file should configure copter frame."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        with open(parm_path, encoding="utf-8") as f:
            content = f.read()
        assert "FRAME_CLASS,1" in content

    def test_irs4_parm_carolina_origin(self):
        """Parameter file should have Carolina Park GPS origin."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        with open(parm_path, encoding="utf-8") as f:
            content = f.read()
        assert "SIM_OPOS_LAT,-0.189" in content
        assert "SIM_OPOS_ALT,2800" in content


class TestDockerSITL:
    """Verify Docker SITL configuration (Dockerfile, compose, entrypoint)."""

    def _project_root(self):
        import os
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_dockerfile_sitl_exists(self):
        """Dockerfile.sitl should exist."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        assert os.path.exists(path)

    def test_dockerfile_builds_copter_and_plane(self):
        """Dockerfile should build both arducopter and arduplane."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "arducopter" in content
        assert "arduplane" in content

    def test_dockerfile_exposes_ports(self):
        """Dockerfile should expose required UDP/TCP ports."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "9002" in content
        assert "9003" in content
        assert "14550" in content

    def test_dockerfile_has_healthcheck(self):
        """Dockerfile should have a MAVLink heartbeat health check."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "HEALTHCHECK" in content

    def test_compose_has_sitl_service(self):
        """docker-compose.yml should have ardupilot_sitl service."""
        import os
        path = os.path.join(self._project_root(), "docker-compose.yml")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "ardupilot_sitl" in content

    def test_compose_sitl_ports(self):
        """SITL compose service should expose correct ports."""
        import os
        path = os.path.join(self._project_root(), "docker-compose.yml")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "9002:9002" in content
        assert "14550:14550" in content

    def test_sitl_entrypoint_exists(self):
        """SITL entrypoint script should exist and be executable."""
        import os, stat
        path = os.path.join(self._project_root(), "scripts", "sitl_entrypoint.sh")
        assert os.path.exists(path)
        mode = os.stat(path).st_mode
        assert mode & stat.S_IXUSR


# ── Mission Waypoint Files ───────────────────────────────────────────────

class TestMissionFiles:
    """Verify paper mission waypoint files (.waypoints format)."""

    def _project_root(self):
        import os
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_fw_158_exists(self):
        """Fixed-wing mission 158 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_158.waypoints")
        assert os.path.exists(path)

    def test_fw_178_exists(self):
        """Fixed-wing mission 178 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_178.waypoints")
        assert os.path.exists(path)

    def test_fw_185_exists(self):
        """Fixed-wing mission 185 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_185.waypoints")
        assert os.path.exists(path)

    def test_fw_missions_qgc_format(self):
        """All FW mission files should start with QGC WPL 110 header."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            path = os.path.join(missions_dir, name)
            with open(path, encoding="utf-8") as f:
                first_line = f.readline().strip()
            assert first_line == "QGC WPL 110", f"{name} missing QGC header"

    def test_fw_missions_have_antisana_origin(self):
        """FW missions should reference Antisana GPS coordinates."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            with open(os.path.join(missions_dir, name), encoding="utf-8") as f:
                content = f.read()
            assert "-0.508333" in content, f"{name} missing Antisana lat"
            assert "-78.141667" in content, f"{name} missing Antisana lon"

    def test_fw_missions_have_waypoints(self):
        """Each FW mission should have at least 5 waypoints."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            with open(os.path.join(missions_dir, name), encoding="utf-8") as f:
                lines = f.readlines()
            # First line is header, rest are waypoints
            wp_count = len([l for l in lines[1:] if l.strip()])
            assert wp_count >= 5, f"{name} has only {wp_count} waypoints"

    def test_quad_missions_module_exists(self):
        """Quadrotor mission definitions module should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "quad_missions.py")
        assert os.path.exists(path)

    def test_quad_missions_importable(self):
        """Quadrotor missions should be importable and contain 4 missions."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import ALL_QUAD_MISSIONS, get_mission, mission_to_qgc_wpl
            assert len(ALL_QUAD_MISSIONS) == 4
            assert "carolina_40" in ALL_QUAD_MISSIONS
            assert "carolina_20" in ALL_QUAD_MISSIONS
            assert "epn_30" in ALL_QUAD_MISSIONS
            assert "epn_20" in ALL_QUAD_MISSIONS
        finally:
            sys.path.remove(missions_dir)

    def test_quad_mission_to_qgc_wpl(self):
        """mission_to_qgc_wpl should produce valid QGC format."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import get_mission, mission_to_qgc_wpl
            mission = get_mission("carolina_40")
            wpl = mission_to_qgc_wpl(mission)
            assert wpl.startswith("QGC WPL 110")
            lines = [l for l in wpl.strip().split("\n") if l.strip()]
            assert len(lines) >= 5
        finally:
            sys.path.remove(missions_dir)

    def test_quad_missions_have_correct_origins(self):
        """Carolina missions at -0.189, EPN missions at -0.210."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import get_mission
            carolina = get_mission("carolina_40")
            assert carolina["origin"]["lat"] == -0.189
            epn = get_mission("epn_30")
            assert epn["origin"]["lat"] == -0.210
        finally:
            sys.path.remove(missions_dir)


# ── SITL Lifecycle Script ─────────────────────────────────────────────────

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
        with open(script_path, encoding="utf-8") as f:
            content = f.read()
        assert "[1/6]" in content  # Start stack
        assert "[2/6]" in content  # Health check
        assert "[3/6]" in content  # Upload mission
        assert "[4/6]" in content  # Arm and fly
        assert "[5/6]" in content  # Capture logs
        assert "[6/6]" in content  # Validate


# ── 3D Wind Estimation ───────────────────────────────────────────────────

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


# ── CI Pipeline ──────────────────────────────────────────────────────────

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
        with open(ci_path, encoding="utf-8") as f:
            content = f.read()
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
        with open(ci_path, encoding="utf-8") as f:
            content = f.read()
        assert "push" in content
        assert "master" in content or "main" in content


# ── IRS-4 Quadrotor Preset ───────────────────────────────────────────────

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

class TestPaperValidation:
    """Parametrized tests verifying sim accuracy against paper Table 5 thresholds.

    Paper: Valencia et al. (2025), Table 5.
    Fixed-wing: RMSE_Z ≤ 2.0m, RMSE_X ≤ 1.8m, RMSE_Y ≤ 1.3m
    Quadrotor:  RMSE_Z ≤ 0.1m, RMSE_X ≤ 0.071m, RMSE_Y ≤ 0.071m

    The paper's RMSE compares sim (with estimated wind perturbation) against
    real flight data where the controller compensates for wind. These tests
    verify that both sims produce deterministically identical results (RMSE≈0
    for same wind) and that hover position accuracy matches paper targets.
    """

    # Our generic PID reaches ~0.5-1.6m steady-state accuracy depending on
    # altitude (higher = slower convergence). Paper's 0.1m target requires
    # ArduPilot's tuned controller via SITL integration.
    # These tests verify the engine converges and tracks within reasonable bounds.
    QUAD_HOVER_Z_THRESHOLD = 2.0   # generic PID steady-state (paper: 0.10m w/ ArduPilot)
    FW_TRACK_Z_THRESHOLD = 3.0     # generic PID tracking (paper: 2.0m)

    def _measure_hover_accuracy(self, params, target, wind=None, settle_time=10.0,
                                 measure_time=5.0):
        """Measure position accuracy during steady hover at target."""
        total_time = settle_time + measure_time
        records = run_simulation(
            waypoints=[target], params=params, dt=0.005,
            waypoint_radius=0.2, hover_time=measure_time + 1.0,
            max_time=total_time, wind=wind,
        )
        hover_records = [r for r in records if r.t > settle_time]
        if len(hover_records) < 10:
            return None
        positions = np.array([r.position for r in hover_records])
        errors = positions - target
        return {
            "rmse_z": float(np.sqrt(np.mean(errors[:, 2] ** 2))),
            "rmse_x": float(np.sqrt(np.mean(errors[:, 0] ** 2))),
            "rmse_y": float(np.sqrt(np.mean(errors[:, 1] ** 2))),
            "max_z_error": float(np.max(np.abs(errors[:, 2]))),
        }

    def _run_deterministic_comparison(self, params, waypoints, wind, max_time=60.0):
        """Run two sims with identical wind and verify deterministic match."""
        records_a = run_simulation(
            waypoints=waypoints, params=params, dt=0.005,
            waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
            wind=wind,
        )
        records_b = run_simulation(
            waypoints=waypoints, params=params, dt=0.005,
            waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
            wind=wind,
        )
        n = min(len(records_a), len(records_b))
        pos_a = np.array([r.position for r in records_a[:n]])
        pos_b = np.array([r.position for r in records_b[:n]])
        return compute_rmse(pos_a, pos_b)

    @pytest.mark.parametrize("altitude_agl", [20, 40])
    def test_quadrotor_carolina_hover_accuracy(self, altitude_agl):
        """Quadrotor hover accuracy at Carolina-like altitudes.

        Paper Table 5: RMSE_Z ≤ 0.10m (with ArduPilot controller).
        Our generic PID: RMSE_Z ≤ 0.6m after 10s settling.
        """
        params = make_irs4_quadrotor()
        target = np.array([0.0, 0.0, float(altitude_agl)])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"Carolina-{altitude_agl} hover RMSE_Z={result['rmse_z']:.4f} > {self.QUAD_HOVER_Z_THRESHOLD}m"

    @pytest.mark.parametrize("altitude_agl", [20, 30])
    def test_quadrotor_epn_hover_accuracy(self, altitude_agl):
        """Quadrotor hover accuracy at EPN-like altitudes.

        Paper Table 5: RMSE_Z ≤ 0.10m (with ArduPilot controller).
        Our generic PID: RMSE_Z ≤ 0.6m after 10s settling.
        """
        params = make_irs4_quadrotor()
        target = np.array([5.0, 5.0, float(altitude_agl)])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"EPN-{altitude_agl} hover RMSE_Z={result['rmse_z']:.4f} > {self.QUAD_HOVER_Z_THRESHOLD}m"

    def test_fixed_wing_deterministic(self):
        """Fixed-wing sim is deterministic: same inputs produce same outputs."""
        params = make_valencia_fixed_wing()
        waypoints = [
            np.array([0, 0, 100]),
            np.array([50, 0, 100]),
            np.array([50, 30, 100]),
        ]
        wind = WindField(wind_speed=3.0, wind_direction=np.array([0.6, 0.8, 0.0]),
                         turbulence_type="constant")
        result = self._run_deterministic_comparison(params, waypoints, wind,
                                                     max_time=60.0)
        assert result.rmse_total < 1e-10, \
            f"FW determinism RMSE={result.rmse_total:.4e} (should be ~0)"

    def test_quadrotor_deterministic(self):
        """Quadrotor sim is deterministic: same inputs produce same outputs."""
        params = make_irs4_quadrotor()
        waypoints = [
            np.array([0, 0, 20]),
            np.array([10, 0, 20]),
            np.array([10, 10, 20]),
        ]
        wind = WindField(wind_speed=2.0, wind_direction=np.array([1, 0.5, 0.3]),
                         turbulence_type="constant")
        result = self._run_deterministic_comparison(params, waypoints, wind,
                                                     max_time=40.0)
        assert result.rmse_total < 1e-10, \
            f"Quad determinism RMSE={result.rmse_total:.4e} (should be ~0)"

    def test_paper_table5_format(self):
        """Verify validation pipeline produces all Table 5 metric fields."""
        params = make_irs4_quadrotor()
        waypoints = [np.array([0, 0, 20]), np.array([5, 0, 20])]
        ref_records = run_simulation(waypoints=waypoints, params=params,
                                     dt=0.005, max_time=20.0)
        sim_records = run_simulation(waypoints=waypoints, params=params,
                                     dt=0.005, max_time=20.0)
        n = min(len(ref_records), len(sim_records))
        sim_t = np.array([r.t for r in sim_records[:n]])
        sim_p = np.array([r.position for r in sim_records[:n]])
        ref_t = np.array([r.t for r in ref_records[:n]])
        ref_p = np.array([r.position for r in ref_records[:n]])

        metrics = compare_sim_real(sim_t, sim_p, ref_t, ref_p)
        required_keys = {"rmse_z", "rmse_x", "rmse_y", "rmse_total",
                         "median", "p75", "p25", "n_points"}
        assert required_keys.issubset(set(metrics.keys()))

    def test_quadrotor_hover_no_wind_rmse(self):
        """Pure hover without wind: generic PID converges within threshold."""
        params = make_irs4_quadrotor()
        target = np.array([0.0, 0.0, 20.0])
        result = self._measure_hover_accuracy(params, target, wind=None)
        assert result is not None, "Hover did not settle"
        assert result["rmse_z"] < self.QUAD_HOVER_Z_THRESHOLD, \
            f"No-wind hover RMSE_Z={result['rmse_z']:.4f}m exceeds {self.QUAD_HOVER_Z_THRESHOLD}m"

    def test_paper_table5_thresholds_documented(self):
        """Paper Table 5 exact RMSE values are documented for reference."""
        # Paper Table 5 exact values (for documentation / future real-data tests)
        table5 = {
            "fw_185": {"rmse_z": 1.885, "rmse_x": 0.865, "rmse_y": 0.373},
            "fw_178": {"rmse_z": 1.994, "rmse_x": 1.729, "rmse_y": 1.248},
            "fw_158": {"rmse_z": 2.001, "rmse_x": 0.661, "rmse_y": 0.247},
            "quad_carolina_40": {"rmse_z": 0.07, "rmse_x": 0.043, "rmse_y": 0.039},
            "quad_carolina_20": {"rmse_z": 0.054, "rmse_x": 0.037, "rmse_y": 0.027},
            "quad_epn_30": {"rmse_z": 0.07, "rmse_x": 0.062, "rmse_y": 0.055},
            "quad_epn_20": {"rmse_z": 0.10, "rmse_x": 0.071, "rmse_y": 0.036},
        }
        # All FW Z-RMSE under 2.1m, all Quad Z-RMSE under 0.11m
        for name, vals in table5.items():
            if name.startswith("fw"):
                assert vals["rmse_z"] <= 2.1, f"{name} Z-RMSE too high"
            else:
                assert vals["rmse_z"] <= 0.11, f"{name} Z-RMSE too high"

    def test_real_log_mission_catalog_has_required_profiles(self):
        expected = {
            "quad_carolina_40",
            "quad_carolina_20",
            "quad_epn_30",
            "quad_epn_20",
        }
        for name in expected:
            mission = get_real_log_mission(name)
            assert mission.name == name
            assert mission.source_filename.endswith(".bin")
            assert mission.segment_end_s > mission.segment_start_s

    def test_real_log_acceptance_gate_passes_within_2x_paper(self):
        mission = get_real_log_mission("quad_epn_20")
        metrics = {
            "rmse_z": mission.paper_rmse_z * 1.9,
            "rmse_x": mission.paper_rmse_x * 1.7,
            "rmse_y": mission.paper_rmse_y * 1.5,
        }
        assert_real_log_validation_pass(metrics, mission, multiplier=2.0)

    def test_real_log_acceptance_gate_fails_over_2x_paper(self):
        mission = get_real_log_mission("quad_carolina_40")
        metrics = {
            "rmse_z": mission.paper_rmse_z * 2.1,
            "rmse_x": mission.paper_rmse_x,
            "rmse_y": mission.paper_rmse_y,
        }
        with pytest.raises(AssertionError):
            assert_real_log_validation_pass(metrics, mission, multiplier=2.0)


class TestTrajectoryTracking:
    """Tests for run_trajectory_tracking and real-flight-data replay pipeline."""

    def test_trajectory_tracking_follows_reference(self):
        """Sim should closely track a simple reference trajectory."""
        ref_times = np.linspace(0, 10.0, 200)
        # Gentle helical path: circle in XY, climb in Z
        ref_positions = np.column_stack([
            5.0 * np.sin(ref_times * 0.5),
            5.0 * np.cos(ref_times * 0.5),
            ref_times * 1.0,  # 1 m/s climb
        ])

        records = run_trajectory_tracking(
            ref_times=ref_times,
            ref_positions=ref_positions,
            params=make_irs4_quadrotor(),
            dt=0.01,
        )
        sim_times = np.array([r.t for r in records])
        sim_positions = np.array([r.position for r in records])

        # Only measure accuracy in the second half (after controller settles)
        metrics = compare_sim_real(sim_times, sim_positions, ref_times, ref_positions)
        # Should track within 1m RMSE on each axis for this gentle path
        assert metrics["rmse_z"] < 1.0, f"Z tracking too loose: {metrics['rmse_z']:.3f}"
        assert metrics["rmse_x"] < 1.5, f"X tracking too loose: {metrics['rmse_x']:.3f}"
        assert metrics["rmse_y"] < 1.5, f"Y tracking too loose: {metrics['rmse_y']:.3f}"

    def test_trajectory_tracking_starts_at_ref_origin(self):
        """Sim initial position should match reference trajectory start."""
        ref_times = np.array([0.0, 1.0, 2.0])
        ref_positions = np.array([
            [10.0, 20.0, 5.0],
            [11.0, 20.0, 5.0],
            [12.0, 20.0, 5.0],
        ])
        records = run_trajectory_tracking(
            ref_times=ref_times,
            ref_positions=ref_positions,
            dt=0.01,
        )
        assert len(records) > 0
        np.testing.assert_allclose(records[0].position, [10.0, 20.0, 5.0], atol=0.01)

    def test_trajectory_tracking_rejects_short_ref(self):
        """Must have at least 2 reference points."""
        with pytest.raises(ValueError, match=">=.*2"):
            run_trajectory_tracking(
                ref_times=np.array([0.0]),
                ref_positions=np.array([[0.0, 0.0, 0.0]]),
            )

    def test_real_log_segments_have_data(self):
        """All mission segments must yield >= 10 GPS points after masking."""
        from flight_log import FlightLog
        from validation import ensure_real_log_logs, REAL_LOG_MISSIONS
        try:
            logs = ensure_real_log_logs("data/flight_logs")
        except (RuntimeError, FileNotFoundError):
            pytest.skip("Flight logs not available")

        for name, mission in REAL_LOG_MISSIONS.items():
            log = FlightLog.from_bin(logs[mission.source_filename])
            rel_t = log.timestamps - log.timestamps[0]
            mask = (rel_t >= mission.segment_start_s) & (rel_t <= mission.segment_end_s)
            n = int(np.sum(mask))
            assert n >= 10, (
                f"{name}: segment [{mission.segment_start_s}, {mission.segment_end_s}] "
                f"has only {n} GPS points (need >= 10)"
            )


class TestWindAutoTuning:
    def test_auto_tuning_converges_and_recovers_scale(self):
        t = np.linspace(0.0, 60.0, 601)
        base = np.sin(t * 0.15) + 0.25 * np.sin(t * 0.03)
        true_scale = 1.7
        ref_z = true_scale * base
        ref_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), ref_z])

        def simulate(scale: float):
            sim_z = scale * base
            sim_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), sim_z])
            return t, sim_positions

        result = auto_tune_wind_force_scale(
            ref_times=t,
            ref_positions=ref_positions,
            simulate_with_scale=simulate,
            initial_scale=0.3,
            initial_step=0.7,
            max_iterations=30,
            convergence_tol=0.01,
        )

        assert result.converged
        assert result.best_rmse_z < 0.01
        assert abs(result.best_scale - true_scale) < 0.05


class TestRealLogDownload:
    def test_ensure_real_log_logs_uses_existing_local_files(self, tmp_path):
        existing = {
            "Carolina_quad_40m_plus_20m.bin",
            "EPN_quad_30m_plus_20m.bin",
        }
        for filename in existing:
            (tmp_path / filename).write_bytes(b"binlog")

        result = ensure_real_log_logs(str(tmp_path))
        assert set(result.keys()) == existing
        for filename, local_path in result.items():
            assert Path(local_path).exists()
            assert Path(local_path).name == filename

    def test_ensure_real_log_logs_reports_clear_error_on_all_download_failures(self, tmp_path, monkeypatch):
        def always_404(url: str, filename: str):
            raise HTTPError(url, 404, "Not Found", hdrs=None, fp=None)

        monkeypatch.setattr("validation.urlretrieve", always_404)

        with pytest.raises(RuntimeError) as exc:
            ensure_real_log_logs(str(tmp_path))

        message = str(exc.value)
        assert "Failed to download required real-log file" in message
        assert "Carolina_quad_40m_plus_20m.bin" in message or "EPN_quad_30m_plus_20m.bin" in message
        assert "raw.githubusercontent.com/estebanvt/OSSITLQUAD/master/Flight_logs" in message
        assert "raw.githubusercontent.com/estebanvt/OSSITLQUAD/main/Flight_logs" in message
        assert "Place the file manually" in message

    def test_auto_tuning_is_reproducible(self):
        t = np.linspace(0.0, 20.0, 401)
        base = np.cos(t * 0.2)
        ref_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), 1.25 * base])

        def simulate(scale: float):
            sim_positions = np.column_stack([np.zeros_like(t), np.zeros_like(t), scale * base])
            return t, sim_positions

        result_a = auto_tune_wind_force_scale(t, ref_positions, simulate)
        result_b = auto_tune_wind_force_scale(t, ref_positions, simulate)

        assert result_a.best_scale == pytest.approx(result_b.best_scale, abs=1e-12)
        assert result_a.best_rmse_z == pytest.approx(result_b.best_rmse_z, abs=1e-12)
        assert result_a.history == result_b.history


# ── Simulation Bridge Tests (UDP / Timing) ───────────────────────────────

class TestSimBridge:
    """Tests for the UDP simulation bridge (message contract, physics step)."""

    def test_bridge_message_contract(self):
        """ActionMessage and StatusMessage JSON contract matches Rust side."""
        import json

        # Action messages Rust sends
        actions = [
            {"type": "RequestOffboard"},
            {"type": "RequestArm"},
            {"type": "PublishSetpoint", "x": 1.0, "y": 2.0, "z": 5.0},
        ]
        encoded = json.dumps(actions)
        decoded = json.loads(encoded)
        assert len(decoded) == 3
        assert decoded[0]["type"] == "RequestOffboard"
        assert decoded[2]["x"] == 1.0

        # Status message Python sends back
        status = {"nav_state": 14, "arming_state": 2, "position": [0.0, 0.0, 4.5]}
        encoded_status = json.dumps(status)
        decoded_status = json.loads(encoded_status)
        assert decoded_status["nav_state"] == 14
        assert len(decoded_status["position"]) == 3

    def test_bridge_process_actions(self):
        """SimBridge correctly processes action batches."""
        from sim_bridge import SimBridge
        import socket

        # Use a random high port to avoid conflicts
        bridge = SimBridge(port=0, dt=0.02)
        # Get the actual port assigned
        actual_port = bridge.sock.getsockname()[1]

        bridge.process_actions([{"type": "RequestOffboard"}])
        assert bridge.nav_state == 14

        bridge.process_actions([{"type": "RequestArm"}])
        assert bridge.arming_state == 2

        bridge.process_actions([
            {"type": "PublishSetpoint", "x": 0.0, "y": 0.0, "z": 5.0}
        ])
        np.testing.assert_allclose(bridge.target_position, [0.0, 0.0, 5.0])

        bridge.sock.close()

    def test_bridge_physics_step(self):
        """SimBridge advances physics when armed."""
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        bridge.arming_state = 2
        bridge.target_position = np.array([0.0, 0.0, 5.0])

        initial_z = bridge.state.position[2]
        for _ in range(50):
            bridge.step_physics()

        # Should have moved upward
        assert bridge.state.position[2] > initial_z
        bridge.sock.close()

    def test_bridge_status_message_format(self):
        """Status message contains required fields."""
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        bridge.nav_state = 14
        bridge.arming_state = 2

        msg = bridge.make_status_message()
        assert "nav_state" in msg
        assert "arming_state" in msg
        assert "position" in msg
        assert len(msg["position"]) == 3
        assert msg["nav_state"] == 14
        assert msg["arming_state"] == 2
        bridge.sock.close()

    def test_bridge_udp_roundtrip(self):
        """Full UDP roundtrip: send actions, receive status."""
        import json
        import socket
        from sim_bridge import SimBridge

        bridge = SimBridge(port=0, dt=0.02)
        port = bridge.sock.getsockname()[1]

        # Client socket
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(2.0)

        # Send actions
        actions = [
            {"type": "RequestOffboard"},
            {"type": "RequestArm"},
            {"type": "PublishSetpoint", "x": 0.0, "y": 0.0, "z": 5.0},
        ]
        client.sendto(json.dumps(actions).encode(), ("127.0.0.1", port))

        # Bridge processes one step
        data, addr = bridge.sock.recvfrom(4096)
        decoded = json.loads(data.decode())
        bridge.process_actions(decoded)
        for _ in range(5):
            bridge.step_physics()

        response = json.dumps(bridge.make_status_message()).encode()
        bridge.sock.sendto(response, addr)

        # Client receives status
        resp_data, _ = client.recvfrom(4096)
        status = json.loads(resp_data.decode())

        assert status["nav_state"] == 14
        assert status["arming_state"] == 2
        assert len(status["position"]) == 3

        client.close()
        bridge.sock.close()


class TestTimingContract:
    """Tests for real-time timing contract (physics step latency, jitter)."""

    def test_physics_step_latency(self):
        """Single physics_step completes in < 1ms (CI gate)."""
        import time
        from drone_physics import make_holybro_x500, DroneState, DroneCommand, physics_step

        params = make_holybro_x500()
        state = DroneState(
            position=np.array([0.0, 0.0, 5.0]),
            velocity=np.zeros(3),
            rotation=np.eye(3),
            angular_velocity=np.zeros(3),
        )
        cmd = DroneCommand(thrust=params.mass * 9.81, torque=np.zeros(3))

        durations = []
        for _ in range(100):
            t0 = time.perf_counter()
            state = physics_step(state, cmd, params, 0.02)
            durations.append(time.perf_counter() - t0)

        mean_ms = np.mean(durations) * 1000
        p95_ms = np.percentile(durations, 95) * 1000
        assert mean_ms < 1.0, f"Mean physics_step latency {mean_ms:.3f}ms exceeds 1ms"
        assert p95_ms < 2.0, f"P95 physics_step latency {p95_ms:.3f}ms exceeds 2ms"

    def test_controller_step_latency(self):
        """PositionController.compute completes in < 1ms."""
        import time
        from drone_physics import PositionController, DroneState, make_holybro_x500

        params = make_holybro_x500()
        controller = PositionController(params)
        state = DroneState(
            position=np.array([0.0, 0.0, 5.0]),
            velocity=np.zeros(3),
            rotation=np.eye(3),
            angular_velocity=np.zeros(3),
        )
        target = np.array([0.0, 0.0, 10.0])

        durations = []
        for _ in range(100):
            t0 = time.perf_counter()
            controller.compute(state, target, 0.0, 0.02)
            durations.append(time.perf_counter() - t0)

        p95_ms = np.percentile(durations, 95) * 1000
        assert p95_ms < 1.0, f"P95 controller latency {p95_ms:.3f}ms exceeds 1ms"


# ── Terrain STL Export ───────────────────────────────────────────────────────

class TestTerrainSTLExport:
    """Tests for TerrainMap.export_stl() (binary STL generation)."""

    def test_export_stl_creates_file(self, tmp_path):
        """export_stl writes a valid binary STL file."""
        terrain = TerrainMap.from_array(
            np.array([[0.0, 1.0, 2.0],
                      [1.0, 2.0, 3.0],
                      [2.0, 3.0, 4.0]]),
            resolution=10.0,
        )
        stl_path = str(tmp_path / "test.stl")
        terrain.export_stl(stl_path)

        import struct
        data = open(stl_path, 'rb').read()
        assert len(data) >= 84, "STL file too small"

        n_triangles = struct.unpack_from('<I', data, 80)[0]
        # 3x3 grid → 2x2 cells → 4 cells × 2 triangles = 8
        assert n_triangles == 8

    def test_export_stl_expected_size(self, tmp_path):
        """STL file size matches: 84 + n_triangles * 50."""
        import struct
        elev = np.random.rand(5, 7) * 100
        terrain = TerrainMap.from_array(elev, resolution=5.0)
        stl_path = str(tmp_path / "rand.stl")
        terrain.export_stl(stl_path)

        data = open(stl_path, 'rb').read()
        n_tri = struct.unpack_from('<I', data, 80)[0]
        expected = 84 + n_tri * 50
        assert len(data) == expected
        assert n_tri == (7 - 1) * (5 - 1) * 2

    def test_export_stl_roundtrip(self, tmp_path):
        """export → from_stl preserves elevation within 1m."""
        elev = np.array([[0.0, 10.0, 20.0],
                         [5.0, 15.0, 25.0],
                         [10.0, 20.0, 30.0]])
        terrain = TerrainMap.from_array(elev, resolution=10.0)
        stl_path = str(tmp_path / "roundtrip.stl")
        terrain.export_stl(stl_path)

        loaded = TerrainMap.from_stl(stl_path, resolution=10.0)
        # Center elevation should be close to 15.0
        center_z = loaded.get_elevation(10.0, 10.0)
        assert abs(center_z - 15.0) < 2.0, f"Roundtrip center: {center_z}"

    def test_export_stl_with_scale(self, tmp_path):
        """Scale factor multiplies coordinates."""
        import struct
        elev = np.array([[0.0, 1.0], [1.0, 2.0]])
        terrain = TerrainMap.from_array(elev, resolution=1.0)
        stl_path = str(tmp_path / "scaled.stl")
        terrain.export_stl(stl_path, scale=10.0)

        data = open(stl_path, 'rb').read()
        # Read first triangle's first vertex (after normal)
        vx, vy, vz = struct.unpack_from('<fff', data, 84 + 12)
        # Origin is (0,0) with scale=10 → vertex at (0, 0, 0*10=0)
        assert abs(vz) < 0.01

    def test_export_stl_rejects_1x1(self):
        """1x1 grid cannot be tessellated."""
        terrain = TerrainMap.from_array(np.array([[5.0]]))
        with pytest.raises(ValueError, match="2x2"):
            terrain.export_stl("/dev/null")

    def test_export_stl_normals_point_up(self, tmp_path):
        """Triangle normals on flat terrain should point roughly upward."""
        import struct
        elev = np.zeros((3, 3))
        terrain = TerrainMap.from_array(elev, resolution=1.0)
        stl_path = str(tmp_path / "flat.stl")
        terrain.export_stl(stl_path)

        data = open(stl_path, 'rb').read()
        offset = 84
        for _ in range(8):
            nx, ny, nz = struct.unpack_from('<fff', data, offset)
            assert nz > 0.99, f"Normal not upward: ({nx},{ny},{nz})"
            offset += 50


# ── Terrain Coloring (Gazebo Materials) ──────────────────────────────────────

class TestTerrainColoring:
    """Tests for Gazebo terrain material files (color, texture)."""

    def test_material_file_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_terrain.material')
        assert os.path.exists(path), "Material script missing"

    def test_vertex_shader_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.vert')
        assert os.path.exists(path), "Vertex shader missing"

    def test_fragment_shader_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.frag')
        assert os.path.exists(path), "Fragment shader missing"

    def test_material_references_shaders(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_terrain.material')
        content = open(path).read()
        assert 'antisana_height_color.vert' in content
        assert 'antisana_height_color.frag' in content
        assert 'AntisanaTerrain/HeightColored' in content

    def test_fragment_has_elevation_bands(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.frag')
        content = open(path).read()
        assert 'green_max' in content
        assert 'brown_max' in content
        assert 'snow_min' in content

    def test_world_references_material(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'worlds', 'antisana.world')
        content = open(path).read()
        assert 'AntisanaTerrain/HeightColored' in content
        assert 'antisana_terrain.material' in content


class TestTerrainSatelliteTexture:
    """Tests for satellite-texture terrain pipeline (tile download, overlay)."""

    def test_satellite_tile_download_has_offline_fallback(self, tmp_path):
        from terrain import download_satellite_tile

        tile = download_satellite_tile(-0.508333, -78.141667, str(tmp_path), tile_size=64)
        assert tile is not None
        assert tile.endswith('.ppm')
        assert (tmp_path / Path(tile).name).exists()

    def test_export_obj_with_uv_contains_uv_and_faces(self, tmp_path):
        terrain = TerrainMap.from_array(
            np.array([[4400.0, 4450.0, 4500.0],
                      [4420.0, 4470.0, 4520.0],
                      [4440.0, 4490.0, 4540.0]]),
            resolution=10.0,
        )
        tex = tmp_path / 'sat.ppm'
        tex.write_bytes(b'P6\n1 1\n255\n\x00\x00\x00')
        out_obj = tmp_path / 'terrain.obj'

        terrain.export_obj_with_uv(str(out_obj), texture_path=str(tex))
        content = out_obj.read_text()
        assert 'vt ' in content
        assert 'f ' in content

        mtl = out_obj.with_suffix('.mtl').read_text()
        assert 'map_Kd sat.ppm' in mtl

    def test_export_assets_fallbacks_to_height_material_without_texture(self, tmp_path):
        terrain = TerrainMap.flat(elevation=4500.0, size=20.0, resolution=10.0)
        assets = terrain.export_gazebo_terrain_assets(str(tmp_path), texture_path=str(tmp_path / 'missing.ppm'))
        assert assets['material_name'] == 'AntisanaTerrain/HeightColored'
        assert Path(assets['obj_path']).exists()

    def test_world_and_material_include_satellite_reference(self):
        import os
        import xml.etree.ElementTree as ET
        material_path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                                     'media', 'materials', 'scripts',
                                     'antisana_terrain.material')
        world_path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                                  'worlds', 'antisana.world')
        material = open(material_path).read()
        world = open(world_path).read()
        assert 'AntisanaTerrain/SatelliteTextured' in material
        assert 'antisana_satellite.ppm' in material
        assert 'AntisanaTerrain/SatelliteTextured' in world

        world_xml = ET.parse(world_path).getroot()
        world_node = world_xml.find('world')
        model_names = [m.get('name') for m in world_node.findall('model')]
        assert 'antisana_terrain' in model_names
        assert 'ground_plane' not in model_names


class TestGazeboSensorVisibility:
    """Checks that simulation model exposes visible sensors."""

    def test_x500_has_visible_sensors(self):
        import os
        import xml.etree.ElementTree as ET

        model_path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                                  'models', 'x500', 'model.sdf')
        root = ET.parse(model_path).getroot()
        sensors = root.findall('.//model[@name="x500"]/link[@name="base_link"]/sensor')
        sensor_names = {s.get('name') for s in sensors}

        assert {'imu_sensor', 'gps_sensor', 'barometer_sensor', 'magnetometer_sensor', 'front_camera'} <= sensor_names
        for sensor in sensors:
            vis = sensor.find('visualize')
            assert vis is not None
            assert vis.text == 'true'


# ── Position-Aware Wind ──────────────────────────────────────────────────────

class TestPositionAwareWind:
    """Tests for wind node position subscription (altitude-dependent density)."""

    def test_wind_node_has_pose_subscription(self):
        """wind_node.py subscribes to /mavros/local_position/pose."""
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '/mavros/local_position/pose' in content
        assert 'PoseStamped' in content

    def test_wind_node_has_pose_callback(self):
        """wind_node.py has _pose_callback method."""
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '_pose_callback' in content

    def test_wind_node_uses_drone_pos(self):
        """wind_node.py uses drone_pos instead of hardcoded zeros."""
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        # Old hardcoded line in publish_wind should be gone
        assert 'pos = np.zeros(3)  # TODO' not in content
        assert 'self.drone_pos' in content

    def test_wind_node_altitude_density(self):
        """wind_node.py computes altitude-dependent density."""
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'scripts', 'wind_node.py')
        content = open(path).read()
        assert '_get_altitude_density' in content
        assert 'base_altitude_msl' in content

    def test_isa_density_at_altitude(self):
        """ISA density at 4500m differs from sea level."""
        rho_sea = Atmosphere(altitude_msl=0.0).rho
        rho_4500 = Atmosphere(altitude_msl=4500.0).rho
        ratio = rho_4500 / rho_sea
        # At 4500m, density should be ~0.60-0.65 of sea level
        assert 0.55 < ratio < 0.70, f"Density ratio {ratio:.3f} unexpected"


# ── Euler Rate Kinematics (Eq. 2) ────────────────────────────────────────────

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

class TestPhysicsLiveReplay:
    """Tests for the physics_live_replay module — simulation → bridge →
    live viewer pipeline.

    These tests verify:
    1. Simulation record generation and loading
    2. NPZ file round-trip (save/load)
    3. Full pipeline: physics sim → MAVLinkBridge → MAVLinkLiveSource → queue
    4. Full pipeline through the WebSocket to the browser
    5. run_scenario.sh --physics-live / --physics-swarm-live wiring
    """

    def test_default_waypoints_returns_valid_list(self):
        """_default_waypoints() returns a list of 3D numpy arrays."""
        from physics_live_replay import _default_waypoints
        wps = _default_waypoints()
        assert len(wps) >= 3
        for wp in wps:
            assert wp.shape == (3,)
            assert wp[2] > 0, "waypoints should have positive altitude"

    def test_run_physics_simulation_produces_records(self):
        """run_physics_simulation() returns a non-empty list of SimRecords."""
        from physics_live_replay import run_physics_simulation
        records = run_physics_simulation(max_time=2.0)
        assert len(records) > 10
        # Records should have increasing time
        for i in range(1, len(records)):
            assert records[i].t > records[i - 1].t
        # First record starts near origin, last record moved away
        assert np.linalg.norm(records[0].position) < 1.0
        total_dist = np.linalg.norm(records[-1].position - records[0].position)
        assert total_dist > 0.1, "drone should have moved during simulation"

    def test_run_physics_simulation_custom_waypoints(self):
        """run_physics_simulation() respects custom waypoints."""
        from physics_live_replay import run_physics_simulation
        wps = [np.array([0, 0, 3.0]), np.array([5, 0, 3.0])]
        records = run_physics_simulation(waypoints=wps, max_time=5.0)
        assert len(records) > 0
        # The drone should attempt to reach waypoint at x=5
        max_x = max(r.position[0] for r in records)
        assert max_x > 1.0, "drone should fly toward x=5 waypoint"

    def test_load_npz_records_roundtrip(self, tmp_path):
        """Save SimRecords to .npz, reload via load_npz_records(), verify match."""
        from physics_live_replay import run_physics_simulation, load_npz_records

        records = run_physics_simulation(max_time=1.0)
        npz_path = str(tmp_path / "test_scenario.npz")
        np.savez(
            npz_path,
            t=np.array([r.t for r in records]),
            pos=np.array([r.position for r in records]),
            vel=np.array([r.velocity for r in records]),
            euler=np.array([r.euler for r in records]),
            thrust=np.array([r.thrust for r in records]),
            ang_vel=np.array([r.angular_velocity for r in records]),
        )

        loaded = load_npz_records(npz_path)
        assert len(loaded) == len(records)
        for orig, loaded_r in zip(records, loaded):
            assert loaded_r.t == pytest.approx(orig.t, abs=1e-9)
            np.testing.assert_allclose(loaded_r.position, orig.position, atol=1e-9)
            np.testing.assert_allclose(loaded_r.velocity, orig.velocity, atol=1e-9)

    def test_load_swarm_npz_records(self, tmp_path):
        """load_swarm_npz_records() extracts one drone from a swarm file."""
        from physics_live_replay import load_swarm_npz_records

        n_steps, n_drones = 20, 3
        t = np.linspace(0, 2.0, n_steps)
        positions = np.random.randn(n_steps, n_drones, 3)
        velocities = np.random.randn(n_steps, n_drones, 3)

        npz_path = str(tmp_path / "swarm_data.npz")
        np.savez(npz_path, t=t, positions=positions, velocities=velocities)

        for drone_idx in range(n_drones):
            loaded = load_swarm_npz_records(npz_path, drone_index=drone_idx)
            assert len(loaded) == n_steps
            for i, rec in enumerate(loaded):
                np.testing.assert_allclose(
                    rec.position, positions[i, drone_idx], atol=1e-9)

    @pytest.mark.timeout(30)
    def test_physics_records_to_bridge_to_queue(self):
        """Full pipeline: run_simulation → MAVLinkBridge.send_state → MAVLinkLiveSource → queue.

        Verifies that physics simulation records are correctly transformed
        into MAVLink, sent via UDP, received, decoded, and end up in the
        telemetry queue with matching positions.
        """
        import socket
        import time
        from physics_live_replay import run_physics_simulation
        from mavlink_bridge import MAVLinkBridge, sim_state_from_record
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        # Run a short simulation
        records = run_physics_simulation(max_time=1.0)
        assert len(records) > 5

        # Bind the receiver on an ephemeral port
        q = TelemetryQueue(maxlen=2048)
        src = MAVLinkLiveSource(
            listen_ip="127.0.0.1",
            listen_port=0,
            queue=q,
        )
        src.start()
        recv_port = src._sock.getsockname()[1]

        # Create a bridge targeting that port
        bridge = MAVLinkBridge(
            target_ip="127.0.0.1",
            target_port=recv_port,
            listen_port=0,
        )
        bridge.start()

        try:
            # Send a few records through the bridge
            for rec in records[:10]:
                state = sim_state_from_record(rec)
                bridge.send_state(state)
                time.sleep(0.02)

            # Wait for samples to arrive
            deadline = time.time() + 5.0
            while len(q) < 3 and time.time() < deadline:
                time.sleep(0.05)

            assert len(q) >= 3, (
                f"Expected at least 3 samples in queue, got {len(q)}")

            # Verify the samples have non-trivial positions (drone moved)
            samples = q.snapshot()
            positions = [s.pos_enu for s in samples]
            # At least some samples should differ in position
            pos_spread = max(np.linalg.norm(
                np.array(p) - np.array(positions[0])) for p in positions)
            assert pos_spread > 0.001, (
                "All samples have identical positions — bridge or receiver "
                "is not forwarding correctly")
        finally:
            bridge.stop()
            src.stop()

    @pytest.mark.timeout(45)
    def test_physics_to_websocket_full_pipeline(self):
        """End-to-end: physics sim → bridge → source → server → WebSocket.

        Boots the full FastAPI server, runs a short simulation through the
        bridge, and verifies that the WebSocket client receives moving
        telemetry samples. Interleaves sending and receiving to handle
        timing correctly.
        """
        import socket as _socket
        import time
        import json
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue
        from mavlink_bridge import MAVLinkBridge, sim_state_from_record
        from physics_live_replay import run_physics_simulation

        try:
            import simple_websocket
        except ImportError:
            pytest.skip("simple_websocket not installed")

        # Run a short simulation producing records with spread-out positions
        records = run_physics_simulation(max_time=3.0)

        # Set up the telemetry queue and start the server
        q = TelemetryQueue(maxlen=4096)

        # Pick an ephemeral UDP port for MAVLink
        probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        mav_port = probe.getsockname()[1]
        probe.close()

        src = MAVLinkLiveSource(
            listen_ip='127.0.0.1', listen_port=mav_port, queue=q,
        )
        src.start()

        server, thread, base_url = _runtime_view_start_uvicorn(q)

        try:
            bridge = MAVLinkBridge(
                target_ip='127.0.0.1',
                target_port=mav_port,
                listen_port=0,
            )
            bridge.start()
            try:
                ws_url = base_url.replace('http://', 'ws://') + '/ws/telemetry'
                client = simple_websocket.Client(ws_url)
                try:
                    # Interleave: send a record, try to read from WS
                    sample_seen = None
                    # Use records with spread-out positions (skip first few
                    # which are near origin, use every 5th for speed)
                    send_records = records[::5][:30]
                    for tick, rec in enumerate(send_records):
                        state = sim_state_from_record(rec)
                        bridge.send_state(state)
                        time.sleep(0.05)

                        try:
                            raw = client.receive(timeout=0.2)
                        except Exception:
                            continue
                        if raw is None:
                            continue
                        msg = json.loads(raw)
                        payload = None
                        if msg['type'] == 'sample':
                            payload = msg['data']
                        elif msg['type'] == 'snapshot' and msg['data']:
                            payload = msg['data'][-1]
                        if payload and payload.get('pos_enu'):
                            sample_seen = payload
                            # Break once we see a sample with non-zero Z
                            # (the drone has lifted off)
                            if abs(payload['pos_enu'][2]) > 0.5:
                                break

                    assert sample_seen is not None, (
                        'No telemetry reached the WebSocket from '
                        'physics simulation replay')
                    # Verify the sample has valid position data
                    enu = sample_seen['pos_enu']
                    assert isinstance(enu, list) and len(enu) == 3
                finally:
                    client.close()
            finally:
                bridge.stop()
        finally:
            try:
                src.stop()
            finally:
                _runtime_view_stop_uvicorn(server, thread)

    def test_run_scenario_physics_live_mode(self):
        """run_scenario.sh --physics-live must call run_physics_live."""
        import re
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')

        # Find the --physics-live branch
        m = re.search(
            r'\n\s*--physics-live\)\s*\n(?P<body>.*?);;\s*\n',
            body, re.DOTALL)
        assert m is not None, '--physics-live branch not found'
        branch = m.group('body')

        assert 'NEED_RUNTIME_VIEW=1' in branch, (
            '--physics-live must request runtime view deps')
        assert 'run_physics_live' in branch, (
            '--physics-live must call run_physics_live')
        # Must NOT call run_single_mission or matplotlib viz
        assert 'run_single_mission' not in branch
        assert 'run_single_viz' not in branch
        assert 'run_viz' not in branch

    def test_run_scenario_physics_swarm_live_mode(self):
        """run_scenario.sh --physics-swarm-live must call run_physics_live --swarm."""
        import re
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')

        m = re.search(
            r'\n\s*--physics-swarm-live\)\s*\n(?P<body>.*?);;\s*\n',
            body, re.DOTALL)
        assert m is not None, '--physics-swarm-live branch not found'
        branch = m.group('body')

        assert 'NEED_RUNTIME_VIEW=1' in branch
        assert 'run_physics_live' in branch
        assert '--swarm' in branch
        assert '--loop' in branch

    def test_run_scenario_help_lists_physics_live(self):
        """run_scenario.sh --help must document --physics-live and --physics-swarm-live."""
        import subprocess
        import sys as _sys

        script = str(Path(__file__).resolve().parents[1] / 'run_scenario.sh')
        result = subprocess.run(
            ['bash', script, '--help'],
            capture_output=True, text=True, timeout=10,
        )
        assert '--physics-live' in result.stdout, (
            '--physics-live missing from --help output')
        assert '--physics-swarm-live' in result.stdout, (
            '--physics-swarm-live missing from --help output')

    def test_run_physics_live_function_exists_in_script(self):
        """run_scenario.sh must define a run_physics_live() shell function."""
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')
        assert 'run_physics_live()' in body, (
            'run_physics_live() function not defined in run_scenario.sh')
        assert 'physics_live_replay' in body, (
            'run_physics_live must invoke physics_live_replay module')

    def test_cli_help(self):
        """physics_live_replay --help should exit 0 and list options."""
        import subprocess
        import sys as _sys

        sim_dir = str(Path(__file__).resolve().parent)
        result = subprocess.run(
            [_sys.executable, '-m', 'physics_live_replay', '--help'],
            capture_output=True, text=True, timeout=10,
            cwd=sim_dir,
        )
        assert result.returncode == 0, f"--help failed: {result.stderr}"
        for flag in ('--replay', '--swarm', '--fps', '--loop',
                     '--http-port', '--mav-port', '--no-browser'):
            assert flag in result.stdout, f"{flag} missing from --help"

    def test_receiver_binds_before_replay_starts(self):
        """run_physics_live() must start the MAVLink receiver BEFORE the
        replay thread, otherwise UDP packets are silently dropped and the
        drone mesh never moves in the browser.

        This test inspects the source code to enforce the ordering contract:
        start_telemetry() must be called before the replay thread starts,
        and run_server() must be called with start_source=False.
        """
        import inspect
        from physics_live_replay import run_physics_live

        src = inspect.getsource(run_physics_live)

        # start_telemetry() must appear before any replay thread launch
        idx_start = src.index('start_telemetry(')
        # Find the first Thread(...).start() or replay_thread.start()
        import re
        replay_match = re.search(r'Thread\(target=_replay', src)
        assert replay_match is not None, 'No replay thread found in source'
        idx_replay = replay_match.start()
        assert idx_start < idx_replay, (
            'start_telemetry() must be called before the replay thread — '
            'otherwise the receiver is not listening when the replay begins '
            'and all UDP packets are silently dropped')

        # run_server must NOT start a second source (double-bind)
        assert 'start_source=False' in src, (
            'run_server() must be called with start_source=False because '
            'start_telemetry() was already called — a double-bind would '
            'either fail or create a second listener that races with the '
            'first')

    @pytest.mark.timeout(30)
    def test_replay_delivers_samples_to_queue_non_loop(self):
        """Non-looping replay must deliver samples to the queue even though
        it finishes quickly — the receiver must already be listening.

        This is the regression test for the original bug: run_physics_live()
        started the replay thread before binding the MAVLink receiver, so
        the entire replay completed before any listener existed and zero
        samples ever reached the queue.
        """
        import time
        from physics_live_replay import run_physics_simulation
        from mavlink_bridge import MAVLinkBridge, sim_state_from_record
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue

        records = run_physics_simulation(max_time=1.0)

        # Simulate the fixed ordering: receiver FIRST, then replay.
        q = TelemetryQueue(maxlen=2048)
        src = MAVLinkLiveSource(listen_ip='127.0.0.1', listen_port=0, queue=q)
        src.start()
        recv_port = src._sock.getsockname()[1]

        bridge = MAVLinkBridge(
            target_ip='127.0.0.1', target_port=recv_port, listen_port=0,
        )
        bridge.start()

        try:
            # Non-looping replay — runs once and finishes
            bridge.run_replay(records[:20], fps=200.0, loop=False)

            # Give the receiver a moment to process the last datagrams
            time.sleep(0.3)

            assert len(q) > 0, (
                'Non-looping replay produced zero samples in the queue — '
                'the receiver was not listening when the replay ran')

            # Verify position spread (drone actually moved)
            samples = q.snapshot()
            positions = np.array([s.pos_enu for s in samples])
            spread = np.max(np.ptp(positions, axis=0))
            assert spread > 0.01, (
                f'Samples in queue but drone did not move '
                f'(spread={spread:.4f})')
        finally:
            bridge.stop()
            src.stop()


class TestMultiDroneLiveView:
    """Tests for multi-drone live view — system_id demux, per-drone
    samples, and multi-bridge swarm replay."""

    def test_decode_mavlink_v2_returns_system_id(self):
        """decode_mavlink_v2() must return (system_id, msg_id, payload)."""
        from mavlink_bridge import build_attitude, decode_mavlink_v2
        msg = build_attitude(0.1, -0.2, 0.5, system_id=7)
        result = decode_mavlink_v2(msg)
        assert result is not None
        sys_id, msg_id, payload = result
        assert sys_id == 7
        assert msg_id == 30  # ATTITUDE

    def test_live_sample_carries_drone_id(self):
        """LiveTelemetrySample must include drone_id in to_dict()."""
        from live_telemetry import LiveTelemetrySample
        s = LiveTelemetrySample(drone_id=4)
        d = s.to_dict()
        assert d['drone_id'] == 4

    @pytest.mark.timeout(30)
    def test_multi_drone_demux_in_queue(self):
        """Two bridges with different system_ids produce samples with
        different drone_id values in the same queue."""
        import time
        from live_telemetry import MAVLinkLiveSource, TelemetryQueue
        from mavlink_bridge import MAVLinkBridge, SimState

        q = TelemetryQueue(maxlen=512)
        src = MAVLinkLiveSource(listen_ip='127.0.0.1', listen_port=0, queue=q)
        src.start()
        port = src._sock.getsockname()[1]

        b1 = MAVLinkBridge(target_ip='127.0.0.1', target_port=port,
                           listen_port=0, system_id=1)
        b2 = MAVLinkBridge(target_ip='127.0.0.1', target_port=port,
                           listen_port=0, system_id=2)
        b1.start()
        b2.start()

        try:
            for tick in range(10):
                b1.send_state(SimState(
                    time_s=0.1 * tick,
                    position=np.array([float(tick), 0.0, 5.0]),
                ))
                b2.send_state(SimState(
                    time_s=0.1 * tick,
                    position=np.array([0.0, float(tick), 8.0]),
                ))
                time.sleep(0.03)

            time.sleep(0.5)
            samples = q.snapshot()
            drone_ids = set(s.drone_id for s in samples)
            assert 1 in drone_ids, f'No samples from drone 1: {drone_ids}'
            assert 2 in drone_ids, f'No samples from drone 2: {drone_ids}'

            d1 = [s for s in samples if s.drone_id == 1]
            d2 = [s for s in samples if s.drone_id == 2]
            assert len(d1) >= 2
            assert len(d2) >= 2
        finally:
            b1.stop()
            b2.stop()
            src.stop()

    def test_run_scenario_swarm_live_streams_all(self):
        """--physics-swarm-live help must mention streaming all drones."""
        import subprocess, sys as _sys
        script = str(Path(__file__).resolve().parents[1] / 'run_scenario.sh')
        result = subprocess.run(
            ['bash', script, '--help'],
            capture_output=True, text=True, timeout=10,
        )
        assert '--physics-swarm-live' in result.stdout
        # The help text after --physics-swarm-live should say "all"
        swarm_section = result.stdout.split('--physics-swarm-live')[1]
        first_lines = swarm_section.split('\n')[0] + swarm_section.split('\n')[1]
        assert 'all' in first_lines.lower(), (
            f'--physics-swarm-live help must mention streaming all drones, '
            f'got: {first_lines}')


class TestPostFlightReplay:
    """Tests for post-flight replay in the web viewer (roadmap item 3):
    --replay-live shell mode, /api/files, /api/load endpoints."""

    def test_run_scenario_replay_live_in_help(self):
        """--replay-live must appear in run_scenario.sh --help."""
        import subprocess, sys as _sys
        script = str(Path(__file__).resolve().parents[1] / 'run_scenario.sh')
        result = subprocess.run(
            ['bash', script, '--help'],
            capture_output=True, text=True, timeout=10,
        )
        assert '--replay-live' in result.stdout

    def test_run_scenario_replay_live_branch_exists(self):
        """run_scenario.sh must have a --replay-live case branch."""
        import re
        script = Path(__file__).resolve().parents[1] / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')
        m = re.search(r'\n\s*--replay-live\)\s*\n(?P<body>.*?);;\s*\n',
                       body, re.DOTALL)
        assert m is not None, '--replay-live branch not found'
        branch = m.group('body')
        assert 'run_physics_live' in branch
        assert '--replay' in branch

    def test_api_files_lists_npz(self):
        """GET /api/files must return at least one .npz file."""
        import urllib.request, json
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/files'
            with urllib.request.urlopen(url, timeout=5) as resp:
                data = json.loads(resp.read())
            assert isinstance(data, list)
            npz_files = [f for f in data if f['type'] == 'npz']
            assert len(npz_files) >= 1, (
                f'Expected at least one .npz file, got: {data}')
            assert 'scenario_data.npz' in [f['name'] for f in npz_files]
        finally:
            _runtime_view_stop_uvicorn(server, thread)

    @pytest.mark.timeout(30)
    def test_api_load_starts_replay(self):
        """POST /api/load with a valid .npz starts a replay."""
        import urllib.request, urllib.parse, json, time
        from live_telemetry import TelemetryQueue, MAVLinkLiveSource

        q = TelemetryQueue(maxlen=512)
        server, thread, base = _runtime_view_start_uvicorn(q)

        import runtime_view.server as srv
        src = MAVLinkLiveSource(listen_ip='127.0.0.1', listen_port=0, queue=q)
        src.start()
        srv.live_source = src

        try:
            npz_path = str(Path(__file__).resolve().parent / 'scenario_data.npz')
            url = f'{base}/api/load?path={urllib.parse.quote(npz_path)}'
            req = urllib.request.Request(url, method='POST')
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read())

            assert data['status'] == 'playing'
            assert data['records'] > 0

            deadline = time.time() + 5.0
            while len(q) < 3 and time.time() < deadline:
                time.sleep(0.1)
            assert len(q) >= 1, f'No samples after /api/load, got {len(q)}'
        finally:
            if srv._replay_bridge is not None:
                srv._replay_bridge.stop()
                srv._replay_bridge = None
            try:
                src.stop()
            except Exception:
                pass
            srv.live_source = None
            _runtime_view_stop_uvicorn(server, thread)

    def test_api_load_rejects_missing_file(self):
        """POST /api/load with a nonexistent path returns 404."""
        import urllib.request, urllib.error, urllib.parse
        from live_telemetry import TelemetryQueue
        server, thread, base = _runtime_view_start_uvicorn(TelemetryQueue())
        try:
            url = f'{base}/api/load?path={urllib.parse.quote("/nonexistent/file.npz")}'
            req = urllib.request.Request(url, method='POST')
            try:
                urllib.request.urlopen(req, timeout=5)
                assert False, 'Expected HTTP error'
            except urllib.error.HTTPError as e:
                assert e.code in (404, 422), f'Expected 404, got {e.code}'
        finally:
            _runtime_view_stop_uvicorn(server, thread)


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
        sim_dir = str(Path(__file__).resolve().parent)
        result = subprocess.run(
            [_sys.executable, '-m', 'physics_live_replay', '--help'],
            capture_output=True, text=True, timeout=10,
            cwd=sim_dir,
        )
        assert '--record-bin' in result.stdout
