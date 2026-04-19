"""
Test Sensors

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
