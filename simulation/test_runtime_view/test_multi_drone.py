"""
Test Multi Drone

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
        script = str(PROJECT_ROOT / 'run_scenario.sh')
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
