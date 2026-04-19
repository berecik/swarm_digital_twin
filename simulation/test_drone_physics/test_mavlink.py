"""
Test Mavlink

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
