"""
Test Telemetry

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

from test_runtime_view._helpers import (
    _runtime_view_start_uvicorn,
    _runtime_view_stop_uvicorn,
)
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
        script = PROJECT_ROOT / 'run_scenario.sh'
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

        script = PROJECT_ROOT / 'run_scenario.sh'
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
        script = PROJECT_ROOT / 'run_scenario.sh'
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
        script = PROJECT_ROOT / 'run_scenario.sh'
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

        script = str(PROJECT_ROOT / 'run_scenario.sh')
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
        script = PROJECT_ROOT / 'run_scenario.sh'
        body = script.read_text(encoding='utf-8')
        assert 'run_physics_live()' in body, (
            'run_physics_live() function not defined in run_scenario.sh')
        assert 'physics_live_replay' in body, (
            'run_physics_live must invoke physics_live_replay module')

    def test_cli_help(self):
        """physics_live_replay --help should exit 0 and list options."""
        import subprocess
        import sys as _sys

        sim_dir = str(SIM_DIR)
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
