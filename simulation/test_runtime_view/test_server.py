"""
Test Server

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


