"""
Shared uvicorn-server helpers for the runtime-view test suite.

Originally inlined in `test_drone_physics.TestRunTimeViewIntegration`;
extracted when the file was split per-domain so that
`test_server.py`, `test_telemetry.py`, `test_replay.py`, and
`test_launch.py` can all share the same boot/teardown plumbing.
"""

from __future__ import annotations


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
