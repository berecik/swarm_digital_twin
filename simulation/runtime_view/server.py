"""
Run-time View Flask Server - Swarm Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Serves the pysimverse-style mission launcher (``web/index.html``) and
the live 3D view (``web/live.html``) plus the REST/WebSocket API that
feeds them telemetry pushed into :class:`live_telemetry.TelemetryQueue`.

Run standalone with::

    python -m runtime_view.server --port 8765 --listen-port 14550

from the ``simulation/`` directory, or via ``run_scenario.sh --viz-live``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

# Allow running as ``python -m runtime_view.server`` from the project
# root as well as from the ``simulation/`` directory. The sibling
# ``live_telemetry`` and ``mavlink_bridge`` modules must be importable
# either way.
_THIS_DIR = Path(__file__).resolve().parent  # .../simulation/runtime_view
_SIM_DIR = _THIS_DIR.parent                  # .../simulation
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from flask import Flask, jsonify, send_from_directory, abort  # noqa: E402
from flask_sock import Sock  # noqa: E402

from live_telemetry import (  # noqa: E402
    LiveTelemetrySample,
    MAVLinkLiveSource,
    TelemetryQueue,
)


# ── App factory ──────────────────────────────────────────────────────────

WEB_DIR = _THIS_DIR / "web"
MISSIONS_PATH = _THIS_DIR / "missions.json"

app = Flask(
    __name__,
    static_folder=str(WEB_DIR),
    static_url_path="/static",
    template_folder=str(WEB_DIR),
)
sock = Sock(app)

# Global singletons — the module-level names are imported by the unit
# tests, so keep them stable.
telemetry_queue: TelemetryQueue = TelemetryQueue(maxlen=4096)
live_source: Optional[MAVLinkLiveSource] = None


# ── Routes: HTML pages ──────────────────────────────────────────────────


def _read_template(name: str) -> str:
    """Read an HTML file by name.

    Tries ``app.template_folder`` first (so unit tests that monkey-patch
    it to a tmp_path keep working), then falls back to the canonical
    ``WEB_DIR`` shipped with the package. This makes the routes resilient
    to test pollution that forgets to restore ``template_folder`` between
    cases.
    """
    candidates = []
    if app.template_folder:
        candidates.append(Path(app.template_folder) / name)
    if Path(app.template_folder or "") != WEB_DIR:
        candidates.append(WEB_DIR / name)
    for path in candidates:
        if path.exists():
            return path.read_text(encoding="utf-8")
    abort(404)


@app.route("/")
def index():
    """Render the mission launcher."""
    return _read_template("index.html")


@app.route("/live")
def live_view():
    """Render the live 3D view."""
    return _read_template("live.html")


# ── Routes: static assets ───────────────────────────────────────────────


@app.route("/web/<path:filename>")
def web_assets(filename: str):
    """Serve CSS/JS/img/vendor assets from the ``web`` directory."""
    folder = Path(app.static_folder or WEB_DIR)
    return send_from_directory(str(folder), filename)


# ── Routes: REST API ────────────────────────────────────────────────────


def _load_missions() -> list:
    if MISSIONS_PATH.exists():
        try:
            return json.loads(MISSIONS_PATH.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return []
    return []


@app.route("/api/missions")
def api_missions():
    """Return the mission catalogue as JSON."""
    return jsonify(_load_missions())


@app.route("/api/status")
def api_status():
    """Return the current telemetry/connection status."""
    latest = telemetry_queue.latest()
    running = bool(live_source is not None and getattr(live_source, "_running", False))
    return jsonify({
        "connected": running,
        "sample_count": len(telemetry_queue),
        "latest_sample": latest.to_dict() if latest is not None else None,
    })


@app.route("/api/snapshot")
def api_snapshot():
    """Return the last ``n`` telemetry samples (default 100, max 1000)."""
    from flask import request
    try:
        n = int(request.args.get("n", "100"))
    except (TypeError, ValueError):
        n = 100
    n = max(1, min(1000, n))
    samples = telemetry_queue.snapshot(n=n)
    return jsonify([s.to_dict() for s in samples])


# ── WebSocket route ─────────────────────────────────────────────────────


@sock.route("/ws/telemetry")
def telemetry_stream(ws):
    """Push the latest :class:`LiveTelemetrySample` at ~50 Hz."""
    last_t: float = -1.0
    last_heartbeat = time.time()
    try:
        # Kick off with a snapshot so the client paints immediately.
        initial = telemetry_queue.snapshot(n=200)
        if initial:
            ws.send(json.dumps({
                "type": "snapshot",
                "data": [s.to_dict() for s in initial],
            }))
        while True:
            latest = telemetry_queue.latest()
            if latest is not None and latest.t_wall > last_t:
                ws.send(json.dumps({"type": "sample", "data": latest.to_dict()}))
                last_t = latest.t_wall
            now = time.time()
            if now - last_heartbeat >= 5.0:
                ws.send(json.dumps({"type": "ping", "t": now}))
                last_heartbeat = now
            time.sleep(0.02)  # 50 Hz cap
    except Exception:
        # flask-sock raises on disconnect; treat that as a normal close.
        return


# ── Lifecycle helpers ───────────────────────────────────────────────────


def start_telemetry(listen_port: int = 14550, listen_ip: str = "0.0.0.0") -> None:
    """Open the MAVLink UDP socket and start the receiver thread."""
    global live_source
    if live_source is not None:
        return
    live_source = MAVLinkLiveSource(
        listen_ip=listen_ip,
        listen_port=listen_port,
        queue=telemetry_queue,
    )
    live_source.start()


def stop_telemetry() -> None:
    global live_source
    if live_source is not None:
        try:
            live_source.stop()
        finally:
            live_source = None


def run_server(
    host: str = "127.0.0.1",
    port: int = 8765,
    listen_port: int = 14550,
    start_source: bool = True,
) -> None:
    """Run the Flask development server (blocking)."""
    if start_source:
        start_telemetry(listen_port=listen_port)
    try:
        app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)
    finally:
        stop_telemetry()


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="runtime_view.server",
        description="Flask + Three.js Run-time View for the Swarm Digital Twin.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    parser.add_argument(
        "--listen-port", type=int, default=14550,
        help="UDP port to receive MAVLink telemetry on",
    )
    parser.add_argument(
        "--no-source", action="store_true",
        help="Skip binding the MAVLink UDP listener (useful when an "
             "external bridge is already running on the port, or for "
             "hermetic unit tests).",
    )
    args = parser.parse_args(argv)
    run_server(
        host=args.host,
        port=args.port,
        listen_port=args.listen_port,
        start_source=not args.no_source,
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
