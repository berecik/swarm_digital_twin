"""
Run-time View FastAPI Server - Swarm Digital Twin
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
The HTTP server is FastAPI on top of uvicorn; the WebSocket route uses
Starlette's native ``WebSocket`` API instead of flask-sock so the same
process serves both REST and live telemetry over a single ASGI app.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

# Allow running as ``python -m runtime_view.server`` from the project
# root as well as from the ``simulation/`` directory. The sibling
# ``live_telemetry`` and ``mavlink_bridge`` modules must be importable
# either way.
_THIS_DIR = Path(__file__).resolve().parent  # .../simulation/runtime_view
_SIM_DIR = _THIS_DIR.parent                  # .../simulation
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse  # noqa: E402
import uvicorn  # noqa: E402

from live_telemetry import (  # noqa: E402
    LiveTelemetrySample,
    MAVLinkLiveSource,
    TelemetryQueue,
)


# ── Web/asset paths ──────────────────────────────────────────────────────

WEB_DIR = _THIS_DIR / "web"
MISSIONS_PATH = _THIS_DIR / "missions.json"


# ── App + module-level singletons ───────────────────────────────────────
#
# The unit tests import ``app``, ``telemetry_queue``, ``live_source``,
# ``WEB_DIR``, ``MISSIONS_PATH``, ``start_telemetry`` and
# ``stop_telemetry`` directly, so the symbol shape must stay stable
# across the Flask → FastAPI migration.

app = FastAPI(
    title="Swarm Digital Twin — Run-time View",
    docs_url=None,
    redoc_url=None,
    openapi_url=None,
)

# ``static_folder`` and ``template_folder`` were Flask attributes; the
# tests still set them directly to swap in tmp_path web roots, so we
# expose them as plain attributes on the FastAPI app instance.
app.static_folder = str(WEB_DIR)
app.template_folder = str(WEB_DIR)

telemetry_queue: TelemetryQueue = TelemetryQueue(maxlen=4096)
live_source: Optional[MAVLinkLiveSource] = None

# Waypoints for the current simulation (set by physics_live_replay).
# Dict mapping drone_id → list of [x, y, z] in ENU metres.
# Legacy single-drone callers may set a plain list — the endpoint
# normalises it to ``{1: waypoints}``.
mission_waypoints: dict = {}

# Background replay thread (for /api/load file replay).
_replay_bridge: Optional[object] = None
_replay_thread: Optional[threading.Thread] = None

# Background mission process (for /api/launch).
_launch_proc: Optional["subprocess.Popen"] = None


def _stop_replay() -> None:
    """Stop any running file replay (bridge + thread)."""
    global _replay_bridge, _replay_thread
    if _replay_bridge is not None:
        try:
            _replay_bridge.stop()
        except Exception:
            pass
        _replay_bridge = None
    if _replay_thread is not None and _replay_thread.is_alive():
        _replay_thread.join(timeout=2.0)
        _replay_thread = None


def _kill_proc(proc: "subprocess.Popen") -> None:
    """Terminate a subprocess, escalating to kill after 5 s."""
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


# ── HTML page helpers ───────────────────────────────────────────────────


def _read_template(name: str) -> str:
    """Read an HTML file by name.

    Tries ``app.template_folder`` first (so unit tests that monkey-patch
    it to a tmp_path keep working), then falls back to the canonical
    ``WEB_DIR`` shipped with the package. This makes the routes resilient
    to test pollution that forgets to restore ``template_folder`` between
    cases.
    """
    candidates = []
    template_folder = getattr(app, "template_folder", None)
    if template_folder:
        candidates.append(Path(template_folder) / name)
    if Path(template_folder or "") != WEB_DIR:
        candidates.append(WEB_DIR / name)
    for path in candidates:
        if path.exists():
            return path.read_text(encoding="utf-8")
    raise HTTPException(status_code=404, detail=f"{name} not found")


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    """Render the mission launcher."""
    return HTMLResponse(_read_template("index.html"))


@app.get("/live", response_class=HTMLResponse)
def live_view() -> HTMLResponse:
    """Render the live 3D view."""
    return HTMLResponse(_read_template("live.html"))


# ── Static asset route (parity with the Flask /web/<path> path) ─────────


@app.get("/web/{filename:path}")
def web_assets(filename: str):
    """Serve CSS/JS/img/vendor assets from the ``web`` directory."""
    folder = Path(getattr(app, "static_folder", None) or WEB_DIR)
    target = (folder / filename).resolve()
    # Reject path traversal: the resolved path must stay inside ``folder``.
    try:
        target.relative_to(folder.resolve())
    except ValueError:
        raise HTTPException(status_code=404)
    if not target.is_file():
        raise HTTPException(status_code=404)
    return FileResponse(str(target))


# ── REST API ────────────────────────────────────────────────────────────


def _load_missions() -> list:
    if MISSIONS_PATH.exists():
        try:
            return json.loads(MISSIONS_PATH.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return []
    return []


@app.get("/api/missions")
def api_missions() -> JSONResponse:
    """Return the mission catalogue as JSON."""
    return JSONResponse(_load_missions())


@app.get("/api/status")
def api_status() -> JSONResponse:
    """Return the current telemetry/connection status."""
    latest = telemetry_queue.latest()
    running = bool(live_source is not None and getattr(live_source, "_running", False))
    return JSONResponse({
        "connected": running,
        "sample_count": len(telemetry_queue),
        "latest_sample": latest.to_dict() if latest is not None else None,
    })


@app.get("/api/snapshot")
def api_snapshot(n: int = Query(default=100)) -> JSONResponse:
    """Return the last ``n`` telemetry samples (default 100, max 1000)."""
    n = max(1, min(1000, n))
    samples = telemetry_queue.snapshot(n=n)
    return JSONResponse([s.to_dict() for s in samples])


@app.get("/api/waypoints")
def api_waypoints() -> JSONResponse:
    """Return the active mission waypoints (ENU metres).

    Returns a dict ``{drone_id: [[x,y,z], ...], ...}``.
    Legacy single-drone lists are normalised to ``{"1": [...]}``.
    """
    wps = mission_waypoints
    # Normalise legacy plain-list format.
    if isinstance(wps, list):
        wps = {"1": wps} if wps else {}
    return JSONResponse(wps)


@app.get("/api/files")
def api_files() -> JSONResponse:
    """List .npz and .BIN flight data files available for replay."""
    files: List[dict] = []
    sim_dir = _SIM_DIR
    for p in sorted(sim_dir.glob("*.npz")):
        files.append({
            "name": p.name,
            "path": str(p),
            "size": p.stat().st_size,
            "type": "npz",
        })
    log_dir = sim_dir.parent / "logs"
    if log_dir.is_dir():
        for p in sorted(log_dir.rglob("*.BIN")):
            files.append({
                "name": str(p.relative_to(sim_dir.parent)),
                "path": str(p),
                "size": p.stat().st_size,
                "type": "bin",
            })
    return JSONResponse(files)


@app.post("/api/load")
def api_load(path: str = Query(...)) -> JSONResponse:
    """Load an .npz file and start replaying it to the live viewer.

    The replay runs in a background thread; any previous replay is
    stopped first.
    """
    global _replay_bridge, _replay_thread, mission_waypoints
    import numpy as np
    from mavlink_bridge import MAVLinkBridge

    file_path = Path(path)
    if not file_path.is_file():
        raise HTTPException(status_code=404, detail=f"File not found: {path}")
    if file_path.suffix.lower() != '.npz':
        raise HTTPException(status_code=400, detail="Only .npz files supported")

    # Stop any existing replay.
    _stop_replay()
    telemetry_queue.clear()

    try:
        from physics_live_replay import load_npz_records, load_swarm_npz_records
        # Load once and pass the data through to avoid double I/O.
        try:
            data = np.load(str(file_path), allow_pickle=False)
        except ValueError:
            data = np.load(str(file_path), allow_pickle=True)

        if "positions" in data:
            records = load_swarm_npz_records(str(file_path), drone_index=0, data=data)
            file_type = "swarm_npz"
        elif "pos" in data:
            records = load_npz_records(str(file_path), data=data)
            file_type = "npz"
            if "waypoints" in data:
                mission_waypoints = {
                    "1": [w.tolist() for w in data["waypoints"]]
                }
        else:
            raise HTTPException(
                status_code=400,
                detail="Unsupported file format (no 'pos' or 'positions' key)",
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    if not records:
        raise HTTPException(status_code=400, detail="No records in file")

    # Ensure the telemetry receiver is running.
    if live_source is None:
        start_telemetry(listen_port=14550)

    listen_port = (live_source._sock.getsockname()[1]
                   if live_source and live_source._sock else 14550)
    bridge = MAVLinkBridge(
        target_ip="127.0.0.1",
        target_port=listen_port,
        listen_port=0,
    )
    bridge.start()
    _replay_bridge = bridge

    def _replay():
        bridge.run_replay(records, fps=50.0, loop=True)

    t = threading.Thread(target=_replay, daemon=True)
    t.start()
    _replay_thread = t

    return JSONResponse({
        "status": "playing",
        "file": str(file_path),
        "type": file_type,
        "records": len(records),
    })


# ── Launch endpoint ─────────────────────────────────────────────────────


def _log_launch(action: str, command: str, detail: str = "") -> None:
    """Append a timestamped entry to the launch audit log."""
    log_dir = _SIM_DIR.parent / ".ai"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "launch.log"
    ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
    entry = f"[{ts}] {action}: {command}"
    if detail:
        entry += f" — {detail}"
    with open(log_file, "a", encoding="utf-8") as f:
        f.write(entry + "\n")


@app.post("/api/launch")
def api_launch(id: str = Query(...)) -> JSONResponse:
    """Execute a mission's start_command by mission id.

    Security model:
    - Only commands listed in missions.json are allowed (allowlist).
    - The command runs as a subprocess of the server process.
    - Each launch is recorded in .ai/launch.log.
    - Any previously running launch is stopped first.
    """
    global _launch_proc

    missions = _load_missions()
    mission = next((m for m in missions if m.get("id") == id), None)
    if mission is None:
        raise HTTPException(status_code=404, detail=f"Mission '{id}' not found")
    if mission.get("disabled"):
        raise HTTPException(status_code=403, detail=f"Mission '{id}' is disabled")

    cmd = mission.get("start_command", "")
    if not cmd:
        raise HTTPException(status_code=400, detail="Mission has no start_command")

    # Stop any previously running mission.
    if _launch_proc is not None and _launch_proc.poll() is None:
        _log_launch("STOP", "(previous)", f"pid={_launch_proc.pid}")
        _kill_proc(_launch_proc)
        _launch_proc = None

    # Execute from the project root.
    project_root = _SIM_DIR.parent
    _log_launch("START", cmd, f"mission={id}")
    try:
        _launch_proc = subprocess.Popen(
            cmd,
            shell=True,
            cwd=str(project_root),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        _log_launch("ERROR", cmd, str(e))
        raise HTTPException(status_code=500, detail=str(e))

    return JSONResponse({
        "status": "launched",
        "mission_id": id,
        "command": cmd,
        "pid": _launch_proc.pid,
    })


@app.post("/api/launch/stop")
def api_launch_stop() -> JSONResponse:
    """Stop the currently running launched mission."""
    global _launch_proc
    if _launch_proc is None or _launch_proc.poll() is not None:
        return JSONResponse({"status": "not_running"})
    pid = _launch_proc.pid
    _log_launch("STOP", "(user request)", f"pid={pid}")
    _kill_proc(_launch_proc)
    _launch_proc = None
    return JSONResponse({"status": "stopped", "pid": pid})


@app.get("/api/launch/status")
def api_launch_status() -> JSONResponse:
    """Check if a launched mission is still running."""
    if _launch_proc is None:
        return JSONResponse({"running": False})
    rc = _launch_proc.poll()
    if rc is None:
        return JSONResponse({"running": True, "pid": _launch_proc.pid})
    return JSONResponse({"running": False, "exit_code": rc})


# ── WebSocket route ─────────────────────────────────────────────────────


@app.websocket("/ws/telemetry")
async def telemetry_stream(ws: WebSocket) -> None:
    """Push every queued :class:`LiveTelemetrySample` at ~50 Hz.

    Forwards *every* sample with ``t_wall > last_t`` rather than only the
    most recent one. The previous ``latest()``-only loop coalesced bursts
    into a single frame, so when several GPS updates arrived between two
    20 ms WS ticks the browser saw the drone teleport instead of move —
    matching the "connected but no motion" regression. Async + asyncio
    sleeps replace the Flask thread loop but the wire format is identical.
    """
    await ws.accept()
    last_t: float = -1.0
    last_heartbeat = time.time()
    try:
        # Kick off with a snapshot so the client paints immediately.
        initial = telemetry_queue.snapshot(n=200)
        if initial:
            await ws.send_text(json.dumps({
                "type": "snapshot",
                "data": [s.to_dict() for s in initial],
            }))
            # Treat the snapshot as already-acknowledged so the catch-up
            # loop below doesn't immediately re-send the same samples.
            last_t = max(s.t_wall for s in initial)
        while True:
            # Drain only new samples (avoids copying the full 4096-entry buffer).
            for sample in telemetry_queue.since(last_t):
                await ws.send_text(json.dumps({
                    "type": "sample",
                    "data": sample.to_dict(),
                }))
                last_t = sample.t_wall
            now = time.time()
            if now - last_heartbeat >= 5.0:
                await ws.send_text(json.dumps({"type": "ping", "t": now}))
                last_heartbeat = now
            await asyncio.sleep(0.02)  # 50 Hz cap
    except WebSocketDisconnect:
        return
    except Exception:
        # Mirror the Flask handler: any other failure is treated as a
        # normal close so a hung browser tab doesn't crash the server.
        try:
            await ws.close()
        except Exception:
            pass
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
    """Run uvicorn against the FastAPI app (blocking)."""
    if start_source:
        start_telemetry(listen_port=listen_port)
    config = uvicorn.Config(
        app, host=host, port=port,
        log_level="info", access_log=False, lifespan="off",
    )
    server = uvicorn.Server(config)
    try:
        server.run()
    finally:
        stop_telemetry()


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="runtime_view.server",
        description="FastAPI + Three.js Run-time View for the Swarm Digital Twin.",
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
