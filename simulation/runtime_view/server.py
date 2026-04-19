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

import os  # noqa: E402
import secrets  # noqa: E402
from fastapi import Body, FastAPI, Header, HTTPException, Query, Request, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse  # noqa: E402
from starlette.middleware.base import BaseHTTPMiddleware  # noqa: E402
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

# Active terrain mesh (Phase 7-1). Either None (no terrain published —
# the live viewer falls back to its flat grid) or a dict::
#
#   {"vertices": [[x, y, z], ...],   # ENU metres, Three.js handed Y=up
#    "faces":    [[i0, i1, i2], ...],
#    "bounds":   [x_min, y_min, x_max, y_max]}
#
# Set in-process by physics_live_replay or via POST /api/terrain.
mission_terrain: Optional[dict] = None

# Background replay thread (for /api/load file replay).
_replay_bridge: Optional[object] = None
_replay_thread: Optional[threading.Thread] = None

# Background mission process (for /api/launch).
_launch_proc: Optional["subprocess.Popen"] = None


# ── Authentication & CSRF (Phase 7-3) ────────────────────────────────────────
#
# Off by default — the live view binds to 127.0.0.1 and read-only endpoints
# stay open so the existing local workflow keeps working. Enable by setting
# `RUNTIME_VIEW_AUTH_TOKEN` (or via `--auth-token` on the CLI). When set:
#
#   * Mutating endpoints (POST/PUT/DELETE) require both
#       Authorization: Bearer <token>
#     AND
#       X-CSRF-Token: <token>
#   * GET /api/csrf returns a per-process token clients fetch on load.
#
# The CSRF token equals the auth token in this minimal model — appropriate
# for single-operator setups where the goal is "don't accept random
# unauthenticated POSTs from a co-located browser tab", not full multi-user
# session management. Multi-user is its own follow-up.

_AUTH_TOKEN: Optional[str] = os.environ.get("RUNTIME_VIEW_AUTH_TOKEN") or None

# Multi-user session isolation (Phase 7 close-out): each browser tab
# can mint its own session via `POST /api/session`, then send the
# returned token on `Authorization: Bearer <session_token>` for every
# mutating call. The middleware accepts any of:
#   * the global API key (back-compat),
#   * any active session token in `_SESSIONS` whose `expires_at_s` is in
#     the future.
# Sessions auto-expire after `SESSION_TTL_S` of inactivity. Designed for
# single-operator setups with one or two browser tabs — full RBAC is
# still its own follow-up.
SESSION_TTL_S: float = 3600.0
_SESSIONS: dict = {}  # token -> {"created_s": float, "expires_at_s": float}


def _purge_expired_sessions(now_s: Optional[float] = None) -> None:
    if now_s is None:
        now_s = time.time()
    expired = [t for t, s in _SESSIONS.items() if s["expires_at_s"] <= now_s]
    for t in expired:
        _SESSIONS.pop(t, None)


def create_session(ttl_s: Optional[float] = None) -> str:
    """Mint a new per-browser session token. Returns the token."""
    token = secrets.token_urlsafe(24)
    now = time.time()
    _SESSIONS[token] = {
        "created_s": now,
        "expires_at_s": now + (ttl_s if ttl_s is not None else SESSION_TTL_S),
    }
    return token


def revoke_session(token: str) -> bool:
    """Drop an active session; returns True if it was present."""
    return _SESSIONS.pop(token, None) is not None


def active_sessions() -> int:
    _purge_expired_sessions()
    return len(_SESSIONS)


def reset_sessions() -> None:
    """Clear all sessions (used by tests)."""
    _SESSIONS.clear()

# Endpoints that don't require auth even when it's enabled. The Read-only
# REST surface and the WebSocket stream stay open so the static page can
# load and keep showing telemetry. Add new mutating endpoints to
# `_MUTATING_PATHS` below.
_PUBLIC_PATHS = {
    "/", "/live",
    "/api/missions", "/api/status", "/api/snapshot",
    "/api/waypoints", "/api/files", "/api/launch/status",
    "/api/terrain",
    "/api/csrf",
}
_MUTATING_PATHS = {
    "/api/load",
    "/api/launch", "/api/launch/stop",
    "/api/waypoints",  # POST (the GET path is in _PUBLIC_PATHS)
    "/api/terrain",    # POST
    "/api/session",    # DELETE (POST is gated separately by the middleware)
}


def set_auth_token(token: Optional[str]) -> None:
    """Configure the auth/CSRF token at runtime (also used by tests)."""
    global _AUTH_TOKEN
    _AUTH_TOKEN = token or None


def auth_token() -> Optional[str]:
    return _AUTH_TOKEN


def _is_valid_token(supplied: str) -> bool:
    """Accept the global API key OR any active session token."""
    if _AUTH_TOKEN is not None and secrets.compare_digest(supplied, _AUTH_TOKEN):
        return True
    _purge_expired_sessions()
    return supplied in _SESSIONS


class AuthMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        token = _AUTH_TOKEN
        if token is None:
            return await call_next(request)

        # Static asset path is always open.
        if request.url.path.startswith("/web/"):
            return await call_next(request)

        method = request.method.upper()
        path = request.url.path

        if method in {"GET", "HEAD", "OPTIONS"}:
            return await call_next(request)

        # /api/session is the only POST that the global token alone may
        # call (it's how a browser mints a per-session token in the first
        # place).
        if path in _MUTATING_PATHS or path == "/api/session":
            auth_header = request.headers.get("authorization", "")
            csrf_header = request.headers.get("x-csrf-token", "")
            if not auth_header.lower().startswith("bearer "):
                return JSONResponse(
                    {"detail": "missing Bearer authorization header"},
                    status_code=401,
                )
            supplied = auth_header.split(" ", 1)[1].strip()
            if not _is_valid_token(supplied):
                return JSONResponse(
                    {"detail": "invalid auth token"}, status_code=401)
            # CSRF must match the same token the caller authenticated with.
            if not secrets.compare_digest(csrf_header, supplied):
                return JSONResponse(
                    {"detail": "missing or invalid CSRF token"},
                    status_code=403,
                )

        return await call_next(request)


# Auth middleware is always registered; it short-circuits when no token
# is configured so the local default workflow is unchanged.
app.add_middleware(AuthMiddleware)


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


@app.post("/api/session")
def api_session_create() -> JSONResponse:
    """Mint a per-browser session token (Phase 7 multi-user isolation).

    The caller must be authenticated with the global API key. The
    returned ``token`` carries CSRF authority for ``SESSION_TTL_S``
    seconds; subsequent mutating calls send it on
    ``Authorization: Bearer <session_token>`` and the matching
    ``X-CSRF-Token`` header.

    When auth is disabled (no global token configured) this returns 404
    — sessions are an opt-in addition to the auth flow, not a separate
    surface.
    """
    if _AUTH_TOKEN is None:
        raise HTTPException(
            status_code=404,
            detail="sessions are only available when --auth-token is set",
        )
    token = create_session()
    return JSONResponse({
        "token": token,
        "ttl_s": SESSION_TTL_S,
    })


@app.delete("/api/session")
def api_session_revoke(payload: dict = Body(...)) -> JSONResponse:
    """Revoke a session token early."""
    token = payload.get("token") if isinstance(payload, dict) else None
    if not token:
        raise HTTPException(status_code=400, detail="missing 'token'")
    revoked = revoke_session(token)
    return JSONResponse({"revoked": revoked})


@app.get("/api/csrf")
def api_csrf() -> JSONResponse:
    """Return the CSRF token clients must send on mutating endpoints.

    When auth is disabled returns ``{"token": null, "auth_required": false}``
    so callers can branch off a single GET. When enabled the response
    contains the active token; the caller must echo it on the
    `X-CSRF-Token` header alongside `Authorization: Bearer <token>`.
    """
    token = _AUTH_TOKEN
    return JSONResponse({
        "token": token,
        "auth_required": token is not None,
    })


@app.get("/api/terrain")
def api_terrain() -> JSONResponse:
    """Return the active terrain mesh, or 204 No Content when none is set.

    Phase 7-1. Live viewer fetches this on load to replace the flat grid
    with the simulation's actual terrain mesh.
    """
    if not mission_terrain:
        return JSONResponse({}, status_code=204)
    return JSONResponse(mission_terrain)


@app.post("/api/terrain")
def api_terrain_set(payload: dict = Body(...)) -> JSONResponse:
    """Replace the active terrain mesh.

    Accepts either:
      * ``{"vertices": [...], "faces": [...], "bounds": [...]}`` directly,
      * ``{"terrain_name": "<name>", "manifest_path": "<optional>"}``
        which loads via ``terrain.load_from_manifest`` and converts to
        the wire format here.

    Setting an empty payload clears the terrain (the live viewer
    re-shows the flat grid).
    """
    global mission_terrain
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="payload must be a dict")
    if not payload:
        mission_terrain = None
        return JSONResponse({"status": "cleared"})

    if "terrain_name" in payload:
        from terrain import load_from_manifest
        try:
            terrain = load_from_manifest(
                payload["terrain_name"],
                manifest_path=payload.get("manifest_path"),
            )
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc))
        mission_terrain = _terrain_to_mesh(terrain)
        return JSONResponse({
            "status": "ok",
            "vertex_count": len(mission_terrain["vertices"]),
            "face_count": len(mission_terrain["faces"]),
        })

    for required in ("vertices", "faces", "bounds"):
        if required not in payload:
            raise HTTPException(
                status_code=400,
                detail=f"missing required field '{required}'",
            )
    mission_terrain = {
        "vertices": [list(map(float, v))[:3] for v in payload["vertices"]],
        "faces": [list(map(int, f))[:3] for f in payload["faces"]],
        "bounds": list(map(float, payload["bounds"]))[:4],
    }
    return JSONResponse({
        "status": "ok",
        "vertex_count": len(mission_terrain["vertices"]),
        "face_count": len(mission_terrain["faces"]),
    })


def _terrain_to_mesh(terrain) -> dict:
    """Convert a :class:`terrain.TerrainMap` into the live-viewer wire format.

    Vertices are emitted as Three.js-friendly `[x, y, z]` triples where
    `x = east`, `y = up` (the live viewer uses Y-up, ENU east as X), and
    `z = -north`. Faces are zero-indexed triangles.
    """
    import numpy as np
    elev = terrain.elevations
    ny, nx = elev.shape
    res = terrain.resolution
    ox, oy = terrain.origin
    vertices = []
    for j in range(ny):
        for i in range(nx):
            ex = ox + i * res
            ny_pos = oy + j * res
            uz = float(elev[j, i])
            # ENU → Three.js (Y-up, -Z = north).
            vertices.append([float(ex), uz, float(-ny_pos)])
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            b = a + 1
            c = a + nx
            d = c + 1
            faces.append([a, b, d])
            faces.append([a, d, c])
    x_min, y_min, x_max, y_max = terrain.bounds
    return {
        "vertices": vertices,
        "faces": faces,
        "bounds": [float(x_min), float(y_min), float(x_max), float(y_max)],
    }


@app.post("/api/waypoints")
def api_waypoints_set(payload: dict = Body(...)) -> JSONResponse:
    """Replace the active waypoint set.

    Accepts ``{"waypoints": {drone_id: [[e,n,u], ...], ...}}`` or the bare
    ``{drone_id: [[e,n,u], ...]}`` form. Used by SITL launchers to publish
    mission waypoints into the live view (parity with what
    physics_live_replay.run_physics_live does in-process).
    """
    global mission_waypoints
    wps = payload.get("waypoints", payload)
    if not isinstance(wps, dict):
        raise HTTPException(status_code=400, detail="waypoints must be a dict")
    norm: dict = {}
    for k, v in wps.items():
        if not isinstance(v, list):
            raise HTTPException(status_code=400, detail=f"drone {k}: not a list")
        coords = []
        for p in v:
            if not isinstance(p, (list, tuple)) or len(p) < 3:
                raise HTTPException(
                    status_code=400,
                    detail=f"drone {k}: each waypoint must be [e, n, u]",
                )
            try:
                coords.append([float(p[0]), float(p[1]), float(p[2])])
            except (TypeError, ValueError):
                raise HTTPException(
                    status_code=400,
                    detail=f"drone {k}: non-numeric waypoint",
                )
        norm[str(k)] = coords
    mission_waypoints = norm
    return JSONResponse({"status": "ok", "drones": list(norm.keys())})


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
    suffix = file_path.suffix.lower()
    if suffix not in {".npz", ".bin"}:
        raise HTTPException(
            status_code=400,
            detail="Only .npz and .bin files supported",
        )

    # Stop any existing replay.
    _stop_replay()
    telemetry_queue.clear()

    if suffix == ".bin":
        from physics_live_replay import load_bin_records
        try:
            records = load_bin_records(str(file_path))
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))
        if not records:
            raise HTTPException(
                status_code=400,
                detail=".BIN file has no GPS samples with a 3D fix",
            )
        file_type = "bin"
        return _start_records_replay(records, file_path, file_type)

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

    return _start_records_replay(records, file_path, file_type)


def _start_records_replay(records, file_path, file_type) -> JSONResponse:
    """Spin up the MAVLinkBridge replay thread for a record list."""
    global _replay_bridge, _replay_thread
    from mavlink_bridge import MAVLinkBridge

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


def start_telemetry(listen_port: int = 14550, listen_ip: str = "0.0.0.0",
                    ref_lat: float = 47.3769, ref_lon: float = 8.5417,
                    ref_alt_msl: float = 408.0) -> None:
    """Open the MAVLink UDP socket and start the receiver thread.

    *ref_lat/ref_lon/ref_alt_msl* must match the GPS origin used by
    the telemetry sender (SITL or MAVLinkBridge) so that GPS→ENU
    conversion produces correct local coordinates.
    """
    global live_source
    if live_source is not None:
        return
    live_source = MAVLinkLiveSource(
        listen_ip=listen_ip,
        listen_port=listen_port,
        queue=telemetry_queue,
        ref_lat=ref_lat,
        ref_lon=ref_lon,
        ref_alt_msl=ref_alt_msl,
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
    ref_lat: float = 47.3769,
    ref_lon: float = 8.5417,
    ref_alt_msl: float = 408.0,
) -> None:
    """Run uvicorn against the FastAPI app (blocking)."""
    if start_source:
        start_telemetry(listen_port=listen_port,
                        ref_lat=ref_lat, ref_lon=ref_lon,
                        ref_alt_msl=ref_alt_msl)
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
        help="Skip binding the MAVLink UDP listener.",
    )
    parser.add_argument("--ref-lat", type=float, default=47.3769,
                        help="GPS reference latitude (must match SITL origin)")
    parser.add_argument("--ref-lon", type=float, default=8.5417,
                        help="GPS reference longitude (must match SITL origin)")
    parser.add_argument("--ref-alt", type=float, default=408.0,
                        help="GPS reference altitude MSL in metres")
    args = parser.parse_args(argv)
    run_server(
        host=args.host,
        port=args.port,
        listen_port=args.listen_port,
        start_source=not args.no_source,
        ref_lat=args.ref_lat,
        ref_lon=args.ref_lon,
        ref_alt_msl=args.ref_alt,
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
