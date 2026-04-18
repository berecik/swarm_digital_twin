# Run-time View

A FastAPI + Three.js web app that turns the digital twin's MAVLink stream
into a live 3D view, mirroring the look-and-feel of [pysimverse.com][1]'s
mission launcher.

## Quick start

```bash
# 1. Default mode boots ONLY the live view server (no SITL stack):
./run_scenario.sh                 # opens http://127.0.0.1:8765/live
./run_scenario.sh --viz-live      # same thing, explicit alias

# 2. Or run the server directly (no MAVLink listener for tests/dev):
python -m runtime_view.server --no-source --port 8765

# 3. Or pair the live view with the SITL stack:
./run_scenario.sh --single-live
```

The server is FastAPI on uvicorn; the WebSocket route is Starlette's
native ``WebSocket`` API. There is no Flask in the runtime view.

## Files

| File | Purpose |
| :--- | :--- |
| `server.py` | FastAPI app, REST API, `/ws/telemetry` WebSocket pump |
| `missions.json` | Mission catalogue rendered into the launcher card grid |
| `web/index.html` | Launcher landing page |
| `web/live.html` | Live 3D viewport + HUD |
| `web/styles.css` | Dark-navy theme tokens (single file, no Tailwind) |
| `web/app.js` | Launcher logic + status polling |
| `web/live.js` | Three.js scene + WebSocket consumer |
| `web/vendor/three.module.js` | Vendored Three.js r160 (MIT) |
| `web/vendor/OrbitControls.js` | Vendored OrbitControls (MIT) |
| `web/img/*.png` | Mission card thumbnails (placeholder Pillow renders) |

## API surface

| Route | Method | Description |
| :--- | :---: | :--- |
| `/` | GET | Launcher HTML |
| `/live` | GET | Live 3D view HTML |
| `/web/<path>` | GET | Static assets (CSS/JS/img/vendor) |
| `/api/missions` | GET | Mission catalogue (JSON list) |
| `/api/status` | GET | `{connected, sample_count, latest_sample}` |
| `/api/snapshot?n=N` | GET | Last `N` samples (max 1000) |
| `/ws/telemetry` | WS | 50 Hz `LiveTelemetrySample` push |

## Architecture

```
MAVLinkBridge ──UDP──▶ MAVLinkLiveSource ──▶ TelemetryQueue ──▶ FastAPI
   (sender)            (live_telemetry.py)        (ring buf)        │
                                                                    ▼
                                                          /ws/telemetry
                                                                    │
                                                                    ▼
                                                              Three.js HUD
```

The receiver thread, queue, and CSV recorder all live in
`simulation/live_telemetry.py` so they can be reused by other UIs (or
the matplotlib post-flight viewer if we ever bridge them).

[1]: https://pysimverse.com/
