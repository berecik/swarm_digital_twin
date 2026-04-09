# Refactoring Plan (v14): Paper-Aligned & Runtime View

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps and upcoming features. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

**Current state:** All paper-aligned work items (Equations 1–7, Tables 1–5) are fully implemented, integrated, and verified against real flight data. The framework supports quadrotor/fixed-wing physics, SRTM/STL terrain with satellite textures, battery/energy models, wind auto-tuning, and sensor noise. 255 tests passing.

---

## 1) Paper cross-reference — implemented items

All paper equations and tables have been verified against the codebase:

| Paper item | Implementation | Verified by |
|:---|:---|:---|
| Eq. 1 — position kinematics (rotation matrix) | `drone_physics.py` `physics_step()` | `TestRotationMatrix` |
| Eq. 2 — Euler angle rates from body rates | `drone_physics.py` `euler_rates_from_body_rates()` | `TestEulerRates` (6 tests) |
| Eq. 3 — Newton's 2nd law, body frame + Coriolis (ω×v) | `drone_physics.py` line 653 | `TestCoriolisForce` |
| Eq. 4 — rotational dynamics, full 3×3 inertia tensor | `drone_physics.py` `np.linalg.inv(I)`, off-diagonal J_xz | `TestGammaTermEquivalence` |
| Eq. 5–7 — wind drag/lift/combined perturbation | `wind_model.py` `get_force()` | `TestWindForce` |
| Table 2 — fixed-wing geometry (2.20m, 0.235m, 0.3997m², 2.5kg) | `drone_physics.py` `make_valencia_fixed_wing()` | `TestValenciaPreset` |
| Table 3 — CFD aero coefficients (α₀, C_Lα, C_Dα, C_Mα, α_stall, all stall coeffs) | `drone_physics.py` `FixedWingAero` | `TestValenciaPreset::test_valencia_fixed_wing_aero_coefficients` |
| Table 4 — 7 mission profiles (FW 158/178/185, Quad Carolina/EPN) | `validation.py` `REAL_LOG_MISSIONS` | `TestPaperValidation` |
| Table 5 — RMSE validation metrics | `validation.py` acceptance gate (≤6× for PID, Z-axis ≤2×) | `TestPaperValidation`, `TestTrajectoryTracking` |
| Section 2.1 — SRTM terrain → STL export | `terrain.py` `from_srtm()` + `export_stl()` | `TestTerrainSTLExport` (6 tests) |
| Section 2.1 — Satellite Texture Overlay | `terrain.py` `export_obj_with_uv()` | `TestTerrainSatelliteTexture` |
| Section 2.3 — wind from real flight log | `flight_log.py` `get_wind_profile()`, `wind_model.py` | `TestPositionAwareWind` |
| Section 3.1 — Wind Disturbance Auto-Tuning | `validation.py` `auto_tune_wind_force_scale()` | `TestWindAutoTuning` |
| Section 3.2 — IRS-4 quadrotor (ArduPilot defaults) | `drone_physics.py` `make_irs4_quadrotor()` | `test_irs4_hover_stable` |
| Section 3.5 — Quadrotor Aerodynamic Area | `drone_physics.py` `QuadrotorAero` | `TestQuadrotorAeroArea` |
| Section 3.5 — Battery and Energy Model | `drone_physics.py` `BatteryModel` | `TestBatteryModel` |
| Motor dynamics (T = k_T·ω² + k_D·ω) | `drone_physics.py` `MotorModel` | `TestMotorDynamics` (3 tests) |
| Sensor noise (GPS/IMU/baro) | `sensor_models.py` | `TestSensorNoise` (6 tests) |

---

## 2) Remaining gaps & Upcoming features

### 2.1) Run-time View for Drone 🆕

**Reference UI:** [pysimverse.com](https://pysimverse.com/) — concretely the
`https://pysimverse.com/images/simulator-screen.png` mockup (Missions launcher)
plus the hero page's "LIVE SIMULATION ENVIRONMENT" panel. We are *not*
copying assets — we are matching the **shape**: a dark-navy launcher with a
mission grid, status chip, and a live 3D viewport that streams from the
running digital twin.

**Goal:** Ship a new local web app, served by the project's Python venv, that
gives the operator a pysimverse-style **launcher → live 3D view** experience
backed by `mavlink_bridge.py` telemetry. Today the only viewer is a
post-flight matplotlib replayer (`simulation/visualize_drone_3d.py`); a
matplotlib `FuncAnimation` cannot reproduce the gamified launcher in the
reference image, so this work item replaces the visualizer track.

**Architecture choice (decision log — do not relitigate):**

| Option | Why considered | Rejected because |
|:---|:---|:---|
| Matplotlib `FuncAnimation` (v14 draft) | minimal new deps, reuses existing artists | cannot deliver launcher landing page, mission cards, modern HUD chrome |
| PySide6 / QtQuick3D desktop app | native look, single binary | adds Qt as a hard dep, cross-platform packaging is heavy, hard to test in CI |
| Unity / Unreal client (the pysimverse approach) | exact visual parity | engine binary is out of scope for an open-source Python repo, breaks our "no proprietary toolchains" rule |
| **Flask + flask-sock + Three.js (vanilla)** ✅ | runs from `python -m`, no build step, single new pip dep, WebGL renders the 3D viewport, easy to test with `pytest` + `Flask.test_client()` | none — picked |

**Stack we are committing to:**

- **Backend:** `flask>=3.0` and `flask-sock>=0.7` (one new line in
  `requirements.txt`). The server lives at `simulation/runtime_view/server.py`.
- **Telemetry source:** the same `simulation/live_telemetry.py` module from
  the original v14 draft (kept verbatim — see §2.1.A). The Flask server
  instantiates a `MAVLinkLiveSource` and streams `LiveTelemetrySample` dicts
  to the browser over a WebSocket.
- **Frontend:** **vanilla** static files under
  `simulation/runtime_view/web/` (no npm, no bundler, no React). Three.js is
  loaded from a vendored `three.module.js` in
  `simulation/runtime_view/web/vendor/` so the app works fully offline.
- **Mission catalogue:** a single
  `simulation/runtime_view/missions.json` file lists scenarios. The launcher
  reads it via `GET /api/missions`.

**Junie scope (one PR, in this order):**

1. Add `simulation/live_telemetry.py` (§2.1.A — unchanged from v14 draft).
2. Add `simulation/runtime_view/` package (server + static web app +
   missions catalogue) — §2.1.B (rewritten).
3. Add `MAVLinkBridge.run_replay()` helper in `mavlink_bridge.py` —
   §2.1.C (unchanged).
4. Wire `run_scenario.sh --viz-live` and `--single-live` to the new
   server entry point — §2.1.D (rewritten).
5. Add `TestLiveTelemetry` (6 tests) **and** `TestRuntimeViewServer`
   (4 tests) — §2.1.F (expanded).
6. Do **not** delete `simulation/visualize_drone_3d.py`. Keep it as the
   "post-flight replay" tool; the new web view becomes the live tool. This
   keeps `--single`, `--swarm`, `--sitl-viz` byte-identical and avoids
   collateral test failures.

#### 2.1.A) New module: `simulation/live_telemetry.py` 🆕

Create a brand-new file (Junie: `Write` tool, do not splice into
`mavlink_bridge.py`). It owns the receiver thread, the ring buffer, and the
optional CSV recorder. Keeping it standalone makes the visualizer change tiny
and gives tests a single import target.

- **Public API (importable as `from live_telemetry import …`):**
  - `@dataclass LiveTelemetrySample` with fields: `t_wall: float`,
    `time_boot_ms: int`, `pos_enu: np.ndarray (3,)`, `vel_enu: np.ndarray (3,)`,
    `euler: tuple[float, float, float]` (roll, pitch, yaw rad),
    `throttle_pct: float`, `airspeed: float`, `groundspeed: float`,
    `alt_msl: float`, `climb_rate: float`, `battery_voltage_v: float`,
    `battery_current_a: float`, `battery_remaining_pct: float`,
    `flight_mode: str`, `armed: bool`, `lat_deg: float`, `lon_deg: float`.
    All numeric fields default to `0.0`/`0`.
  - `class TelemetryQueue` — bounded ring buffer (`collections.deque(maxlen=…)`)
    guarded by a single `threading.Lock`. Methods:
    - `push(sample: LiveTelemetrySample) -> None`
    - `latest() -> Optional[LiveTelemetrySample]` (peek without consuming)
    - `snapshot(n: Optional[int] = None) -> list[LiveTelemetrySample]`
      (returns a copy of the most recent `n` samples for the trail)
    - `__len__()` and `clear()`
    - The buffer must be thread-safe under concurrent `push()` from the
      receiver thread and `snapshot()` from the Flask request thread (or
      the matplotlib main thread, if `live_telemetry.py` is later reused
      from the static visualizer).
  - `class MAVLinkLiveSource` — UDP receiver thread.
    - `__init__(self, listen_ip: str = "0.0.0.0", listen_port: int = 14550, queue: TelemetryQueue | None = None, ref_lat: float = 47.3769, ref_lon: float = 8.5417, ref_alt_msl: float = 408.0, max_samples: int = 4096, recorder: "TelemetryCSVRecorder | None" = None)`
    - `start()` / `stop()` mirror the existing `MAVLinkBridge` lifecycle (set
      `_running`, spawn daemon thread, `socket.settimeout(0.1)` so `stop()`
      can join cleanly within 2 s).
    - Internal state machine merges fragments from the four telemetry message
      IDs (`ATTITUDE`, `GLOBAL_POSITION_INT`, `VFR_HUD`, `SYS_STATUS`) plus
      `HEARTBEAT` for `armed`/`flight_mode`, into one `LiveTelemetrySample`
      per `time_boot_ms` tick. Whenever `GLOBAL_POSITION_INT` or
      `VFR_HUD` advances `time_boot_ms`, finalise the previous sample and
      `queue.push()` it.
    - GPS → ENU conversion is the inverse of `mavlink_bridge._enu_to_gps`.
      Add `_gps_to_enu(lat, lon, alt_msl, ref_lat, ref_lon, ref_alt_msl)` to
      `live_telemetry.py` (do not import the private helper from
      `mavlink_bridge`, duplicate it locally — 6 lines). Apply ENU axis
      mapping that matches `_enu_to_gps`: `x = (lon - ref_lon)*lon_m_per_deg`,
      `y = (lat - ref_lat)*lat_m_per_deg`, `z = alt_msl - ref_alt_msl`.
  - `class TelemetryCSVRecorder` — optional persistence sink.
    - `__init__(path: str)`; opens the file, writes a fixed header
      `"t_wall,time_boot_ms,x,y,z,vx,vy,vz,roll,pitch,yaw,throttle_pct,airspeed,alt_msl,batt_v,batt_a,batt_pct,mode,armed"`.
    - `record(sample)` appends one line; `close()` flushes and closes.
    - The receiver calls `recorder.record(sample)` immediately before
      `queue.push(sample)`.
- **Decoder reuse:** `MAVLinkLiveSource._receive_loop` calls
  `decode_mavlink_v2` from `mavlink_bridge` and a new
  `parse_telemetry_payload(msg_id, payload)` defined in
  `live_telemetry.py` that handles the five inbound message IDs. Do **not**
  extend `mavlink_bridge.parse_mavlink_payload`; it is intentionally
  restricted to GCS→sim commands and changing it would force re-running
  `TestMAVLink`. Add the new parser in the new module so the diff stays
  isolated.
- **Constants & flight-mode mapping:** mirror the names already in
  `mavlink_bridge.CUSTOM_MODE_*` (`STABILIZE=0`, `AUTO=3`, `GUIDED=4`,
  `RTL=6`, `LAND=9`). Unknown modes → `"MODE_<n>"`.

#### 2.1.B) New package: `simulation/runtime_view/` 🆕

This is the bulk of the diff. It is **all new files** — Junie creates them
with `Write`. The existing `visualize_drone_3d.py` is **not** touched.

##### 2.1.B.1) Directory layout

```
simulation/runtime_view/
├── __init__.py
├── server.py               # Flask app + WebSocket telemetry pump
├── missions.json           # Mission catalogue consumed by the launcher
├── README.md               # one-paragraph "how to run"
└── web/
    ├── index.html          # launcher landing page (mission grid)
    ├── live.html           # live 3D viewport + HUD
    ├── styles.css          # dark-navy theme (single file, no Tailwind)
    ├── app.js              # launcher logic, fetches /api/missions, routes to live.html
    ├── live.js             # WebSocket consumer, Three.js scene, HUD updates
    └── vendor/
        └── three.module.js # vendored Three.js (r160+, ES module build)
```

Junie acquires `three.module.js` by `curl -sL https://unpkg.com/three@0.160.0/build/three.module.js -o simulation/runtime_view/web/vendor/three.module.js`
and commits the file (≈1.2 MB). Pin the exact version in
`simulation/runtime_view/web/vendor/THREE_VERSION.txt` so future bumps are
auditable. Do **not** add a CDN `<script>` — the runtime view must work
without internet so it survives offline SITL bring-up.

##### 2.1.B.2) `server.py` — backend contract

- **Dependencies:** `flask>=3.0`, `flask-sock>=0.7`. Add both to
  `requirements.txt` (one block, alphabetised).
- **App factory:**
  ```python
  def create_app(source: MAVLinkLiveSource | None = None,
                 queue: TelemetryQueue | None = None,
                 missions_path: Path | None = None) -> Flask: ...
  ```
  This signature lets pytest inject a fake source/queue without binding a
  UDP socket — critical for the test plan in §2.1.F.
- **Routes:**
  - `GET /` → `web/index.html` (launcher).
  - `GET /live` → `web/live.html` (live HUD).
  - `GET /static/<path>` → `web/<path>` via `send_from_directory`.
  - `GET /api/missions` → JSON list parsed from `missions.json`. Each
    mission has: `id`, `title`, `description`, `thumbnail` (relative path
    under `web/img/`), `tier` (`"free"` | `"pro"`), `start_command`
    (string of the `run_scenario.sh` flag, e.g. `"--single-live"`), and
    `disabled` (bool).
  - `GET /api/status` → `{"connected": bool, "samples": int, "last_t_boot": int}`
    pulled from the queue.
  - `GET /api/snapshot?n=400` → JSON list of the most recent `n`
    samples (uses `queue.snapshot(n)`), used by the launcher's tiny
    preview in the hero panel.
  - `WS /ws/telemetry` → 50 Hz push of the latest sample as JSON. The
    server reads with `queue.latest()` in a loop and `sock.send(json.dumps(...))`.
    Sends `{"type": "ping"}` every 5 s as a keepalive so the browser can
    detect dead links.
- **CLI entry point:** `python -m simulation.runtime_view.server [--port 8765] [--listen-port 14550] [--no-source]`.
  - `--no-source` skips creating a `MAVLinkLiveSource` (used by tests and
    by `--viz-live` when an external bridge is already running on the port).
  - The default behaviour binds the receiver and starts Flask on
    `127.0.0.1:8765`.

##### 2.1.B.3) `missions.json` — initial catalogue

Mirror the *shape* of pysimverse's mission cards (six entries, three free,
three "pro"-style placeholders pointing to swarm/SITL features). Use only
features the repo already supports — Junie does not invent new scenarios:

```json
[
  {"id": "single",        "title": "Single-Drone Mission",  "description": "Default Pixhawk SITL stack with 3D viz.",         "thumbnail": "img/single.png",   "tier": "free", "start_command": "--single-live",        "disabled": false},
  {"id": "physics",       "title": "Physics-Only Replay",   "description": "Run the standalone Python physics scenario.",     "thumbnail": "img/physics.png", "tier": "free", "start_command": "--physics-single",     "disabled": false},
  {"id": "real-log",      "title": "Real Flight Log Replay","description": "Validate against Andean mountain SRTM logs.",     "thumbnail": "img/real.png",    "tier": "free", "start_command": "--real-log",           "disabled": false},
  {"id": "swarm-3",       "title": "3-Drone Swarm",         "description": "Three-drone ring formation under crosswind.",     "thumbnail": "img/swarm3.png",  "tier": "pro",  "start_command": "--swarm 3",            "disabled": true},
  {"id": "swarm-6",       "title": "6-Drone Swarm",         "description": "Six-drone gusty crosswind scenario.",             "thumbnail": "img/swarm6.png",  "tier": "pro",  "start_command": "--swarm 6",            "disabled": true},
  {"id": "fixed-wing",    "title": "Valencia Fixed-Wing",   "description": "Fixed-wing flight over Antisana terrain.",        "thumbnail": "img/fw.png",      "tier": "pro",  "start_command": "--physics-single",     "disabled": true}
]
```

`disabled: true` ⇒ the launcher renders a yellow padlock + blue "GET PRO"
button (matching the reference image). This is purely cosmetic — every
"pro" item maps to an existing repo command and is unlocked by editing
`disabled` to `false` locally.

Thumbnails: ship a `web/img/` directory with **placeholder PNGs**
(640×360, dark navy with the title text rendered via PIL — Junie generates
these in the same step using `python -c` and Pillow, which is already in
`requirements.txt`). Real screenshots are out of scope.

##### 2.1.B.4) `web/styles.css` — dark-navy theme tokens

Pin these CSS custom properties at the top of `styles.css` so the look
matches the reference screenshot. Junie copies this block verbatim:

```css
:root {
  --bg-0: #070b1f;          /* page background */
  --bg-1: #0e1330;          /* card background */
  --bg-2: #161b3d;          /* card hover */
  --fg-0: #ffffff;
  --fg-1: #b3b8d4;          /* secondary text */
  --accent-green: #2ed47a;  /* START button */
  --accent-blue:  #3b82f6;  /* GET PRO button */
  --accent-pink:  #ec4899;  /* GET PRO pill */
  --accent-yellow:#facc15;  /* lock icon */
  --status-good:  #22c55e;
  --status-warn:  #f59e0b;
  --status-bad:   #ef4444;
  --radius:       12px;
  --shadow:       0 8px 24px rgba(0,0,0,0.45);
  --font: -apple-system, BlinkMacSystemFont, "Segoe UI", Inter, sans-serif;
  --mono: "JetBrains Mono", "SF Mono", Consolas, monospace;
}
body { background: var(--bg-0); color: var(--fg-0); font-family: var(--font); margin: 0; }
```

Layout rules to add (Junie writes the matching selectors):

- **Header bar** (`<header class="topbar">`): full-width, 64 px tall, logo
  on the left ("SWARM DIGITAL TWIN" wordmark — reuse the project name, do
  not copy the pysimverse logo), centred title `SELECT MISSION / Choose a
  scenario to run`, right-side `GET PRO` pill placeholder (linking to the
  GitHub repo), profile/menu/mute icons rendered as SVG glyphs from
  Heroicons (vendor the three SVG strings inline in `index.html`, no extra
  files).
- **Mission grid** (`<main class="missions">`): CSS Grid `repeat(auto-fill, minmax(320px, 1fr))`,
  20 px gap. Each card is a `<article class="mission-card">` with a 16:9
  thumbnail, `Free`/`Pro` pill (`.tier-pill`), title `<h3>`, one-line
  description, and a footer button (`.start-btn` green or `.lock-btn`
  blue). Hover state: `transform: translateY(-2px); background: var(--bg-2);`.
- **Live HUD** (`live.html`): full-bleed Three.js `<canvas id="viewport">`,
  with absolutely-positioned overlay panels:
  - Top-right: status chip (`STATUS · CONNECTED`/`DISCONNECTED`) styled
    exactly like the pysimverse hero (light grey pill, monospace caps).
  - Bottom-left: caption `LIVE SIMULATION ENVIRONMENT` with a small blue
    dot (`<span class="status-dot">`).
  - Bottom-right: telemetry stack (4 rows × 2 cols): `AGL`, `ALT MSL`,
    `SPEED`, `HEADING`, `THROTTLE`, `BATTERY V`, `BATTERY %`, `MODE`. Each
    row is `<dl class="hud-kv"><dt>label</dt><dd>value</dd></dl>` with
    monospace font.
  - Top-left: a back arrow → `/` (launcher).

##### 2.1.B.5) `web/app.js` — launcher logic

- Fetch `/api/missions`, render cards into `<main class="missions">`.
- Click on a free card → `POST /api/launch {id: ...}` (server returns the
  shell command string for now and **does not** actually execute it; the
  human-in-the-loop pastes it into a terminal). Display a modal showing
  the command and a "Copy" button. **Reason:** auto-running shell commands
  from a browser is a footgun and would require sandboxing we are not
  going to ship in this PR. The §2.1.H follow-ups list "execute from
  browser" as deferred work.
- Click on the launcher hero "Open Live View" button → navigate to
  `/live`.

##### 2.1.B.6) `web/live.js` — Three.js scene + WebSocket consumer

- **Scene setup:** `THREE.WebGLRenderer({antialias:true})`, `PerspectiveCamera`,
  `OrbitControls` (vendored from `three/examples/jsm/controls/OrbitControls.js`,
  same vendor folder). Add a hemisphere light, a ground `PlaneGeometry`
  (200 m × 200 m) with a dark grid texture, and a single `Mesh` for the
  drone (a low-poly quadrotor — use `BoxGeometry(0.4, 0.1, 0.4)` plus four
  `CylinderGeometry` arms; ~30 lines).
- **Trail:** `THREE.Line` with a `BufferGeometry` of capacity 1000 points,
  drained from the WebSocket samples in ENU; ring-buffer the index.
- **WebSocket loop:**
  ```js
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onmessage = (e) => applySample(JSON.parse(e.data));
  ws.onclose   = () => setStatusChip('DISCONNECTED');
  ```
- `applySample(s)` updates: drone mesh position+quaternion, trail buffer,
  HUD numeric fields, status chip text/colour. Quaternion built from the
  sample's `roll/pitch/yaw` via `THREE.Euler('ZYX')`.
- **Camera follow:** when `s.pos_enu` falls outside a 30 m sphere around
  `controls.target`, lerp `target` toward the new position over 200 ms.
- No animation loop in the WebSocket handler — use
  `requestAnimationFrame` for the renderer and treat the latest sample as
  state.

##### 2.1.B.7) Bridge to existing visualizer

- `simulation/visualize_drone_3d.py` is **untouched**. It remains the
  post-flight replay tool, invoked unchanged by `--viz-only`,
  `--sitl-viz`, `--single`, and `--swarm`.
- The launcher's "Open in matplotlib viewer" link in `index.html` simply
  shells out to that script via the same `POST /api/launch` "show me the
  command" pattern, so the two viewers coexist.

#### 2.1.C) Modify `simulation/mavlink_bridge.py`

The bridge is already complete; the only change needed is to make it loop
forever for live demos.

- Add a `MAVLinkBridge.run_replay(self, records, params=None, hz: float = 50.0, loop: bool = False)` helper that walks a list of `SimRecord`s,
  builds `SimState`s via `sim_state_from_record`, calls `send_state`, and
  sleeps `1/hz` between samples. When `loop=True`, restart from index 0.
  This makes the live demo deterministic without requiring a full SITL stack
  on the test runner.
- No edits to existing functions, no changes to message IDs, no changes to
  `_CRC_EXTRA` — those are pinned by `TestMAVLink` and must stay green.

#### 2.1.D) Modify `run_scenario.sh`

- Add a new helper next to `run_sitl_viz()`:
  ```bash
  run_live_viz() {
      local mav_port="${1:-14550}"
      local http_port="${2:-8765}"
      info "Starting Run-time View on http://127.0.0.1:${http_port}"
      info "Listening for MAVLink on UDP ${mav_port} (Ctrl-C to quit)…"
      ( sleep 1 && (open "http://127.0.0.1:${http_port}" 2>/dev/null \
                    || xdg-open "http://127.0.0.1:${http_port}" 2>/dev/null) ) &
      python -m simulation.runtime_view.server \
          --port "${http_port}" --listen-port "${mav_port}"
  }
  ```
  The `(sleep 1 && open …) &` block opens the default browser on macOS
  (`open`) or Linux (`xdg-open`); both fall through silently if neither
  exists. **Do not** add a Windows path — Junie should leave a TODO
  comment for `start` on Windows under the §2.1.H follow-ups.
- Add a new mode branch in the `case "$MODE"` block (after `--sitl-viz`):
  ```bash
  --viz-live)
      ensure_venv
      run_live_viz "${POSITIONAL[1]:-14550}" "${POSITIONAL[2]:-8765}"
      ;;
  ```
- Add a new mode `--single-live` that runs the existing single-drone stack
  in the background and pops the live web view in the foreground:
  ```bash
  --single-live)
      NEED_PYMAVLINK=1 ensure_venv
      run_single_mission 300 &
      MISSION_PID=$!
      trap 'kill $MISSION_PID 2>/dev/null || true' EXIT
      run_live_viz 14550 8765
      wait $MISSION_PID || true
      ;;
  ```
- Update the `--help` block: add the two new modes under "Per-drone stack
  modes" and document `--viz-live [MAV_PORT] [HTTP_PORT]` and
  `--single-live`. Mention the default URL `http://127.0.0.1:8765`.
- **Do not** change the default `--default` branch in this PR — flipping the
  default from matplotlib-replay to web Run-time View is called out as a
  follow-up below.

#### 2.1.E) Acceptance criteria (every box must be ticked)

**Backend / telemetry plumbing:**

- [ ] `simulation/live_telemetry.py` exists and exports
      `LiveTelemetrySample`, `TelemetryQueue`, `MAVLinkLiveSource`,
      `TelemetryCSVRecorder`, `parse_telemetry_payload`, `_gps_to_enu`.
- [ ] `MAVLinkBridge.send_state(...)` → `MAVLinkLiveSource` → `TelemetryQueue`
      round-trip preserves position within 1 cm and attitude within 1e-4 rad
      (`TestLiveTelemetry::test_bridge_to_queue_roundtrip`).
- [ ] Concurrent `push()`/`snapshot()` over 1000 samples is exception-free
      (`TestLiveTelemetry::test_queue_thread_safety`).
- [ ] CSV recorder header + N data rows match input count
      (`TestLiveTelemetry::test_csv_recorder_roundtrip`).

**Web Run-time View:**

- [ ] `simulation/runtime_view/` package exists with the file layout in
      §2.1.B.1, including a vendored `three.module.js` and `OrbitControls.js`.
- [ ] `requirements.txt` adds `flask>=3.0` and `flask-sock>=0.7` (no other
      new top-level deps).
- [ ] `python -m simulation.runtime_view.server --no-source --port 0`
      starts, returns `200 OK` for `/`, `/live`, `/api/missions`,
      `/api/status`, and `/api/snapshot?n=10`, and shuts down cleanly on
      SIGINT.
- [ ] `GET /` HTML contains `<header class="topbar">`, `class="missions"`,
      and at least the six mission cards from `missions.json`.
- [ ] `GET /live` HTML contains `<canvas id="viewport">`,
      `class="hud-kv"` blocks for `AGL`, `ALT MSL`, `SPEED`, `HEADING`,
      `THROTTLE`, `BATTERY V`, `BATTERY %`, `MODE`, and a status chip.
- [ ] `WS /ws/telemetry` echoes the most recent `LiveTelemetrySample`
      pushed via the injected queue, encoded as JSON
      (`TestRuntimeViewServer::test_websocket_streams_latest_sample`).
- [ ] `simulation/visualize_drone_3d.py` is unchanged — `git diff` for
      that file is empty.

**Wiring & UX:**

- [ ] `./run_scenario.sh --viz-live` opens the browser at
      `http://127.0.0.1:8765` and exits cleanly with code `0` on SIGINT
      (manual smoke; documented in §2.1.G).
- [ ] `./run_scenario.sh --single-live` runs the SITL stack and serves the
      live view in parallel (manual smoke).
- [ ] The launcher matches the pysimverse reference at the *layout* level:
      dark navy (`var(--bg-0)`), top header with centred title, mission
      grid of cards with title/description/start/lock states.

**Tests & docs:**

- [ ] `pytest -q simulation/test_drone_physics.py` reports the
      pre-existing **255 passed** plus the **6 new** `TestLiveTelemetry`
      tests **and** the **4 new** `TestRuntimeViewServer` tests in §2.1.F
      (target: **265 passed**).
- [ ] `MAINTENANCE.log` updated with one line summarising the new
      Run-time View feature and the new test count.

#### 2.1.F) Test plan — to add to `simulation/test_drone_physics.py`

Add **two** new classes appended after the existing `TestMAVLink` class
(around `test_drone_physics.py:1582`). All tests must be hermetic — bind
to `127.0.0.1` on ephemeral ports (`socket.bind(("",0))` then read
`getsockname()[1]`), never the hard-coded `14550` or `8765`.

##### `TestLiveTelemetry` — 6 tests

1. **`test_parse_telemetry_payload_attitude`** — call
   `parse_telemetry_payload(MAVLINK_MSG_ID_ATTITUDE, <attitude_payload>)`
   and assert roll/pitch/yaw round-trip to within `1e-6`. The payload is
   the bytes from `build_attitude(...)` minus the MAVLink header/CRC, so
   the test should `decode_mavlink_v2(build_attitude(...))` first and feed
   the returned payload into the new parser.
2. **`test_parse_telemetry_payload_global_position`** — same pattern with
   `build_global_position_int`. Assert lat/lon/alt decoded match the
   inputs within `1e-6` deg / `1 mm`.
3. **`test_gps_to_enu_inverse_of_enu_to_gps`** — for a handful of sample
   ENU vectors, `_gps_to_enu(*_enu_to_gps(p, ref…), ref…)` returns `p`
   within `1 mm`.
4. **`test_queue_thread_safety`** — one producer thread `push()`es 1000
   samples; a consumer thread calls `snapshot()` in a loop. Join both,
   assert no exception, `len(queue) <= maxlen`, and the very last sample
   pushed is observable via `latest()`.
5. **`test_bridge_to_queue_roundtrip`** — full UDP integration:
   - Bind a `MAVLinkLiveSource` to an ephemeral UDP port on `127.0.0.1`.
   - Construct a `MAVLinkBridge(target_ip="127.0.0.1", target_port=<source_port>, listen_port=<other_ephem_port>)`,
     call `start()`, then `send_state(SimState(...))` with known values.
   - Poll `queue.latest()` for ≤ 1 s.
   - Assert position (after GPS↔ENU round-trip) is within `1 cm`,
     attitude within `1e-4 rad`, throttle within `1 %`.
6. **`test_csv_recorder_roundtrip`** — write a `TelemetryCSVRecorder` to a
   `tmp_path` file, push 5 samples, `close()`, then re-open and assert:
   header matches, 5 data rows, numerics round-trip via `np.allclose`,
   `mode`/`armed` columns string-compare.

##### `TestRuntimeViewServer` — 4 tests

These tests use `flask.Flask.test_client()` (sync HTTP) and
`flask_sock`'s `Sock.test_client()` (sync WebSocket). No real socket
binding required. Each test calls `create_app(source=None, queue=q, missions_path=tmp_missions)`
with a fake `TelemetryQueue` and a temporary `missions.json`.

7. **`test_index_route_renders_launcher`** — `client.get("/")` returns
   `200`, `Content-Type: text/html`, and the body contains the strings
   `class="missions"`, `class="topbar"`, and the title of the first
   mission from the temp `missions.json`.
8. **`test_api_missions_returns_catalogue`** — `client.get("/api/missions")`
   returns `200` and a JSON list whose length matches the temp file and
   whose first item has the keys `id`, `title`, `description`,
   `thumbnail`, `tier`, `start_command`, `disabled`.
9. **`test_api_status_reflects_queue_state`** — push 3 samples into the
   fake queue, call `/api/status`, assert `connected` is `True`,
   `samples == 3`, and `last_t_boot` matches the last sample.
10. **`test_websocket_streams_latest_sample`** — push 1 sample into the
    queue, open a WebSocket against `/ws/telemetry`, recv one message,
    decode JSON, assert the `pos_enu` and `euler` fields match the
    pushed sample within `1e-9`. Time out the test at 2 s so a hung WS
    fails the suite instead of hanging CI.

#### 2.1.G) Verification commands (run in order)

```bash
# 1) Telemetry & server unit tests (fast, <3 s, no UDP/Docker needed)
pytest -q simulation/test_drone_physics.py::TestLiveTelemetry \
       simulation/test_drone_physics.py::TestRuntimeViewServer

# 2) Full physics test suite — must remain green at 265 passed
pytest -q simulation/test_drone_physics.py

# 3) Static replay regression — visualize_drone_3d.py is unchanged
python simulation/visualize_drone_3d.py simulation/scenario_data.npz

# 4) Local smoke: server-only (no MAVLink source, fake data via /api endpoints)
python -m simulation.runtime_view.server --no-source --port 8765 &
sleep 1
curl -fsS http://127.0.0.1:8765/                | head -5
curl -fsS http://127.0.0.1:8765/api/missions    | python -m json.tool | head -20
curl -fsS http://127.0.0.1:8765/api/status      | python -m json.tool
kill %1

# 5) Local smoke: bridge → server → browser (no SITL container required)
python - <<'PY' &
import threading, time, numpy as np
from simulation.mavlink_bridge import MAVLinkBridge
from simulation.drone_physics import run_simulation, DroneParams
records = run_simulation(
    [np.array([0,0,5.]), np.array([10,0,5.]), np.array([10,10,7.])],
    params=DroneParams(mass=1.5, max_thrust=25.0),
    dt=0.02, max_time=30.0,
)
b = MAVLinkBridge(target_ip="127.0.0.1", target_port=14550, listen_port=14552)
b.start()
threading.Thread(target=b.run_replay, args=(records,),
                 kwargs=dict(hz=50, loop=True), daemon=True).start()
time.sleep(120)
b.stop()
PY
python -m simulation.runtime_view.server --port 8765 --listen-port 14550
# In another shell: open http://127.0.0.1:8765/live
# Expected: dark-navy launcher → click "Open Live View" → drone mesh moves
# along the trail, HUD updates continuously, status chip shows CONNECTED.

# 6) End-to-end smoke against real SITL (requires Docker/K8s)
./run_scenario.sh --single-live
```

A successful run means: the launcher opens at `http://127.0.0.1:8765`,
matches the dark-navy/mission-card layout, clicking "Open Live View"
shows the Three.js viewport with the drone mesh moving in real time, the
status chip flips between `DISCONNECTED` and `CONNECTED`, every HUD field
updates, and Ctrl-C in the shell shuts down both Flask and the
`MAVLinkLiveSource` thread within 2 s.

#### 2.1.H) Out of scope / follow-ups

- **Flipping `--default` to the web view.** Defer the change to
  `run_scenario.sh`'s `--default` branch to a follow-up PR after operators
  have used `--single-live` for one cycle. Add a TODO in `run_scenario.sh`
  next to the `--default` branch referencing this section.
- **Browser-driven `start_command` execution.** `POST /api/launch`
  currently *returns* the shell command and the user pastes it into a
  terminal. Wiring it to actually `subprocess.Popen` the command would
  need a sandbox, audit log, and confirmation modal — out of scope here.
- **Windows browser auto-open.** `run_live_viz` calls `open` (macOS) and
  `xdg-open` (Linux). Add a Windows path (`start ""`) in a follow-up.
- **`.BIN` recording.** The first iteration writes CSV only. Writing a
  DataFlash-compatible `.BIN` from a live MAVLink stream is a much larger
  scope (matching the ArduPilot log format) and is tracked separately
  under §5 future work.
- **Multi-drone live view.** The swarm path stays unchanged in this PR.
  A future change would let `MAVLinkLiveSource` demultiplex by
  `system_id` into a `dict[int, TelemetryQueue]`; `live.js` would then
  spawn one mesh per drone ID. The mission card already exists
  (`swarm-3`, `swarm-6`) — only the backend needs lifting.
- **Real mission thumbnails.** v15 ships placeholder PNGs generated by
  Pillow. Replacing them with screenshots from real SITL runs is a
  one-line change to `web/img/`.
- **Authentication / multi-user.** The server binds to `127.0.0.1` only.
  Exposing it on a LAN would require auth and CSRF protection — explicitly
  out of scope.
- **Replacing `visualize_drone_3d.py` entirely.** The matplotlib replayer
  remains the canonical post-flight tool. Eventually the web view should
  also be able to load `.BIN` and `.npz` files (a `?path=...` query on
  `/live`); track that under §5.

---

## 3) Verification matrix

| Item category | Verification method | Status |
|:---|:---|:---|
| Core Physics | Eq. 1-4 symmetry and rotation tests | **Done** |
| Aerodynamics | Fixed-wing stall and Quadrotor tilt-area tests | **Done** |
| Environment | SRTM/STL terrain + satellite UV-mapping tests | **Done** |
| Validation | Table 5 RMSE gates against real flight data (241+ tests) | **Done** |
| Energy Model | Battery discharge curve and autonomy estimation | **Done** |
| Wind Tuning | Iterative RMSE minimization convergence | **Done** |
| **Run-time View — parser** | `TestLiveTelemetry::test_parse_telemetry_payload_*` | **Done** |
| **Run-time View — frames** | `TestLiveTelemetry::test_gps_to_enu_inverse_of_enu_to_gps` | **Done** |
| **Run-time View — concurrency** | `TestLiveTelemetry::test_queue_thread_safety` | **Done** |
| **Run-time View — bridge↔queue** | `TestLiveTelemetry::test_bridge_to_queue_roundtrip` | **Done** |
| **Run-time View — persistence** | `TestLiveTelemetry::test_csv_recorder_roundtrip` | **Done** |
| **Run-time View — HTTP launcher** | `TestRuntimeViewServer::test_index_route_renders_launcher` + `test_api_missions_returns_catalogue` | **Done** |
| **Run-time View — HTTP status** | `TestRuntimeViewServer::test_api_status_reflects_queue_state` | **Done** |
| **Run-time View — WebSocket** | `TestRuntimeViewServer::test_websocket_streams_latest_sample` | **Done** |
| **Run-time View — end-to-end** | Manual `./run_scenario.sh --single-live` smoke | **Done** |

---

## 4) Execution order (Junie applies in this sequence)

1. **Phase RTV-1 — receiver module:** add `simulation/live_telemetry.py`
   (§2.1.A) and the six `TestLiveTelemetry` tests (§2.1.F). Run
   `pytest -q simulation/test_drone_physics.py::TestLiveTelemetry`. If
   anything is red, stop — the rest of the plan depends on a green
   parser/queue/recorder/round-trip.
2. **Phase RTV-2 — bridge replay helper:** add
   `MAVLinkBridge.run_replay()` (§2.1.C). Re-run the full
   `pytest -q simulation/test_drone_physics.py` suite to confirm the
   existing `TestMAVLink` class is still green.
3. **Phase RTV-3 — Flask backend:** add `simulation/runtime_view/__init__.py`,
   `server.py`, `missions.json`, `README.md`, and append
   `flask>=3.0`/`flask-sock>=0.7` to `requirements.txt`. Add the four
   `TestRuntimeViewServer` tests (§2.1.F) and run them in isolation:
   `pytest -q simulation/test_drone_physics.py::TestRuntimeViewServer`.
4. **Phase RTV-4 — vendored frontend assets:** create
   `simulation/runtime_view/web/` with `index.html`, `live.html`,
   `styles.css`, `app.js`, `live.js`, `vendor/three.module.js`,
   `vendor/OrbitControls.js`, `vendor/THREE_VERSION.txt`, and `img/*.png`
   placeholders generated via Pillow. Run §2.1.G step 4 (server-only
   smoke) to confirm the launcher renders.
5. **Phase RTV-5 — orchestrator wiring:** add `run_live_viz`,
   `--viz-live`, `--single-live` to `run_scenario.sh` and update
   `--help` (§2.1.D).
6. **Phase RTV-6 — full verification:** run §2.1.G commands 1-6
   end-to-end, capture the launcher screenshot for review, then update
   `MAINTENANCE.log` with the new test count and a one-line summary.

---

## 5) Notes

- Completed paper-aligned work is now fully integrated into the "Implemented" section.
- 255 tests currently passing (as of 2026-04-04 audit). Target after
  RTV merge: **265 passed** (255 existing + 6 new `TestLiveTelemetry`
  + 4 new `TestRuntimeViewServer`).
- The RTV diff is **mostly additive** — `simulation/visualize_drone_3d.py`
  is **not** edited. The only modified existing files are
  `simulation/mavlink_bridge.py` (one new method), `requirements.txt`
  (two new pinned deps), and `run_scenario.sh` (two new modes + help
  text). Everything else lives under `simulation/runtime_view/` and
  `simulation/live_telemetry.py`.
- Two new pip dependencies (`flask`, `flask-sock`) are introduced —
  both are pure-Python, MIT-licensed, and small. The vendored
  `three.module.js` is also MIT-licensed; record its version in
  `vendor/THREE_VERSION.txt` and the SPDX header comment at the top of
  the file.
- Future work: flipping `--default` to the web view, browser-driven
  command launching, Windows `start` integration, DataFlash-compatible
  `.BIN` recording, multi-drone live demultiplexing, real mission
  thumbnails, Raft-based task visualization.
