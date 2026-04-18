# Changelog

All notable changes to the Swarm Digital Twin project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [Unreleased] — life_vision branch

### Added

#### Run-time View (Live 3D Visualization)
- **FastAPI + Three.js live viewer** as the default visualization
  (`./run_scenario.sh` with no args runs a physics simulation and streams
  it to the browser at `http://127.0.0.1:8765/live`).
- `simulation/runtime_view/` package: FastAPI server with WebSocket
  telemetry streaming at 50 Hz, mission launcher UI, dark-navy theme.
- `simulation/runtime_view/web/live.js`: Three.js scene with low-poly
  quadrotor mesh, 1000-point trail, HUD overlay (AGL, ALT MSL, SPEED,
  HEADING, THROTTLE, BATTERY V/%, MODE), status chip, OrbitControls camera.
- **Waypoint markers** in the live view: yellow spheres, vertical poles,
  ground rings, WP labels, and a dashed flight-plan path fetched from
  `GET /api/waypoints`.
- `simulation/live_telemetry.py`: MAVLink v2 receiver thread
  (`MAVLinkLiveSource`), thread-safe `TelemetryQueue` ring buffer,
  `LiveTelemetrySample` dataclass, `TelemetryCSVRecorder`, GPS↔ENU
  conversion.
- `simulation/physics_live_replay.py`: runs the Python physics engine and
  streams results through `MAVLinkBridge.run_replay()` → UDP →
  `MAVLinkLiveSource` → FastAPI → browser. Supports `--replay` (NPZ),
  `--swarm`, `--loop`, `--fps`.
- `MAVLinkBridge.run_replay()` method for deterministic playback of
  `SimRecord` lists without a SITL container.
- `GET /api/waypoints` endpoint returning the active mission waypoints.
- `GET /api/status`, `GET /api/snapshot?n=N`, `GET /api/missions` REST
  endpoints.
- `WS /ws/telemetry` WebSocket: initial 200-sample snapshot on connect,
  then per-sample push with 5 s keepalive pings.
- Mission launcher landing page (`/`) with 6 mission cards (3 free,
  3 pro-placeholder), pysimverse-inspired dark-navy layout.
- Vendored `three.module.js` (r160) and `OrbitControls.js` for offline
  operation.

#### run_scenario.sh Modes
- `--physics-live [--loop]` — Python physics sim → live Three.js viewer
  (no Docker/SITL needed).
- `--physics-swarm-live [N]` — swarm sim → live viewer (first drone,
  looping).
- `--viz-live [MAV_PORT] [HTTP_PORT]` — bare server waiting for external
  MAVLink telemetry.
- `--single-live` / `--single` — SITL stack + live viewer (requires
  Docker/K8s).
- Default (no args) changed from bare server to physics-live with
  `--loop`.

#### Tests (40 new, 282 total)
- `TestLiveTelemetry` (6): MAVLink payload parsing, GPS↔ENU inverse,
  queue thread safety, bridge-to-queue roundtrip, CSV recorder.
- `TestRuntimeViewServer` (4): HTTP route rendering, API catalogue,
  status endpoint, WebSocket sample streaming.
- `TestRunTimeViewIntegration` (10): real uvicorn server, HTTP asset
  delivery, WebSocket snapshot/streaming, full MAVLink bridge → server →
  WebSocket pipeline.
- `TestLiveViewNoMotionRegression` (6): empty-queue WS behavior, motion
  detection, SITL frame forwarding, orchestrator poll relay, full
  pipeline, run_scenario.sh script assertions.
- `TestPhysicsLiveReplay` (14): simulation record generation, NPZ
  roundtrip, swarm NPZ extraction, bridge-to-queue pipeline, full
  WebSocket pipeline, run_scenario.sh mode wiring, CLI help, receiver
  startup ordering, non-loop replay delivery.

### Changed
- **Server migrated from Flask to FastAPI** (commit 7384adb). ASGI-native
  async WebSocket replaces Flask + flask-sock threads. Dependencies:
  `fastapi>=0.110`, `uvicorn>=0.27`, `websockets>=12` (replacing
  `flask>=3.0`, `flask-sock>=0.7`).
- `run_scenario.sh` default mode changed from bare live-view server to
  `run_physics_live --loop` (runs simulation + streams to browser).
- All "Flask" references in comments and help text updated to "FastAPI".
- ENU recentering in `live.js`: first telemetry sample sets the origin
  so the drone stays in view regardless of geodetic reference point.

### Fixed
- **Replay-before-receiver race condition** in `run_physics_live()`:
  `start_telemetry()` now binds the UDP listener before the replay
  thread starts, preventing silent packet loss that caused the drone
  mesh to never move in the browser.

---

## Paper-Aligned Physics (implemented prior to life_vision branch)

All items from Valencia et al. (2025) verified against the codebase:

| Paper item | Implementation |
|:---|:---|
| Eq. 1 — position kinematics (rotation matrix) | `drone_physics.py` `physics_step()` |
| Eq. 2 — Euler angle rates from body rates | `drone_physics.py` `euler_rates_from_body_rates()` |
| Eq. 3 — Newton's 2nd law, body frame + Coriolis | `drone_physics.py` line 653 |
| Eq. 4 — rotational dynamics, full 3×3 inertia | `drone_physics.py` `np.linalg.inv(I)` |
| Eq. 5–7 — wind drag/lift/combined perturbation | `wind_model.py` `get_force()` |
| Table 2 — fixed-wing geometry | `drone_physics.py` `make_valencia_fixed_wing()` |
| Table 3 — CFD aero coefficients | `drone_physics.py` `FixedWingAero` |
| Table 4 — 7 mission profiles | `validation.py` `REAL_LOG_MISSIONS` |
| Table 5 — RMSE validation metrics | `validation.py` acceptance gate |
| Section 2.1 — SRTM terrain → STL export | `terrain.py` `from_srtm()` + `export_stl()` |
| Section 2.1 — Satellite texture overlay | `terrain.py` `export_obj_with_uv()` |
| Section 2.3 — Wind from real flight log | `flight_log.py` `get_wind_profile()` |
| Section 3.1 — Wind auto-tuning | `validation.py` `auto_tune_wind_force_scale()` |
| Section 3.2 — IRS-4 quadrotor | `drone_physics.py` `make_irs4_quadrotor()` |
| Section 3.5 — Quadrotor aero area | `drone_physics.py` `QuadrotorAero` |
| Section 3.5 — Battery and energy model | `drone_physics.py` `BatteryModel` |
| Motor dynamics | `drone_physics.py` `MotorModel` |
| Sensor noise (GPS/IMU/baro) | `sensor_models.py` |
