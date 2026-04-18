# Changelog

All notable changes to the Swarm Digital Twin project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [2026-04-18] — Kubernetes Gazebo simulation planning docs

### Added

- `ROADMAP.md`: new phased roadmap for Kubernetes-native Gazebo swarm simulation, covering realistic physics, terrain model integration, collision detection, wind simulation, and full-system validation gates.
- `TODO.md`: actionable backlog with implementation tasks, acceptance KPIs, and documentation synchronization items.
- `todo/gazebo_k8s_playground.md`: detailed operational scenario describing how to use Gazebo as a Kubernetes playground, including topology, execution flow, fault injection, test matrix, and acceptance report template.

### Notes

- This entry introduces planning and scenario documentation only; it does not claim implementation of new simulation mechanics yet.

### Changed

- Migrated former `docs/REFACTOR_PLAN.md` follow-up items into `ROADMAP.md`, `TODO.md`, and `todo/gazebo_k8s_playground.md`.
- Updated documentation references to use unified roadmap/backlog documents.
- Removed direct dependency on `docs/REFACTOR_PLAN.md` in active documentation links.

---

## [2026-04-18] — Live Run-time View release

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
  ground rings, WP labels, and dashed flight-plan paths (per-drone in
  swarm mode).
- `simulation/live_telemetry.py`: MAVLink v2 receiver thread
  (`MAVLinkLiveSource`), thread-safe `TelemetryQueue` ring buffer,
  `LiveTelemetrySample` dataclass with `drone_id` field,
  `TelemetryCSVRecorder`, GPS↔ENU conversion.
- `simulation/physics_live_replay.py`: runs the Python physics engine and
  streams results through `MAVLinkBridge.run_replay()` → UDP →
  `MAVLinkLiveSource` → FastAPI → browser. Supports `--replay` (NPZ),
  `--swarm`, `--loop`, `--fps`, `--record-bin`.
- `MAVLinkBridge.run_replay()` method for deterministic playback of
  `SimRecord` lists without a SITL container.
- Vendored `three.module.js` (r160) and `OrbitControls.js` for offline
  operation.

#### Multi-drone live view
- `decode_mavlink_v2()` returns `(system_id, msg_id, payload)` — extracts
  the MAVLink system ID from byte 5 of the header.
- `MAVLinkLiveSource` demultiplexes telemetry by `system_id` into
  per-drone assembly state. Each drone's samples carry a `drone_id` field.
- `live.js` dynamically creates drone meshes keyed by `drone_id`, each
  with its own colour (8-colour palette), trail, and `D1`/`D2`/... label.
  Camera follows the centroid of all drones.
- Swarm replay spawns N `MAVLinkBridge` instances (one per drone with
  unique `system_id`), interleaving all drones in a single replay thread.

#### Post-flight web replay
- `--replay-live [FILE]` mode in `run_scenario.sh` — replays an existing
  `.npz` file in the live Three.js viewer (looping).
- `GET /api/files` lists `.npz` and `.BIN` files available for replay.
- `POST /api/load?path=...` loads an `.npz` file and starts a background
  bridge replay to the live viewer.
- Launcher UI "Replay Flight Data" section with file cards and REPLAY
  buttons.

#### Browser-driven command execution
- `POST /api/launch?id=<mission>` executes a mission's `start_command`
  from `missions.json` via `subprocess.Popen`.
- Allowlist security: only commands in the mission catalogue are accepted.
- `POST /api/launch/stop` terminates a running mission.
- `GET /api/launch/status` checks if a mission process is running.
- Audit log written to `.ai/launch.log` with timestamps.
- Confirmation modal with Launch button in the launcher UI.

#### DataFlash `.BIN` recording
- `simulation/dataflash_recorder.py`: writes ArduPilot-compatible binary
  log files with FMT header records + ATT, GPS, BAT, MODE data records.
- `record_sample()` converts `LiveTelemetrySample` to ATT+GPS+BAT records.
- `--record-bin FILE` flag in `physics_live_replay` wires the recorder
  via the `MAVLinkLiveSource` sample hook.

#### REST API
- `GET /api/missions` — mission catalogue from `missions.json`.
- `GET /api/status` — connection/sample status.
- `GET /api/snapshot?n=N` — last N telemetry samples.
- `GET /api/waypoints` — per-drone waypoints (dict format).
- `GET /api/files` — available `.npz`/`.BIN` data files.
- `POST /api/load?path=...` — load and replay a data file.
- `POST /api/launch?id=...` — execute a mission command.
- `POST /api/launch/stop` — stop running mission.
- `GET /api/launch/status` — mission process status.
- `WS /ws/telemetry` — 50 Hz telemetry push with snapshot on connect.

#### run_scenario.sh modes
- `(default)` — physics sim + live viewer (looping).
- `--physics-live [--loop]` — single-drone physics sim → live viewer.
- `--physics-swarm-live [N]` — N-drone swarm → live viewer (all drones).
- `--replay-live [FILE]` — replay `.npz` file in live viewer.
- `--viz-live [MAV_PORT] [HTTP_PORT]` — bare server for external MAVLink.
- `--single-live` / `--single` — SITL stack + live viewer.

#### Tests (57 new, 299 total)
- `TestLiveTelemetry` (6), `TestRuntimeViewServer` (4),
  `TestRunTimeViewIntegration` (10), `TestLiveViewNoMotionRegression` (6),
  `TestPhysicsLiveReplay` (14), `TestMultiDroneLiveView` (4),
  `TestPostFlightReplay` (5), `TestBrowserLaunch` (3),
  `TestDataFlashRecorder` (5).

### Changed
- **Server migrated from Flask to FastAPI** (ASGI-native async WebSocket).
  Dependencies: `fastapi>=0.110`, `uvicorn>=0.27`, `websockets>=12,<13`.
- Default `./run_scenario.sh` runs physics sim + live viewer (was bare
  server, was matplotlib replayer before that).
- Windows browser auto-open via `start ""` in `run_live_viz()`.
- `load_npz_records()` handles missing optional keys gracefully.
- `mission_waypoints` changed from list to dict `{drone_id: [wps]}`.

### Fixed
- **Replay-before-receiver race condition**: `start_telemetry()` now binds
  the UDP listener before the replay thread starts.

---

## Paper-Aligned Physics (implemented prior to this release)

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
