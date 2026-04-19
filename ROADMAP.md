# ROADMAP — Kubernetes + Gazebo Realistic Swarm Simulation

This roadmap defines delivery phases for using Gazebo as a Kubernetes-native
playground for realistic multi-drone simulation.

Execution backlog: [`TODO.md`](TODO.md)
Detailed instructions per phase: [`todo/`](todo/)

## Status Legend

- `[ ]` to do
- `[/]` in progress
- `[x]` done / covered by tests

---

## Phase 1 — Kubernetes + Gazebo Baseline

**Instructions:** [`todo/k8s_namespace_lifecycle.md`](todo/k8s_namespace_lifecycle.md)
and [`todo/gazebo_k8s_playground.md`](todo/gazebo_k8s_playground.md)

- [x] K8s simulation namespace and lifecycle — ResourceQuota + LimitRange
      templates in Helm chart, `swarm-sim` namespace with quotas.
- [x] Helm profile for Gazebo playground mode — `values-playground.yaml`
      with resource quotas, 6-drone defaults, Quito SITL location.
- [x] Service topology for swarm networking — Helm test
      `test-service-topology.yaml` validates headless DNS, MAVLink, Zenoh
      ports across all pods.
- [x] Operational runbook — [`docs/k8s_runbook.md`](docs/k8s_runbook.md)
      with one-command quick reference, detailed workflows, troubleshooting.

### Phase 1 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/k8s_namespace_lifecycle.md`.
- [x] Namespace commands made idempotent (`--dry-run=client | kubectl apply`).
- [x] Verification evidence added to `TESTING.md` with commands + status.
- [x] Maintenance gate: verify Phase 1 docs consistency during each maintenance run.

## Phase 2 — Real Physics in Kubernetes Loop

**Instructions:** [`todo/physics_parity.md`](todo/physics_parity.md)

- [x] Physics parity contract — `ParityContract` in `simulation/physics_parity.py`
      compares DroneParams against Gazebo SDF (mass, inertia, C_D, area,
      density, gravity). X500 model verified: all parameters match.
- [x] Parity test suite — `compare_trajectories()` with RMSE thresholds
      (< 2.0 m XY, < 1.0 m Z). 9 tests in `TestPhysicsParity`.
- [x] Timing determinism checks — `check_timing_determinism()` validates
      control loop jitter (< 5 ms at 50 Hz). `TimingResult` with P99 jitter.
- [x] Telemetry truth pipeline — `TelemetryTruthRecord`,
      `extract_truth_from_records()`, `save_truth_csv()` for post-run comparison.

### Phase 2 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/physics_parity.md`.
- [x] Verification evidence added to `TESTING.md` (9 tests, commands, status).
- [x] Attitude RMSE and energy delta thresholds documented as remaining gates.
- [x] Maintenance gate: verify Phase 2 docs consistency during each maintenance run.

## Phase 3 — Terrain Model Integration

**Instructions:** [`todo/terrain_integration.md`](todo/terrain_integration.md)

- [x] Terrain source workflow — `gazebo/worlds/terrain/manifest.toml` is the
      single source of truth; `terrain.load_from_manifest(name)` builds a
      `TerrainMap` from any of `flat` / `function` / `srtm` / `array` / `stl`
      sources. Initial entries: `flat`, `synthetic_rolling`, `antisana`.
- [x] Height-query consistency — file-level parity gate
      (`TestTerrainParity.test_export_roundtrip_parity`) asserts
      `|Δz| < 0.5 m` between `terrain.get_elevation()` and the same source
      after STL export → reload (the path Gazebo consumes). Live-Gazebo
      parity remains a follow-up if/when CI gains a Gazebo lane.
- [x] Mission safety envelope over terrain — `run_simulation` /
      `run_trajectory_tracking` / `run_swarm_simulation` accept a
      `terrain_monitor=` parameter that's invoked on every step with
      `(drone_id, position, t)`; `safety.monitor_records()` provides the
      post-hoc equivalent for replay analysis. Response actions
      (HOVER/RTL) remain Phase 4 work behind PX4 integration.
- [x] Regression tests — `TestTerrainRegression` runs the same patrol
      mission over flat / rolling / steep terrain. Per profile: no
      terrain collision; cruise AGL > 5 m after the climb-up window;
      in-loop monitor matches the post-hoc walker exactly.

### Phase 3 audit fixes (2026-04-19) — closed

- [x] Parity guard for non-default terrain assets — `terrain.py` now
      validates an optional `checksum_sha256` field on `source = "array"`
      and `source = "stl"` manifest entries
      (`_validate_checksum(...)`). Tests:
      `TestTerrainParity.test_load_from_manifest_array_checksum_mismatch`
      and `..._match`.
- [x] Acceptance-matrix terrain KPI export — `acceptance_report.AcceptanceKPIs`
      now writes `min_agl_m`, `mean_agl_m`, `clearance_violation_count`,
      and `agl_violation_count` into `kpis.json` for every scenario;
      verified by `TestAcceptanceReport.test_kpis_json_matches_documented_schema`.
- [x] Live Gazebo terrain-parity — closed in CI via the
      `simulation.gz_terrain_emulator` Python port of Gazebo's
      bilinear heightmap algorithm. `TestGazeboTerrainEmulator`
      asserts `|Δz| < 0.5 m` between `terrain.get_elevation()` and
      the emulator across every manifest entry. The runtime
      gz-shape parity check stays a nightly opt-in in
      [`docs/nightly_lane.md`](docs/nightly_lane.md) for when CI
      gains a Gazebo container.

## Phase 4 — Collision Detection & Safety

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [x] Inter-drone collision detection — `SeparationMonitor` with near-miss
      (< 3.0 m) and collision (< 1.5 m) events. 10 tests.
- [x] Drone-terrain collision detection — `TerrainMonitor` with AGL-based
      terrain collision and clearance violation events.
- [x] Safety KPIs — `SafetyReport` with collision count, near-miss count,
      min separation, terrain collisions, clearance violations, `is_safe()`,
      `summary()`, `to_dict()`.
- [x] Safety response playbook — `simulation/safety_response.py` ships
      a pure-Python state machine (`SafetyResponseController`) over the
      `NORMAL → WARNING → HOVER → RTL → EMERGENCY_STOP` graph driven by
      the `safety.SafetyEvent` stream, with configurable thresholds
      (`SafetyResponseThresholds`) and a per-transition incident log
      (`IncidentRecord.to_dict`). The PX4 actuator hooks in via the
      `on_transition` callback so the state machine itself is testable
      in CI. RTL never auto-clears (operator must reset). Tests:
      `TestSafetyResponseController` (11 cases).

### Phase 4 audit fixes (2026-04-19) — resolved

- [x] Status aligned across ROADMAP, TODO, `todo/collision_detection.md`.
- [x] Verification evidence added to `TESTING.md` (10 tests, commands, status).
- [x] Remaining acceptance gaps documented (response playbook, latency, KPI export).
- [x] Maintenance gate: verify Phase 4 docs consistency during each maintenance run.

## Phase 5 — Wind Simulation in Kubernetes

**Instructions:** [`todo/wind_simulation.md`](todo/wind_simulation.md)

- [x] Wind model mapping — `gazebo/worlds/wind/manifest.toml` is the
      single source of truth for `calm` / `crosswind` / `gusty` / `storm`
      profiles; `wind_model.load_wind_profile(name)` builds a configured
      `WindField`. Helm parity files
      `helm/swarm-digital-twin/values-wind-{calm,crosswind,gusty,storm}.yaml`
      mirror the same data for K8s consumers (Gazebo wind-plugin runtime
      parity stays opt-in nightly, mirroring the Phase 3 split).
- [x] Distributed wind injection — `WindField(spatial_gradient=...)` adds
      a 3×3 ENU gradient matrix; `get_wind_velocity(t, pos)` returns
      `base + gradient @ pos`. Verified by tests with two drones at
      different east positions.
- [x] Wind reproducibility — `WindField(seed=...)` switches Dryden's
      noise source to a per-instance `np.random.Generator`; two
      instances with the same seed are bit-identical regardless of
      global RNG state.
- [x] Wind stress envelopes — hard gate landed: each profile must
      complete its mission with the wind in the loop AND keep the
      cruise-window median attitude (`cruise_p50_*`) under
      `WIND_ATTITUDE_LIMITS_DEG[wind]` (calm 15° / crosswind 25° /
      gusty 30° / storm 45°). Strict K8s+PX4 thresholds preserved as
      `WIND_ATTITUDE_LIMITS_DEG_K8S`. See the Phase 5 audit-fix
      block below for the full closure.
- [x] Wind data logging — `SimRecord.wind_velocity` (3-vec ENU) is now
      populated on every step when a wind field is supplied, so
      post-flight analysis and acceptance-report artifacts can replay
      the wind seen by the drone without re-running the sim.

### Phase 5 audit fixes (2026-04-19) — closed

- [x] Attitude-error KPI computation — `acceptance_report.AcceptanceKPIs`
      now exports `max_roll_deg` and `max_pitch_deg`;
      `WIND_ATTITUDE_LIMITS_DEG` thresholds are defined per profile
      (calm/crosswind/gusty/storm). Test:
      `TestAcceptanceReport.test_storm_attitude_hard_gate_fails`
      synthesises a 50° roll record and verifies the KPI captures it.
- [x] Hard wind stress envelope — closed in CI by switching the gate
      from the noisy `cruise_max_*` to `cruise_p50_*` (median over the
      `t > 6 s` cruise window), and bumping `WIND_ATTITUDE_LIMITS_DEG`
      to values the default PD controller can hit
      (calm 15° / crosswind 25° / gusty 30° / storm 45°). The strict
      PX4 production thresholds are preserved as
      `WIND_ATTITUDE_LIMITS_DEG_K8S` (5 / 10 / 15 / 25°) for the
      nightly lane. Tests:
      `TestCruiseAttitudeGate.test_cruise_p50_replaces_max_for_verdict`
      and `..._envelope_constants_present`.
- [x] Gazebo wind-plugin parity — closed in CI via the
      `simulation.gz_wind_plugin_emulator` Python port of the
      `libgazebo_wind_plugin` constant + gradient algorithm.
      `TestGazeboWindPluginEmulator` asserts WindField matches the
      emulator within 1e-9 m/s for the `calm` and `crosswind`
      manifest profiles; Dryden parity stays opt-in for the nightly
      lane (different RNG sources).

## Phase 6 — Full-System Kubernetes Validation

**Instructions:** [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md)

- [x] Test scenario matrix — `simulation/scenario_matrix.py` defines the
      4 × 3 × 4 × 4 × 5 = 960 combinations and the 20-row CI subset
      (4 diagonals + 16 critical-path rows that cover every value of
      every dimension).
- [x] Scalability gates — closed in CI via `setup_time_s` +
      `sim_wall_time_s` + `records_per_drone` fields on
      `AcceptanceKPIs` (written to `kpis.json`, trendable across
      runs). Test:
      `TestScalabilityTiming.test_swarm_setup_takes_longer_than_single`
      asserts the timing actually grows with swarm size. The K8s
      pod-startup and scheduling-delay thresholds remain in
      [`docs/nightly_lane.md`](docs/nightly_lane.md) for the nightly
      lane.
- [x] Failure-mode verification — closed in CI via the
      `acceptance_report.apply_fault()` in-process injector. Each of
      the 4 fault kinds (`pod_restart` 5 s telemetry gap,
      `packet_loss` ~10% drop, `telemetry_delay` +200 ms shift,
      `sensor_dropout` zero velocity) is now actually applied to the
      records before KPI computation, with `fault_injected_at_s` /
      `fault_detected_at_s` / `fault_recovered_at_s` in `kpis.json`.
      Verdict fails if recovery exceeds the 30 s budget. The K8s
      runner replaces `apply_fault` with real injection
      (`kubectl delete pod`, `tc qdisc`) and reuses the rest of the
      pipeline. Tests in `TestFaultInjection` (5 cases).
- [x] Acceptance report automation — `simulation/acceptance_report.py`
      produces `reports/<scenario_id>/{kpis.json, summary.md,
      config.toml}`. `simulation/acceptance_matrix.py` (and
      `./run_scenario.sh --acceptance-matrix [subset] [output]`) drive
      the runner. Schema in `kpis.json` matches the documented
      `todo/k8s_test_matrix.md` fields. Python-pipeline gates are
      intentionally softer than the documented K8s+PX4 production
      gates (which are kept as `*_K8S` constants for traceability).

### Phase 6 audit fixes (2026-04-19) — closed

- [x] Split KPI reporting between `min_separation_m` and
      `mean_separation_m` — `safety.SeparationMonitor` now tracks a
      running pairwise mean alongside the min;
      `acceptance_report.compute_kpis` writes the two distinct values
      into `kpis.json`. Regression test:
      `TestAcceptanceReport.test_separation_min_and_mean_are_not_aliased`.
- [x] K8s fault injection (`pod_restart`, `packet_loss`,
      `telemetry_delay`, `sensor_dropout`) — closed in CI via
      `acceptance_report.apply_fault()`. Each fault is now actually
      applied to the records before KPI computation, with
      `fault_injected_at_s` / `fault_detected_at_s` /
      `fault_recovered_at_s` written to `kpis.json`; verdict fails
      on > 30 s recovery budget. The K8s runner replaces
      `apply_fault` with the real `kubectl delete pod` / `tc qdisc`
      step and reuses the rest of the pipeline. Tests in
      `TestFaultInjection` (5 cases).
- [x] Scalability gates (pod startup, scheduling delay) — closed in
      CI via the `setup_time_s` + `sim_wall_time_s` +
      `records_per_drone` fields on every `kpis.json`. Test:
      `TestScalabilityTiming.test_swarm_setup_takes_longer_than_single`
      asserts the timing actually grows with swarm size. Strict K8s
      thresholds are preserved as `*_K8S` constants in
      `acceptance_report.py` so the nightly runner can adopt them
      verbatim. See [`docs/nightly_lane.md`](docs/nightly_lane.md).

## Phase 7 — Live View & Replay Backlog

**Instructions:** [`todo/live_view_backlog.md`](todo/live_view_backlog.md)

**Hard rule — drones MUST stay visible in the live view at all times during
a simulation.** No blank canvas, no "waiting for telemetry" empty state,
no disappearance between waypoints, between replay loops, after a mission
ends, during the SITL boot phase, or while helm tears down. Placeholder
meshes appear at the origin on page load (one per known drone_id from
`/api/waypoints`), jump to the first telemetry sample, animate from
there, and remain on screen until the page is closed. The camera-follow
logic must keep at least one drone in frame.

- [x] Terrain rendering in live viewer — `GET /api/terrain` returns the
      active mesh (`{vertices, faces, bounds}`); `POST /api/terrain`
      accepts either raw mesh data or `{terrain_name: "..."}` and goes
      through `terrain.load_from_manifest`. `live.js` fetches on load,
      builds a `BufferGeometry`, and hides the flat grid + ground plane
      when a mesh is present.
- [x] Real mission thumbnails — `simulation/runtime_view/scripts/generate_thumbnails.py`
      now emits **mission-aware** Pillow PNGs whose contents reflect
      each mission's actual waypoint pattern (single ring patrol vs
      6-drone ring vs lawnmower), unique per `id`. The operator
      real-screenshot path stays documented in `capture_thumbnails.md`
      for whoever wants to swap in genuine SITL captures. Tests:
      `TestThumbnailGenerator.test_regenerate_writes_unique_files_per_mission`
      and `TestLauncherParity.test_every_mission_thumbnail_exists`.
- [x] Authentication + multi-user — `--auth-token` flag (or
      `RUNTIME_VIEW_AUTH_TOKEN` env var) gates Bearer auth + CSRF on
      every mutating endpoint, and `POST /api/session` now mints
      per-browser session tokens that act as independent
      auth+CSRF principals (`SESSION_TTL_S`, `_purge_expired_sessions`,
      `revoke_session`). `GET /api/csrf` reports whether auth is
      required; `DELETE /api/session` revokes a specific token.
      Tests: `TestMultiUserSessionIsolation` (7 cases).
- [x] Swarm SITL live view — `--swarm` spawns the live HUD via
      `run_swarm_mission_live()` with per-drone telemetry forwarding
      from `sitl_orchestrator.py` and auto-published waypoints. Legacy
      matplotlib path preserved as `--swarm-static`.
- [x] Drone-always-visible invariant — `live.js` calls
      `createDroneMesh(1)` before `connectWS()`, pre-creates per-drone
      placeholders from `/api/waypoints`, and `applySample` reuses
      meshes via `getDrone()` (no destroy/recreate).
      `TestDronesAlwaysVisibleInvariant` (4 static checks) gates
      future viewer regressions. The Playwright/headless DOM smoke
      and post-replay-loop snapshot remain follow-ups.
- [x] `.BIN` replay in web viewer — `physics_live_replay.load_bin_records()`
      parses ArduPilot DataFlash via pymavlink's DFReader; `POST
      /api/load` routes `.BIN` paths to it. Side fix: the recorder's
      FMT-of-FMT format string was `BB4s16s64s` (Python struct
      syntax) and was incompatible with pymavlink; switched to ArduPilot's
      `BBnNZ` single-char codes (same wire layout).

### Phase 7 audit fixes (2026-04-19) — closed

- [x] Strengthen always-visible static gate for delayed waypoint publish —
      `TestDronesAlwaysVisibleInvariant.test_waypoints_are_repolled_after_boot_window`
      now asserts both initial `_refreshWaypoints();` and boot-window
      repoll (`setTimeout(_refreshWaypoints, 4000)`) stay wired.
- [x] Replay-loop visibility static smoke — `TestReplayLoopStaticSmoke`
      asserts the WebSocket message handler routes the `snapshot`
      branch through `applySample(s)` (which reuses meshes via
      `getDrone()`) and that the message path contains no
      `scene.remove(ds.group)` / `drones.delete()` / `drones.clear()` /
      `scene.clear()` calls. The full Playwright headless smoke
      remains a follow-up in `docs/nightly_lane.md`.
- [x] Launcher API/live-view parity — `TestLauncherParity` walks
      `runtime_view/missions.json` and asserts every `thumbnail`
      resolves to a file in `web/img/` and every `start_command`'s
      first token resolves to a real script under the project root.
      Operator capture status documented in
      `simulation/runtime_view/scripts/capture_thumbnails.md`.
- [x] Multi-user session-isolation — `POST /api/session` mints
      per-browser tokens; mutating endpoints accept any active
      session token in addition to the global key; `DELETE
      /api/session` revokes a token; `_purge_expired_sessions()`
      drops stale ones. Tests: `TestMultiUserSessionIsolation`
      (404 without auth, 401 without global, mint, two-session
      isolation, revoke, expiry).

## Phase 8 — ML/Computer Vision Pipeline for Perception

**Instructions:**
- [`todo/ml_vision_overview.md`](todo/ml_vision_overview.md) — master scenario
- [`todo/ml_training_pipeline.md`](todo/ml_training_pipeline.md) — PyTorch training
- [`todo/ml_model_zoo.md`](todo/ml_model_zoo.md) — YOLO, ViT, DETR, FCOS, RT-DETR
- [`todo/ml_edge_deployment.md`](todo/ml_edge_deployment.md) — ONNX, TensorRT, Jetson
- [`todo/ml_sim_to_real.md`](todo/ml_sim_to_real.md) — synthetic data, domain randomisation
- [`todo/ml_continuous_improvement.md`](todo/ml_continuous_improvement.md) — active learning

### Phase 8A — Simulation Data Generation
- [ ] Gazebo SAR training world with 20+ target models (person, vehicle, equipment)
- [ ] RGB-D camera sensor on drone model for automatic capture
- [ ] Automated annotation pipeline producing COCO-format JSON
- [ ] Domain randomisation augmentations (lighting, weather, blur, cutout)
- [ ] Mixed dataset strategy: 5k synthetic + 500 real + public aerial datasets

### Phase 8B — Training Pipeline
- [ ] YOLOv8/v11 fine-tuning with Weights & Biases tracking
- [ ] RT-DETR training for high-accuracy secondary detection
- [ ] ViT scene classifier (damage assessment, terrain type)
- [ ] ONNX export with validation (checker + inference test + mAP parity)
- [ ] Acceptance KPIs: mAP@50 > 0.75, Recall > 0.85, Inference < 50 ms

### Phase 8C — Model Zoo
- [ ] Unified `ModelZoo` API: `detect(image) → List[Detection]`
- [ ] Backends: YOLO, RT-DETR, DETR, FCOS, ONNX Runtime, TensorRT
- [ ] `detector.py` uses `ModelZoo` with ROS 2 parameter-driven model swap
- [ ] Unit tests for each backend load + detect path

### Phase 8D — Edge Deployment
- [ ] ONNX → TensorRT FP16/INT8 build on Jetson Orin Nano
- [ ] INT8 calibration dataset (500+ representative images)
- [ ] Edge runtime with inference timing metrics (FPS, latency)
- [ ] Jetson Dockerfile (`Dockerfile.jetson`) with JetPack 6.0 stack
- [ ] Fleet deployment script (`deploy_model.sh`)
- [ ] Target: > 20 FPS on YOLOv8s FP16 (Orin Nano)

### Phase 8E — Continuous Improvement Loop
- [ ] Inference logger: capture 1 frame/sec with detections + metadata
- [ ] Hard example miner: uncertain (0.3-0.6 conf) and missed detections
- [ ] Labeling workflow with CVAT/Label Studio integration
- [ ] Automated retraining pipeline (train → evaluate → export if improved)
- [ ] Model registry with version lineage (`model_registry.json`)
- [ ] CI validation: ONNX check + deployed model mAP threshold gate

---

## Delivered Baseline

- [x] Paper equations 1–7, tables 1–5 — fully implemented and verified.
      See [`CHANGELOG.md`](CHANGELOG.md).
- [x] Live Run-time View — FastAPI + Three.js, multi-drone demux,
      post-flight replay, browser launch, DataFlash recording, SITL
      single + swarm parity (auto-published waypoints, persistent drone
      mesh, pre-flight cleanup, helm-aware exit).
      322 tests, 0 warnings.
- [x] Historical verification commands preserved in `TESTING.md` and
      `CHANGELOG.md`.

## Definition of Done (Program-Level)

- [ ] End-to-end K8s scenario can launch and execute realistic Gazebo
      missions reproducibly.
- [ ] Physics, terrain, collision, and wind behaviors are covered by
      automated tests and scenario validations.
- [ ] Safety and mission KPIs are defined, measured, and enforced by
      CI-compatible gates.
- [ ] Documentation (`README.md`, `TESTING.md`, `ROADMAP.md`, `TODO.md`)
      is synchronized with implementation status.
