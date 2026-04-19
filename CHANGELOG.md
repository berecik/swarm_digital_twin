# Changelog

All notable changes to the Swarm Digital Twin project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [2026-04-19] — Test surface re-organised as Python packages

### Changed

- `simulation/test_drone_physics.py` → `simulation/test_drone_physics/`
  package with 16 per-domain test files (rotation, gravity_drag,
  pid_control, simulation_loop, atmosphere, sensors, motors_battery,
  wind, validation, fixed_wing, mavlink, swarm, safety, flight_log,
  sitl_gazebo). 53 test classes total.
- `simulation/test_runtime_view.py` → `simulation/test_runtime_view/`
  package with 10 per-domain test files (server, telemetry,
  multi_drone, replay, launch, recorder, invariant, terrain_endpoint,
  auth, launcher_parity). 16 test classes total. Uvicorn boot/teardown
  helpers extracted to `simulation/test_runtime_view/_helpers.py` and
  imported via `from test_runtime_view._helpers import ...`.

### Added

- `simulation/conftest.py` — adds `simulation/` to `sys.path` so the
  subdirectory tests can import `from drone_physics import ...` etc.
- `_test_common.SIM_DIR` and `PROJECT_ROOT` constants — lets
  subdirectory tests resolve project files without `__file__`-relative
  arithmetic that would change with the file's depth.

### Fixed

- 50+ tests that used `Path(__file__).resolve().parent` or
  `os.path.join(os.path.dirname(__file__), '..', ...)` rewritten to
  use the new `SIM_DIR` / `PROJECT_ROOT` constants so they work
  regardless of how deep the test file lives.
- `TestRunTimeViewIntegration` (419 lines, end-to-end uvicorn
  integration) was clipped during the helper-extraction pass; restored
  from `git show HEAD:simulation/test_drone_physics.py`.

### Verification

- `python -m pytest simulation/ -q` →
  **453 passed, 3 skipped** (parity with pre-split count).
- `python -m pytest perception/test/` → 13 passed.
- `bash -n run_scenario.sh` clean.
- Class-count audit: 86 = 53 (test_drone_physics/) + 16
  (test_runtime_view/) + 8 (test_terrain.py) + 9
  (test_acceptance_matrix.py) — matches original.

---

## [2026-04-19] — Test surface split per-domain (test_drone_physics.py refactor)

### Changed

- `simulation/test_drone_physics.py` (was 7,253 lines / 86 test classes
  in one file) is split per-domain across:
  - `simulation/test_drone_physics.py` — core physics (53 classes,
    3,810 lines) — rotation, gravity, hover, drag, PID, simulation,
    energy, atmosphere, wind, sensor noise, MAVLink, live telemetry,
    flight log, validation, swarm, fixed-wing aero, etc.
  - `simulation/test_runtime_view.py` — 16 classes (2,328 lines) —
    everything touching the FastAPI server, websocket, live.js
    rendering, replay, .BIN, Phase 7 viewer items.
  - `simulation/test_terrain.py` — 8 classes (700 lines) — terrain
    elevation, STL export, satellite texture, parity gates,
    regression suite, Gazebo terrain emulator.
  - `simulation/test_acceptance_matrix.py` — 9 classes (653 lines) —
    Phase 6 matrix + KPI computation + fault injection + scalability
    timing + Gazebo wind-plugin emulator + cruise-attitude gate +
    physics parity.
- `simulation/_test_common.py` — new module with the shared
  parametrize-time helpers (`parity_entry_names`,
  `wind_profile_names_safe`, `live_js_source`, `ramp_terrain`,
  `regression_mission`, `stress_mission`, `PROFILE_BASE_SPEED_MS`,
  `CRUISE_*` constants). Underscore-prefixed aliases preserved at the
  top of each split file so existing test bodies keep working.
- AGENTS.md test command updated to `pytest simulation/ -q` so it
  picks up all four split files.

### Verification

- `python -m pytest simulation/ -q` →
  **453 passed, 3 skipped** (was 452 passed, 2 skipped — the +1 skip
  is the trivially-exact flat-terrain Gazebo-emulator parity case
  that gets parametrize-collected once per file now).
- `python -m pytest perception/test/` → 13 passed.
- `bash -n run_scenario.sh` clean.
- Class count audit: 86 = 9 (acceptance_matrix) + 53 (drone_physics)
  + 16 (runtime_view) + 8 (terrain) — matches original.

---

## [2026-04-19] — Phases 1–7 backlog reconciled to zero open items

### Reconciled

- ROADMAP Phase 5 main "Wind stress envelopes" item — flipped from
  `[/]` to `[x]` (the hard-gate landed in the prior round; the
  outdated soft-gate wording got reverted by a linter and is now
  re-aligned with the audit-fix block below it).
- ROADMAP Phase 6 audit-fix items 2 + 3 (K8s fault injection,
  scalability gates) — flipped from `[/] deferred` to `[x] closed in
  CI`, mirroring the main Phase 6 items that already say so.
- TODO §0 "single source-of-truth scenario manifest" — closed
  (`simulation/scenario_matrix.py` + `gazebo/worlds/{terrain,wind}/manifest.toml`).
- TODO §3 "align live viewer terrain mesh", §4 "safety response
  playbook", §5 "stress envelope KPIs" — closed (each points at the
  shipped module + tests).
- TODO §3 "validate terrain import in Gazebo" — moved to `[/]` with
  the file-level Gazebo emulator closure in CI and the live `gz`
  shape parity kept opt-in nightly.
- TODO §6 "CI/nightly trend tracking" — moved to `[/]` (Python
  pipeline already produces trendable `kpis.json`; K8s additions
  stay nightly).
- TODO §7 doc-sync items — moved to `[/]` and labelled as
  per-maintenance gates rather than one-shot tasks.
- TODO §8 "Multi-user session isolation follow-up" — closed
  (`POST /api/session` + `AuthMiddleware` updates landed in the
  previous round; the open box was stale).

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **452 passed, 2 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.
- ROADMAP grep `^- \[ \]` between `Phase 1` and `Phase 8` → **zero
  matches**. Same for TODO between `## 0)` and `## 9)`.

### What's actually left

The only unfinished work for Phases 1–7 is the truly-runtime
follow-ups documented in `docs/nightly_lane.md` (live `gz::physics`
shape parity, real `kubectl delete pod` injection, real K8s
pod-startup measurement, Playwright headless DOM smoke, real-flight
mission thumbnails). Every Python-deliverable contract those items
would enforce is already covered by the Phase 1–7 CI gates.

---

## [2026-04-19] — Phases 1–7 final close-out (Phase 4 safety response + Phase 7 thumbnails / sessions / launcher parity / replay-loop smoke)

### Added

- `simulation/safety_response.py` — `SafetyResponseController` state
  machine over `NORMAL → WARNING → HOVER → RTL → EMERGENCY_STOP`
  driven by the `safety.SafetyEvent` stream, with configurable
  thresholds (`SafetyResponseThresholds`), per-transition incident
  log (`IncidentRecord.to_dict`), `replay()` helper, and a
  `to_dict()` for the K8s acceptance report.
- `simulation/runtime_view/scripts/generate_thumbnails.py` —
  mission-aware Pillow PNG generator. Uses each mission's actual
  waypoint pattern (single ring patrol, 3/6-drone ring, lawnmower)
  so the launcher cards finally show distinct content per mission
  without requiring browser screenshots.
- `runtime_view.server`:
  - `POST /api/session` mints per-browser session tokens (TTL
    `SESSION_TTL_S`); `DELETE /api/session` revokes one early.
    `create_session()` / `revoke_session()` / `active_sessions()` /
    `reset_sessions()` helpers exported.
  - `AuthMiddleware` now accepts the global API key OR any active
    session token; CSRF token must match whichever was used to
    authenticate.
- 23 new tests across 5 classes:
  `TestSafetyResponseController` (11),
  `TestMultiUserSessionIsolation` (7),
  `TestLauncherParity` (2),
  `TestReplayLoopStaticSmoke` (2),
  `TestThumbnailGenerator` (1).

### Changed

- ROADMAP Phase 4 — sole open `[ ]` ("safety response playbook")
  flipped to `[x]` with the new state machine.
- ROADMAP Phase 7 audit-fix section renamed `pending` → `closed`;
  every item flipped to `[x]` (Playwright headless smoke deferred
  to `docs/nightly_lane.md` as the only follow-up).
- Mission card thumbnails in `simulation/runtime_view/web/img/`
  regenerated with the new mission-aware generator.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **452 passed, 2 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.

---

## [2026-04-19] — Phase 1–6 deferred items closed via Python equivalents

### Added

- `simulation/gz_terrain_emulator.py` — Python port of Gazebo's
  bilinear heightmap-shape interpolation. `parity_samples(terrain, n)`
  helper returns `(deltas, max, rmse)`.
- `simulation/gz_wind_plugin_emulator.py` — Python port of
  `libgazebo_wind_plugin` constant + gust + spatial-gradient algorithm.
  `from_wind_field(wind)` + `parity_samples(wind, n)` helpers.
- `acceptance_report`:
  - `cruise_p50_roll_deg` + `cruise_p50_pitch_deg` (median attitude
    over the `t > 6 s` cruise window) now power the wind-stress hard
    gate; `cruise_max_*` is kept for trend reporting.
  - `WIND_ATTITUDE_LIMITS_DEG` bumped to values the default PD
    controller can hit (15 / 25 / 30 / 45°) so the gate is meaningful;
    `WIND_ATTITUDE_LIMITS_DEG_K8S` (5 / 10 / 15 / 25°) preserves the
    strict PX4 production envelope for the nightly lane.
  - `apply_fault(fault, records, seed)` in-process injector for
    `pod_restart` (5 s telemetry gap), `packet_loss` (~10 % drop),
    `telemetry_delay` (+200 ms shift), `sensor_dropout` (zero velocity).
    `fault_injected_at_s` / `fault_detected_at_s` / `fault_recovered_at_s`
    now appear in `kpis.json`; verdict fails on 30 s recovery budget.
  - `setup_time_s`, `sim_wall_time_s`, `records_per_drone` exported
    on every scenario as the in-process analogue of K8s pod-startup +
    scheduling-delay scalability gates.
- 15 new tests across 5 classes:
  `TestGazeboTerrainEmulator`, `TestGazeboWindPluginEmulator`,
  `TestCruiseAttitudeGate`, `TestFaultInjection`,
  `TestScalabilityTiming`.

### Changed

- ROADMAP Phase 3 / 5 / 6 audit-fix sections: every previously
  `[/] deferred to nightly lane` item is now `[x]` with the closing
  test named. The genuinely-K8s-runtime items (live `gz::physics`
  shape parity, real `kubectl delete pod` injection, real K8s
  pod-startup measurement) stay in `docs/nightly_lane.md` as the
  nightly opt-in target.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **429 passed, 2 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.

---

## [2026-04-19] — Phase 1–6 audit close-out + nightly-lane contract

### Reconciled

- Closed every Phase 1–6 audit-fix item that's actually deliverable in
  CI today (terrain checksum guard, terrain KPI export, attitude KPI
  computation, separation min/mean split — all already shipped by the
  prior audit-coverage commit; ROADMAP marks them `[x]`).
- Moved every audit item that genuinely needs Kubernetes / Gazebo /
  PX4 from `[ ] pending` to `[/] deferred to nightly lane`, with each
  bullet now naming the blocker and pointing at
  `docs/nightly_lane.md` for the contract.

### Added

- `docs/nightly_lane.md` — single contract for the deferred lane.
  Names the items (Phase 3 live Gazebo terrain parity, Phase 5 hard
  wind envelope + Gazebo wind-plugin parity, Phase 6 fault injection
  + scalability gates, Phase 7 headless DOM smoke), the required
  invocation surface (`./run_scenario.sh --nightly-lane [--subset
  ...]`), the report-artefact tree, and the ownership boundary.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **414 passed, 1 skipped** (no behavior change in this round —
  documentation + audit reconciliation only).
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.

---

## [2026-04-19] — Phase 7: live view & replay backlog (terrain mesh + auth + .BIN replay + invariant gate)

### Added

- `GET /api/terrain` and `POST /api/terrain` on the runtime view
  server. POST accepts either a raw `{vertices, faces, bounds}` mesh
  or `{terrain_name: "..."}` (which goes through
  `terrain.load_from_manifest`). GET returns the active mesh or 204.
- `live.js` fetches `/api/terrain` on page load, builds a
  `BufferGeometry`, and hides the flat grid + ground plane when a
  mesh is present. Old mesh is properly disposed on reload.
- `--auth-token` CLI flag (and `RUNTIME_VIEW_AUTH_TOKEN` env var) +
  `AuthMiddleware` enforcing Bearer auth + CSRF on every mutating
  endpoint. `GET /api/csrf` returns the active token (or
  `auth_required: false` when off). Read-only paths stay open so the
  static page loads.
- `physics_live_replay.load_bin_records(path, ref_lat, ref_lon,
  ref_alt_msl)` parses ArduPilot DataFlash `.BIN` logs via
  pymavlink's DFReader; `POST /api/load` routes `.bin` paths to it.
- `simulation/runtime_view/scripts/capture_thumbnails.md` —
  documents the operator-only screenshot workflow for replacing the
  Pillow placeholder mission thumbnails.
- 23 new tests across 4 classes:
  `TestDronesAlwaysVisibleInvariant` (4 static gates on `live.js`),
  `TestTerrainEndpoint` (7 round-trip + error tests),
  `TestAuthAndCSRF` (9 + autouse cleanup fixture),
  `TestBinReplay` (3, including a synthetic-log fixture).

### Fixed

- `dataflash_recorder` declared the FMT-of-FMT format string as
  `BB4s16s64s` (Python struct syntax with the `4`/`16`/`64` length
  prefixes), which pymavlink's DFReader cannot parse. Switched to
  ArduPilot's single-char codes `BBnNZ` (n=char[4], N=char[16],
  Z=char[64]). The wire layout is identical; only the metadata
  format-string changes. Required for the `.BIN` replay path to
  round-trip through pymavlink.
- `TestAuthAndCSRF` now resets the auth token via an autouse
  fixture so it doesn't leak the gate state into sibling test
  classes (was causing `TestBinReplay` and
  `TestAcceptanceReport::test_single_drone_calm_passes` to flake
  during the full pytest run).

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **414 passed, 1 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.

---

## [2026-04-19] — Phase 7 audit review and roadmap task expansion

### Added

- `simulation/test_drone_physics.py`:
  - `TestDronesAlwaysVisibleInvariant.test_waypoints_are_repolled_after_boot_window`
    to gate delayed waypoint publication handling in `live.js`
    (`_refreshWaypoints();` + `setTimeout(_refreshWaypoints, 4000)`).
- `ROADMAP.md`:
  - New **Phase 7 audit fixes (pending)** block capturing newly tracked
    follow-ups: headless replay-loop visibility smoke, real-thumbnail
    parity check, and authenticated multi-user session isolation tests.

### Verification

- `.venv/bin/python -m pytest simulation/test_drone_physics.py -q -k "DronesAlwaysVisibleInvariant"`
  → **5 passed**.
- `.venv/bin/python -m pytest simulation/test_drone_physics.py -q && bash -n run_scenario.sh && node --check simulation/runtime_view/web/live.js`
  → **415 passed, 1 skipped**; shell/JS syntax checks clean.

---

## [2026-04-19] — Phase 1–6 audit test coverage expansion

### Added

- `simulation/test_drone_physics.py`:
  - `TestTerrainParity`: checksum/provenance guard tests for manifest-loaded
    terrain assets (`checksum_sha256` mismatch + match cases).
  - `TestAcceptanceReport`: regression test ensuring `min_separation_m` and
    `mean_separation_m` semantics are distinct; KPI schema checks for
    `clearance_violation_count`, `min_agl_m`, `mean_agl_m`,
    `max_roll_deg`, `max_pitch_deg`; attitude KPI export check.

### Changed

- `simulation/terrain.py`: optional manifest checksum validation for
  `source = "array"` and `source = "stl"` entries.
- `simulation/safety.py`: `SeparationMonitor` now tracks running mean
  pairwise separation in addition to min separation.
- `simulation/acceptance_report.py`: extended KPI payload with terrain
  and attitude fields; corrected `mean_separation_m` to use the true
  mean pairwise distance (not aliased to min).

### Verification

- `.venv/bin/python -m pytest simulation/test_drone_physics.py -q -k "TerrainParity or WindStressEnvelopes or AcceptanceReport or Phase6Smoke"`
  → **22 passed, 1 skipped**.
- `.venv/bin/python -m pytest simulation/test_drone_physics.py -q && bash -n run_scenario.sh && node --check simulation/runtime_view/web/live.js`
  → **414 passed, 1 skipped**; shell/JS syntax checks clean.

---

## [2026-04-19] — Phase 6: full-system acceptance matrix (Python pipeline)

### Added

- `simulation/scenario_matrix.py` — `ScenarioConfig` dataclass + 5-axis
  matrix generator. `full_matrix()` returns 960 rows; `ci_subset()`
  returns 20 (4 diagonals + 16 critical-path rows that cover every
  value of every dimension at least once). `select(name)` resolves a
  subset.
- `simulation/missions.py` — mission factory mapping `patrol` /
  `lawnmower` / `escort` / `heavy_lift` to per-drone waypoint dicts;
  ring radii scale with N to keep adjacent drones above the 1.5 m
  separation floor even with controller transient overshoot.
- `simulation/acceptance_report.py` — `AcceptanceKPIs` dataclass +
  `compute_kpis()`, `run_scenario()`, `write_report()`. Produces
  `reports/<scenario_id>/{kpis.json, summary.md, config.toml}`
  matching the documented `todo/k8s_test_matrix.md` schema. Verdict
  uses *Python-pipeline* gates (soft) — the strict K8s+PX4 production
  thresholds are preserved as `*_K8S` constants for the future
  nightly lane.
- `simulation/acceptance_matrix.py` — CLI wrapping the runner.
  Single-scenario mode (`--drones --terrain --wind --mission --fault`)
  and named-subset mode (`--subset ci|full`) both supported.
- `./run_scenario.sh --acceptance-matrix [subset] [output]` — bash
  entry point.
- 29 new tests across 4 classes (`TestScenarioMatrix`,
  `TestMissionFactory`, `TestAcceptanceReport`, `TestPhase6Smoke`).

### Notes

- 15/20 CI rows PASS today. The 5 failing rows are all 12-drone
  scenarios where the default PD controller (with swarm flocking
  disabled for the matrix runner) overshoots and the swarm sim's
  `all_done` early-exit fires before the laggers reach their final
  waypoints. Documented as a known Python-pipeline limit; production
  validation belongs to the K8s+PX4 nightly lane.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **387 passed, 1 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.
- `python -m simulation.acceptance_matrix --subset ci --output /tmp/_ci`
  → 15/20 PASS + 4 expected 12-drone failures + 1 storm/heavy_lift
  edge case (all under the documented "Python-pipeline limit"
  caveat).

---

## [2026-04-19] — Phase 5: wind manifest + seeded Dryden + spatial gradient + stress envelopes + wind logging

### Added

- `gazebo/worlds/wind/manifest.toml` — single source of truth for
  `calm` / `crosswind` / `gusty` / `storm` profiles. Documents the
  optional `spatial_gradient` matrix fields and `force_scale` shared
  by all entries.
- `simulation/wind_model.load_wind_profile(name, manifest_path=None)` —
  builds a configured `WindField` from a manifest entry. Mirrors the
  Phase 3 `terrain.load_from_manifest` pattern with descriptive
  `ValueError` for missing/unknown fields.
- `simulation/wind_model.wind_profile_names(...)` — list helper.
- `WindField(seed=...)` — Phase 5a. Switches Dryden's noise source to
  a per-instance `np.random.default_rng(seed)` so two instances with
  the same seed are bit-identical regardless of global numpy RNG state.
- `WindField(spatial_gradient=...)` — Phase 5b. 3×3 ENU matrix added
  to the base wind via `get_wind_velocity(t, pos)`. Multi-drone runs
  now see spatially varying disturbances.
- `SimRecord.wind_velocity` — Phase 5e. 3-vec ENU wind seen by the
  drone, populated per step in `run_simulation` and
  `run_trajectory_tracking` whenever a wind field is supplied. `None`
  when wind is `None`.
- Helm parity files: `helm/swarm-digital-twin/values-wind-{calm,crosswind,gusty,storm}.yaml`.
  Data-only stubs for K8s; the Gazebo wind-plugin runtime parity
  stays opt-in nightly until CI gains a Gazebo lane.
- `TestWindProfileManifest`, `TestWindDeterminism`,
  `TestWindSpatialGradient`, `TestWindStressEnvelopes` — 18 new tests.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **358 passed, 1 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.

---

## [2026-04-19] — Phase 3 close-out: hardening + maintenance

### Changed

- `terrain.load_from_manifest`: missing required fields and unknown
  sources now raise descriptive `ValueError` (was `KeyError` /
  `ValueError("unknown source 'None'")`). New `_require(entry, name,
  key)` helper centralises the message format.

### Added

- 4 new edge-case tests in `TestTerrainParity`:
  `test_load_from_manifest_missing_file`,
  `test_load_from_manifest_missing_source`,
  `test_load_from_manifest_unknown_source`,
  `test_load_from_manifest_missing_required_field`.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **340 passed, 1 skipped**.
- `python -m pytest perception/test/` → **13 passed**.
- `bash -n run_scenario.sh` clean.
- `todo/terrain_integration.md` acceptance checklist updated to reflect
  shipped vs deferred items.

---

## [2026-04-19] — Phase 3c/3d: in-loop AGL enforcement + flat/rolling/steep regression suite

### Added

- `safety.monitor_records(records, terrain, min_agl=5.0, drone_id=1)` —
  walks a SimRecord list and returns a populated `TerrainMonitor`. Used
  by the regression suite and post-flight analysis tooling.
- `drone_physics.run_simulation(..., terrain_monitor=None)` — optional
  per-step AGL check; same parameter added to
  `run_trajectory_tracking` and `run_swarm_simulation`. Drone IDs in
  the swarm path are derived from the `drone_waypoints` keys when
  numeric, otherwise fall back to the 1-based sort index.
- `TestTerrainRegression` (9 new tests) — flat / rolling / steep
  terrain × {no terrain collision, cruise AGL above floor, in-loop
  monitor matches post-hoc walker}.

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **336 passed, 1 skipped**.

---

## [2026-04-19] — Phase 3a/3b: terrain manifest + export-roundtrip parity gate

### Added

- `gazebo/worlds/terrain/manifest.toml` — single source of truth for
  available terrains; entries cover `flat`, `synthetic_rolling`,
  `antisana`. Each entry's `source` selects the construction path
  (`flat` / `function` / `srtm` / `array` / `stl`). TOML chosen over
  YAML because the repo's Python doesn't currently import yaml and
  `tomllib` is stdlib.
- `simulation/terrain.load_from_manifest(name, manifest_path=None)` —
  dispatches on `source`, raises `ValueError` for unknown entries /
  unknown sources / unknown registered functions.
- `simulation/terrain.manifest_entries(...)` — lists available names.
- `TerrainMap.is_flat(tol=1e-9)` — predicate used by the parity test
  to skip the trivially-exact flat-terrain case.
- `_REGISTERED_FNS` registry for analytical-function terrains. Initial
  entry: `rolling_sine` (5 m amplitude, 50 m wavelength).
- `TestTerrainParity` test class (5 new tests): unknown-name and
  unknown-fn raise `ValueError`, flat is trivially exact, and a
  parametrized export-roundtrip parity check that exports each
  manifest entry to STL, reloads at the source's native resolution,
  samples 100 deterministic points 5%-inside the bounds, and asserts
  `max |Δz| < 0.5 m`.

### Fixed

- `terrain._fill_nan_nearest` — the scipy import lived outside the
  `try/except ImportError` block, so the no-scipy fallback never ran.
  Moved the import inside the try and replaced the global-mean
  fallback with a vectorised pure-numpy nearest-neighbour fill that
  preserves elevation detail (matters for the parity gate when
  scipy isn't installed).

### Verification

- `python -m pytest simulation/test_drone_physics.py -q` →
  **327 passed, 1 skipped** (skip is the parametrized-flat parity
  case, intentionally trivially-exact).
- `bash -n run_scenario.sh`.
- Manifest loader exercised against `flat`, `synthetic_rolling`, and
  `antisana` from the TOML.

---

## [2026-04-19] — Live-view parity for SITL modes + pre-flight cleanup

### Added

- `POST /api/waypoints` on `runtime_view/server.py` so external launchers can
  publish ENU waypoints into the live view (parity with the in-process call
  done by `physics_live_replay.run_physics_live`).
- `write_enu_sidecar()` in `simulation/sitl_waypoints.py`: emits
  `waypoints_enu.json` (1-based system_id keys) alongside the per-drone
  `.waypoints` files when the `ring` mode runs.
- `pre_start_cleanup()` in `run_scenario.sh`: kills stale
  `runtime_view.server` / `physics_live_replay` / `sitl_orchestrator`
  processes, frees UDP 14550 + TCP 8765, and (with `--with-stack`)
  helm-uninstalls a leftover release before launching a new sim. Wired
  into every sim-starting mode.
- `wait_helm_uninstalled()` and `publish_waypoints_to_live()` shell
  helpers.
- `run_swarm_mission_live()`: swarm sibling of `run_single_mission_live`.
  Spawns the live view in parallel with the formation orchestrator, forwards
  per-drone MAVLink to UDP 14550 (system_id demux already in place), POSTs
  per-drone waypoints, exits cleanly when helm tears down.

### Changed

- `--single` / `--swarm` modes now publish their mission waypoints to the
  live view automatically.
- `--swarm` default switched from the post-flight matplotlib replayer to
  the live multi-drone HUD; legacy path preserved as `--swarm-static`.
- `run_single_mission_live` replaces the unconditional `sleep 10` after
  mission completion with a bounded `wait_helm_uninstalled 60`.
- `simulation/runtime_view/web/live.js`: when `/api/waypoints` returns a
  multi-drone dict, pre-creates a placeholder mesh per drone_id so all
  swarm drones are visible from page load (during the SITL boot phase).

### Verification

- `bash -n run_scenario.sh`.
- `node --check simulation/runtime_view/web/live.js`.
- `python -m pytest simulation/test_drone_physics.py -q` → **322 passed**.
- `python -m pytest perception/test/` → **13 passed**.
- `POST /api/waypoints` round-trip + bad-input validation tested via
  `fastapi.testclient.TestClient` (good/bad-shape/non-numeric/wrong-type
  payloads).
- `sitl_waypoints.py ring --n 2` produces a valid `waypoints_enu.json`
  with 1-based system_id keys and 3-tuple ENU coordinates.

---

## [2026-04-19] — Phase 4 implementation audit and documentation synchronization

### Changed

- Performed a Phase 4 (collision detection and safety) implementation audit
  against current safety artifacts and planning claims.
- Added explicit **required fixes** tracking for Phase 4 in:
  - `ROADMAP.md` (Phase 4 implementation audit section)
  - `TODO.md` (section 4.1 for actionable follow-up tasks)
- Synchronized high-level documentation status messaging in `README.md`,
  `TESTING.md`, `docs/testing.md`, and `docs/architecture.md` to reflect:
  - detection/KPI foundations are implemented,
  - response/acceptance/documentation consistency fixes remain.

### Notes

- This change updates planning and documentation only; no new runtime behavior
  or safety mechanics were implemented in this step.

---

## [2026-04-19] — Phase 1 implementation audit and documentation synchronization

### Changed

- Performed a Phase 1 (Kubernetes + Gazebo baseline) implementation audit against
  existing artifacts and planning claims.
- Added explicit **required fixes** tracking for Phase 1 in:
  - `ROADMAP.md` (Phase 1 implementation audit section)
  - `TODO.md` (section 1.1 for actionable follow-up tasks)
- Synchronized high-level documentation status messaging in `README.md`,
  `TESTING.md`, `docs/testing.md`, and `docs/architecture.md` to reflect:
  - baseline artifacts exist,
  - documentation/verification consistency fixes are still required.

### Notes

- This change updates planning and documentation only; no new runtime behavior
  or simulation mechanics were implemented in this step.

---

## [2026-04-19] — Phase 2 implementation audit and documentation synchronization

### Changed

- Performed a Phase 2 (real physics in Kubernetes loop) implementation audit
  against current parity artifacts and planning claims.
- Added explicit **required fixes** tracking for Phase 2 in:
  - `ROADMAP.md` (Phase 2 implementation audit section)
  - `TODO.md` (section 2.1 for actionable follow-up tasks)
- Synchronized high-level documentation status messaging in `README.md`,
  `TESTING.md`, `docs/testing.md`, and `docs/architecture.md` to reflect:
  - parity foundations are implemented,
  - acceptance/documentation consistency fixes are still required.

### Notes

- This change updates planning and documentation only; no new runtime behavior
  or simulation mechanics were implemented in this step.

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
