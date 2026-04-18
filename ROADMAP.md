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

- [ ] K8s simulation namespace and lifecycle — standardize namespace, resource
      quotas, and cleanup for repeated simulation runs.
- [ ] Helm profile for Gazebo playground mode — add `values-playground.yaml`
      enabling Gazebo/SITL-oriented workload topology for multi-drone runs.
- [ ] Service topology for swarm networking — validate ROS 2 + Zenoh +
      MAVLink addressing model for pod-to-pod communication in-cluster.
- [ ] Operational runbook — document one-command launch, stop, and reset
      workflow for playground sessions.

## Phase 2 — Real Physics in Kubernetes Loop

**Instructions:** [`todo/physics_parity.md`](todo/physics_parity.md)

- [ ] Physics parity contract — define required parity between standalone
      physics (`simulation/drone_physics.py`) and Gazebo/K8s runtime outputs.
      Mirror mass, inertia, motor lag, drag/lift, atmosphere settings.
- [ ] Parity test suite — compare Gazebo traces against standalone references
      with pass/fail thresholds (RMSE < 2.0 m XY, < 1.0 m Z).
- [ ] Timing determinism checks — validate `dt`, sensor rates, and
      controller update rates under K8s scheduling pressure.
- [ ] Telemetry truth pipeline — synchronized logging of state, actuator
      commands, and derived metrics for post-run validation.

## Phase 3 — Terrain Model Integration

**Instructions:** [`todo/terrain_integration.md`](todo/terrain_integration.md)

- [ ] Terrain source workflow — support flat, grid, STL, and SRTM terrain
      assets with reproducible versioned datasets and a manifest file.
- [ ] Height-query consistency — align terrain elevation lookups across
      Gazebo world, physics replay, validation tooling, and live viewer.
- [ ] Mission safety envelope over terrain — enforce AGL/clearance
      constraints against terrain mesh during autonomous missions.
- [ ] Regression tests — flat + rolling + steep terrain profiles all pass.

## Phase 4 — Collision Detection & Safety

**Instructions:** [`todo/collision_detection.md`](todo/collision_detection.md)

- [x] Inter-drone collision detection — `SeparationMonitor` with near-miss
      (< 3.0 m) and collision (< 1.5 m) events. 10 tests.
- [x] Drone-terrain collision detection — `TerrainMonitor` with AGL-based
      terrain collision and clearance violation events.
- [x] Safety KPIs — `SafetyReport` with collision count, near-miss count,
      min separation, terrain collisions, clearance violations, `is_safe()`,
      `summary()`, `to_dict()`.
- [ ] Safety response playbook — Warning → HOVER → RTL / emergency stop
      with configurable thresholds and logged incidents (requires PX4 integration).

## Phase 5 — Wind Simulation in Kubernetes

**Instructions:** [`todo/wind_simulation.md`](todo/wind_simulation.md)

- [ ] Wind model mapping — map constant, Dryden turbulence, and flight-log
      replay profiles into Gazebo/K8s scenario definitions.
- [ ] Distributed wind injection — spatially varying wind fields for
      multi-drone area operations.
- [ ] Wind reproducibility — fixed seeds and profile snapshots for
      deterministic reruns (seeded Dryden must be bit-identical).
- [ ] Wind stress envelopes — pass/fail gates for baseline (calm),
      crosswind (5 m/s), gusty (8+), and storm-like (12 m/s) profiles.

## Phase 6 — Full-System Kubernetes Validation

**Instructions:** [`todo/k8s_test_matrix.md`](todo/k8s_test_matrix.md)

- [ ] Test scenario matrix — 4 drone counts x 3 terrains x 4 wind profiles
      x 4 mission types x 5 fault types (960 combinations, CI subset of 20).
- [ ] Scalability gates — validate 1/3/6/12 drone swarm sizes with pod
      startup time and scheduling delay thresholds.
- [ ] Failure-mode verification — pod restart, packet loss, network delay,
      sensor dropout. Each must detect, respond, and recover within limits.
- [ ] Acceptance report automation — machine-readable KPIs (`kpis.json`),
      human report (`summary.md`), plots, logs, replay artifacts.

## Phase 7 — Live View & Replay Backlog

**Instructions:** [`todo/live_view_backlog.md`](todo/live_view_backlog.md)

- [ ] Terrain rendering in live viewer — replace flat Three.js grid with
      SRTM/STL mesh rendering aligned with `simulation/terrain.py`.
- [ ] Real mission thumbnails — replace Pillow placeholders with captures
      from real SITL runs.
- [ ] Authentication and multi-user — auth/CSRF/session isolation if the
      runtime view is exposed beyond localhost.
- [ ] Swarm SITL live view — wire `--swarm-live` mode with per-drone
      telemetry forwarding from `sitl_orchestrator.py`.
- [ ] `.BIN` replay in web viewer — extend `POST /api/load` to parse and
      replay ArduPilot DataFlash `.BIN` logs.

---

## Delivered Baseline

- [x] Paper equations 1–7, tables 1–5 — fully implemented and verified.
      See [`CHANGELOG.md`](CHANGELOG.md).
- [x] Live Run-time View — FastAPI + Three.js, multi-drone demux,
      post-flight replay, browser launch, DataFlash recording.
      299 tests, 0 warnings.
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
