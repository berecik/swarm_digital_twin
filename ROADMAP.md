# ROADMAP.md — Kubernetes + Gazebo Realistic Swarm Simulation

This roadmap defines delivery phases for using Gazebo as a Kubernetes-native playground for realistic multi-drone simulation.

## Status Legend

- `[ ]` to do
- `[/]` in progress
- `[x]` done / cover by tests

## Phase 1 — Kubernetes + Gazebo Baseline (Test Coverage: 0%)

- [ ] **K8s simulation namespace and lifecycle**: Standardize namespace, resource quotas, and cleanup for repeated simulation runs.
- [ ] **Helm profile for Gazebo playground mode**: Add values profile enabling Gazebo/SITL-oriented workload topology for multi-drone runs.
- [ ] **Service topology for swarm networking**: Validate ROS 2 + Zenoh + MAVLink addressing model for pod-to-pod communication in-cluster.
- [ ] **Operational runbook**: Document one-command launch, stop, and reset workflow for playground sessions.

## Phase 2 — Real Physics in Kubernetes Loop (Test Coverage: 0%)

- [ ] **Physics parity contract**: Define required parity between standalone physics (`simulation/drone_physics.py`) and Gazebo/K8s runtime outputs.
- [ ] **Dynamics fidelity profile**: Enable realistic mass, inertia, motor lag, drag/lift, and atmosphere settings in cluster runs.
- [ ] **Timing determinism checks**: Validate simulation `dt`, sensor rates, and controller update rates under K8s scheduling pressure.
- [ ] **Telemetry truth pipeline**: Ensure synchronized logging of state, actuator commands, and derived metrics for post-run validation.

## Phase 3 — Terrain Model Integration (Test Coverage: 0%)

- [ ] **Terrain source workflow**: Support flat, grid, and STL terrain assets with reproducible versioned datasets.
- [ ] **Height-query consistency**: Align terrain elevation lookups across Gazebo world, physics replay, and validation tooling.
- [ ] **Mission safety envelope over terrain**: Enforce AGL/clearance constraints against terrain mesh during autonomous missions.
- [ ] **Visualization alignment**: Keep live and post-flight viewers consistent with terrain-enabled scenarios.

## Phase 4 — Collision Detection & Safety (Test Coverage: 0%)

- [ ] **Inter-drone collision detection**: Add deterministic minimum-separation checks and incident logging.
- [ ] **Drone-terrain collision detection**: Trigger collision/near-miss events based on terrain and pose trajectory.
- [ ] **Obstacle contact handling**: Define safety response (`HOVER`, `RTL`, emergency stop/detach where applicable) after collision events.
- [ ] **Safety KPIs**: Track collision count, near-miss count, minimum separation, and safety-recovery latency.

## Phase 5 — Wind Simulation in Kubernetes (Test Coverage: 0%)

- [ ] **Wind model mapping**: Map constant, Dryden turbulence, and flight-log replay profiles into Gazebo/K8s scenario definitions.
- [ ] **Distributed wind injection**: Support spatially varying wind fields for multi-drone area operations.
- [ ] **Wind reproducibility**: Fixed seeds and profile snapshots for deterministic reruns.
- [ ] **Wind stress envelopes**: Define pass/fail gates for mission completion under baseline, crosswind, gusty, and storm-like profiles.

## Phase 6 — Full-System Kubernetes Validation (Test Coverage: 0%)

- [ ] **Test scenario matrix**: Build matrix across drone counts, terrain complexity, wind class, mission type, and failure injections.
- [ ] **Scalability gates**: Validate target swarm sizes (including minimum 6-agent heavy-lift control assumptions where relevant).
- [ ] **Failure-mode verification**: Validate behavior under pod restarts, network jitter/loss, sensor dropout, and delayed control loops.
- [ ] **Acceptance report automation**: Generate run artifacts (metrics, plots, logs) and machine-readable pass/fail summaries.

## Phase 7 — Live View & Replay Backlog Migration (from former `REFACTOR_PLAN`) (Test Coverage: 0%)

- [ ] **Terrain rendering in live viewer**: Replace flat Three.js grid with SRTM/STL mesh rendering aligned with `simulation/terrain.py`.
- [ ] **Real mission thumbnails**: Replace placeholder Pillow-generated mission thumbnails with captures from real SITL runs.
- [ ] **Authentication and multi-user readiness**: Add auth/CSRF/session isolation if runtime view is exposed beyond localhost.
- [ ] **Swarm SITL live view parity**: Enable `--swarm` SITL telemetry forwarding for per-drone live browser rendering.
- [ ] **`.BIN` replay in web viewer**: Extend `POST /api/load` pipeline to parse and replay ArduPilot DataFlash `.BIN` logs.

## Migrated Baseline (from former `REFACTOR_PLAN`)

- [x] Paper equations/tables alignment is implemented and tracked in `CHANGELOG.md`.
- [x] Live Run-time View, multi-drone telemetry demux, post-flight replay, launch API, and DataFlash recording are delivered.
- [x] Historical verification commands from the prior plan are preserved in `TESTING.md` and `CHANGELOG.md` entries.

## Definition of Done (Program-Level)

- [ ] End-to-end Kubernetes scenario can launch and execute realistic Gazebo missions reproducibly.
- [ ] Physics, terrain, collision, and wind behaviors are covered by automated tests and scenario validations.
- [ ] Safety and mission KPIs are defined, measured, and enforced by CI-compatible gates.
- [ ] Documentation (`README.md`, `TESTING.md`, `AGENTS.md`, `ROADMAP.md`, `TODO.md`) is synchronized with implementation status.
