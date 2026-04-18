# Gazebo as Kubernetes Playground — Realistic Swarm Simulation Scenario

Part of [ROADMAP Phase 1](../ROADMAP.md). See also
[`k8s_namespace_lifecycle.md`](k8s_namespace_lifecycle.md) for namespace setup.

This scenario describes how to run Gazebo as a Kubernetes-native playground
for realistic drone swarm simulation, including terrain, wind, collision
detection, and full validation.

## 1. Objective

Validate that Kubernetes orchestration is production-ready for realistic swarm simulation with:

- Gazebo SITL in-cluster execution
- physics fidelity checks against standalone reference models
- terrain-aware mission execution
- inter-drone and drone-terrain collision detection
- configurable wind disturbance models
- repeatable, metrics-driven pass/fail criteria

## 2. Preconditions

- Kubernetes cluster available (`minikube`, `kind`, `k3s`, EKS/GKE/AKS)
- Helm chart available: `helm/swarm-digital-twin/`
- Container images available in registry or preloaded into cluster runtime
- `kubectl` + `helm` configured to target the desired cluster
- Optional: external visualization endpoint forwarding for live monitoring

## 3. Scenario Topology

- **Pods per drone**: SITL + companion/control stack (ROS 2 + Zenoh bridge)
- **Shared services**: telemetry gateway, mission control API, optional visualization
- **ConfigMaps/Secrets**: mission profile, wind profile, terrain asset metadata, safety thresholds
- **Persistent artifacts**: logs, metrics, telemetry dumps, replay files

## 4. Execution Flow

### Step A — Cluster Preparation

1. Create/verify dedicated simulation namespace.
2. Apply resource quotas and limit ranges to avoid noisy-neighbor effects.
3. Verify node capacity for requested drone count.

Expected outcome:
- Namespace ready and resource policy applied without admission errors.

### Step B — Deploy Playground Stack

1. Choose a values profile (local/cloud, swarm size, wind/terrain presets).
2. Deploy via Helm release.
3. Wait for all pods to become `Ready`.

Expected outcome:
- All required services and pods healthy; no crash loops; stable startup logs.

### Step C — Load Mission + Terrain + Wind

1. Load terrain configuration (flat/grid/STL).
2. Load mission pattern (patrol/lawnmower/escort/heavy-lift mock).
3. Apply wind profile (constant/Dryden/replay) with deterministic seed when needed.

Expected outcome:
- Mission starts with confirmed terrain and wind profile tags in telemetry metadata.

### Step D — Run Simulation and Safety Monitoring

1. Execute mission for planned duration.
2. Continuously monitor:
   - drone pose/velocity telemetry
   - AGL and terrain-clearance margins
   - inter-drone separation metrics
   - collision and near-miss event streams
   - control-loop timing jitter

Expected outcome:
- Mission runs end-to-end with no unhandled safety events.

### Step E — Fault Injection Validation

1. Inject one fault at a time:
   - pod restart
   - packet loss / network delay
   - temporary sensor dropout
2. Observe fail-operational response and recovery time.

Expected outcome:
- System enters safe degraded mode and recovers within configured thresholds.

### Step F — Artifact Collection & Teardown

1. Collect logs, metrics, and telemetry traces.
2. Export replay artifacts (`.npz`, `.BIN` if enabled).
3. Generate machine-readable and markdown summary.
4. Tear down release and verify clean namespace state.

Expected outcome:
- Reproducible artifact bundle and no residual leaked resources.

## 5. Physics Realism Validation Gates

- **Trajectory RMSE**: Gazebo vs standalone reference within configured envelope.
- **Attitude stability**: roll/pitch/yaw variance under wind remains bounded.
- **Energy plausibility**: battery/power trends align with mission profile expectations.
- **Timing fidelity**: control and sensor update jitter below defined limit.

## 6. Terrain & Collision Gates

- **AGL compliance**: no unauthorized terrain penetration.
- **Clearance compliance**: minimum terrain clearance respected per mission policy.
- **Separation compliance**: minimum inter-drone distance always above threshold.
- **Collision handling**: on collision or near-miss, safety mode transition is triggered and logged.

## 7. Wind Robustness Gates

- Baseline, crosswind, gusty, and storm-like profiles all executable.
- Deterministic re-run produces equivalent KPI ranges for seeded profiles.
- Mission completion and safety KPIs remain above acceptance thresholds per profile.

## 8. Test Matrix (Minimum)

- Drone count: `1`, `3`, `6`, `12`
- Terrain: flat, rolling, steep
- Wind: calm, crosswind, gusty, storm-like
- Mission: patrol, lawnmower, escort, heavy-lift mock
- Faults: restart, packet loss, delayed telemetry, sensor dropout

## 9. Acceptance Report Template

For each run, report:

- scenario ID and git revision
- cluster profile and Helm values used
- mission + terrain + wind configuration
- KPIs: mission success, RMSE, min separation, collision/near-miss count, AGL violations, recovery time
- verdict: `PASS` / `FAIL`
- links/paths to logs, metrics, and replay artifacts

## 10. Done Criteria

This scenario is considered ready when:

- all test matrix rows have defined pass/fail thresholds
- nightly execution can run the selected matrix subset automatically
- acceptance report generation is reproducible and archived
- operational documentation is synced in `ROADMAP.md`, `TODO.md`, `TESTING.md`, and `README.md`

## 11. Related Instructions

- **Live view & replay backlog:** [`live_view_backlog.md`](live_view_backlog.md)
- **Collision detection & safety:** [`collision_detection.md`](collision_detection.md)
- **Wind simulation:** [`wind_simulation.md`](wind_simulation.md)
- **Full test matrix:** [`k8s_test_matrix.md`](k8s_test_matrix.md)
- **Historical verification:** see `TESTING.md` and `CHANGELOG.md`
