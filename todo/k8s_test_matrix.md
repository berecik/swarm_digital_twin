# Full-System Kubernetes Validation

Detailed instructions for Phase 6 of the [ROADMAP](../ROADMAP.md).

## Goal

Build and automate a comprehensive test matrix that validates the full
K8s swarm simulation stack across all dimensions.

## Test Matrix

### Dimensions

| Dimension | Values |
|:---|:---|
| Drone count | 1, 3, 6, 12 |
| Terrain | flat, rolling (hills), steep (Antisana SRTM) |
| Wind | calm, crosswind (5 m/s), gusty (8+gusts), storm (12 m/s) |
| Mission | patrol (ring), lawnmower search, payload escort, heavy-lift mock |
| Faults | none, pod restart, packet loss (10%), delayed telemetry (200ms), sensor dropout |

### Total combinations

4 x 3 x 4 x 4 x 5 = **960 scenarios** (full matrix)

### Recommended CI subset

For nightly CI, run a reduced matrix (1 per dimension = 20 scenarios):

```
1-drone  + flat    + calm      + patrol     + none
3-drone  + rolling + crosswind + lawnmower  + pod restart
6-drone  + steep   + gusty     + escort     + packet loss
12-drone + flat    + storm     + heavy-lift + sensor dropout
```

Plus 16 critical-path scenarios covering the riskiest combinations.

## Acceptance KPIs

| KPI | Target | Source |
|:---|:---|:---|
| Mission completion rate | > 95% (calm), > 50% (storm) | Mission controller logs |
| Mean inter-drone separation | > 3.0 m | Separation monitor |
| Min inter-drone separation | > 1.5 m | Separation monitor |
| Collision count | 0 | Safety events |
| Near-miss count | < 3 per mission | Safety events |
| Trajectory RMSE (vs reference) | < 2.0 m (XY), < 1.0 m (Z) | Parity test |
| AGL violation count | 0 | Terrain collision detector |
| Failover recovery time | < 5 s | Event timestamps |
| Control loop jitter | < 5 ms at 50 Hz | Timing metrics |

## Scalability Gates

| Swarm size | Max pod startup time | Max scheduling delay |
|:---|:---|:---|
| 1 drone (4 pods) | 60 s | 5 s |
| 6 drones (24 pods) | 120 s | 10 s |
| 12 drones (48 pods) | 180 s | 15 s |

## Failure-Mode Verification

For each fault type, verify:

1. **Detection**: fault is detected within 2 s
2. **Response**: system enters safe degraded mode
3. **Recovery**: normal operation resumes within configured timeout
4. **Logging**: fault event + recovery are logged with timestamps

### Pod restart

```bash
kubectl delete pod <drone-pod> -n swarm-sim
# Expect: new pod starts, drone rejoins formation within 30 s
```

### Network disruption

```bash
# Inject 10% packet loss
kubectl exec <pod> -- tc qdisc add dev eth0 root netem loss 10%
# Expect: telemetry degrades but mission continues
```

## Acceptance Report Format

Each run produces:

```
reports/
  <scenario-id>/
    config.yaml          # Helm values + mission + wind + fault
    kpis.json            # Machine-readable KPI results
    summary.md           # Human-readable report
    telemetry/           # Raw telemetry files
      drone_1.npz
      drone_2.npz
      ...
    logs/                # Container logs
    plots/               # Auto-generated comparison plots
```

### `kpis.json` schema

```json
{
  "scenario_id": "6drone-steep-gusty-patrol-none",
  "git_revision": "abc1234",
  "verdict": "PASS",
  "mission_completion_rate": 1.0,
  "mean_separation_m": 4.2,
  "min_separation_m": 1.8,
  "collision_count": 0,
  "near_miss_count": 1,
  "trajectory_rmse_xy_m": 1.3,
  "trajectory_rmse_z_m": 0.6,
  "agl_violation_count": 0,
  "failover_recovery_s": null,
  "control_jitter_ms": 3.1
}
```

## Automation

```bash
# Run the CI subset
./run_scenario.sh --k8s-test-matrix --subset=ci

# Run a specific scenario
./run_scenario.sh --k8s-test-matrix \
  --drones=6 --terrain=steep --wind=gusty --mission=patrol --fault=none

# Generate acceptance report
python simulation/k8s_report.py reports/<scenario-id>/
```

## Acceptance Criteria

- [x] CI subset (20 scenarios) runs and reports KPIs — Python pipeline:
      `python -m simulation.acceptance_matrix --subset ci` (also
      `./run_scenario.sh --acceptance-matrix ci`). Each scenario
      produces `reports/<scenario_id>/{kpis.json, summary.md,
      config.toml}` matching the documented schema. Nightly K8s lane
      with real pod startup, fault injection, and Gazebo-runtime
      parity is the next iteration.
- [/] All critical-path scenarios pass acceptance thresholds — 15/20
      pass the soft Python-pipeline envelope today; the 5 failing rows
      are all 12-drone scenarios where the default PD controller +
      flocking-disabled swarm sim hits chaotic transients. Production
      K8s+PX4 thresholds (preserved as `*_K8S` constants in
      `acceptance_report.py`) are the gate for the nightly lane.
- [/] Acceptance reports are archived and trend-tracked — per-scenario
      report tree is produced; CI archival + trend tracking is the
      next iteration.
- [x] `TESTING.md` documents the test matrix and how to run it — see
      the matrix-runner section there and the `Phase 6` row in the
      status table.
