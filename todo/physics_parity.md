# Real Physics Parity in Kubernetes

Detailed instructions for Phase 2 of the [ROADMAP](../ROADMAP.md).

## Goal

Ensure the Gazebo/K8s simulation produces results that match the standalone
Python physics engine (`simulation/drone_physics.py`) within defined
tolerances.

## Physics Parameters to Mirror

| Parameter | Standalone source | K8s/Gazebo target |
|:---|:---|:---|
| Mass | `DroneParams.mass` | Gazebo SDF `<inertial><mass>` |
| Inertia tensor | `DroneParams.I_body` | Gazebo SDF `<inertia>` |
| Motor response | `MotorModel.tau_spinup` | Gazebo motor plugin time constant |
| Drag coefficient | `AeroCoefficients.C_D` | Gazebo `LiftDragPlugin` `cda` |
| Lift coefficient | `AeroCoefficients.C_L_alpha` | Gazebo `LiftDragPlugin` `cla` |
| Atmosphere density | `Atmosphere.rho` | Gazebo world `<atmosphere>` |
| Gravity | `GRAVITY` (9.81) | Gazebo `<physics><gravity>` |
| Battery capacity | `BatteryModel.capacity_ah` | Gazebo battery plugin |

## Parity Test Suite

For each parameter set, run both pipelines on the same waypoint mission:

```bash
# Standalone
python simulation/drone_scenario.py --benchmark moderate

# K8s (extract telemetry)
./run_scenario.sh --single-live
# After mission: download logs
kubectl cp swarm-sim/<pod>:/sitl/logs/. logs/k8s_run/
```

Compare trajectories:

```python
from validation import compute_rmse
rmse = compute_rmse(standalone_positions, k8s_positions)
assert rmse.x < 2.0  # metres
assert rmse.z < 1.0  # metres (altitude is tighter)
```

## Acceptance Thresholds

| Metric | Threshold |
|:---|:---|
| Position RMSE (XY) | < 2.0 m |
| Position RMSE (Z) | < 1.0 m |
| Attitude RMSE (roll/pitch) | < 5 deg |
| Energy consumption delta | < 15% |
| Control loop jitter | < 5 ms at 50 Hz |

## Files to Create/Modify

- `simulation/k8s_parity_test.py` — automated comparison script
- `helm/swarm-digital-twin/values-parity.yaml` — values matching standalone params
- `gazebo/models/x500/model.sdf` — verify inertia/mass match `DroneParams`

## Acceptance Criteria

- [ ] Parity test passes for `moderate` and `crosswind` benchmark profiles
- [ ] Test is runnable in CI (with K8s cluster available)
- [ ] Results are logged to `MAINTENANCE.log`
