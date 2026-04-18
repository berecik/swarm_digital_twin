# Wind Simulation in Kubernetes

Detailed instructions for Phase 5 of the [ROADMAP](../ROADMAP.md).

## Goal

Map the standalone wind models into K8s Gazebo scenarios with
reproducible, spatially varying disturbances.

## Wind Profile Types

| Profile | Source | Standalone | K8s/Gazebo |
|:---|:---|:---|:---|
| Constant | Config param | `WindField(wind_speed, direction)` | Gazebo `<wind>` element |
| Dryden turbulence | Random seed | `WindField(turbulence_type="dryden")` | Gazebo wind plugin + seed |
| Flight-log replay | `.csv` file | `WindField.from_log(path)` | ConfigMap with wind timeseries |
| Spatial gradient | Grid config | New: `WindField(gradient=...)` | Gazebo multi-zone wind plugin |

## Scenario Configuration

Wind profiles are specified in the Helm values or a ConfigMap:

```yaml
# values-wind-crosswind.yaml
wind:
  profile: constant
  speed_ms: 5.0
  direction: [1.0, 0.3, 0.0]  # ENU
  seed: 42  # for Dryden/gust modes
```

For Dryden turbulence:

```yaml
wind:
  profile: dryden
  base_speed_ms: 3.0
  turbulence_intensity: 0.15
  seed: 42
```

## Deterministic Reproducibility

- All random wind profiles must accept a `seed` parameter
- Two runs with the same seed must produce identical wind timeseries
- Verify with: `wind_profile_run1.csv` vs `wind_profile_run2.csv` → bit-identical

## Wind Stress Envelopes

| Profile | Max wind (m/s) | Mission completion | Max attitude error |
|:---|:---|:---|:---|
| Calm | 0–1 | > 99% | < 3 deg |
| Crosswind | 3–5 | > 95% | < 8 deg |
| Gusty | 5–8 (gusts to 12) | > 85% | < 15 deg |
| Storm-like | 10–15 | > 50% (abort OK) | < 25 deg |

## Spatial Wind Gradients

For multi-drone operations over large areas, wind should vary spatially:

```python
def wind_at_position(pos_enu, t):
    """Return wind vector at position and time."""
    base = np.array([5.0, 0.0, 0.0])  # base wind
    gradient = 0.1 * pos_enu[0]  # increases with East distance
    return base + np.array([gradient, 0.0, 0.0])
```

## Acceptance Criteria

- [ ] All four wind profiles executable in K8s Gazebo runs
- [ ] Seeded Dryden produces identical results across re-runs
- [ ] Stress envelope KPIs met for each profile class
- [ ] Spatial gradient supported for multi-drone distributed missions
- [ ] Wind data logged and available in acceptance report artifacts
