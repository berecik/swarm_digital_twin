# Collision Detection & Safety

Detailed instructions for Phase 4 of the [ROADMAP](../ROADMAP.md).

## Goal

Detect and respond to inter-drone and drone-terrain collisions with
deterministic safety responses and measurable KPIs.

## Inter-Drone Collision Detection

### Separation monitor

Add a `SeparationMonitor` that runs at the control loop rate (50 Hz):

```python
class SeparationMonitor:
    def __init__(self, min_separation: float = 1.5,
                 near_miss_threshold: float = 3.0):
        self.min_separation = min_separation
        self.near_miss_threshold = near_miss_threshold
        self.events: list = []

    def check(self, positions: dict[int, np.ndarray], t: float) -> None:
        for (i, pi), (j, pj) in combinations(positions.items(), 2):
            dist = np.linalg.norm(pi - pj)
            if dist < self.min_separation:
                self.events.append(CollisionEvent(t, i, j, dist))
            elif dist < self.near_miss_threshold:
                self.events.append(NearMissEvent(t, i, j, dist))
```

### Location

- Standalone: `simulation/safety.py` (new file)
- K8s: runs inside the swarm control node (`swarm_control/src/safety.rs`)
- Parity: both implementations must produce identical event counts for
  the same trajectory data

## Drone-Terrain Collision Detection

Check AGL at every control step:

```python
agl = pos[2] - terrain.get_elevation(pos[0], pos[1])
if agl < 0:
    events.append(TerrainCollisionEvent(t, drone_id, pos, agl))
elif agl < min_clearance:
    events.append(ClearanceViolationEvent(t, drone_id, pos, agl))
```

## Safety Response Playbook

| Event | Severity | Response |
|:---|:---|:---|
| Near miss (< 3.0 m) | Warning | Log event, increase separation gain |
| Collision (< 1.5 m) | Critical | Trigger HOVER mode, log incident |
| Terrain clearance (< MIN_AGL) | Warning | Increase altitude target |
| Terrain collision (AGL < 0) | Critical | Emergency LAND, log incident |

## Safety KPIs

| KPI | Target | Measured by |
|:---|:---|:---|
| Collision count | 0 per mission | `SeparationMonitor.events` |
| Near-miss count | < 3 per mission | `SeparationMonitor.events` |
| Min separation | > 1.5 m | `min(pairwise_distances)` |
| AGL violation count | 0 per mission | Terrain collision detector |
| Safety recovery latency | < 2 s | Time from event to safe mode |

## Test Scenarios

- **Crossing trajectories**: two drones fly head-on, verify avoidance
- **Formation compression**: reduce formation radius until near-miss triggers
- **Terrain clipping**: fly low over steep terrain, verify AGL enforcement
- **Obstacle avoidance**: static obstacle in flight path

## Acceptance Criteria

- [x] `SeparationMonitor` detects collisions and near-misses — 10 tests in `TestSafetyMonitor`
- [x] `TerrainMonitor` catches AGL violations and terrain collisions
- [x] `SafetyReport` aggregates KPIs with `is_safe()`, `summary()`, `to_dict()`
- [x] Full swarm benchmark produces a valid report (verified in `test_full_swarm_simulation_produces_report`)
- [ ] Safety response triggers within 2 s of event detection (requires PX4 integration)
- [ ] KPI export in acceptance report JSON format (structure defined in `k8s_test_matrix.md`)

## Verification Commands

```bash
# All safety monitor tests
.venv/bin/python -m pytest simulation/test_drone_physics.py::TestSafetyMonitor -v

# Specific checks
.venv/bin/python -m pytest simulation/test_drone_physics.py::TestSafetyMonitor -v -k "collision"
.venv/bin/python -m pytest simulation/test_drone_physics.py::TestSafetyMonitor -v -k "terrain"
.venv/bin/python -m pytest simulation/test_drone_physics.py::TestSafetyMonitor -v -k "report"
```
