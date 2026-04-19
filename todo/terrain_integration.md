# Terrain Model Integration

Detailed instructions for Phase 3 of the [ROADMAP](../ROADMAP.md).

## Goal

Support realistic terrain in K8s Gazebo simulations with consistent
elevation queries across all components.

## Terrain Source Workflow

### Supported formats

| Source | Pipeline | Output |
|:---|:---|:---|
| Flat | No-op (default) | `TerrainMap.flat()` |
| SRTM | `terrain.py` `from_srtm(lat, lon, size)` | Elevation grid |
| STL mesh | `terrain.py` `export_stl()` | Gazebo collision mesh |
| Analytical | `TerrainMap.from_function(fn)` | Custom terrain |

### Asset versioning

Store terrain assets in `gazebo/worlds/terrain/` with a manifest:

```yaml
# gazebo/worlds/terrain/manifest.yaml
antisana:
  source: SRTM
  lat: -0.4838
  lon: -78.1412
  size_km: 5
  resolution_m: 30
  stl: antisana.stl
  texture: antisana_satellite.png
  checksum: sha256:abc123...
```

### Gazebo world integration

Add terrain mesh to the world file:

```xml
<model name="terrain">
  <static>true</static>
  <link name="terrain_link">
    <collision name="terrain_collision">
      <geometry><mesh><uri>model://terrain/antisana.stl</uri></mesh></geometry>
    </collision>
    <visual name="terrain_visual">
      <geometry><mesh><uri>model://terrain/antisana.stl</uri></mesh></geometry>
      <material><script>
        <uri>file://media/materials/scripts</uri>
        <name>AntisanaTerrain/SatelliteTextured</name>
      </script></material>
    </visual>
  </link>
</model>
```

## Height-Query Consistency

All components must agree on terrain elevation at any (x, y):

| Component | Method | Must match |
|:---|:---|:---|
| `drone_physics.py` | `terrain.get_elevation(x, y)` | Reference |
| Gazebo | `gz::physics::HeightmapShape` | Within 0.5 m |
| Live viewer | `live.js` terrain mesh | Visual only |
| Validation | `validation.py` AGL check | Exact match with reference |

## AGL Enforcement

Mission controllers must enforce minimum AGL:

```python
agl = position[2] - terrain.get_elevation(position[0], position[1])
if agl < MIN_AGL:
    trigger_safety_response("terrain_clearance_violation")
```

## Acceptance Criteria

- [x] SRTM terrain loads via `terrain.load_from_manifest("antisana")` —
      manifest entry resolves through `TerrainMap.from_srtm`, with the
      synthetic-tile fallback in `_create_synthetic_hgt` keeping CI
      offline-safe.
- [x] `get_elevation()` matches the STL export pipeline within 0.5 m —
      enforced by `TestTerrainParity.test_export_roundtrip_parity` over
      100 deterministic samples per manifest entry. Live-Gazebo runtime
      parity (`gz::physics::HeightmapShape::HeightAt`) remains opt-in
      (no Gazebo CI lane today).
- [/] AGL violation triggers safety response in mission controller —
      *detection* shipped: `run_simulation` / `run_trajectory_tracking` /
      `run_swarm_simulation` accept a `terrain_monitor=` and emit
      `ClearanceViolationEvent` / `TerrainCollisionEvent` per step. The
      *response* (HOVER/RTL) belongs to Phase 4 once PX4 integration is
      in place.
- [x] Flat, rolling, and steep terrain profiles all pass regression
      tests — `TestTerrainRegression` covers each profile with the
      same patrol mission, asserting (no terrain collision) ×
      (cruise AGL > 5 m after climb-up) × (in-loop monitor matches
      post-hoc walker).
- [ ] Live viewer renders terrain mesh (Three.js) aligned with
      simulation — Phase 7 follow-up.
