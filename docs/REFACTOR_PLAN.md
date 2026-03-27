# Refactoring Plan (v7): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

---

## 1) Remaining gaps

| # | Area | Current status | Gap | Priority |
|:--|:---|:---|:---|:---|
| L1 | SRTM → STL mesh for Gazebo | `from_srtm()` downloads .hgt, no STL export | Need heightmap-to-STL converter (paper Section 2.1 pipeline) | **P1** |
| L2 | Terrain texture pipeline | Flat green ground plane | Need elevation-based coloring or satellite texture (paper Fig. 3) | **P2** |
| M1 | Wind node position subscription | `wind_node.py` uses hardcoded `pos = np.zeros(3)` | Need ROS sub to `/mavros/local_position/pose` for altitude-dependent wind | **P1** |
| M2 | Euler angle rate kinematics (Eq. 2) | Rotation matrix used, no explicit Euler rate utility | Need `euler_rates_from_body_rates()` for telemetry analysis | **P2** |

---

## 2) Active roadmap

### Phase L — Terrain mesh pipeline (**P1/P2**)

#### L1. SRTM heightmap to STL converter

- **Target files:** `simulation/terrain.py` (new `export_stl()` method)
- **Deliverables:**
  - `TerrainMap.export_stl(path)` — writes binary STL mesh from elevation grid
  - Proper triangle tessellation of the heightmap grid
  - Scale factor to match Gazebo world coordinates
- **Acceptance criteria:**
  - Generated STL loads in Gazebo and matches SRTM elevations within 1m

#### L2. Elevation-based terrain coloring (optional)

- **Target files:** `gazebo/worlds/antisana.world` or separate material file
- **Deliverables:**
  - Height-based color gradient (green lowlands → brown highlands → white peaks)
  - Or satellite texture UV-mapped onto terrain mesh
- **Acceptance criteria:**
  - Visual terrain distinguishable from flat ground in Gazebo

### Phase M — Wind node and kinematic utilities (**P1/P2**)

#### M1. Position-aware wind node

- **Target files:** `gazebo/scripts/wind_node.py`
- **Deliverables:**
  - Subscribe to `/mavros/local_position/pose` for drone position
  - Wind force varies with altitude (density changes) and optionally terrain proximity
- **Acceptance criteria:**
  - Wind force at 4500m differs from sea level by the ISA density ratio

#### M2. Euler angle rate utility (Eq. 2)

- **Target files:** `simulation/drone_physics.py`
- **Deliverables:**
  - `euler_rates_from_body_rates(phi, theta, p, q, r)` implementing paper Eq. 2
  - Useful for telemetry analysis and comparison with real ATT log messages
- **Acceptance criteria:**
  - Test verifies `euler_rates` matches numerical differentiation of Euler angles from rotation matrix

---

## 3) Verification matrix

| Item category | Verification method | Status |
|:---|:---|:---|
| STL terrain export (L1) | Test STL loads + elevation check | **Pending** |
| Terrain coloring (L2) | Visual check in Gazebo | **Pending** |
| Wind node position-aware (M1) | Manual test with ROS 2 | **Pending** |
| Euler rate utility (M2) | TestEulerRates vs numerical differentiation | **Pending** |

---

## 4) Execution order

1. **Phase L (P1/P2)** — terrain mesh export + terrain visuals.
2. **Phase M (P1/P2)** — wind node improvements + telemetry kinematic utilities.

---

## 5) Notes

- This is a **delta backlog** only — completed work is not listed here.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
