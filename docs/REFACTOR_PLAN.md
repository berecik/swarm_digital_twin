# Refactoring Plan (v6): Paper-Aligned Delta Plan

**Reference paper:** Valencia et al., *An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains*, J. Intell. & Robot. Syst. 111:71 (2025), DOI: `10.1007/s10846-025-02276-7`

**Scope of this document:** delta plan listing only remaining gaps. This is a backlog — completed work lives in the codebase and `MAINTENANCE.log`.

---

## 1) Remaining gaps

| # | Area | Current status | Gap | Priority |
|:--|:---|:---|:---|:---|
| K1 | ArduPilot SITL in Docker compose | Placeholder containers (`tail -f /dev/null`) | Need actual ArduPilot SITL container with UDP ports (9003/9004, 14550/14555) for Gazebo | **P1** |
| K2 | Quadrotor Gazebo model (IRS-4) | Only `x500` basic model | Need SDF with correct mass, motor layout, PID params for paper Fig. 7 | **P1** |
| L1 | SRTM → STL mesh for Gazebo | `from_srtm()` downloads .hgt, no STL export | Need heightmap-to-STL converter (paper Section 2.1 pipeline) | **P1** |
| L2 | Terrain texture pipeline | Flat green ground plane | Need elevation-based coloring or satellite texture (paper Fig. 3) | **P2** |
| M1 | Wind node position subscription | `wind_node.py` uses hardcoded `pos = np.zeros(3)` | Need ROS sub to `/mavros/local_position/pose` for altitude-dependent wind | **P1** |
| M2 | Euler angle rate kinematics (Eq. 2) | Rotation matrix used, no explicit Euler rate utility | Need `euler_rates_from_body_rates()` for telemetry analysis | **P2** |
| N1 | Paper mission waypoint files (Table 4) | No mission files for 7 paper experiments | Need QGC WPL 110 files for missions 158/178/185, Python defs for Carolina/EPN | **P1** |

---

## 2) Active roadmap

### Phase K — Docker SITL and quadrotor Gazebo model (**P1**)

#### K1. ArduPilot SITL Docker container

- **Target files:** `docker-compose.yml`, `Dockerfile.sitl` (new)
- **Deliverables:**
  - Dockerfile building ArduPilot SITL (plane + copter)
  - Compose service `ardupilot_sitl` with proper UDP ports (9002/9003 for Gazebo, 14550 for QGC)
  - Health check: wait for MAVLink heartbeat on port 14550
- **Acceptance criteria:**
  - `docker compose up ardupilot_sitl` starts SITL and responds to MAVLink heartbeat within 30s

#### K2. IRS-4 quadrotor Gazebo model

- **Target files:** `gazebo/models/irs4_quadrotor/model.sdf`, `model.config`
- **Deliverables:**
  - SDF with 4 rotors, correct mass/inertia for paper's IRS-4 platform
  - ArduPilot plugin configured for copter frame
  - `.parm` file with PID gains matching paper's urban experiments
- **Acceptance criteria:**
  - Model loads in Gazebo, hover stable, responds to MAVLink commands

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

### Phase N — Paper mission definitions (**P1**)

#### N1. Mission waypoint files

- **Target files:** `missions/` directory (new)
- **Deliverables:**
  - QGC WPL 110 format files for Antisana missions (158, 178, 185)
  - Python dict definitions for Carolina-40/20 and EPN-30/20 quadrotor missions
  - GPS origins, altitudes, and approximate waypoints from paper Fig. 8/11
- **Acceptance criteria:**
  - Mission files load with `run_sitl_mission.sh --mission missions/fw_185.waypoints`

---

## 3) Verification matrix

| Item category | Verification method | Status |
|:---|:---|:---|
| ArduPilot SITL Docker (K1) | `docker compose up ardupilot_sitl` + heartbeat | **Pending** |
| IRS-4 Gazebo model (K2) | TestGazeboModels extended | **Pending** |
| STL terrain export (L1) | Test STL loads + elevation check | **Pending** |
| Wind node position-aware (M1) | Manual test with ROS 2 | **Pending** |
| Euler rate utility (M2) | TestEulerRates vs numerical differentiation | **Pending** |
| Mission files (N1) | TestMissionFiles — files exist + parseable | **Pending** |

---

## 4) Execution order

1. **Phase K (P1)** — ArduPilot SITL Docker + quadrotor Gazebo.
2. **Phase N (P1)** — mission waypoint files.
3. **Phase L (P1/P2)** — terrain mesh export + terrain visuals.
4. **Phase M (P1/P2)** — wind node improvements + telemetry kinematic utilities.

---

## 5) Notes

- This is a **delta backlog** only — completed work is not listed here.
- Any new phase must include: target files, measurable acceptance criteria, verification commands.
- When paper-aligned metrics are updated, update `TESTING.md` and `MAINTENANCE.log`.
- Real flight data from the paper is publicly available at `github.com/estebanvt/OSSITLQUAD`.
