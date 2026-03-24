# Refactoring Plan: Align with Open-Source UAV Digital Twin Framework

**Reference:** Valencia et al., "An Open-source UAV Digital Twin framework: A Case Study on Remote Sensing in the Andean Mountains," *J. Intell. & Robot. Syst.* 111:71 (2025). DOI: 10.1007/s10846-025-02276-7

**Goal:** Upgrade our standalone physics engine and simulation pipeline to match the fidelity level described in the paper, adapted for our swarm/quadrotor use case.

---

## Gap Analysis: Current State vs Paper

| Aspect | Our Current State | Paper's Framework | Priority |
|:---|:---|:---|:---|
| Aerodynamic drag | Linear drag `F = -kd·v` (world frame) | Quadratic drag `F_D = 0.5·ρ·A·C_D·V²` with AoA-dependent coefficients (Eq. 5) | **P0** |
| Aerodynamic lift | None | `F_L = 0.5·ρ·A·C_L·V²` with AoA-dependent coefficients, stall model (Eq. 6, Table 3) | **P1** |
| Wind perturbation | None | Single/multi-axis perturbation force from real altitude data; `ΔF_W = \|\|F_D + F_L\|\|` (Eq. 7) | **P0** |
| Air density | Constant (implicit) | Altitude-dependent ρ (0.83 kg/m³ at 4500m ASL vs 1.225 at sea level) | **P1** |
| Inertia tensor | Diagonal only `diag([Jx, Jy, Jz])` | Full 3×3 with products of inertia (Jxy, Jxz, Jyz) and Γ terms (Eq. 4) | **P1** |
| Body-frame dynamics | Forces computed in world frame | Newton's 2nd law in body frame with cross-coupling terms (Eq. 3) | **P1** |
| Euler kinematics | First-order rotation matrix | Proper `[φ̇,θ̇,ψ̇] = T(φ,θ)·[p,q,r]` (Eq. 2) | **P2** |
| Terrain/environment | Empty space (z≥0 ground plane) | 3D height map from satellite data (SRTM → BlenderGIS → STL → Gazebo) | **P2** |
| Ardupilot SITL | None | Full SITL with Pixhawk firmware, PID tuning, MAVLink | **P2** |
| QGroundControl | None | Mission planning, flight log, MAVLink bridge | **P3** |
| Gazebo integration | None (matplotlib viz only) | Full Gazebo world with terrain, collisions, clouds, LiftDrag plugin | **P2** |
| Flight log import | None | Parse Ardupilot .bin/.log for validation against real data | **P1** |
| Validation metrics | None | RMSE in X/Y/Z, error percentiles, boxplots (Table 5, Fig. 13) | **P1** |
| Vehicle types | Quadrotor only | Quadrotor + fixed-wing (configurable aerodynamic profiles) | **P3** |

---

## Phase 1: Physics Engine Upgrade (P0 + P1)

### 1.1 Quadratic Aerodynamic Drag Model

**File:** `simulation/drone_physics.py`

**What to change:** Replace the linear drag `F = -kd·v` with proper quadratic aerodynamic drag computed in the body frame.

**Implementation:**

```python
@dataclass
class AeroCoefficients:
    """Aerodynamic coefficients, derived from CFD or lookup tables."""
    reference_area: float = 0.04      # m² (quadrotor frontal area)
    C_D: float = 1.0                  # drag coefficient (constant for quad, or f(AoA) for fixed-wing)
    C_L: float = 0.0                  # lift coefficient (0 for quad, f(AoA) for fixed-wing)
    C_M: float = 0.0                  # moment coefficient

@dataclass
class Atmosphere:
    """ISA-based atmosphere model."""
    rho_sea_level: float = 1.225      # kg/m³
    altitude_msl: float = 0.0         # m above sea level

    @property
    def rho(self) -> float:
        # ISA troposphere: ρ = ρ0 · (1 - 2.2558e-5·h)^4.2559
        return self.rho_sea_level * (1 - 2.2558e-5 * self.altitude_msl) ** 4.2559
```

In `physics_step`, replace:
```python
# OLD: drag_force = -params.drag_coeff * state.velocity
# NEW:
V_body = R.T @ state.velocity  # velocity in body frame
V_mag = np.linalg.norm(V_body)
if V_mag > 1e-6:
    V_hat = V_body / V_mag
    # Quadratic drag: F_D = 0.5 * ρ * A * C_D * V²
    F_drag_body = -0.5 * atmo.rho * aero.reference_area * aero.C_D * V_mag**2 * V_hat
    # Lift (for fixed-wing): F_L = 0.5 * ρ * A * C_L * V², perpendicular to velocity
    # For quadrotors, C_L ≈ 0
    drag_force = R @ F_drag_body  # transform back to world
else:
    drag_force = np.zeros(3)
```

**Tests to add:**
- `test_quadratic_drag_scales_with_v_squared`: Verify drag force quadruples when velocity doubles.
- `test_high_altitude_less_drag`: Same velocity at 4500m ASL should produce less drag than sea level.
- `test_terminal_velocity`: Freefall should converge to a finite terminal velocity.

---

### 1.2 Wind Perturbation Model

**New file:** `simulation/wind_model.py`

**What to implement (from paper Eq. 5-7, Section 2.3.1):**

```python
@dataclass
class WindField:
    """Wind perturbation model following Valencia et al. (2025)."""
    wind_speed: float = 0.0                   # m/s (constant or from profile)
    wind_direction: np.ndarray = field(        # unit vector, world frame
        default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    gust_intensity: float = 0.0               # m/s (stochastic component)
    turbulence_type: str = "none"              # "none", "constant", "dryden", "from_log"
    altitude_profile: Optional[np.ndarray] = None  # altitude-based wind from flight log

    def get_wind_velocity(self, t: float, position: np.ndarray) -> np.ndarray:
        """Return wind velocity vector [m/s] at given time and position."""
        ...

    def get_perturbation_force(self, t: float, position: np.ndarray,
                                aero: AeroCoefficients, atmo: Atmosphere) -> np.ndarray:
        """
        Compute wind perturbation force (paper Eq. 5-7):
          ΔF_D ≈ 0.5 · ρ · A · C_D · V_wind²
          ΔF_L ≈ 0.5 · ρ · A · C_L · V_wind²
          ΔF_W = ||F_D + F_L||
        Applied at the UAV's center of mass.
        """
        ...
```

**Wind types to support:**
1. **No wind** — baseline (what we have now)
2. **Constant wind** — uniform vector field
3. **Dryden turbulence** — standard MIL-F-8785C stochastic model (gusts)
4. **From flight log** — replay altitude-derived perturbation from real `.bin` data (paper's primary validation approach)

**Integration into `physics_step`:**
```python
# Add wind force to total_force
wind_force = wind.get_perturbation_force(t, state.position, aero, atmo)
total_force = gravity_force + thrust_world + drag_force + wind_force
```

**Tests to add:**
- `test_no_wind_unchanged`: Wind with speed=0 produces zero force.
- `test_constant_wind_deflects_hover`: Drone hovering in wind should drift downwind.
- `test_stronger_wind_more_force`: Force scales with V².
- `test_wind_from_log_matches_data`: Replayed log produces expected perturbation profile.

---

### 1.3 Full Inertia Tensor

**File:** `simulation/drone_physics.py`

**What to change:** Allow full 3×3 inertia tensor with products of inertia (Jxy, Jxz, Jyz) as in paper's Eq. 4.

```python
@dataclass
class DroneParams:
    mass: float = 1.5
    # Full inertia tensor (paper Eq. 4)
    inertia: np.ndarray = field(
        default_factory=lambda: np.array([
            [ 0.02,  0.0,   0.0  ],
            [ 0.0,   0.02,  0.0  ],
            [ 0.0,   0.0,   0.04 ],
        ])
    )
    # ... rest of params
```

The existing `physics_step` already handles a full matrix via `I_inv = np.linalg.inv(I)` and `np.cross(omega, I @ omega)`, so this is mainly about enabling non-diagonal defaults and adding presets for real airframes.

**Add airframe presets:**
```python
HOLYBRO_X500 = DroneParams(mass=2.0, inertia=np.array([...]), ...)
GENERIC_QUAD = DroneParams(mass=1.5, ...)
```

---

### 1.4 Body-Frame Force Decomposition

**File:** `simulation/drone_physics.py`

**What to change:** Align with paper's Eq. 3 — compute forces in body frame, then transform.

The paper formulates Newton's 2nd law as (Eq. 3):
```
[u̇]   [rv - qw]   1  [fx]
[v̇] = [pw - ru] + ─  [fy]
[ẇ]   [qu - pv]   m  [fz]
```
where (u,v,w) are body-frame velocities and (fx,fy,fz) are body-frame external forces (aero + gravity + thrust).

Currently we compute forces in world frame. The refactored approach:
1. Transform velocity to body frame: `V_body = R.T @ velocity`
2. Compute all forces in body frame (thrust is already body-frame)
3. Add Coriolis term: `ω × V_body`
4. Integrate in body frame, then transform back to world

**This matters for:** accuracy at high angular rates and when aerodynamic forces are naturally body-frame quantities.

---

### 1.5 Flight Log Import & Validation Metrics

**New file:** `simulation/flight_log.py`

**Purpose:** Parse Ardupilot `.bin` or `.log` telemetry files and extract:
- GPS position (lat, lon, alt) → local NED
- Attitude (roll, pitch, yaw)
- Airspeed
- Throttle/elevator/aileron signals
- Timestamps

```python
class FlightLog:
    """Parse and replay Ardupilot flight logs for DT validation."""

    @staticmethod
    def from_bin(path: str) -> 'FlightLog':
        """Parse Ardupilot .bin dataflash log."""
        ...

    @staticmethod
    def from_csv(path: str) -> 'FlightLog':
        """Parse exported CSV log."""
        ...

    def get_trajectory(self) -> np.ndarray:
        """Return Nx3 position array in local NED."""
        ...

    def get_wind_profile(self) -> np.ndarray:
        """Estimate wind perturbation from altitude deviations (paper method)."""
        ...
```

**New file:** `simulation/validation.py`

**Purpose:** Compare sim output against reference data (flight log or another sim).

```python
def compute_rmse(sim_trajectory: np.ndarray, ref_trajectory: np.ndarray) -> dict:
    """
    Compute RMSE per axis and total, matching paper's Table 5.
    Returns: {rmse_x, rmse_y, rmse_z, rmse_total, median, p25, p75}
    """
    ...

def plot_comparison(sim_records, ref_log, output_path: str):
    """
    Generate paper-style comparison plots:
    - Altitude vs time (sim vs real, Fig. 9/10/12)
    - 3D trajectory comparison (Fig. 8/11)
    - Error boxplots (Fig. 13)
    """
    ...
```

---

## Phase 2: Environment & Gazebo (P2)

### 2.1 Terrain Height Map

**New file:** `simulation/terrain.py`

**Purpose:** Load terrain data and provide ground-height queries for collision detection.

```python
class TerrainMap:
    """Terrain elevation model for the physics engine."""

    @staticmethod
    def from_stl(path: str) -> 'TerrainMap':
        """Load terrain from STL file (BlenderGIS output)."""
        ...

    @staticmethod
    def from_srtm(lat: float, lon: float, radius_km: float) -> 'TerrainMap':
        """Download SRTM elevation data for a region."""
        ...

    def get_elevation(self, x: float, y: float) -> float:
        """Return ground elevation at (x,y) in local frame."""
        ...

    def check_collision(self, position: np.ndarray) -> bool:
        """True if position is below terrain surface."""
        ...
```

**Integration into `physics_step`:**
```python
# Replace flat ground:
# OLD: if new_pos[2] < 0.0
# NEW:
ground_z = terrain.get_elevation(new_pos[0], new_pos[1])
if new_pos[2] < ground_z:
    new_pos[2] = ground_z
    new_vel[2] = max(new_vel[2], 0.0)
```

### 2.2 Gazebo World Generation

**New directory:** `gazebo/`

Following the paper's pipeline (Fig. 1, Fig. 2):
1. `gazebo/worlds/` — Gazebo `.world` files (empty, terrain-based)
2. `gazebo/models/` — UAV `.sdf` model files with LiftDrag plugin config
3. `gazebo/launch/` — ROS `.launch` files for SITL startup
4. `gazebo/scripts/wind_node.py` — ROS node that publishes wind perturbation forces

**Not urgent** — our standalone sim is the priority. Gazebo integration is for when we need full SITL with Ardupilot.

---

## Phase 3: Extended Vehicle Support (P3)

### 3.1 Fixed-Wing Aerodynamics

Add AoA-dependent lift/drag curves from CFD data (paper's Table 3, Fig. 5):

```python
class FixedWingAero(AeroCoefficients):
    """Aerodynamic model with AoA-dependent CL/CD and stall."""
    alpha_0: float = 0.05236          # zero-lift AoA [rad]
    alpha_stall: float = 0.26180      # stall angle [rad]
    C_La: float = 3.50141             # lift slope before stall
    C_Da: float = 0.63662             # drag slope before stall
    C_La_stall: float = -1.1459       # lift slope after stall
    C_Da_stall: float = 2.29183       # drag slope after stall

    def get_CL(self, alpha: float) -> float:
        if abs(alpha) < self.alpha_stall:
            return self.C_La * (alpha - self.alpha_0)
        else:
            return self.C_La_stall * (alpha - self.alpha_0)

    def get_CD(self, alpha: float) -> float:
        if abs(alpha) < self.alpha_stall:
            return self.C_Da * alpha**2 + C_D0
        else:
            return self.C_Da_stall * alpha**2 + C_D0
```

### 3.2 QGroundControl / MAVLink Bridge

**Scope:** Add MAVLink telemetry output so our sim can connect to QGC for mission planning and monitoring. Use `pymavlink` library.

---

## Execution Order

```
Phase 1 (physics upgrades — do first, all in simulation/):
  1.1  Quadratic drag + atmosphere model     → drone_physics.py
  1.2  Wind perturbation model               → wind_model.py (new)
  1.3  Full inertia tensor + airframe presets → drone_physics.py
  1.4  Body-frame force decomposition         → drone_physics.py
  1.5  Flight log import + validation metrics → flight_log.py, validation.py (new)

Phase 2 (environment — after Phase 1):
  2.1  Terrain height map                    → terrain.py (new)
  2.2  Gazebo world generation               → gazebo/ (new dir)

Phase 3 (extensions — after Phase 2):
  3.1  Fixed-wing aerodynamics               → drone_physics.py
  3.2  MAVLink bridge                        → mavlink_bridge.py (new)
```

## Testing Strategy

Each phase must maintain all existing 19 tests passing, plus add:

| Phase | New Tests | Focus |
|:---|:---|:---|
| 1.1 | 3 tests | Quadratic scaling, altitude effect, terminal velocity |
| 1.2 | 4 tests | Zero wind, drift, force scaling, log replay |
| 1.3 | 2 tests | Off-diagonal inertia coupling, preset loading |
| 1.4 | 2 tests | Body-frame equivalence, high-angular-rate accuracy |
| 1.5 | 3 tests | Log parsing, RMSE computation, comparison plot generation |
| 2.1 | 3 tests | STL loading, elevation query, terrain collision |

**Total expected: 19 existing + ~17 new = ~36 physics tests**

## Key Equations Reference (from paper)

- **Eq. 1:** Translational kinematics (rotation matrix × body velocity → inertial velocity)
- **Eq. 2:** Euler angle rates: `[φ̇,θ̇,ψ̇]ᵀ = T(φ,θ) · [p,q,r]ᵀ`
- **Eq. 3:** Newton's 2nd law in body frame: `[u̇,v̇,ẇ]ᵀ = [rv-qw, pw-ru, qu-pv]ᵀ + (1/m)·[fx,fy,fz]ᵀ`
- **Eq. 4:** Rotational dynamics: `[ṗ,q̇,ṙ]ᵀ = J⁻¹·(τ - ω×Jω)` with full J
- **Eq. 5:** Drag perturbation: `ΔF_D ≈ 0.5·ρ·A·C_D·V²`
- **Eq. 6:** Lift perturbation: `ΔF_L ≈ 0.5·ρ·A·C_L·V²`
- **Eq. 7:** Total wind force: `ΔF_W = ||F_D + F_L||`

---

## Notes for Claude / AI Agent

- **Always run `./run_scenario.sh --test` after every change** to ensure no regressions.
- **Phase 1 is self-contained** — all changes are in `simulation/`, no Docker/ROS needed.
- **Keep backward compatibility:** `DroneParams()` with no args must still work (defaults to current simplified model). New features are opt-in via `AeroCoefficients`, `Atmosphere`, `WindField` parameters.
- **Do not remove the linear drag option** — keep it as a fast-path for when `AeroCoefficients` is not provided.
- The paper uses Ardupilot's IRS-4 quadrotor model for validation. Our Holybro X500 V2 has similar specs.
- Wind perturbation along Z-axis only is the paper's validated approach (Section 2.3.1). Full 3D wind is future work.
