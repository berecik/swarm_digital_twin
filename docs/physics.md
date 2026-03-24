# Drone Physics Engine & Standalone Simulation

The **Swarm Digital Twin** includes a custom-built, high-fidelity, standalone rigid-body quadrotor physics engine written in Python (using NumPy). This allows for rapid algorithm development and testing without the overhead of ROS 2 or Gazebo.

For the full mathematical derivation and equations, see **[Physics Details](physics_details.md)**.

---

## Physics Model

The engine (`simulation/drone_physics.py`) simulates the following dynamics:

### 1. Rigid-Body Dynamics
- **State Vector:** Position, Velocity, Rotation Matrix (SO(3)), and Angular Velocity.
- **Gravity:** Applied globally (g = 9.81 m/s^2).
- **Thrust:** Applied along the body-frame Z-axis.
- **Aerodynamic Drag (legacy):** Linear drag proportional to velocity. Used when no `AeroCoefficients` are set.
- **Aerodynamic Drag (quadratic):** Body-frame quadratic drag following Valencia et al. (2025) Eq. 5. Enabled via `AeroCoefficients`. AoA-dependent C_D for fixed-wing via `FixedWingAero`.
- **Aerodynamic Lift:** AoA-dependent lift with stall model (Eq. 6). Pre/post-stall CL/CD curves via `FixedWingAero`. Zero for quadrotors.
- **ISA Atmosphere:** Altitude-dependent air density for drag and wind calculations.
- **Body-Frame Dynamics:** When quadratic drag is active, forces are computed in the body frame with Coriolis coupling (paper Eq. 3).
- **Wind Perturbation:** Optional `WindField` model supporting constant wind, Dryden turbulence, and flight-log replay (paper Eq. 5-7).
- **Full Inertia Tensor:** Supports off-diagonal products of inertia per paper Eq. 4.
- **Terrain Height Map:** Optional `TerrainMap` for ground collision against 3D elevation data. Replaces the flat z=0 ground plane when provided.

### 2. Cascaded Control
- **Outer Loop (Position):** PID controller for position error, computing desired acceleration with gravity compensation.
- **Inner Loop (Attitude):** PID controller for attitude error, outputting torque commands.
- **Thrust computation:** Derived from desired acceleration magnitude.

---

## Key Files

| File | Description |
| :--- | :--- |
| `drone_physics.py` | Core engine: rigid-body dynamics, quadratic drag, lift with stall, atmosphere, PID controllers, airframe presets (quad + fixed-wing). |
| `wind_model.py` | Wind perturbation model (constant, Dryden turbulence, flight-log replay). |
| `terrain.py` | Terrain elevation model (flat, grid, STL, analytical function) with bilinear interpolation. |
| `flight_log.py` | Ardupilot flight log parser (CSV) for validation against real data. |
| `validation.py` | RMSE metrics and comparison plots (paper Table 5, Fig. 13 style). |
| `drone_scenario.py` | Full-featured scenario: 7 waypoints over terrain with wind and quadratic drag. |
| `visualize_drone_3d.py` | 3D animated visualization with terrain surface, wind indicator, AGL tracking. |
| `mavlink_bridge.py` | MAVLink v2 UDP bridge to QGroundControl (heartbeat, attitude, GPS, HUD). |
| `test_drone_physics.py` | 64 pytest tests covering all physics modules + MAVLink. |

---

## Usage

### Full Workflow
Run everything (tests -> sim -> viz) with:
```bash
./run_scenario.sh --all
```

### Standalone Simulator
Generate scenario data (`scenario_data.npz`):
```bash
./run_scenario.sh --sim-only
```

### 3D Visualization
Visualize the last recorded flight:
```bash
./run_scenario.sh --viz-only
```

---

## Scenario

The default scenario (`drone_scenario.py`) demonstrates all physics features:

- **Drone:** 1.5 kg quadrotor with quadratic drag (C_D=1.0, A=0.04 m^2)
- **Atmosphere:** ISA sea level (rho=1.225 kg/m^3)
- **Wind:** 3.0 m/s constant NE breeze
- **Terrain:** Rolling hills with Gaussian peaks (0-13 m elevation, 0.5 m resolution)
- **Waypoints:** 7 terrain-relative waypoints (8 m AGL cruise, 1 m AGL landing)
- **Path planning:** Straight-line legs with terrain clearance verification (min 5 m AGL)

The visualization shows the 3D terrain surface with the drone's trajectory, attitude axes, velocity vector, wind direction, ground shadow projected onto terrain, and timeline panels for altitude (MSL), AGL, speed, and thrust.

---

## Testing & Verification

The physics engine is verified by 64 unit tests covering:

1. **Math Integrity:** Rotation matrix identity, roundtrip Euler conversions, orthogonality.
2. **Physics Correctness:** Analytical freefall, ground constraints.
3. **Control Performance:** PID convergence, target tracking.
4. **Energy Conservation:** KE + PE conserved within 0.5% (drag-free).
5. **Quadratic Drag:** V^2 scaling, altitude-dependent density, terminal velocity.
6. **Atmosphere:** ISA sea-level and high-altitude density.
7. **Wind Model:** Zero-wind baseline, constant wind drift, force scaling, log replay.
8. **Inertia & Presets:** Off-diagonal coupling, airframe preset loading.
9. **Body-Frame Dynamics:** Hover equivalence, freefall accuracy.
10. **Validation:** RMSE computation, flight log CSV parsing.
11. **Terrain:** Flat/grid/function elevation, collision detection, physics integration, STL errors.
12. **Fixed-Wing Aerodynamics:** AoA computation, pre/post-stall CL/CD, lift perpendicularity, stall behaviour, preset loading, AoA-dependent drag.
13. **MAVLink Bridge:** CRC computation, message encoding/decoding roundtrips (heartbeat, attitude, GPS, HUD), command parsing, ENU→GPS conversion, SimState conversion.

---

## Further Reading

- **[Physics Details](physics_details.md)** — Full equations, derivations, and parameter tables.
- **[Refactoring Plan](REFACTOR_PLAN.md)** — Gap analysis and implementation roadmap.
- **[TESTING.md](../TESTING.md)** — Complete test catalog with descriptions.
