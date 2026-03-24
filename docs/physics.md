# 🧪 Drone Physics Engine & Standalone Simulation

The **Swarm Digital Twin** includes a custom-built, high-fidelity, standalone rigid-body quadrotor physics engine written in Python (using NumPy). This allows for rapid algorithm development and testing without the overhead of ROS 2 or Gazebo.

---

## 🏗 Physics Model

The engine (`simulation/drone_physics.py`) simulates the following dynamics:

### 1. Rigid-Body Dynamics
- **State Vector:** Position ($x, y, z$), Velocity ($v_x, v_y, v_z$), Rotation Matrix ($R_{3\times3}$), and Angular Velocity ($\omega_x, \omega_y, \omega_z$).
- **Gravity:** Applied globally ($g = 9.81$ m/s²).
- **Thrust:** Applied along the body-frame Z-axis ($R \times [0, 0, T]^T$).
- **Aerodynamic Drag (legacy):** Linear drag proportional to velocity ($F_{drag} = -k_d \cdot v$). Used when no `AeroCoefficients` are set.
- **Aerodynamic Drag (quadratic):** Body-frame quadratic drag following Valencia et al. (2025) Eq. 5: $F_D = -\frac{1}{2} \rho A C_D |V|^2 \hat{V}$. Enabled via `AeroCoefficients`.
- **ISA Atmosphere:** Altitude-dependent air density: $\rho = \rho_0 (1 - 2.2558 \times 10^{-5} h)^{4.2559}$.
- **Body-Frame Dynamics:** When quadratic drag is active, forces are computed in the body frame with Coriolis coupling (paper Eq. 3): $\dot{V}_b = F_b/m - \omega \times V_b$.
- **Wind Perturbation:** Optional `WindField` model supporting constant wind, Dryden turbulence, and flight-log replay (paper Eq. 5-7).
- **Full Inertia Tensor:** Supports off-diagonal products of inertia ($J_{xy}, J_{xz}, J_{yz}$) per paper Eq. 4.

### 2. Cascaded Control
- **Outer Loop (Position):** P-controller for position error, computing target velocities.
- **Inner Loop (Velocity):** PID-controller for velocity error, computing target thrust and attitude (roll, pitch).
- **Attitude Control:** Simplified high-bandwidth attitude stabilization for target roll/pitch.

---

## 📂 Key Files

| File | Description |
| :--- | :--- |
| `drone_physics.py` | Core engine: rigid-body dynamics, quadratic drag, atmosphere, PID controllers, airframe presets. |
| `wind_model.py` | Wind perturbation model (constant, Dryden turbulence, flight-log replay). |
| `flight_log.py` | Ardupilot flight log parser (CSV) for validation against real data. |
| `validation.py` | RMSE metrics and comparison plots (paper Table 5, Fig. 13 style). |
| `drone_scenario.py` | Mission runner: Takeoff → 6-waypoint cruise → Return → Land. |
| `visualize_drone_3d.py` | Matplotlib-based 3D animator (30 FPS) with timeline panels. |
| `test_drone_physics.py` | 35 pytest tests verifying math, physics, aero, wind, and validation. |

---

## 🏃 Usage

### Full Workflow
Run everything (tests → sim → viz) with:
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

## 📉 Testing & Verification
The physics engine is verified by 35 unit tests covering:
1. **Math Integrity:** Identity rotations, roundtrip Euler/Matrix conversions, and matrix orthogonality.
2. **Physics Correctness:** Drag-free freefall follows analytical solutions ($z = z_0 - \frac{1}{2}gt^2$), ground constraints stop vertical fall.
3. **Control Performance:** PID loop convergence to zero error, target tracking within tolerance.
4. **Energy Conservation:** Total Mechanical Energy (KE + PE) is conserved within 0.5% during drag-free flight.
5. **Quadratic Drag:** V² scaling, altitude-dependent density effect, terminal velocity convergence.
6. **Atmosphere:** ISA sea-level and high-altitude density validation.
7. **Wind Model:** Zero-wind baseline, constant wind drift, force scaling, flight-log replay interpolation.
8. **Inertia & Presets:** Off-diagonal coupling, airframe preset loading.
9. **Body-Frame Dynamics:** Hover equivalence with world-frame, freefall accuracy.
10. **Validation:** RMSE computation, flight log CSV parsing.
