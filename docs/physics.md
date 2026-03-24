# 🧪 Drone Physics Engine & Standalone Simulation

The **Swarm Digital Twin** includes a custom-built, high-fidelity, standalone rigid-body quadrotor physics engine written in Python (using NumPy). This allows for rapid algorithm development and testing without the overhead of ROS 2 or Gazebo.

---

## 🏗 Physics Model

The engine (`simulation/drone_physics.py`) simulates the following dynamics:

### 1. Rigid-Body Dynamics
- **State Vector:** Position ($x, y, z$), Velocity ($v_x, v_y, v_z$), Rotation Matrix ($R_{3\times3}$), and Angular Velocity ($\omega_x, \omega_y, \omega_z$).
- **Gravity:** Applied globally ($g = 9.81$ m/s²).
- **Thrust:** Applied along the body-frame Z-axis ($R \times [0, 0, T]^T$).
- **Aerodynamic Drag:** Proportional to velocity ($F_{drag} = -k_d \cdot v$).

### 2. Cascaded Control
- **Outer Loop (Position):** P-controller for position error, computing target velocities.
- **Inner Loop (Velocity):** PID-controller for velocity error, computing target thrust and attitude (roll, pitch).
- **Attitude Control:** Simplified high-bandwidth attitude stabilization for target roll/pitch.

---

## 📂 Key Files

| File | Description |
| :--- | :--- |
| `drone_physics.py` | The core engine, state integrator (Euler), and PID controllers. |
| `drone_scenario.py` | Mission runner: Takeoff → 6-waypoint cruise → Return → Land. |
| `visualize_drone_3d.py` | Matplotlib-based 3D animator (30 FPS) with timeline panels. |
| `test_drone_physics.py` | 19 pytest tests verifying math, gravity, and control convergence. |

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
The physics engine is verified by 19 unit tests covering:
1. **Math Integrity:** Identity rotations, roundtrip Euler/Matrix conversions, and matrix orthogonality.
2. **Physics Correctness:** Drag-free freefall follows analytical solutions ($z = z_0 - \frac{1}{2}gt^2$), ground constraints stop vertical fall.
3. **Control Performance:** PID loop convergence to zero error, target tracking within tolerance.
4. **Energy Conservation:** Total Mechanical Energy (KE + PE) is conserved within 0.5% during drag-free flight.
