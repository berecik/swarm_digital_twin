# sar_simulation

Simulation module for the Swarm Digital Twin digital twin. Provides both a **standalone physics engine** (no ROS 2 required) and **ROS 2-based mock simulators** for swarm testing.

## Standalone Physics Simulation

### Quick Start

From the repo root:

```bash
./run_scenario.sh           # run scenario + open 3D visualization
./run_scenario.sh --test    # run 19 physics unit tests
./run_scenario.sh --all     # tests → scenario → visualization
```

### Files

| File | Description |
| :--- | :--- |
| `drone_physics.py` | Quadrotor rigid-body physics engine — gravity, thrust, drag, attitude dynamics, PID controller |
| `drone_scenario.py` | Flight scenario: takeoff → waypoints → return → land. Outputs `scenario_data.npz` |
| `visualize_drone_3d.py` | 3D animated visualization (matplotlib) with attitude axes, velocity, timelines |
| `test_drone_physics.py` | 19 pytest tests: rotation, gravity, hover, drag, PID, controller, energy conservation |

### Physics Model

- **Rigid body**: 6-DOF with mass, inertia tensor
- **Forces**: Gravity, thrust (body-Z), linear aerodynamic drag
- **Attitude**: Rotation matrix with angular velocity and Euler's rotation equation
- **Ground constraint**: Position z >= 0, velocity clamped at contact
- **Integration**: First-order with SVD re-orthogonalization to prevent rotation drift
- **Controller**: Cascaded PID — position error → desired acceleration → desired attitude → thrust + torque

## ROS 2 Mock Simulators

| File | Description |
| :--- | :--- |
| `mock_drone_sim.py` | Lightweight ROS 2 mock drone with P-control physics |
| `test_swarm_flight.py` | ROS 2 integration test: swarm goal tracking |
| `search_task.py` | Lawnmower search pattern waypoint publisher |
| `visualize_swarm.py` | ASCII terminal swarm visualizer (ROS 2) |
| `visualize_swarm_graphical.py` | Matplotlib swarm visualizer (ROS 2) |
| `visualize_udp_relay.py` | UDP relay for host-side visualization |
