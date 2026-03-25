# Testing Strategy for `swarm_control_core`

This document describes the testing approach for the Rust ROS 2 node that manages the PX4 Offboard Control Loop.

## 1. Unit Testing (Core Logic)

Core mathematical and utility functions are separated into `src/utils.rs` and `src/boids.rs` to allow testing without a full ROS 2 environment.

### Key Tests (Detailed Catalog):

#### A. Coordinate Systems & Math
- **`test_enu_to_ned_conversion`**:
    - **Purpose**: Validates the translation between ROS 2 ENU (East-North-Up) and PX4 NED (North-East-Down) frames.
    - **Input**: `[10.0, 5.0, 2.0]` (ENU).
    - **Expected Outcome**: `[5.0, 10.0, -2.0]` (NED). Correctly handles axis swapping and Z-axis inversion.
- **`test_timestamp_generation`**:
    - **Purpose**: Ensures that the `get_clock_microseconds` helper provides monotonically increasing timestamps in microseconds, as required by PX4's uORB middleware.
    - **Verification**: Asserts that two sequential calls with a delay return increasing values with appropriate deltas.

#### B. Swarm Intelligence (Boids Algorithm)
- **`test_separation_force`**:
    - **Purpose**: Verifies that drones generate a repulsive force when too close to neighbors (< 1.0m).
    - **Input**: Two Boids separated by 0.1m.
    - **Expected Outcome**: A force vector pointing directly away from the neighbor.
- **`test_alignment_force`**:
    - **Purpose**: Checks if a drone attempts to match the velocity of its neighbors within the sensing radius (5.0m).
    - **Input**: One stationary Boid and one neighbor moving at `[1.0, 1.0, 0.0]`.
    - **Expected Outcome**: A force vector contributing to velocity matching.
- **`test_cohesion_force`**:
    - **Purpose**: Ensures drones are attracted to the center of mass of their neighbors.
    - **Input**: Neighbors within the 5.0m radius.
    - **Expected Outcome**: A force vector pointing towards the average neighbor position.
- **`test_boids_edge_cases`**:
    - **Purpose**: Stress tests the flocking logic against boundary conditions.
    - **Scenarios**: Neighbors at exactly 5.0m (excluded), identical positions (zero force to avoid division by zero), and neighbors just inside/outside the radius.
- **`test_flocking_vector_matches_rust_reference_case`** (Python parity in `simulation/test_drone_physics.py`):
    - **Purpose**: Ensures the standalone simulator computes the boids steering vector with the same equation and default weights as `src/boids.rs::calculate_flocking_vector`.
    - **Input**: One controlled reference configuration with two neighbors (inside neighbor radius, one inside separation radius).
    - **Expected Outcome**: Python result exactly matches the hand-derived vector from the Rust formula within numerical tolerance.
- **`test_flocking_vector_returns_zero_without_neighbors`** (Python parity in `simulation/test_drone_physics.py`):
    - **Purpose**: Verifies stable no-neighbor behavior parity with Rust early-return path.
    - **Input**: Empty `neighbors` list.
    - **Expected Outcome**: Steering vector is exactly `[0, 0, 0]`.
- **`test_flocking_vector_excludes_neighbor_at_radius_boundary`** (Python parity in `simulation/test_drone_physics.py`):
    - **Purpose**: Confirms strict `< neighbor_radius` contract parity with Rust implementation.
    - **Input**: One neighbor exactly at `neighbor_radius`.
    - **Expected Outcome**: Neighbor is ignored by boids aggregation and result follows non-neighbor branch.

#### D. Cross-Module Parity (Rust vs Standalone Twin)
- **Parity Contract (Phase C)**:
    - **Purpose**: Keep standalone swarm simulation behavior aligned with Rust swarm logic before SITL integration.
    - **Reference**: `src/boids.rs` (`FlockingParams` defaults and `calculate_flocking_vector`).
    - **Verification**:
      1. Run `pytest -q simulation/test_drone_physics.py -k SwarmStandaloneTwin`.
      2. Confirm `test_flocking_vector_matches_rust_reference_case` passes.
      3. Confirm `test_six_agent_run_maintains_min_separation` passes (no collisions in 6-agent demo envelope).
    - **Expected Outcome**: Boids steering parity for the reference case and stable 6-agent standalone run with enforced separation.

#### C. Communication & Middleware
- **`test_boid_serialization`**:
    - **Purpose**: Validates the `bincode` encoding/decoding of the `Boid` struct.
    - **Requirement**: Essential for reliable Zenoh-based state sharing between drones.
    - **Outcome**: Decoded struct must exactly match the original.
- **`test_handshake_logic`**:
    - **Purpose**: Verifies the internal state machine for the PX4 Offboard handshake (Heartbeat -> Mode Switch -> Arming).
    - **Verification**: Simulates 20 cycles and confirms that the mode switch and arming commands are triggered exactly at cycle 10.

### Running Unit Tests:
Since the project depends on `rclrs`, a sourced ROS 2 environment (Humble) is required even for `cargo test`.
```bash
# Inside a ROS 2 environment
cd swarm_control
cargo test
```

## 2. Integration Testing (Node Logic)

The `OffboardControlNode` in `src/main.rs` implements a state machine for the Offboard handshake:
1. **Heartbeat Phase:** Stream `OffboardControlMode` at 20Hz for >0.5s.
2. **Mode Switch:** Send `VehicleCommand` to set `OFFBOARD` mode.
3. **Arming Phase:** Send `VehicleCommand` to `ARM` the motors.
4. **Active Control:** Stream `TrajectorySetpoint` (NED) with NaN masking.

### Verification (Manual/SITL):
Full integration testing should be performed using the PX4 SITL (Software-in-the-Loop) environment.
1. Start PX4 SITL with Gazebo.
2. Start the XRCE-DDS Agent:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
3. Run the swarm node:
   ```bash
   ros2 run swarm_control_core swarm_node
   ```
4. Observe the drone in Gazebo switching to Offboard mode, arming, and hovering at 5m.

## 3. Mocking Strategy

The `px4_msgs_mock` package is used during development to provide the necessary message definitions without requiring the full `px4_msgs` ROS package to be compiled, which accelerates the development cycle.
