# 🏗 System Architecture - Swarm Digital Twin

This document provides a detailed technical overview of the **Swarm Digital Twin** (DAS-SAR) system architecture, components, and communication protocols.

---

## 🚁 Project Essence
**Swarm Digital Twin** is a dual-phase Search and Rescue (SAR) system designed for rapid deployment and high-capacity evacuation.

1.  **Phase 1 (Scout Swarm):**
    - **Hardware:** Agile, man-portable drones (e.g., Holybro X500 V2).
    - **Objective:** Autonomous area search and human detection.
    - **Autonomy:** Boids-inspired flocking, state-machine-driven search patterns (Lawnmower/Boustrophedon).
2.  **Phase 2 (Heavy Lift):**
    - **Hardware:** Coaxial X8 heavy-lift agents.
    - **Objective:** Evacuate 100kg+ casualties using a Distributed Lift System (DLS).
    - **Control:** 6-agent minimum for 6-DOF payload control, Admittance Control, and Distributed Control Allocation (DCA).

---

## 🛠 Software Stack

| Layer | Technology | Responsibility |
| :--- | :--- | :--- |
| **Firmware** | PX4 Autopilot | Real-time stabilization, motor control, and EKF. |
| **Logic (Rust)** | `rclrs` / `swarm_control` | Safety-critical coordination, FSM, and swarm algorithms. |
| **Logic (Python)** | `rclpy` / `perception` | AI perception, 3D localization, and computer vision. |
| **Middleware** | ROS 2 (Humble/Jazzy) | Intra-drone node communication. |
| **Global Mesh** | Eclipse Zenoh | Inter-drone communication and GCS link. |
| **Simulation** | Python (NumPy) | Standalone physics engine for rapid development. |

---

## 🏗 Component Breakdown

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity running on an NVIDIA Jetson compute platform (Orin Nano/AGX).

- **Low-Level (PX4):** Handles the "Dirty Air" — stabilization and basic flight.
- **High-Level (ROS 2):** Handles the "Smart Brain" — perception, path planning, and swarm logic.
- **Zenoh Bridge:** Bridges local ROS 2 topics to the global swarm mesh using `zenoh-bridge-ros2dds`.

### 2. Swarm Control (`swarm_control`)
Written in **Rust** for memory safety and deterministic performance.
- **FSM:** Manages mission states (IDLE, TAKEOFF, SEARCH, FOUND, RTL).
- **Boids Engine:** Computes separation, alignment, and cohesion forces.
- **PX4 Interface:** Communicates with PX4 via `TrajectorySetpoint` (NED frame).

### 3. Perception & Localization (`perception`)
Written in **Python** to leverage the AI ecosystem.
- **Detection:** YOLO-based human detection.
- **3D Projection:** Deprojects 2D pixel coordinates to 3D world coordinates using depth maps and drone odometry.
- **Coordination:** Publishes global human positions to the swarm.

### 4. Distributed Lift System (Phase 2)
The DLS is a critical innovation allowing multiple drones to act as a single "virtual crane".
- **6-DOF Control:** Requires 6 agents to control both position and orientation of the payload.
- **Admittance Control:** Drones admit external forces (tether tension) to prevent fighting against each other.
- **Emergency Detach:** Safety logic for immediate tether release.

---

## 📡 Communication Protocol

### Intra-Drone (ROS 2)
Standard ROS 2 publishers/subscribers for local sensing and control.

### Inter-Drone (Zenoh Mesh)
- **Mesh Connectivity:** Low-latency, decentralised communication.
- **Topics:**
    - `/swarm/telemetry`: Position and status of each drone.
    - `/swarm/detections`: Shared human discovery events.
    - `/swarm/consensus`: Raft-based task allocation (planned).

---

## 📍 Coordinate Systems

| System | Orientation | Origin | Usage |
| :--- | :--- | :--- | :--- |
| **ENU** | East-North-Up | Launch Point | ROS 2 default frame. |
| **NED** | North-East-Down | Launch Point | PX4 firmware default frame. |
| **Body** | Forward-Left-Up | Drone Center | IMU and sensor orientation. |

**Conversion:** All high-level logic works in ENU. Conversions to NED are performed at the PX4 interface layer.
