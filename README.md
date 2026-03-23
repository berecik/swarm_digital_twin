# Swarm Digital Twin: Drone Swarm Simulation & Testing Framework

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FJazzy-blue.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Language](https://img.shields.io/badge/Language-Python-blue.svg)](https://www.python.org/)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh-green.svg)](https://zenoh.io/)

## Overview

**Swarm Digital Twin** is a digital twin framework for developing and testing autonomous drone swarm behavior without physical hardware. It provides a complete simulation environment for the [DAS-SAR Vingilot](https://github.com/berecik/das_sar) project's distributed aerial search and rescue system.

The framework enables:
- **Swarm control algorithm development** using Boids, FSM, and distributed coordination
- **AI perception pipeline testing** with mocked sensors and 3D localization
- **Heavy-lift distributed control** simulation for 6-agent tethered payloads
- **End-to-end mission validation** via Docker-based multi-drone environments

---

## Project Structure

```text
.
├── sar_swarm_ws/               # ROS 2 Workspace
│   └── src/
│       ├── sar_swarm_control/  # Swarm coordination & flight control (Rust)
│       ├── heavy_lift_core/    # Distributed lift system logic (Rust)
│       ├── sar_perception/     # AI/Vision detection & localization (Python)
│       ├── sar_simulation/     # Mock simulators & test runners (Python)
│       └── px4_msgs/           # PX4-ROS 2 message definitions
├── docker/                     # Zenoh configuration and Docker setups
├── Dockerfile                  # Development environment container
├── docker-compose.yml          # Multi-container swarm orchestration
├── visualize_on_host.py        # Real-time swarm visualization
├── AGENTS.md                   # Technical context & development guide
├── TESTING.md                  # Test catalog & verification status
└── MAINTENANCE.log             # Maintenance history
```

## Tech Stack

| Component | Technology |
| :--- | :--- |
| **Safety-Critical Control** | **Rust** (rclrs, MAVSDK-Rust) |
| **AI & Computer Vision** | **Python** (PyTorch, YOLOv8/11) |
| **Middleware** | **Eclipse Zenoh** & **ROS 2** (Humble/Jazzy) |
| **Simulation** | Gazebo Harmonic / PX4 SITL |

## Getting Started

### Prerequisites
- Docker & Docker Compose
- Ubuntu 22.04 LTS (recommended)
- ROS 2 Humble/Jazzy
- Rust Toolchain

### Running Simulation

1. **Build the Docker environment:**
   ```bash
   docker-compose build
   ```

2. **Launch the swarm simulation:**
   ```bash
   docker-compose up
   ```

3. **Visualize on host:**
   ```bash
   python3 visualize_on_host.py
   ```

## Testing

### Running Unit Tests
```bash
# Rust swarm control
cd sar_swarm_ws/src/sar_swarm_control
cargo test

# Python perception
cd sar_swarm_ws/src/sar_perception
python -m pytest test/
```

See [TESTING.md](TESTING.md) for the full test catalog and [AGENTS.md](AGENTS.md) for development workflows.

## Related Projects

- **[DAS-SAR Vingilot](https://github.com/berecik/das_sar)** — Project documentation, architecture, safety case, and business analysis for the DAS-SAR system.

## Authors & Contact

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **Website:** [https://marysia.app](https://marysia.app)
