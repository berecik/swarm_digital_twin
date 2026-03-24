# Swarm Digital Twin: Drone Swarm Simulation & Testing Framework

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FJazzy-blue.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Language](https://img.shields.io/badge/Language-Python-blue.svg)](https://www.python.org/)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh-green.svg)](https://zenoh.io/)

## Overview

**Swarm Digital Twin** is a digital twin framework for developing and testing autonomous drone swarm behavior without physical hardware. It provides a complete simulation environment for the [DAS-SAR Vingilot](https://github.com/berecik/das_sar) project's distributed aerial search and rescue system.

The framework enables:
- **Standalone drone physics simulation** with rigid-body dynamics, PID control, and 3D visualization
- **Swarm control algorithm development** using Boids, FSM, and distributed coordination
- **AI perception pipeline testing** with mocked sensors and 3D localization
- **Heavy-lift distributed control** simulation for 6-agent tethered payloads
- **End-to-end mission validation** via Docker-based multi-drone environments

---

## Project Structure

```text
.
├── docs/                       # Detailed Documentation
│   ├── architecture.md         # System architecture & components
│   ├── testing.md              # Test strategy & catalog
│   ├── development.md          # Setup & workflow guide
│   └── physics.md              # Standalone physics engine details
├── swarm_control/              # Swarm coordination & flight control (Rust)
├── heavy_lift_core/            # Distributed lift system logic (Rust)
├── perception/                 # AI/Vision detection & localization (Python)
├── simulation/                 # Simulators, physics engine & tests (Python)
│   ├── drone_physics.py        # Quadrotor rigid-body physics engine
│   ├── drone_scenario.py       # Flight scenario (takeoff→fly→land)
│   ├── visualize_drone_3d.py   # 3D animated flight visualization
│   └── test_drone_physics.py   # 19 physics unit tests
├── px4_msgs/                   # PX4-ROS 2 message definitions
├── run_scenario.sh             # One-command scenario runner & visualizer
├── docker/                     # Zenoh configuration and Docker setups
├── Dockerfile                  # Development environment container
├── docker-compose.yml          # Multi-container swarm orchestration
├── visualize_on_host.py        # Real-time swarm visualization (ROS 2)
├── AGENTS.md                   # Technical context & development guide
├── TESTING.md                  # Test catalog & verification status
└── MAINTENANCE.log             # Maintenance history
```

## Documentation

Detailed documentation is available in the [docs/](docs/) directory:
- [**Architecture Overview**](docs/architecture.md) — System design, components, and communication.
- [**Testing Guide**](docs/testing.md) — Test strategy, catalog, and protocols.
- [**Development & Setup**](docs/development.md) — Environment setup and coding standards.
- [**Physics Engine**](docs/physics.md) — Deep dive into the standalone simulation.
- [**Agent Guide**](AGENTS.md) — Protocols and context for autonomous developers.

## Tech Stack

| Component | Technology |
| :--- | :--- |
| **Safety-Critical Control** | **Rust** (rclrs, MAVSDK-Rust) |
| **AI & Computer Vision** | **Python** (PyTorch, YOLOv8/11) |
| **Middleware** | **Eclipse Zenoh** & **ROS 2** (Humble/Jazzy) |
| **Physics Simulation** | **Python** (NumPy) — standalone rigid-body quadrotor |
| **3D Visualization** | **Matplotlib** (animated 3D + timeline panels) |
| **Full Simulation** | Gazebo Harmonic / PX4 SITL |

## Quick Start — Standalone Physics Simulation

No Docker, no ROS 2 — just Python 3 and a terminal:

```bash
# Run scenario and open 3D visualization (auto-creates venv)
./run_scenario.sh

# Or step by step:
./run_scenario.sh --test       # run 19 physics unit tests
./run_scenario.sh --sim-only   # run scenario (no GUI)
./run_scenario.sh --viz-only   # open visualization with existing data
./run_scenario.sh --all        # tests → scenario → visualization
```

The scenario flies a drone through 6 waypoints (takeoff → cruise → return → land) with full rigid-body physics: gravity, thrust, aerodynamic drag, attitude dynamics, and a cascaded PID controller.

## Getting Started

### Prerequisites

**Standalone simulation (no external deps):**
- Python 3.10+ (NumPy, Matplotlib — installed automatically by `run_scenario.sh`)

**Full swarm simulation (Docker):**
- Docker & Docker Compose
- Ubuntu 22.04 LTS (recommended)
- ROS 2 Humble/Jazzy
- Rust Toolchain

### Running Full Swarm Simulation

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

### Running All Tests
```bash
# Standalone physics tests (no ROS 2 needed)
./run_scenario.sh --test

# Rust swarm control (requires ROS 2 environment)
cd swarm_control
cargo test

# Python perception
cd perception
python -m pytest test/
```

See [TESTING.md](TESTING.md) for the full test catalog and [AGENTS.md](AGENTS.md) for development workflows.

## Related Projects

- **[DAS-SAR Vingilot](https://github.com/berecik/das_sar)** — Project documentation, architecture, safety case, and business analysis for the DAS-SAR system.

## Authors & Contact

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **Website:** [https://marysia.app](https://marysia.app)
