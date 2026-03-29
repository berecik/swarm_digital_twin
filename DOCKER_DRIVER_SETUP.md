# Docker Driver & ROS 2 Multi-Drone Setup

## Overview

Each drone in the swarm requires:
1. **Driver Bridge** — Micro-XRCE-DDS Agent (PX4 ↔ ROS 2 bridge)
2. **ROS 2 Control Node** — Swarm control logic (Rust `cargo run`)
3. **Perception Node** — Vision/detection pipeline (Python)
4. **Zenoh Bridge** — Inter-drone mesh networking
5. **Isolated ROS Domain** — Each drone has unique `ROS_DOMAIN_ID`

---

## Docker Compose Service Structure (6-Drone Example)

### Per-Drone Services

For **each drone N** (1–6), the following services are created:

#### 1. `sitl_drone_N` (Simulation/Driver Container)
- **Purpose:** Simulates the Pixhawk autopilot (in SITL mode) or provides the driver interface to real hardware
- **ROS_DOMAIN_ID:** Unique ID per drone (1, 2, 3, 4, 5, 6)
- **Role:** Runs `Micro-XRCE-DDS Agent` (PX4 driver bridge)
- **Default Command:** `tail -f /dev/null` (idle; agent runs in entrypoint)
- **Device Access:** 
  - SITL: No hardware needed
  - Real Drone: Uncomment `devices:` to map `/dev/ttyUSB0`, `/dev/ttyACM0`, `/dev/video*`

**Key Settings:**
```yaml
sitl_drone_N:
  environment:
    - ROS_DOMAIN_ID=N
  # For real hardware, uncomment:
  # devices:
  #   - /dev/ttyUSB0:/dev/ttyUSB0      # Pixhawk serial (adjust for your board)
  #   - /dev/video0:/dev/video0        # RGB camera
  #   - /dev/video1:/dev/video1        # Depth camera (OAK-D)
  healthcheck:
    test: ["CMD-SHELL", "bash -lc 'test -f /dev/null'"]
```

#### 2. `swarm_node_N` (Control Logic)
- **Purpose:** Runs the high-level swarm control logic (Rust/`cargorun`)
- **Command:** `cargo run -- N` (passes drone ID)
- **Dependencies:** Waits for `sitl_drone_N` to be ready
- **Restart Policy:** `always` (fail-safe restart)
- **Healthcheck:** Monitors cargo process

**Key Settings:**
```yaml
swarm_node_N:
  environment:
    - ROS_DOMAIN_ID=N
  command: bash -c "cd /root/workspace/swarm_control && cargo run -- N"
  restart: always
  healthcheck:
    test: ["CMD-SHELL", "bash -lc 'pgrep -f cargo >/dev/null'"]
    interval: 15s
    timeout: 5s
    retries: 5
```

#### 3. `perception_node_N` (Vision & Detection)
- **Purpose:** Runs vision pipeline (YOLOv8/11 detection, OAK-D depth, 3D localization)
- **Command:** `ros2 run perception_core detector`
- **Dependencies:** Waits for `sitl_drone_N`
- **Restart Policy:** `always`
- **Healthcheck:** Monitors detector process

**Key Settings:**
```yaml
perception_node_N:
  environment:
    - ROS_DOMAIN_ID=N
  command: bash -c "export PYTHONPATH=$PYTHONPATH:/root/workspace/perception && ros2 run perception_core detector"
  restart: always
  healthcheck:
    test: ["CMD-SHELL", "bash -lc 'pgrep -f detector >/dev/null'"]
```

### Global Services

#### `zenoh_bridge_N` (Mesh Networking)
- **Purpose:** Bridges ROS 2 DDS topics from isolated domains to a shared Zenoh mesh
- **Image:** `eclipse/zenoh-bridge-ros2dds:latest`
- **Config:** Bridge config per drone (`./docker/zenoh/bridge_drone_N.json5`)
- **Isolation:** Uses `ROS_DOMAIN_ID=N` to connect to the correct drone

---

## ROS 2 Domain Isolation & Zenoh Bridging

### Why Domain Isolation?

Each drone runs ROS 2 with a **separate `ROS_DOMAIN_ID`** to:
- Prevent auto-discovery interference between drones
- Control which topics are exposed to the network
- Enable safer parallel debugging (e.g., introspect one drone without affecting others)

### Zenoh Role

Zenoh acts as the **mesh router** between isolated ROS 2 domains:
- **Drone 1 (Domain 1)** ⇌ **Zenoh Bridge 1** → **Shared Zenoh Mesh**
- **Drone 2 (Domain 2)** ⇌ **Zenoh Bridge 2** → **Shared Zenoh Mesh**
- **...** (and so on for drones 3–6)

This allows inter-drone communication via:
```python
# In swarm_control (Rust)
// Subscribe to Drone 2's odometry (published on Zenoh)
let sub = node.create_subscription::<VehicleOdometry>(
    "/drone_2/odometry",  // Zenoh publishes this from domain 2
    QOS_PROFILE_SENSOR_DATA,
)?;
```

---

## Driver Setup in Dockerfile

### Installed Components

1. **Micro-XRCE-DDS Agent** (compiled from source)
   - Bridges PX4 (over MAVLink/serial) to ROS 2 (DDS)
   - Runs as part of the drone startup

2. **Serial & USB Support**
   - `libusb-1.0-0`, `libusb-1.0-0-dev` — USB device library
   - `libudev-dev` — Device enumeration
   - `minicom`, `stm32flash` — Serial tools
   - **udev rules** — Grant permission to `/dev/ttyUSB*`

3. **ROS 2 Humble**
   - Full desktop installation
   - `rclpy` + `rclrs` support
   - Colcon build tools

### udev Rules (in Dockerfile)

```bash
echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/50-usb-serial.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2341", MODE="0666"' >> /etc/udev/rules.d/50-usb-serial.rules
```

These allow Docker containers to access serial ports without `--privileged`.

---

## Enabling Real Hardware (Non-SITL)

### Step 1: Identify Device Paths (Host)
```bash
# Pixhawk serial
ls -la /dev/ttyUSB* /dev/ttyACM*

# Cameras (RGB, depth)
ls -la /dev/video*
```

### Step 2: Update docker-compose.yml

For `sitl_drone_1` (replace with your device paths):
```yaml
sitl_drone_1:
  devices:
    - /dev/ttyUSB0:/dev/ttyUSB0         # Pixhawk serial
    - /dev/video0:/dev/video0           # RGB camera
    - /dev/video1:/dev/video1           # Intel RealSense / OAK-D depth
```

**OR global privileged access (not recommended):**
```yaml
sitl_drone_1:
  privileged: true  # Full device access (less secure)
```

### Step 3: Launch with Specific Services
```bash
# Start Drone 1 with real hardware
docker-compose --profile swarm_sitl up sitl_drone_1 swarm_node_1 perception_node_1 zenoh_bridge_1

# Start all 6 drones (SITL mode - no real devices needed)
docker-compose --profile swarm_sitl up -d
```

---

## Troubleshooting

### Issue: "Permission denied" on `/dev/ttyUSB0`

**Solution 1:** Use udev rules (container will have permission):
```bash
# In docker-compose, map the device:
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

**Solution 2:** Run with limited privilege (macro):
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/ttyACM0:/dev/ttyACM0
group_add:
  - dialout  # Group that owns serial ports
```

### Issue: Drone N not connecting to ROS 2 mesh

**Check:**
```bash
# Inside container
docker exec swarm_node_N bash -c "source /opt/ros/humble/setup.bash && ros2 node list"

# Should show: /cargo_node_${DRONE_ID}
```

If empty:
1. Verify `ROS_DOMAIN_ID=N` is set
2. Check Zenoh bridge logs: `docker logs zenoh_bridge_N`
3. Ensure `sitl_drone_N` is running (healthcheck passes)

### Issue: Perception node crashes

**Debug:**
```bash
docker logs perception_node_N --follow

# If CUDA errors: perception is expecting GPU
# For CPU mode, update `perception_core/detector.py` to use CPU:
# model = YOLO("yolov8n.pt").to("cpu")
```

---

## Multi-Drone Swarm Network Topology

```
┌──────────────────────────────────────────────┐
│          Docker Bridge Network (10.10.1.0/24) │
├──────────────────┬──────────────────┬────────┤
│  Drone 1 Stack   │  Drone 2 Stack   │  ...D6 │
├──────────────────┼──────────────────┼────────┤
│ sitl_drone_1     │ sitl_drone_2     │ ...    │  10.10.1.11-16
│   (Pixhawk sim)  │   (Pixhawk sim)  │        │
│                  │                  │        │
│ swarm_node_1     │ swarm_node_2     │ ...    │  10.10.1.21-26
│  (Control logic) │  (Control logic) │        │
│   ROS Domain 1   │   ROS Domain 2   │   D3-6 │
│                  │                  │        │
│ perception_1     │ perception_2     │ ...    │  10.10.1.31-36
│  (Vision)        │  (Vision)        │        │
│   ROS Domain 1   │   ROS Domain 2   │        │
│                  │                  │        │
│ zenoh_bridge_1   │ zenoh_bridge_2   │ ...    │  10.10.1.41-46
│  (Mesh router)   │  (Mesh router)   │        │
└──────────────────┴──────────────────┴────────┘
         ↓                ↓                  ↓
      ┌──────────────────────────────────────┐
      │     Zenoh Mesh Network               │
      │  (Inter-Drone Topic Exchange)        │
      └──────────────────────────────────────┘
```

---

## Key Takeaways

✅ **Each drone has:**
- 1 driver/sim container (Micro-XRCE-DDS Agent, Pixhawk bridge)
- 1 swarm control node (Rust, `cargo run`)
- 1 perception node (Python, vision pipeline)
- 1 Zenoh bridge (mesh networking)
- Unique `ROS_DOMAIN_ID` (1-6)

✅ **Communication:**
- **Intra-drone:** ROS 2 DDS (within `ROS_DOMAIN_ID`)
- **Inter-drone:** Zenoh mesh (bridges isolated domains)
- **Hardware:** Serial (Pixhawk), USB (cameras), configured via device mappings

✅ **High Availability:**
- `restart: always` on control/perception nodes
- Healthchecks verify process liveness
- Zenoh bridges auto-reconnect on network disruption

---

*Reference: `/Users/beret/dev/swarm_digital_twin/DOCKER_DRIVER_SETUP.md`*
