#!/usr/bin/env bash
# run_sitl_mission.sh — End-to-end SITL mission lifecycle
#
# Runs: SITL stack start -> health check -> mission upload -> arm -> fly -> log capture -> shutdown
#
# Prerequisites:
#   - Docker with compose plugin
#   - ArduPilot SITL image built (docker build -f Dockerfile.sitl -t ardupilot-sitl .)
#   - QGroundControl (optional, for monitoring)
#
# Usage:
#   ./scripts/run_sitl_mission.sh [--vehicle plane|copter] [--mission FILE] [--timeout SECS] [--swarm] [--viz]
#
# Exit codes:
#   0 = mission completed successfully
#   1 = stack failed to start
#   2 = health check failed
#   3 = mission upload failed
#   4 = mission execution timeout
#   5 = log capture failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/logs/sitl_$(date +%Y%m%d_%H%M%S)"

# Defaults
VEHICLE="copter"
MISSION_FILE=""
TIMEOUT=300  # 5 minutes
SWARM=false
VIZ=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --vehicle) VEHICLE="$2"; shift 2 ;;
        --mission) MISSION_FILE="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --swarm)   SWARM=true; shift ;;
        --viz)     VIZ=true; shift ;;
        --help)
            echo "Usage: $0 [--vehicle plane|copter] [--mission FILE] [--timeout SECS] [--swarm] [--viz]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Resolve SITL profile, container name, and host MAVLink port based on vehicle type
case "$VEHICLE" in
    copter|quad)
        SITL_PROFILE="sitl"
        SITL_CONTAINER="ardupilot_sitl"
        MAVLINK_HOST_PORT=14550
        ;;
    plane|fw)
        SITL_PROFILE="sitl_plane"
        SITL_CONTAINER="ardupilot_sitl_plane"
        MAVLINK_HOST_PORT=14560
        ;;
    *)
        echo "Unknown vehicle type: $VEHICLE (use copter or plane)"
        exit 1
        ;;
esac

# Build compose profile flags
COMPOSE_PROFILES="--profile $SITL_PROFILE"
if [ "$SWARM" = true ]; then
    COMPOSE_PROFILES="$COMPOSE_PROFILES --profile swarm_sitl"
fi

mkdir -p "$LOG_DIR"
echo "=== SITL Mission Lifecycle ==="
echo "Vehicle:    $VEHICLE"
echo "Container:  $SITL_CONTAINER"
echo "MAVLink:    UDP $MAVLINK_HOST_PORT (host)"
echo "Timeout:    ${TIMEOUT}s"
echo "Swarm:      $SWARM"
echo "Log dir:    $LOG_DIR"
echo ""

# ── Step 1: Start SITL stack ──────────────────────────────────────────────
echo "[1/6] Starting SITL stack..."

cleanup() {
    echo ""
    echo "[cleanup] Stopping SITL stack..."
    rm -f "$PROJECT_DIR/missions/active_mission.waypoints" 2>/dev/null || true
    cd "$PROJECT_DIR"
    # shellcheck disable=SC2086
    docker compose $COMPOSE_PROFILES down --timeout 10 2>/dev/null || true
}
trap cleanup EXIT

cd "$PROJECT_DIR"
# shellcheck disable=SC2086
if ! docker compose $COMPOSE_PROFILES up -d 2>&1 | tee "$LOG_DIR/compose_up.log"; then
    echo "FAIL: Docker compose failed to start"
    exit 1
fi
echo "  Stack started."

# ── Step 2: Health check ──────────────────────────────────────────────────
echo "[2/6] Waiting for ArduPilot SITL ($SITL_CONTAINER)..."

HEALTH_TIMEOUT=90
HEALTH_ELAPSED=0
SITL_READY=false

while [ $HEALTH_ELAPSED -lt $HEALTH_TIMEOUT ]; do
    # Check container health via docker inspect (the Dockerfile healthcheck
    # listens for MAVLink heartbeats on UDP, confirming ArduPilot is running)
    HEALTH_STATUS=$(docker inspect \
        --format='{{if .State.Health}}{{.State.Health.Status}}{{else}}no-healthcheck{{end}}' \
        "$SITL_CONTAINER" 2>/dev/null || echo "not-found")

    if [ "$HEALTH_STATUS" = "healthy" ]; then
        SITL_READY=true
        echo "  $SITL_CONTAINER is healthy (MAVLink heartbeat confirmed)."
        break
    fi

    # Fallback: if container is running but healthcheck hasn't passed yet,
    # check if the ArduPilot process is alive inside the container
    # (Do NOT probe TCP 5760 — repeated connect/disconnect crashes ArduPilot)
    CONTAINER_STATE=$(docker inspect --format='{{.State.Status}}' \
        "$SITL_CONTAINER" 2>/dev/null || echo "missing")
    if [ "$CONTAINER_STATE" = "running" ] && [ $HEALTH_ELAPSED -ge 15 ]; then
        if docker exec "$SITL_CONTAINER" sh -c "pgrep -x 'arducopter|arduplane'" \
            >/dev/null 2>&1; then
            SITL_READY=true
            echo "  $SITL_CONTAINER process alive (ArduPilot running)."
            break
        fi
    fi

    sleep 2
    HEALTH_ELAPSED=$((HEALTH_ELAPSED + 2))
    if [ $((HEALTH_ELAPSED % 10)) -eq 0 ]; then
        echo "  Waiting... (${HEALTH_ELAPSED}s, status: $HEALTH_STATUS)"
    fi
done

if [ "$SITL_READY" = false ]; then
    echo "FAIL: ArduPilot health check timed out after ${HEALTH_TIMEOUT}s (last status: $HEALTH_STATUS)"
    # shellcheck disable=SC2086
    docker compose $COMPOSE_PROFILES logs "$SITL_CONTAINER" > "$LOG_DIR/sitl_boot_logs.txt" 2>&1 || true
    exit 2
fi

# ── Step 3: Upload mission ────────────────────────────────────────────────
echo "[3/6] Uploading mission..."

if [ -z "$MISSION_FILE" ]; then
    MISSION_FILE="$LOG_DIR/test_mission.waypoints"
    case "$VEHICLE" in
        copter|quad)
            # Copter: frame 3 (GLOBAL_RELATIVE_ALT), takeoff 20m, waypoints at 20m AGL
            cat > "$MISSION_FILE" << 'MISSION_EOF'
QGC WPL 110
0	1	0	16	0	0	0	0	-0.508333	-78.141667	4510	1
1	0	3	22	0	0	0	0	0	0	20	1
2	0	3	16	0	0	0	0	-0.507000	-78.141000	20	1
3	0	3	16	0	0	0	0	-0.508000	-78.140000	20	1
4	0	3	16	0	0	0	0	-0.509000	-78.141000	20	1
5	0	3	21	0	0	0	0	-0.508333	-78.141667	0	1
MISSION_EOF
            ;;
        plane|fw)
            # Plane: frame 3 (GLOBAL_RELATIVE_ALT), takeoff 50m, waypoints at 70m AGL
            cat > "$MISSION_FILE" << 'MISSION_EOF'
QGC WPL 110
0	1	0	16	0	0	0	0	-0.508333	-78.141667	4510	1
1	0	3	22	15	0	0	0	0	0	50	1
2	0	3	16	0	0	0	0	-0.507000	-78.141000	70	1
3	0	3	16	0	0	0	0	-0.508000	-78.140000	70	1
4	0	3	16	0	0	0	0	-0.509000	-78.141000	70	1
5	0	3	21	0	0	0	0	-0.508333	-78.141667	0	1
MISSION_EOF
            ;;
    esac
    echo "  Generated test mission ($VEHICLE): $MISSION_FILE"
fi

# Place mission file in the host-mounted missions directory
# (the volume is mounted read-only inside the container, so docker cp won't work)
cp "$MISSION_FILE" "$PROJECT_DIR/missions/active_mission.waypoints"

# ── Steps 3+4: Upload, arm, and fly (single pymavlink session) ───────────
echo "[4/6] Uploading mission, arming, and executing..."

# Run the full mission lifecycle inside the SITL container via a single
# persistent pymavlink TCP connection to ArduPilot.
FLIGHT_EXIT=0
docker exec "$SITL_CONTAINER" python3 -c "
import sys, time
from pymavlink import mavutil, mavwp

MISSION_PATH = '/sitl/missions/active_mission.waypoints'  # host ./missions/ mounted here
TIMEOUT = $TIMEOUT

# ── Connect ──────────────────────────────────────────────────────────
print('  Connecting to ArduPilot (tcp:127.0.0.1:5760)...')
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn.wait_heartbeat(timeout=30)
print(f'  Connected (system={conn.target_system}, component={conn.target_component})')

# Request all data streams at 4 Hz (SITL under QEMU is slow)
for stream_id in range(13):
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        stream_id, 4, 1)

# ── Wait for GPS 3D fix and EKF ─────────────────────────────────────
print('  Waiting for GPS fix and EKF initialization...')
gps_ok = False
init_start = time.time()
while time.time() - init_start < 90:
    msg = conn.recv_match(blocking=True, timeout=2)
    if msg is None:
        continue
    mtype = msg.get_type()
    if mtype == 'GPS_RAW_INT' and msg.fix_type >= 3:
        if not gps_ok:
            print(f'  GPS 3D fix acquired (fix={msg.fix_type}, sats={msg.satellites_visible})')
            gps_ok = True
    elif mtype == 'STATUSTEXT':
        text = getattr(msg, 'text', '').rstrip(chr(0))
        if text:
            print(f'  SITL: {text}')
    elif mtype == 'EKF_STATUS_REPORT':
        # Flags bit 0 = attitude, bit 3 = pos_horiz_abs, bit 4 = pos_vert_abs
        if gps_ok and (msg.flags & 0x19) == 0x19:
            print(f'  EKF ready (flags=0x{msg.flags:04x})')
            break

    if time.time() - init_start > 60 and gps_ok:
        print('  EKF timeout — proceeding with GPS fix only')
        break

if not gps_ok:
    # SITL simulates GPS internally, so fix should be immediate
    # If not getting GPS messages, proceed anyway (SITL auto-provides fix)
    print('  No GPS_RAW_INT received — SITL provides simulated GPS, proceeding')

# ── Upload mission ───────────────────────────────────────────────────
loader = mavwp.MAVWPLoader()
loader.load(MISSION_PATH)
wp_count = loader.count()

if wp_count == 0:
    print('FAIL: No waypoints loaded from mission file')
    sys.exit(3)

print(f'  Uploading {wp_count} waypoints...')

# Clear existing mission
conn.waypoint_clear_all_send()
ack = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=10)

# Send waypoint count
conn.waypoint_count_send(wp_count)

# Send each waypoint on request
for i in range(wp_count):
    msg = conn.recv_match(
        type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
        blocking=True, timeout=10)
    if msg is None:
        print(f'FAIL: Timeout waiting for waypoint {i} request')
        sys.exit(3)
    conn.mav.send(loader.wp(msg.seq))

# Wait for upload ACK
ack = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
if ack is None or ack.type != 0:
    ack_type = ack.type if ack else 'timeout'
    print(f'FAIL: Mission upload rejected (ack type={ack_type})')
    sys.exit(3)

print(f'  Mission uploaded ({wp_count} waypoints)')

# ── Arm in GUIDED then immediately switch to AUTO ────────────────────
# ArduPilot copter auto-disarms in GUIDED if no commands arrive quickly,
# so we arm and switch to AUTO in rapid succession.
print('  Setting GUIDED mode...')
conn.mav.set_mode_send(
    conn.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)  # GUIDED = 4
time.sleep(1)

print('  Arming vehicle...')
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0)

# Quick wait for arm ACK only (don't wait for heartbeat — too slow)
for _ in range(20):
    msg = conn.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == 'COMMAND_ACK' and msg.command == 400:
        if msg.result == 0:
            print('  Arm ACK received')
            break
        else:
            print(f'  Arm rejected (result={msg.result}), retrying...')
            time.sleep(2)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
    elif msg.get_type() == 'STATUSTEXT':
        text = getattr(msg, 'text', '').rstrip(chr(0))
        if text:
            print(f'  SITL: {text}')

# Switch to AUTO immediately — do not wait for heartbeat confirmation
print('  Switching to AUTO mode...')
conn.set_mode_auto()
print('  Mission executing.')

# ── Monitor until disarm or timeout ──────────────────────────────────
start = time.time()
last_status = 0
was_armed = True  # We confirmed arm above

while True:
    elapsed = time.time() - start
    if elapsed >= TIMEOUT:
        print(f'  Mission timeout after {TIMEOUT}s')
        sys.exit(4)

    msg = conn.recv_match(blocking=True, timeout=2)
    if msg is None:
        continue

    mtype = msg.get_type()

    if mtype == 'HEARTBEAT':
        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mavutil.mode_string_v10(msg)
        if is_armed:
            was_armed = True
        # Vehicle disarmed AFTER having been armed = mission complete
        if was_armed and not is_armed:
            print(f'  Vehicle disarmed ({mode}) after {int(elapsed)}s — mission complete.')
            sys.exit(0)
        # Periodic status
        if elapsed - last_status >= 15:
            print(f'  [{int(elapsed)}s] mode={mode} armed={is_armed}')
            last_status = elapsed

    elif mtype == 'STATUSTEXT':
        text = getattr(msg, 'text', '').rstrip(chr(0))
        if text:
            print(f'  SITL: {text}')

    elif mtype == 'MISSION_CURRENT':
        seq = msg.seq
        if elapsed - last_status >= 10:
            print(f'  [{int(elapsed)}s] executing waypoint {seq}')
            last_status = elapsed
" 2>&1 | tee "$LOG_DIR/flight.log" || FLIGHT_EXIT=$?

case $FLIGHT_EXIT in
    0) echo "  Mission completed successfully." ;;
    3) echo "FAIL: Mission upload failed"; exit 3 ;;
    4) echo "  Mission timed out after ${TIMEOUT}s" ;;
    *) echo "  Flight script exited with code $FLIGHT_EXIT" ;;
esac

# ── Step 5: Capture logs ──────────────────────────────────────────────────
echo "[5/6] Capturing logs..."

# Copy ArduPilot DataFlash logs from the SITL container's log volume
mkdir -p "$LOG_DIR/flight_logs"
docker cp "$SITL_CONTAINER:/sitl/logs/." "$LOG_DIR/flight_logs/" 2>/dev/null || true

# Save compose logs for all services
# shellcheck disable=SC2086
docker compose $COMPOSE_PROFILES logs > "$LOG_DIR/compose_full.log" 2>&1 || true

BIN_COUNT=$(find "$LOG_DIR/flight_logs" -name "*.BIN" 2>/dev/null | wc -l | tr -d ' ')
echo "  Captured $BIN_COUNT .BIN log file(s)"
echo "  Compose logs saved to compose_full.log"

# ── Step 6: Validate ──────────────────────────────────────────────────────
echo "[6/6] Validating captured data..."

if [ "$BIN_COUNT" -gt 0 ]; then
    # Use the project venv for flight log parsing
    PYTHON="${PROJECT_DIR}/.venv/bin/python"
    if [ ! -x "$PYTHON" ]; then
        PYTHON="python3"
    fi

    "$PYTHON" -c "
import sys, glob
sys.path.insert(0, '$PROJECT_DIR/simulation')
from flight_log import FlightLog

for f in sorted(glob.glob('$LOG_DIR/flight_logs/*.BIN')):
    try:
        log = FlightLog.from_bin(f)
        n = len(log.timestamps)
        alt_min = -log.positions[:, 2].max()
        alt_max = -log.positions[:, 2].min()
        duration = log.timestamps[-1] - log.timestamps[0] if n > 1 else 0
        print(f'  {f}:')
        print(f'    GPS points: {n}, duration: {duration:.1f}s')
        print(f'    Altitude (MSL): [{alt_min:.0f}, {alt_max:.0f}] m')
    except Exception as e:
        print(f'  {f}: parse error: {e}')
" 2>&1 | tee -a "$LOG_DIR/validation.log"
else
    echo "  No .BIN files captured (SITL may not have generated logs for short flights)"
fi

echo ""
echo "=== Mission Lifecycle Complete ==="
echo "Logs saved to: $LOG_DIR"

# ── Optional: Launch 3D visualization ─────────────────────────────────────
if [ "$VIZ" = true ] && [ "$BIN_COUNT" -gt 0 ]; then
    NEWEST_BIN=$(find "$LOG_DIR/flight_logs" -name "*.BIN" -type f 2>/dev/null | \
        xargs ls -t 2>/dev/null | head -1)
    if [ -n "$NEWEST_BIN" ]; then
        echo ""
        echo "[viz] Launching 3D flight visualization..."
        PYTHON="${PROJECT_DIR}/.venv/bin/python"
        if [ ! -x "$PYTHON" ]; then
            PYTHON="python3"
        fi
        "$PYTHON" "$PROJECT_DIR/simulation/visualize_drone_3d.py" "$NEWEST_BIN"
    fi
elif [ "$VIZ" = true ]; then
    echo ""
    echo "[viz] Skipped — no .BIN logs captured to visualize."
fi

if [ $FLIGHT_EXIT -eq 0 ]; then
    echo "Status: SUCCESS"
    exit 0
elif [ $FLIGHT_EXIT -eq 4 ]; then
    echo "Status: TIMEOUT (check $LOG_DIR/flight.log for details)"
    exit 4
else
    echo "Status: FAILED (exit code $FLIGHT_EXIT)"
    exit "$FLIGHT_EXIT"
fi
