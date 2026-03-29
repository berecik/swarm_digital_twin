#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Usage:
#   ./run_scenario.sh              # run single-drone SITL mission then open 3D visualization
#   ./run_scenario.sh --single     # run single-drone SITL mission then visualize
#   ./run_scenario.sh --swarm [N]  # run N-drone SITL swarm (default: 3) then visualize
#   ./run_scenario.sh --sim-only   # run single-drone SITL mission only (no GUI)
#   ./run_scenario.sh --viz-only   # open visualization (using existing data)
#   ./run_scenario.sh --sitl-viz [F] # visualize SITL .BIN flight log (newest or specified file)
#   ./run_scenario.sh --test       # run physics tests
#   ./run_scenario.sh --benchmark  # run deterministic benchmark validation gates
#   ./run_scenario.sh --real-log   # run real-log validation against paper Table 5
#   ./run_scenario.sh --ci-local   # run local CI/CD-equivalent pipeline (tests + all benchmarks)
#   ./run_scenario.sh --all        # run tests, benchmark, SITL scenario, and visualization
#   ./run_scenario.sh --physics-single   # [legacy] run Python physics single-drone scenario
#   ./run_scenario.sh --physics-swarm [N] # [legacy] run Python physics swarm scenario
#   ./run_scenario.sh --physics-sim-only  # [legacy] run Python physics scenario (no GUI)
# ──────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_DIR="$ROOT_DIR/simulation"
VENV_DIR="$ROOT_DIR/.venv"

# ── Colors ───────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

# ── Virtual environment ──────────────────────────────────────────────────────
ensure_venv() {
    if [ ! -d "$VENV_DIR" ]; then
        info "Creating virtual environment..."
        if command -v uv &>/dev/null; then
            uv venv "$VENV_DIR"
        else
            python3 -m venv "$VENV_DIR"
        fi
    fi

    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"

    # Check deps
    if ! python -c "import numpy, matplotlib" &>/dev/null; then
        info "Installing dependencies (numpy, matplotlib)..."
        if command -v uv &>/dev/null; then
            uv pip install numpy matplotlib
        else
            pip install numpy matplotlib
        fi
    fi

    # pytest needed for --test/--all
    if [[ "${RUN_TESTS:-0}" == "1" ]] && ! python -c "import pytest" &>/dev/null; then
        info "Installing pytest..."
        if command -v uv &>/dev/null; then
            uv pip install pytest
        else
            pip install pytest
        fi
    fi

    # pymavlink needed for --sitl/--sitl-swarm
    if [[ "${NEED_PYMAVLINK:-0}" == "1" ]] && ! python -c "import pymavlink" &>/dev/null; then
        info "Installing pymavlink for SITL orchestration..."
        if command -v uv &>/dev/null; then
            uv pip install pymavlink future lxml
        else
            pip install pymavlink future lxml
        fi
    fi

    ok "Virtual environment ready"
}

# ── Actions ──────────────────────────────────────────────────────────────────
run_tests() {
    info "Running drone physics tests..."
    python -m pytest "$SIM_DIR/test_drone_physics.py" -v
    ok "All physics tests passed"
}

run_scenario() {
    local drone_count="${1:-6}"
    info "Running swarm flight scenario..."
    python "$SIM_DIR/swarm_scenario.py" --drones "$drone_count"
    ok "Swarm scenario complete — data saved"
}

run_single_scenario() {
    info "Running single-drone flight scenario..."
    python "$SIM_DIR/drone_scenario.py"
    ok "Single-drone scenario complete — data saved"
}

run_benchmark() {
    info "Running deterministic benchmark validation gates..."
    local profiles=(moderate strong_wind crosswind storm)
    local swarm_profiles=(baseline crosswind gusty)
    local profile
    for profile in "${profiles[@]}"; do
        info "Running single-drone benchmark: $profile"
        python "$SIM_DIR/drone_scenario.py" --benchmark "$profile"
    done

    for profile in "${swarm_profiles[@]}"; do
        info "Running swarm benchmark: $profile"
        python "$SIM_DIR/swarm_scenario.py" --benchmark "$profile"
    done

    ok "Deterministic benchmark validation passed"
}

run_real_log() {
    info "Running real-log validation (paper Table 5)..."
    python "$SIM_DIR/drone_scenario.py" --real-log
    ok "Real-log validation passed"
}

run_ci_local() {
    info "Running local CI/CD pipeline equivalent (.github/workflows/ci.yml)..."

    info "[CI local] Physics test suite"
    python -m pytest "$SIM_DIR/test_drone_physics.py" -v --tb=short

    info "[CI local] Fixed-wing benchmarks"
    python "$SIM_DIR/drone_scenario.py" --benchmark moderate
    python "$SIM_DIR/drone_scenario.py" --benchmark strong_wind
    python "$SIM_DIR/drone_scenario.py" --benchmark crosswind
    python "$SIM_DIR/drone_scenario.py" --benchmark storm

    info "[CI local] IRS-4 quadrotor benchmarks"
    python "$SIM_DIR/drone_scenario.py" --benchmark irs4_carolina
    python "$SIM_DIR/drone_scenario.py" --benchmark irs4_epn

    info "[CI local] Swarm benchmarks"
    python "$SIM_DIR/swarm_scenario.py" --benchmark baseline
    python "$SIM_DIR/swarm_scenario.py" --benchmark crosswind
    python "$SIM_DIR/swarm_scenario.py" --benchmark gusty

    ok "Local CI/CD pipeline equivalent passed"
}

run_viz() {
    local data_file="${1:-$SIM_DIR/scenario_data.npz}"
    if [ ! -f "$data_file" ]; then
        fail "No data found at $data_file — run a simulation first"
    fi
    info "Launching 3D visualization..."
    python "$SIM_DIR/visualize_drone_3d.py" "$data_file"
}

run_single_viz() {
    run_viz "$SIM_DIR/scenario_data.npz"
}

run_sitl_viz() {
    local bin_file="${1:-}"
    if [ -z "$bin_file" ]; then
        # Find the newest SITL .BIN log
        bin_file=$(find "$ROOT_DIR/logs" -name "*.BIN" -type f 2>/dev/null | \
            xargs ls -t 2>/dev/null | head -1)
    fi
    if [ -z "$bin_file" ] || [ ! -f "$bin_file" ]; then
        fail "No SITL .BIN log found. Run a SITL mission first: ./scripts/run_sitl_mission.sh"
    fi
    info "Visualizing SITL flight log: $bin_file"
    python "$SIM_DIR/visualize_drone_3d.py" "$bin_file"
}

# ── SITL Actions ──────────────────────────────────────────────────────────────

SITL_IMAGE="ardupilot-sitl:latest"
SITL_NETWORK="swarm_digital_twin_swarm_net"
SITL_REF_LAT="-0.508333"
SITL_REF_LON="-78.141667"
SITL_REF_ALT="4500"

sitl_ensure_network() {
    # Ensure the compose-managed network exists with correct labels.
    # If a stale (non-compose) network exists, remove it and let compose recreate.
    if docker network inspect "$SITL_NETWORK" &>/dev/null; then
        local net_label
        net_label=$(docker network inspect "$SITL_NETWORK" \
            --format '{{index .Labels "com.docker.compose.network"}}' 2>/dev/null || true)
        if [ "$net_label" != "swarm_net" ]; then
            warn "Removing stale network $SITL_NETWORK (wrong labels)..."
            docker network rm "$SITL_NETWORK" 2>/dev/null || true
        fi
    fi
    if ! docker network inspect "$SITL_NETWORK" &>/dev/null; then
        info "Creating Docker network via compose..."
        cd "$ROOT_DIR"
        docker compose --profile sitl up --no-start 2>&1 | tail -3
        docker compose --profile sitl rm -f 2>/dev/null || true
    fi
}

sitl_wait_healthy() {
    local container="$1"
    local timeout="${2:-90}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        if docker exec "$container" sh -c "pgrep -x 'arducopter|arduplane'" \
            >/dev/null 2>&1; then
            return 0
        fi
        sleep 3
        elapsed=$((elapsed + 3))
        if [ $((elapsed % 15)) -eq 0 ]; then
            info "  Waiting for $container... (${elapsed}s)"
        fi
    done
    return 1
}

run_sitl_single() {
    local timeout="${1:-300}"
    local log_dir="$ROOT_DIR/logs/sitl_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$log_dir"

    info "Starting single-drone SITL mission..."

    # Generate mission
    local mission_dir="$log_dir/missions"
    python "$SIM_DIR/sitl_waypoints.py" --n 1 --output-dir "$mission_dir"
    local mission_file="$mission_dir/drone_0.waypoints"

    # Start SITL via compose (compose manages its own network)
    cd "$ROOT_DIR"
    docker compose --profile sitl up -d 2>&1 | tail -5
    local sitl_container="ardupilot_sitl"

    # Cleanup on exit
    SITL_SINGLE_CLEANUP_DONE=false
    sitl_single_cleanup() {
        if [ "$SITL_SINGLE_CLEANUP_DONE" = false ]; then
            SITL_SINGLE_CLEANUP_DONE=true
            info "Stopping SITL stack..."
            cd "$ROOT_DIR"
            docker compose --profile sitl down --timeout 10 2>/dev/null || true
        fi
    }
    trap sitl_single_cleanup EXIT

    # Wait for healthy
    info "Waiting for ArduPilot SITL..."
    if ! sitl_wait_healthy "$sitl_container" 90; then
        fail "SITL health check failed"
    fi
    ok "SITL is healthy"

    # Run orchestrator from host venv
    info "Running SITL mission (timeout=${timeout}s)..."
    python "$SIM_DIR/sitl_orchestrator.py" single \
        --port 5760 \
        --mission "$mission_file" \
        --timeout "$timeout" || true

    # Capture logs
    info "Capturing flight logs..."
    mkdir -p "$log_dir/flight_logs"
    docker cp "$sitl_container:/sitl/logs/." "$log_dir/flight_logs/" 2>/dev/null || true

    # Find newest .BIN
    local newest_bin
    newest_bin=$(find "$log_dir/flight_logs" -name "*.BIN" -type f 2>/dev/null | \
        xargs ls -t 2>/dev/null | head -1)

    if [ -z "$newest_bin" ]; then
        warn "No .BIN flight log captured"
        return 1
    fi

    # Convert to scenario_data.npz
    info "Converting flight log to NPZ..."
    python "$SIM_DIR/sitl_log_merger.py" single "$newest_bin"

    ok "SITL single-drone mission complete (log: $log_dir)"
}

run_sitl_swarm() {
    local n_drones="${1:-3}"
    local timeout="${2:-600}"
    local log_dir="$ROOT_DIR/logs/sitl_swarm_$(date +%Y%m%d_%H%M%S)"
    local base_port=5760
    local port_step=10
    mkdir -p "$log_dir"

    info "Starting ${n_drones}-drone SITL swarm..."

    # Generate ring-formation missions
    local mission_dir="$log_dir/missions"
    python "$SIM_DIR/sitl_waypoints.py" \
        --n "$n_drones" --output-dir "$mission_dir" \
        --ref-lat "$SITL_REF_LAT" --ref-lon "$SITL_REF_LON" --ref-alt "$SITL_REF_ALT"

    # Get per-drone home GPS from the generated missions
    # (parse from the waypoint files — home is line 2 of each file)
    sitl_ensure_network

    # Cleanup on exit (use global so the trap can access it after local scope ends)
    _SITL_SWARM_N_DRONES="$n_drones"
    sitl_swarm_cleanup() {
        info "Cleaning up swarm containers..."
        for i in $(seq 0 $((_SITL_SWARM_N_DRONES - 1))); do
            docker rm -f "sitl_swarm_${i}" 2>/dev/null || true
        done
    }
    trap sitl_swarm_cleanup EXIT

    # Launch N containers with staggered startup
    info "Launching $n_drones SITL containers..."
    for i in $(seq 0 $((n_drones - 1))); do
        local container_name="sitl_swarm_${i}"
        local ip="10.10.1.$((60 + i))"
        local host_port=$((base_port + i * port_step))

        # Read home GPS from the generated mission file (line 2, fields 9-11)
        local mission_file="$mission_dir/drone_${i}.waypoints"
        local home_lat home_lon
        home_lat=$(awk 'NR==2 {print $9}' "$mission_file")
        home_lon=$(awk 'NR==2 {print $10}' "$mission_file")

        docker rm -f "$container_name" 2>/dev/null || true
        docker run -d \
            --name "$container_name" \
            --platform linux/amd64 \
            --network "$SITL_NETWORK" \
            --ip "$ip" \
            -p "${host_port}:5760/tcp" \
            -e "SIM_LAT=${home_lat}" \
            -e "SIM_LNG=${home_lon}" \
            -e "SIM_ALT=${SITL_REF_ALT}" \
            -e "INSTANCE=0" \
            "$SITL_IMAGE" copter >/dev/null

        info "  drone_${i}: ${container_name} (${ip}, port ${host_port})"

        # Stagger startup to avoid QEMU overload
        if [ "$i" -lt $((n_drones - 1)) ]; then
            sleep 5
        fi
    done

    # Wait for all containers to be healthy
    info "Waiting for all SITL instances..."
    for i in $(seq 0 $((n_drones - 1))); do
        if ! sitl_wait_healthy "sitl_swarm_${i}" 120; then
            fail "sitl_swarm_${i} health check failed"
        fi
        ok "  sitl_swarm_${i} is healthy"
    done

    # Run swarm orchestrator from host
    info "Running swarm mission (${n_drones} drones, timeout=${timeout}s)..."
    python "$SIM_DIR/sitl_orchestrator.py" swarm \
        --n "$n_drones" \
        --base-port "$base_port" \
        --port-step "$port_step" \
        --mission-dir "$mission_dir" \
        --timeout "$timeout" || true

    # Capture logs from each container
    info "Capturing flight logs from $n_drones containers..."
    local log_dirs=()
    for i in $(seq 0 $((n_drones - 1))); do
        local drone_log_dir="$log_dir/drone_${i}"
        mkdir -p "$drone_log_dir"
        docker cp "sitl_swarm_${i}:/sitl/logs/." "$drone_log_dir/" 2>/dev/null || true
        local bin_count
        bin_count=$(find "$drone_log_dir" -name "*.BIN" 2>/dev/null | wc -l | tr -d ' ')
        info "  drone_${i}: ${bin_count} .BIN file(s)"
        if [ "$bin_count" -gt 0 ]; then
            log_dirs+=("$drone_log_dir")
        fi
    done

    if [ ${#log_dirs[@]} -eq 0 ]; then
        warn "No .BIN logs captured from any drone"
        return 1
    fi

    # Merge into swarm_data.npz
    info "Merging ${#log_dirs[@]} drone logs into swarm data..."
    python "$SIM_DIR/sitl_log_merger.py" swarm --log-dirs "${log_dirs[@]}"

    ok "SITL swarm mission complete (${#log_dirs[@]}/${n_drones} drones, log: $log_dir)"
}

# ── Parse args ───────────────────────────────────────────────────────────────
MODE="${1:---default}"
SWARM_DRONES="${2:-3}"

case "$MODE" in
    # ── Primary modes (ArduPilot SITL — requires Docker) ─────────────────────
    --single|--sitl)
        NEED_PYMAVLINK=1 ensure_venv
        run_sitl_single 300
        run_single_viz
        ;;
    --swarm|--sitl-swarm)
        NEED_PYMAVLINK=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
            SWARM_DRONES=3
        fi
        run_sitl_swarm "$SWARM_DRONES" 600
        run_viz "$SIM_DIR/swarm_data.npz"
        ;;
    --sim-only)
        NEED_PYMAVLINK=1 ensure_venv
        run_sitl_single 300
        ;;
    --viz-only)
        ensure_venv
        run_single_viz
        ;;
    --sitl-viz)
        ensure_venv
        run_sitl_viz "${2:-}"
        ;;

    # ── Legacy modes (Python physics engine — no Docker needed) ──────────────
    --physics-single)
        ensure_venv
        run_single_scenario
        run_single_viz
        ;;
    --physics-swarm)
        ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
            SWARM_DRONES=6
        fi
        run_scenario "$SWARM_DRONES"
        run_viz "$SIM_DIR/swarm_data.npz"
        ;;
    --physics-sim-only)
        ensure_venv
        run_single_scenario
        ;;

    # ── Testing & validation ─────────────────────────────────────────────────
    --test)
        RUN_TESTS=1 ensure_venv
        run_tests
        ;;
    --ci-local)
        RUN_TESTS=1 ensure_venv
        run_ci_local
        ;;
    --all)
        RUN_TESTS=1 NEED_PYMAVLINK=1 ensure_venv
        run_tests
        run_benchmark
        run_real_log
        run_sitl_single 300
        run_single_viz
        ;;
    --benchmark)
        ensure_venv
        run_benchmark
        ;;
    --real-log)
        ensure_venv
        run_real_log
        ;;

    # ── Help ─────────────────────────────────────────────────────────────────
    --help|-h)
        echo "Usage: $0 [MODE]"
        echo ""
        echo "ArduPilot SITL (default — requires Docker):"
        echo "  (default)       Run single-drone SITL mission then open 3D visualization"
        echo "  --single        Run single-drone SITL mission then visualize"
        echo "  --swarm [N]     Run N-drone SITL swarm (default: 3) then visualize"
        echo "  --sim-only      Run single-drone SITL mission only (no GUI)"
        echo "  --viz-only      Open visualization (uses existing data)"
        echo "  --sitl-viz [F]  Visualize newest SITL .BIN log (or specify path)"
        echo ""
        echo "Legacy physics simulation (no Docker needed):"
        echo "  --physics-single     Run Python physics single-drone scenario + visualize"
        echo "  --physics-swarm [N]  Run Python physics swarm for N drones (default: 6)"
        echo "  --physics-sim-only   Run Python physics scenario only (no GUI)"
        echo ""
        echo "Testing & validation:"
        echo "  --test          Run physics tests"
        echo "  --benchmark     Run deterministic benchmark validation gates"
        echo "  --real-log      Run real-log validation against paper Table 5"
        echo "  --ci-local      Run local CI/CD-equivalent pipeline (tests + all CI benchmarks)"
        echo "  --all           Run tests, benchmark, SITL scenario, and visualization"
        echo "  --help          Show this help"
        ;;

    # ── Default (no args) ────────────────────────────────────────────────────
    --default)
        NEED_PYMAVLINK=1 ensure_venv
        run_sitl_single 300
        run_single_viz
        ;;
    *)
        fail "Unknown option: $MODE (use --help)"
        ;;
esac
