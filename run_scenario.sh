#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Per-Drone Stack (6 drones × 4 services each = 24 containers):
#   sitl_drone_N      — Pixhawk simulator with Micro-XRCE-DDS Agent
#   swarm_node_N      — Rust swarm control logic
#   perception_node_N — Vision pipeline (Python)
#   zenoh_bridge_N    — Mesh networking router
#
# Usage:
#   ./run_scenario.sh              # run single-drone stack (4 containers) then visualize
#   ./run_scenario.sh --single     # run single-drone stack (4 containers) then visualize
#   ./run_scenario.sh --swarm [N]  # run N-drone stack (default: 6, N×4 containers) then visualize
#   ./run_scenario.sh --sim-only   # run single-drone stack only (no GUI)
#   ./run_scenario.sh --swarm-only [N] # run N-drone stack only (no GUI)
#   ./run_scenario.sh --status     # show health of running swarm containers
#   ./run_scenario.sh --down       # tear down all swarm containers
#   ./run_scenario.sh --viz-only   # open visualization (using existing data)
#   ./run_scenario.sh --sitl-viz [F] # visualize SITL .BIN flight log (newest or specified file)
#   ./run_scenario.sh --test       # run physics + integration tests
#   ./run_scenario.sh --benchmark  # run deterministic benchmark validation gates
#   ./run_scenario.sh --real-log   # run real-log validation against paper Table 5
#   ./run_scenario.sh --ci-local   # run local CI/CD-equivalent pipeline (tests + all benchmarks)
#   ./run_scenario.sh --all        # run tests, benchmarks, single-drone mission, integration tests, and visualization
#   ./run_scenario.sh --test --timeout=300  # run tests with 300s pytest timeout
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
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*" >&2; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*" >&2; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*" >&2; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*" >&2; exit 1; }

# ── Constants ────────────────────────────────────────────────────────────────
MAX_DRONES=6
COMPOSE_PROFILE="swarm_sitl"
COMPOSE_CMD="docker compose"

# Service name templates — N is replaced with drone number (1-6)
SERVICE_SITL="sitl_drone"
SERVICE_SWARM="swarm_node"
SERVICE_PERCEPTION="perception_node"
SERVICE_ZENOH="zenoh_bridge"

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
        info "Installing pytest + pytest-timeout..."
        if command -v uv &>/dev/null; then
            uv pip install pytest pytest-timeout
        else
            pip install pytest pytest-timeout
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

# ── Docker Compose helpers ───────────────────────────────────────────────────

# Build list of services for N drones
# Usage: drone_services 3  →  "sitl_drone_1 swarm_node_1 perception_node_1 zenoh_bridge_1 sitl_drone_2 ..."
drone_services() {
    local n="${1:-$MAX_DRONES}"
    local services=""
    for i in $(seq 1 "$n"); do
        services+="${SERVICE_SITL}_${i} ${SERVICE_SWARM}_${i} ${SERVICE_PERCEPTION}_${i} ${SERVICE_ZENOH}_${i} "
    done
    echo "$services"
}

# Ensure Docker images are built
ensure_images() {
    if ! docker image inspect ardupilot-sitl:latest &>/dev/null; then
        info "Building ardupilot-sitl Docker image..."
        cd "$ROOT_DIR"
        docker build --platform linux/amd64 -f Dockerfile.sitl -t ardupilot-sitl:latest .
        ok "ardupilot-sitl image built"
    fi

    if ! docker image inspect swarm_companion:latest &>/dev/null; then
        info "Building swarm_companion Docker image..."
        cd "$ROOT_DIR"
        docker build -t swarm_companion:latest .
        ok "swarm_companion image built"
    fi
}

# Wait for a single container's healthcheck to pass
wait_container_healthy() {
    local container="$1"
    local timeout="${2:-120}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        local health
        health=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "missing")
        case "$health" in
            healthy) return 0 ;;
            unhealthy) return 1 ;;
            missing)
                # Container doesn't exist or has no healthcheck — check if running
                local state
                state=$(docker inspect --format='{{.State.Status}}' "$container" 2>/dev/null || echo "missing")
                if [ "$state" = "running" ]; then
                    return 0
                elif [ "$state" = "missing" ]; then
                    return 1
                fi
                ;;
        esac
        sleep 3
        elapsed=$((elapsed + 3))
    done
    return 1
}

# Wait for all services of N drones to be healthy
wait_swarm_healthy() {
    local n="${1:-$MAX_DRONES}"
    local timeout="${2:-180}"
    local all_ok=true

    info "Waiting for ${n}-drone stack to become healthy (timeout: ${timeout}s)..."

    for i in $(seq 1 "$n"); do
        for svc in "$SERVICE_SITL" "$SERVICE_SWARM" "$SERVICE_PERCEPTION" "$SERVICE_ZENOH"; do
            local container="${svc}_${i}"
            if wait_container_healthy "$container" "$timeout"; then
                ok "  ${container}"
            else
                warn "  ${container} — not healthy after ${timeout}s"
                all_ok=false
            fi
        done
    done

    if [ "$all_ok" = true ]; then
        ok "All $((n * 4)) containers healthy"
    else
        warn "Some containers did not pass health checks"
    fi
}

# ── Swarm lifecycle ──────────────────────────────────────────────────────────

swarm_up() {
    local n="${1:-$MAX_DRONES}"
    local log_dir="$ROOT_DIR/logs/swarm_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$log_dir"

    if [ "$n" -gt "$MAX_DRONES" ]; then
        fail "Maximum $MAX_DRONES drones supported (docker-compose defines 1-$MAX_DRONES)"
    fi

    ensure_images

    local services
    services=$(drone_services "$n")

    info "Launching ${n}-drone stack (${n}×4 = $((n * 4)) containers)..."
    echo "" >&2
    echo -e "${BOLD}  Per-Drone Stack:${NC}" >&2
    for i in $(seq 1 "$n"); do
        echo -e "    Drone $i: ${SERVICE_SITL}_${i}  ${SERVICE_SWARM}_${i}  ${SERVICE_PERCEPTION}_${i}  ${SERVICE_ZENOH}_${i}" >&2
    done
    echo "" >&2

    cd "$ROOT_DIR"
    # shellcheck disable=SC2086
    $COMPOSE_CMD --profile "$COMPOSE_PROFILE" up -d $services 2>&1 | tail -10 >&2

    wait_swarm_healthy "$n" 180

    ok "Swarm stack running ($((n * 4)) containers)"
    echo "" >&2
    swarm_status "$n"

    echo "$n" > "$log_dir/.drone_count"
    echo "$log_dir"  # sole stdout — captured by callers
}

swarm_down() {
    info "Tearing down swarm stack..."
    cd "$ROOT_DIR"
    $COMPOSE_CMD --profile "$COMPOSE_PROFILE" down --timeout 15 2>&1 | tail -5
    ok "Swarm stack stopped"
}

swarm_status() {
    local n="${1:-$MAX_DRONES}"

    echo -e "${BOLD}┌────────┬──────────┬───────────────┬───────────────┬────────────────┬───────────────┐${NC}" >&2
    echo -e "${BOLD}│ Drone  │ Domain   │ sitl_drone    │ swarm_node    │ perception     │ zenoh_bridge  │${NC}" >&2
    echo -e "${BOLD}├────────┼──────────┼───────────────┼───────────────┼────────────────┼───────────────┤${NC}" >&2

    for i in $(seq 1 "$n"); do
        local sitl_status swarm_status percep_status zenoh_status

        sitl_status=$(container_status_icon "${SERVICE_SITL}_${i}")
        swarm_status=$(container_status_icon "${SERVICE_SWARM}_${i}")
        percep_status=$(container_status_icon "${SERVICE_PERCEPTION}_${i}")
        zenoh_status=$(container_status_icon "${SERVICE_ZENOH}_${i}")

        printf "│   %-4s │   %-6s │  %-12s │  %-12s │  %-13s │  %-12s │\n" \
            "$i" "$i" "$sitl_status" "$swarm_status" "$percep_status" "$zenoh_status" >&2
    done

    echo -e "${BOLD}└────────┴──────────┴───────────────┴───────────────┴────────────────┴───────────────┘${NC}" >&2

    local running
    running=$(docker ps --filter "label=com.docker.compose.project" --format '{{.Names}}' 2>/dev/null | \
        grep -cE "(sitl_drone|swarm_node|perception_node|zenoh_bridge)_[1-6]" || true)
    echo -e "  Running containers: ${BOLD}${running}${NC} / $((n * 4))" >&2
}

container_status_icon() {
    local container="$1"
    local state health
    state=$(docker inspect --format='{{.State.Status}}' "$container" 2>/dev/null || echo "absent")
    health=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "none")

    if [ "$state" = "absent" ]; then
        echo -e "${RED}DOWN${NC}"
    elif [ "$state" != "running" ]; then
        echo -e "${RED}${state}${NC}"
    elif [ "$health" = "healthy" ]; then
        echo -e "${GREEN}healthy${NC}"
    elif [ "$health" = "unhealthy" ]; then
        echo -e "${RED}unhealthy${NC}"
    elif [ "$health" = "starting" ]; then
        echo -e "${YELLOW}starting${NC}"
    else
        echo -e "${GREEN}running${NC}"
    fi
}

# ── Swarm mission orchestration ──────────────────────────────────────────────

# Globals for trap cleanup (locals aren't accessible during EXIT unwinding with set -u)
_CLEANUP_DRONE_COUNT=""
_CLEANUP_LOG_DIR=""
_SWARM_CLEANUP_DONE=false

swarm_cleanup() {
    if [ "$_SWARM_CLEANUP_DONE" = false ]; then
        _SWARM_CLEANUP_DONE=true
        info "Capturing logs before shutdown..."

        local n="${_CLEANUP_DRONE_COUNT:-0}"
        local log_dir="${_CLEANUP_LOG_DIR:-}"

        if [ "$n" -gt 0 ] && [ -n "$log_dir" ]; then
            for i in $(seq 1 "$n"); do
                local drone_log_dir="$log_dir/drone_${i}"
                mkdir -p "$drone_log_dir"
                docker logs "${SERVICE_SWARM}_${i}" > "$drone_log_dir/swarm_node.log" 2>&1 || true
                docker logs "${SERVICE_PERCEPTION}_${i}" > "$drone_log_dir/perception_node.log" 2>&1 || true
                docker logs "${SERVICE_ZENOH}_${i}" > "$drone_log_dir/zenoh_bridge.log" 2>&1 || true
                docker logs "${SERVICE_SITL}_${i}" > "$drone_log_dir/sitl_drone.log" 2>&1 || true
            done
            ok "Logs saved to $log_dir"
        fi

        swarm_down
    fi
}

run_swarm_mission() {
    local n="${1:-$MAX_DRONES}"
    local timeout="${2:-600}"
    local log_dir

    log_dir=$(swarm_up "$n")

    # Set globals for trap handler
    _CLEANUP_DRONE_COUNT="$n"
    _CLEANUP_LOG_DIR="$log_dir"
    _SWARM_CLEANUP_DONE=false
    trap swarm_cleanup EXIT

    # Generate waypoint missions for each drone
    info "Generating waypoint missions for $n drones..."
    local mission_dir="$log_dir/missions"
    python "$SIM_DIR/sitl_waypoints.py" \
        --n "$n" --output-dir "$mission_dir" \
        --ref-lat "-0.508333" --ref-lon "-78.141667" --ref-alt "4500"

    # Run swarm orchestrator against the compose stack
    info "Running swarm mission (${n} drones, timeout=${timeout}s)..."
    info "  Orchestrator connects to swarm_node containers via ROS 2 domains 1-${n}"
    python "$SIM_DIR/sitl_orchestrator.py" swarm \
        --n "$n" \
        --base-port 5760 \
        --port-step 10 \
        --mission-dir "$mission_dir" \
        --timeout "$timeout" || true

    # Merge logs into swarm_data.npz if BIN logs exist
    local log_dirs=()
    for i in $(seq 1 "$n"); do
        local drone_log_dir="$log_dir/drone_${i}"
        mkdir -p "$drone_log_dir"
        docker cp "${SERVICE_SITL}_${i}:/sitl/logs/." "$drone_log_dir/" 2>/dev/null || true
        local bin_count
        bin_count=$(find "$drone_log_dir" -name "*.BIN" 2>/dev/null | wc -l | tr -d ' ')
        if [ "$bin_count" -gt 0 ]; then
            info "  drone_${i}: ${bin_count} .BIN file(s)"
            log_dirs+=("$drone_log_dir")
        fi
    done

    if [ ${#log_dirs[@]} -gt 0 ]; then
        info "Merging ${#log_dirs[@]} drone logs into swarm data..."
        python "$SIM_DIR/sitl_log_merger.py" swarm --log-dirs "${log_dirs[@]}"
    fi

    ok "Swarm mission complete (${n} drones, log: $log_dir)"
}

_SINGLE_CLEANUP_DONE=false

single_cleanup() {
    if [ "$_SINGLE_CLEANUP_DONE" = false ]; then
        _SINGLE_CLEANUP_DONE=true
        local log_dir="${_CLEANUP_LOG_DIR:-}"

        if [ -n "$log_dir" ]; then
            info "Capturing logs..."
            local drone_log_dir="$log_dir/drone_1"
            mkdir -p "$drone_log_dir"
            docker logs "${SERVICE_SWARM}_1" > "$drone_log_dir/swarm_node.log" 2>&1 || true
            docker logs "${SERVICE_PERCEPTION}_1" > "$drone_log_dir/perception_node.log" 2>&1 || true
            docker logs "${SERVICE_ZENOH}_1" > "$drone_log_dir/zenoh_bridge.log" 2>&1 || true
            docker logs "${SERVICE_SITL}_1" > "$drone_log_dir/sitl_drone.log" 2>&1 || true
            ok "Logs saved to $log_dir"
        fi

        info "Stopping single-drone stack..."
        cd "$ROOT_DIR"
        local services
        services=$(drone_services 1)
        # shellcheck disable=SC2086
        $COMPOSE_CMD --profile "$COMPOSE_PROFILE" stop $services 2>/dev/null || true
        # shellcheck disable=SC2086
        $COMPOSE_CMD --profile "$COMPOSE_PROFILE" rm -f $services 2>/dev/null || true
    fi
}

run_single_mission() {
    local timeout="${1:-300}"
    local n=1

    local log_dir
    log_dir=$(swarm_up "$n")

    # Set globals for trap handler
    _CLEANUP_DRONE_COUNT="$n"
    _CLEANUP_LOG_DIR="$log_dir"
    _SINGLE_CLEANUP_DONE=false
    trap single_cleanup EXIT

    # Generate mission
    local mission_dir="$log_dir/missions"
    python "$SIM_DIR/sitl_waypoints.py" --n 1 --output-dir "$mission_dir"
    local mission_file="$mission_dir/drone_0.waypoints"

    # Run orchestrator
    info "Running single-drone mission (timeout=${timeout}s)..."
    python "$SIM_DIR/sitl_orchestrator.py" single \
        --port 5760 \
        --mission "$mission_file" \
        --timeout "$timeout" || true

    # Capture flight logs
    local drone_log_dir="$log_dir/drone_1"
    mkdir -p "$drone_log_dir"
    docker cp "${SERVICE_SITL}_1:/sitl/logs/." "$drone_log_dir/" 2>/dev/null || true

    local newest_bin
    newest_bin=$(find "$drone_log_dir" -name "*.BIN" -type f 2>/dev/null | \
        xargs ls -t 2>/dev/null | head -1)

    if [ -n "$newest_bin" ]; then
        info "Converting flight log to NPZ..."
        python "$SIM_DIR/sitl_log_merger.py" single "$newest_bin"
    else
        warn "No .BIN flight log captured"
    fi

    ok "Single-drone mission complete (log: $log_dir)"
}

# ── Test & benchmark actions ─────────────────────────────────────────────────

run_rust_tests() {
    info "Running Rust unit tests..."
    (cd "$ROOT_DIR/swarm_control" && cargo test)
    ok "All Rust tests passed"
}

run_physics_tests() {
    info "Running drone physics tests..."
    python -m pytest "$SIM_DIR/test_drone_physics.py" -v
    ok "All physics tests passed"
}

run_integration_tests() {
    info "Running integration tests..."
    local timeout_flag=()
    if [ -n "$PYTEST_TIMEOUT" ]; then
        timeout_flag=(--timeout="$PYTEST_TIMEOUT")
    fi
    python -m pytest "$ROOT_DIR/tests/test_integration_sitl.py" -v "${timeout_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_swarm_node.py" -v "${timeout_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_perception.py" -v "${timeout_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_zenoh.py" -v "${timeout_flag[@]}"
    ok "All integration tests passed"
}

run_tests() {
    run_rust_tests
    run_physics_tests
    run_integration_tests
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

    info "[CI local] Rust test suite"
    run_rust_tests

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
        bin_file=$(find "$ROOT_DIR/logs" -name "*.BIN" -type f 2>/dev/null | \
            xargs ls -t 2>/dev/null | head -1)
    fi
    if [ -z "$bin_file" ] || [ ! -f "$bin_file" ]; then
        fail "No SITL .BIN log found. Run a SITL mission first."
    fi
    info "Visualizing SITL flight log: $bin_file"
    python "$SIM_DIR/visualize_drone_3d.py" "$bin_file"
}

# ── Parse args ───────────────────────────────────────────────────────────────
PYTEST_TIMEOUT=""
POSITIONAL=()

for arg in "$@"; do
    case "$arg" in
        --timeout=*)
            PYTEST_TIMEOUT="${arg#--timeout=}"
            ;;
        *)
            POSITIONAL+=("$arg")
            ;;
    esac
done

MODE="${POSITIONAL[0]:---default}"
SWARM_DRONES="${POSITIONAL[1]:-6}"

case "$MODE" in
    # ── Primary modes (Docker Compose per-drone stack) ──────────────────────
    --single|--sitl)
        NEED_PYMAVLINK=1 ensure_venv
        run_single_mission 300
        run_single_viz
        ;;
    --swarm|--sitl-swarm)
        NEED_PYMAVLINK=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        run_swarm_mission "$SWARM_DRONES" 600
        run_viz "$SIM_DIR/swarm_data.npz"
        ;;
    --sim-only)
        NEED_PYMAVLINK=1 ensure_venv
        run_single_mission 300
        ;;
    --swarm-only)
        NEED_PYMAVLINK=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        run_swarm_mission "$SWARM_DRONES" 600
        ;;
    --viz-only)
        ensure_venv
        run_single_viz
        ;;
    --sitl-viz)
        ensure_venv
        run_sitl_viz "${2:-}"
        ;;

    # ── Stack management ────────────────────────────────────────────────────
    --status)
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        swarm_status "$SWARM_DRONES"
        ;;
    --down)
        swarm_down
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
        # Phase 1: offline tests & validation (no Docker needed)
        run_rust_tests
        run_physics_tests
        run_benchmark
        run_real_log
        # Phase 2: Docker-based mission (starts containers)
        run_single_mission 300
        # Phase 3: integration tests (containers now running)
        run_integration_tests
        # Phase 4: visualization
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
        echo "Docker Compose per-drone stack (default — requires Docker):"
        echo "  (default)       Run single-drone stack (4 containers) then visualize"
        echo "  --single        Run single-drone stack (4 containers) then visualize"
        echo "  --swarm [N]     Run N-drone stack (default: 6, max: 6) then visualize"
        echo "                  Each drone: sitl_drone_N + swarm_node_N + perception_node_N + zenoh_bridge_N"
        echo "  --sim-only      Run single-drone stack only (no GUI)"
        echo "  --swarm-only [N] Run N-drone stack only (no GUI)"
        echo "  --status [N]    Show health status of running containers"
        echo "  --down          Tear down all swarm containers"
        echo "  --viz-only      Open visualization (uses existing data)"
        echo "  --sitl-viz [F]  Visualize newest SITL .BIN log (or specify path)"
        echo ""
        echo "Legacy physics simulation (no Docker needed):"
        echo "  --physics-single     Run Python physics single-drone scenario + visualize"
        echo "  --physics-swarm [N]  Run Python physics swarm for N drones (default: 6)"
        echo "  --physics-sim-only   Run Python physics scenario only (no GUI)"
        echo ""
        echo "Testing & validation:"
        echo "  --test          Run physics + integration tests"
        echo "  --benchmark     Run deterministic benchmark validation gates"
        echo "  --real-log      Run real-log validation against paper Table 5"
        echo "  --ci-local      Run local CI/CD-equivalent pipeline (tests + all CI benchmarks)"
        echo "  --all           Run tests, benchmarks, single-drone mission, integration tests, and viz"
        echo "  --timeout=N     Set pytest timeout in seconds (default: no timeout)"
        echo "  --help          Show this help"
        echo ""
        echo "Per-Drone Stack (6 drones × 4 services = 24 containers):"
        echo "  sitl_drone_N      — Pixhawk sim + Micro-XRCE-DDS Agent (ROS_DOMAIN_ID=N)"
        echo "  swarm_node_N      — Rust swarm control (restart: always, healthcheck)"
        echo "  perception_node_N — Python vision pipeline (restart: always, healthcheck)"
        echo "  zenoh_bridge_N    — Mesh networking router (restart: unless-stopped)"
        ;;

    # ── Default (no args) ────────────────────────────────────────────────────
    --default)
        NEED_PYMAVLINK=1 ensure_venv
        run_single_mission 300
        run_single_viz
        ;;
    *)
        fail "Unknown option: $MODE (use --help)"
        ;;
esac
