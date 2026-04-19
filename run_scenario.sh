#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Supports two orchestration backends (auto-detected or --backend= flag):
#   Docker Compose: 6 drones × 4 services = 24 containers
#   Kubernetes + Helm: StatefulSet with 4-container pods + zenoh router
#
# Per-Drone Stack:
#   Docker:  sitl_drone_N / swarm_node_N / perception_node_N / zenoh_bridge_N
#   K8s:     sitl / swarm-node / perception / zenoh-bridge (in pod drone-N)
#
# Usage:
#   ./run_scenario.sh                          # single-drone then visualize
#   ./run_scenario.sh --swarm [N]              # N-drone formation flight
#   ./run_scenario.sh --swarm 2 --backend=k8s  # deploy to Kubernetes
#   ./run_scenario.sh --status                 # show health of running stack
#   ./run_scenario.sh --down                   # tear down stack
#   ./run_scenario.sh --test                   # run physics + integration tests
#   ./run_scenario.sh --test --timeout=300     # with pytest timeout
#   ./run_scenario.sh --help                   # full usage
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

# Kubernetes / Helm
HELM_CHART="$ROOT_DIR/helm/swarm-digital-twin"
K8S_NAMESPACE="swarm"
HELM_RELEASE="swarm"
K8S_STS_NAME="swarm-swarm-digital-twin"

# ── Backend detection ─────────────────────────────────────────────────────

detect_backend() {
    # Explicit BACKEND already set (from --backend= flag)
    if [ -n "${BACKEND:-}" ]; then
        echo "$BACKEND"
        return
    fi
    # Default: k8s (uses current kubectl context)
    echo "k8s"
}

# ── Kubernetes helpers ────────────────────────────────────────────────────

k8s_pod_name() {
    local ordinal="$1"
    echo "${K8S_STS_NAME}-${ordinal}"
}

k8s_ensure_images() {
    local registry="${SWARM_REGISTRY:-beret}"
    local tag="${SWARM_IMAGE_TAG:-latest}"

    local sitl_remote="${registry}/ardupilot-sitl:${tag}"
    local companion_remote="${registry}/swarm_companion:${tag}"

    # Check if images exist in registry (pull test)
    local need_build=false
    if ! docker manifest inspect "$sitl_remote" &>/dev/null; then
        need_build=true
    fi
    if ! docker manifest inspect "$companion_remote" &>/dev/null; then
        need_build=true
    fi

    if [ "$need_build" = true ]; then
        info "Images not found in registry, building and pushing..."
        "$ROOT_DIR/scripts/push_images.sh" --registry "$registry" --tag "$tag" >&2
    else
        ok "Images already in registry: $registry"
    fi
}

k8s_swarm_up() {
    local n="${1:-$MAX_DRONES}"
    local mission_file="${2:-}"
    local log_dir="$ROOT_DIR/logs/swarm_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$log_dir"

    k8s_ensure_images

    # Wait for namespace termination if it's being deleted
    local ns_status
    ns_status=$(kubectl get ns "$K8S_NAMESPACE" -o jsonpath='{.status.phase}' 2>/dev/null || echo "NotFound")
    if [ "$ns_status" = "Terminating" ]; then
        info "Waiting for namespace '$K8S_NAMESPACE' to finish terminating..."
        while kubectl get ns "$K8S_NAMESPACE" &>/dev/null; do
            sleep 2
        done
    fi

    info "Deploying ${n}-drone Helm release..."

    local registry="${SWARM_REGISTRY:-beret}"

    local helm_args=(
        upgrade --install "$HELM_RELEASE" "$HELM_CHART"
        -n "$K8S_NAMESPACE" --create-namespace
        --set "drones=$n"
        --set "images.registry=$registry"
    )

    # Deploy mission config via --set-file if provided
    if [ -n "$mission_file" ] && [ -f "$mission_file" ]; then
        helm_args+=(--set-file "missionConfig=$mission_file")
    fi

    # Use local values if minikube/kind detected
    if command -v minikube &>/dev/null && minikube status --format='{{.Host}}' 2>/dev/null | grep -q Running; then
        helm_args+=(-f "$HELM_CHART/values-local.yaml" --set "drones=$n")
    elif command -v kind &>/dev/null && kind get clusters 2>/dev/null | grep -q .; then
        helm_args+=(-f "$HELM_CHART/values-local.yaml" --set "drones=$n")
    fi

    if ! helm "${helm_args[@]}" >&2; then
        fail "Helm deploy failed"
    fi

    k8s_wait_swarm_healthy "$n" 300

    ok "Helm release deployed (${n} drone pods)"
    echo "" >&2
    k8s_swarm_status "$n"

    echo "$n" > "$log_dir/.drone_count"
    echo "$log_dir"
}

k8s_swarm_down() {
    info "Uninstalling Helm release..."
    helm uninstall "$HELM_RELEASE" -n "$K8S_NAMESPACE" 2>/dev/null || true
    ok "Helm release uninstalled"
}

k8s_wait_swarm_healthy() {
    local n="${1:-$MAX_DRONES}"
    local timeout="${2:-300}"

    info "Waiting for ${n} drone pods to become ready (timeout: ${timeout}s)..."

    # Wait for StatefulSet rollout
    kubectl rollout status statefulset/"$K8S_STS_NAME" \
        -n "$K8S_NAMESPACE" --timeout="${timeout}s" 2>&1 | tail -5 >&2 || true

    # Additionally wait for all pods to be Ready
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        local ready
        ready=$(kubectl get statefulset "$K8S_STS_NAME" \
            -n "$K8S_NAMESPACE" \
            -o jsonpath='{.status.readyReplicas}' 2>/dev/null || echo "0")
        ready="${ready:-0}"
        if [ "$ready" -ge "$n" ]; then
            ok "All $n drone pods ready"
            return 0
        fi
        sleep 3
        elapsed=$((elapsed + 3))
    done

    warn "Only ${ready:-0} of $n pods ready after ${timeout}s"
    return 1
}

k8s_swarm_status() {
    local n="${1:-$MAX_DRONES}"

    echo -e "${BOLD}┌────────┬─────────────────────────────────┬───────────────┬───────────────┬────────────────┬───────────────┐${NC}" >&2
    echo -e "${BOLD}│ Drone  │             Pod                 │ sitl          │ swarm-node    │ perception     │ zenoh-bridge  │${NC}" >&2
    echo -e "${BOLD}├────────┼─────────────────────────────────┼───────────────┼───────────────┼────────────────┼───────────────┤${NC}" >&2

    for i in $(seq 0 $((n - 1))); do
        local drone_id=$((i + 1))
        local pod
        pod=$(k8s_pod_name "$i")

        local pod_json
        pod_json=$(kubectl get pod "$pod" -n "$K8S_NAMESPACE" -o json 2>/dev/null || echo "{}")

        local sitl_status swarm_status percep_status zenoh_status
        sitl_status=$(k8s_container_status_icon "$pod_json" "sitl")
        swarm_status=$(k8s_container_status_icon "$pod_json" "swarm-node")
        percep_status=$(k8s_container_status_icon "$pod_json" "perception")
        zenoh_status=$(k8s_container_status_icon "$pod_json" "zenoh-bridge")

        printf "│   %-4s │ %-9s │  %-12s │  %-12s │  %-13s │  %-12s │\n" \
            "$drone_id" "$pod" "$sitl_status" "$swarm_status" "$percep_status" "$zenoh_status" >&2
    done

    echo -e "${BOLD}└────────┴─────────────────────────────────┴───────────────┴───────────────┴────────────────┴───────────────┘${NC}" >&2

    local ready
    ready=$(kubectl get statefulset "$K8S_STS_NAME" \
        -n "$K8S_NAMESPACE" \
        -o jsonpath='{.status.readyReplicas}' 2>/dev/null || echo "0")
    echo -e "  Ready pods: ${BOLD}${ready:-0}${NC} / $n" >&2
}

k8s_container_status_icon() {
    local pod_json="$1"
    local container="$2"

    if [ "$pod_json" = "{}" ]; then
        echo -e "${RED}DOWN${NC}"
        return
    fi

    local ready state
    ready=$(echo "$pod_json" | python3 -c "
import sys, json
pod = json.load(sys.stdin)
for cs in pod.get('status', {}).get('containerStatuses', []):
    if cs['name'] == '$container':
        print('true' if cs.get('ready') else 'false')
        break
else:
    print('missing')
" 2>/dev/null || echo "missing")

    case "$ready" in
        true)  echo -e "${GREEN}ready${NC}" ;;
        false) echo -e "${YELLOW}not-ready${NC}" ;;
        *)     echo -e "${RED}missing${NC}" ;;
    esac
}

k8s_swarm_cleanup() {
    if [ "$_SWARM_CLEANUP_DONE" = false ]; then
        _SWARM_CLEANUP_DONE=true
        info "Capturing pod logs before teardown..."

        local n="${_CLEANUP_DRONE_COUNT:-0}"
        local log_dir="${_CLEANUP_LOG_DIR:-}"

        if [ "$n" -gt 0 ] && [ -n "$log_dir" ]; then
            for i in $(seq 0 $((n - 1))); do
                local drone_id=$((i + 1))
                local drone_log_dir="$log_dir/drone_${drone_id}"
                mkdir -p "$drone_log_dir"
                local pod
                pod=$(k8s_pod_name "$i")
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c swarm-node > "$drone_log_dir/swarm_node.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c perception > "$drone_log_dir/perception_node.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c zenoh-bridge > "$drone_log_dir/zenoh_bridge.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c sitl > "$drone_log_dir/sitl_drone.log" 2>&1 || true
            done
            ok "Logs saved to $log_dir"
        fi

        k8s_swarm_down
    fi
}

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

    # fastapi + uvicorn + websockets needed for the Run-time View web app
    if [[ "${NEED_RUNTIME_VIEW:-0}" == "1" ]] && ! python -c "import fastapi, uvicorn, websockets" &>/dev/null; then
        info "Installing fastapi + uvicorn + websockets for Run-time View..."
        if command -v uv &>/dev/null; then
            uv pip install 'fastapi>=0.110' 'uvicorn>=0.27' 'websockets>=12,<13'
        else
            pip install 'fastapi>=0.110' 'uvicorn>=0.27' 'websockets>=12,<13'
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
    local backend
    backend=$(detect_backend)

    if [ "$backend" = "k8s" ]; then
        k8s_swarm_up "$n"
        return
    fi

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
    local backend
    backend=$(detect_backend)

    if [ "$backend" = "k8s" ]; then
        k8s_swarm_down
        return
    fi

    info "Tearing down swarm stack..."
    cd "$ROOT_DIR"
    $COMPOSE_CMD --profile "$COMPOSE_PROFILE" down --timeout 15 2>&1 | tail -5
    ok "Swarm stack stopped"
}

swarm_status() {
    local n="${1:-$MAX_DRONES}"
    local backend
    backend=$(detect_backend)

    if [ "$backend" = "k8s" ]; then
        k8s_swarm_status "$n"
        return
    fi

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
    local backend
    backend=$(detect_backend)

    if [ "$backend" = "k8s" ]; then
        k8s_swarm_cleanup
        return
    fi

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
    local timeout="${2:-}"
    local backend
    backend=$(detect_backend)

    # Generate formation mission config (shared by all swarm_nodes)
    local tmp_mission="$ROOT_DIR/logs/.swarm_mission_tmp.json"
    local tmp_wp_dir="$ROOT_DIR/logs/.swarm_waypoints_tmp"
    mkdir -p "$ROOT_DIR/logs"
    rm -rf "$tmp_wp_dir"
    info "Generating formation mission for $n drones..."
    python "$SIM_DIR/sitl_waypoints.py" formation \
        --n "$n" \
        --radius 8 \
        --altitude 20 \
        --patrol-size 40 \
        --cruise-speed 4.0 \
        --output "$tmp_mission"

    # Also generate per-drone QGC waypoint files for the orchestrator
    # to upload to ArduPilot SITL via MAVLink (Rust swarm_node speaks
    # PX4 messages over Zenoh, which ArduPilot does not understand).
    info "Generating per-drone ring waypoints for $n drones..."
    python "$SIM_DIR/sitl_waypoints.py" ring \
        --n "$n" \
        --radius 8 \
        --altitude 20 \
        --output-dir "$tmp_wp_dir"

    local log_dir
    if [ "$backend" = "k8s" ]; then
        # K8s: deploy via Helm with mission config baked into ConfigMap
        log_dir=$(k8s_swarm_up "$n" "$tmp_mission")
    else
        # Docker: start stack then copy config into containers
        log_dir=$(swarm_up "$n")
        cp "$tmp_mission" "$log_dir/swarm_mission.json"
        for i in $(seq 1 "$n"); do
            docker cp "$tmp_mission" "${SERVICE_SWARM}_${i}:/root/workspace/swarm_mission.json" 2>/dev/null || true
        done
        info "Formation config deployed to $n swarm_node containers"
    fi

    # Set globals for trap handler
    _CLEANUP_DRONE_COUNT="$n"
    _CLEANUP_LOG_DIR="$log_dir"
    _SWARM_CLEANUP_DONE=false
    trap swarm_cleanup EXIT

    # Run formation orchestrator (SITL GPS init + offboard monitoring)
    local timeout_flag=()
    if [ -n "$timeout" ]; then
        timeout_flag=(--timeout "$timeout")
        info "Running swarm formation flight (${n} drones, timeout=${timeout}s)..."
    else
        info "Running swarm formation flight (${n} drones, no timeout)..."
    fi
    info "  Drones fly as swarm in ring formation, offboard-controlled by swarm_nodes"

    # K8s: use NodePort services to reach SITL MAVLink ports (more
    # reliable than `kubectl port-forward` for long-lived connections).
    local mav_host="127.0.0.1"
    local mav_base_port=5760
    local mav_port_step=10
    if [ "$backend" = "k8s" ]; then
        mav_host=$(kubectl get nodes -o jsonpath='{.items[0].status.addresses[?(@.type=="InternalIP")].address}')
        mav_base_port=$(kubectl get svc "${K8S_STS_NAME}-mavlink-1" -n "$K8S_NAMESPACE" \
            -o jsonpath='{.spec.ports[0].nodePort}' 2>/dev/null || echo 30760)
        info "  MAVLink via NodePort ${mav_host}:${mav_base_port}.."
    fi

    python "$SIM_DIR/sitl_orchestrator.py" swarm-formation \
        --n "$n" \
        --host "$mav_host" \
        --base-port "$mav_base_port" \
        --port-step "$mav_port_step" \
        --mission-dir "$tmp_wp_dir" \
        "${timeout_flag[@]}" || true

    # Capture SITL logs
    local log_dirs=()
    for i in $(seq 1 "$n"); do
        local drone_log_dir="$log_dir/drone_${i}"
        mkdir -p "$drone_log_dir"
        if [ "$backend" = "k8s" ]; then
            local pod
            pod=$(k8s_pod_name $((i - 1)))
            kubectl cp "$K8S_NAMESPACE/$pod:/sitl/logs/." "$drone_log_dir/" -c sitl 2>/dev/null || true
        else
            docker cp "${SERVICE_SITL}_${i}:/sitl/logs/." "$drone_log_dir/" 2>/dev/null || true
        fi
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

    ok "Swarm formation flight complete (${n} drones, log: $log_dir)"
}

_SINGLE_CLEANUP_DONE=false

single_cleanup() {
    if [ "$_SINGLE_CLEANUP_DONE" = false ]; then
        _SINGLE_CLEANUP_DONE=true
        local backend
        backend=$(detect_backend)
        local log_dir="${_CLEANUP_LOG_DIR:-}"

        if [ -n "$log_dir" ]; then
            info "Capturing logs..."
            local drone_log_dir="$log_dir/drone_1"
            mkdir -p "$drone_log_dir"
            if [ "$backend" = "k8s" ]; then
                local pod
                pod=$(k8s_pod_name 0)
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c swarm-node > "$drone_log_dir/swarm_node.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c perception > "$drone_log_dir/perception_node.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c zenoh-bridge > "$drone_log_dir/zenoh_bridge.log" 2>&1 || true
                kubectl logs "$pod" -n "$K8S_NAMESPACE" -c sitl > "$drone_log_dir/sitl_drone.log" 2>&1 || true
            else
                docker logs "${SERVICE_SWARM}_1" > "$drone_log_dir/swarm_node.log" 2>&1 || true
                docker logs "${SERVICE_PERCEPTION}_1" > "$drone_log_dir/perception_node.log" 2>&1 || true
                docker logs "${SERVICE_ZENOH}_1" > "$drone_log_dir/zenoh_bridge.log" 2>&1 || true
                docker logs "${SERVICE_SITL}_1" > "$drone_log_dir/sitl_drone.log" 2>&1 || true
            fi
            ok "Logs saved to $log_dir"
        fi

        if [ "$backend" = "k8s" ]; then
            k8s_swarm_down
        else
            info "Stopping single-drone stack..."
            cd "$ROOT_DIR"
            local services
            services=$(drone_services 1)
            # shellcheck disable=SC2086
            $COMPOSE_CMD --profile "$COMPOSE_PROFILE" stop $services 2>/dev/null || true
            # shellcheck disable=SC2086
            $COMPOSE_CMD --profile "$COMPOSE_PROFILE" rm -f $services 2>/dev/null || true
        fi
    fi
}

run_single_mission() {
    local timeout="${1:-300}"
    local n=1
    # When set (typically by run_single_mission_live), the orchestrator
    # relays every received MAVLink frame to this pymavlink URL so the
    # Run-time View live HUD has data to render. Without it the launcher
    # connects but the drone mesh never moves.
    local telemetry_forward="${SITL_TELEMETRY_FORWARD:-}"

    local log_dir
    log_dir=$(swarm_up "$n")

    # Set globals for trap handler
    _CLEANUP_DRONE_COUNT="$n"
    _CLEANUP_LOG_DIR="$log_dir"
    _SINGLE_CLEANUP_DONE=false
    trap single_cleanup EXIT

    # Generate mission
    local mission_dir="$log_dir/missions"
    python "$SIM_DIR/sitl_waypoints.py" ring --n 1 --output-dir "$mission_dir"
    local mission_file="$mission_dir/drone_0.waypoints"

    # Run orchestrator
    info "Running single-drone mission (timeout=${timeout}s)..."

    local backend
    backend=$(detect_backend)

    # K8s: use NodePort to reach SITL MAVLink (more stable than port-forward)
    local mav_host="127.0.0.1"
    local mav_port=5760
    if [ "$backend" = "k8s" ]; then
        mav_host=$(kubectl get nodes -o jsonpath='{.items[0].status.addresses[?(@.type=="InternalIP")].address}')
        mav_port=$(kubectl get svc "${K8S_STS_NAME}-mavlink-1" -n "$K8S_NAMESPACE" \
            -o jsonpath='{.spec.ports[0].nodePort}' 2>/dev/null || echo 30760)
        info "  MAVLink via NodePort ${mav_host}:${mav_port}"
    fi

    local forward_args=()
    if [ -n "$telemetry_forward" ]; then
        forward_args=(--telemetry-forward "$telemetry_forward")
        info "  Forwarding telemetry to $telemetry_forward"
    fi

    python "$SIM_DIR/sitl_orchestrator.py" single \
        --host "$mav_host" \
        --port "$mav_port" \
        --mission "$mission_file" \
        --timeout "$timeout" \
        "${forward_args[@]}" || true

    # Capture flight logs
    local drone_log_dir="$log_dir/drone_1"
    mkdir -p "$drone_log_dir"
    if [ "$backend" = "k8s" ]; then
        local pod
        pod=$(k8s_pod_name 0)
        kubectl cp "$K8S_NAMESPACE/$pod:/sitl/logs/." "$drone_log_dir/" -c sitl 2>/dev/null || true
    else
        docker cp "${SERVICE_SITL}_1:/sitl/logs/." "$drone_log_dir/" 2>/dev/null || true
    fi

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
    local backend_flag=()
    local backend
    backend=$(detect_backend)
    backend_flag=(--backend="$backend")
    python -m pytest "$ROOT_DIR/tests/test_integration_sitl.py" -v "${timeout_flag[@]}" "${backend_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_swarm_node.py" -v "${timeout_flag[@]}" "${backend_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_perception.py" -v "${timeout_flag[@]}" "${backend_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_zenoh.py" -v "${timeout_flag[@]}" "${backend_flag[@]}"
    python -m pytest "$ROOT_DIR/tests/test_integration_swarm_formation.py" -v "${timeout_flag[@]}" "${backend_flag[@]}"
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

# ── Run-time View (FastAPI + Three.js web app) ───────────────────────────────
# Default browser landing path. The third positional argument lets callers
# pick the launcher (`/`) or the live HUD (`/live`) — `--single-live` and
# the default flow open `/live` so the user lands on the HUD directly.
run_live_viz() {
    local mav_port="${1:-14550}"
    local http_port="${2:-8765}"
    local open_path="${3:-/live}"
    [[ "$open_path" != /* ]] && open_path="/${open_path}"
    local url="http://127.0.0.1:${http_port}${open_path}"
    info "Starting Run-time View on ${url}"
    info "Listening for MAVLink on UDP ${mav_port} (Ctrl-C to quit)…"

    (
        sleep 1
        if command -v open &>/dev/null; then
            open "$url" 2>/dev/null || true
        elif command -v xdg-open &>/dev/null; then
            xdg-open "$url" 2>/dev/null || true
        elif command -v start &>/dev/null; then
            start "" "$url" 2>/dev/null || true
        fi
    ) &

    # Pass SITL GPS reference to the server so GPS→ENU conversion
    # produces correct local coordinates. For K8s, these come from
    # Helm values (SITL_REF_LAT/LNG/ALT set by run_single_mission_live).
    local ref_args=()
    if [ -n "${SITL_REF_LAT:-}" ]; then
        ref_args+=(--ref-lat "$SITL_REF_LAT" --ref-lon "$SITL_REF_LNG" --ref-alt "$SITL_REF_ALT")
    fi

    (
        cd "$SIM_DIR"
        python -m runtime_view.server \
            --host 127.0.0.1 \
            --port "$http_port" \
            --listen-port "$mav_port" \
            "${ref_args[@]}"
    )
}

# Run a single SITL mission in the background while serving the live
# Run-time View (FastAPI + Three.js) in the foreground. The orchestrator
# (sitl_orchestrator.py) is the only host process that actually receives
# MAVLink from the SITL container (over TCP 5760), so we tell it to relay
# every received frame to UDP 14550 where MAVLinkLiveSource is listening.
# Without that relay the launcher status chip flips to CONNECTED but the
# drone mesh never moves because no samples ever reach the queue.
run_single_mission_live() {
    local timeout="${1:-300}"
    info "Launching single-drone SITL mission with the live Run-time View"
    info "  Mission timeout: ${timeout}s | Live view: http://127.0.0.1:8765/live"

    # Export SITL GPS reference so run_live_viz passes matching
    # coordinates to MAVLinkLiveSource (GPS→ENU conversion origin).
    export SITL_REF_LAT="${SITL_REF_LAT:--0.508333}"
    export SITL_REF_LNG="${SITL_REF_LNG:--78.141667}"
    export SITL_REF_ALT="${SITL_REF_ALT:-4500}"

    # Generate the single-drone ring waypoints up front so we can publish
    # the ENU sidecar to the live view (parity with --physics-live).
    local wp_dir="$ROOT_DIR/logs/.single_waypoints_tmp"
    rm -rf "$wp_dir"
    python "$SIM_DIR/sitl_waypoints.py" ring \
        --n 1 \
        --ref-lat "$SITL_REF_LAT" --ref-lon "$SITL_REF_LNG" \
        --ref-alt "$SITL_REF_ALT" \
        --output-dir "$wp_dir" >/dev/null

    # Start the mission in background.
    SITL_TELEMETRY_FORWARD="udpout:127.0.0.1:14550" \
        run_single_mission "$timeout" &
    local mission_pid=$!

    # Start the live view in background.
    run_live_viz 14550 8765 /live &
    local viz_pid=$!

    # Clean up both on any exit.
    trap 'kill '"$viz_pid"' '"$mission_pid"' 2>/dev/null || true' EXIT INT TERM

    # Publish waypoints to the live view (uvicorn just booted; helper retries).
    publish_waypoints_to_live "$wp_dir/waypoints_enu.json" || true

    # Wait for the mission to finish (it has a timeout).
    wait "$mission_pid" 2>/dev/null || true
    local mission_rc=$?

    info "Mission finished (exit code $mission_rc)"
    info "Waiting for helm uninstall to complete before exiting..."
    wait_helm_uninstalled 60 || warn "Helm release still present after 60s"

    # Stop the live view server.
    kill "$viz_pid" 2>/dev/null || true
    wait "$viz_pid" 2>/dev/null || true
    info "Live view stopped"
}

# Swarm sibling of run_single_mission_live: deploys the SITL stack, runs
# the formation orchestrator with per-drone telemetry forwarding to the
# live view, publishes per-drone waypoints, exits cleanly when helm tears
# down.
run_swarm_mission_live() {
    local n="${1:-$MAX_DRONES}"
    local timeout="${2:-}"
    info "Launching ${n}-drone swarm SITL mission with the live Run-time View"
    info "  Live view: http://127.0.0.1:8765/live"

    export SITL_REF_LAT="${SITL_REF_LAT:--0.508333}"
    export SITL_REF_LNG="${SITL_REF_LNG:--78.141667}"
    export SITL_REF_ALT="${SITL_REF_ALT:-4500}"

    # Pre-generate waypoints (orchestrator also needs them on disk).
    local tmp_mission="$ROOT_DIR/logs/.swarm_mission_tmp.json"
    local tmp_wp_dir="$ROOT_DIR/logs/.swarm_waypoints_tmp"
    mkdir -p "$ROOT_DIR/logs"
    rm -rf "$tmp_wp_dir"
    info "Generating formation mission for $n drones..."
    python "$SIM_DIR/sitl_waypoints.py" formation \
        --n "$n" --radius 8 --altitude 20 --patrol-size 40 --cruise-speed 4.0 \
        --output "$tmp_mission" >/dev/null
    info "Generating per-drone ring waypoints for $n drones..."
    python "$SIM_DIR/sitl_waypoints.py" ring \
        --n "$n" --radius 8 --altitude 20 \
        --ref-lat "$SITL_REF_LAT" --ref-lon "$SITL_REF_LNG" \
        --ref-alt "$SITL_REF_ALT" \
        --output-dir "$tmp_wp_dir" >/dev/null

    local backend
    backend=$(detect_backend)
    local log_dir
    if [ "$backend" = "k8s" ]; then
        log_dir=$(k8s_swarm_up "$n" "$tmp_mission")
    else
        log_dir=$(swarm_up "$n")
        cp "$tmp_mission" "$log_dir/swarm_mission.json"
        for i in $(seq 1 "$n"); do
            docker cp "$tmp_mission" "${SERVICE_SWARM}_${i}:/root/workspace/swarm_mission.json" 2>/dev/null || true
        done
    fi

    _CLEANUP_DRONE_COUNT="$n"
    _CLEANUP_LOG_DIR="$log_dir"
    _SWARM_CLEANUP_DONE=false

    # Start live view in background.
    run_live_viz 14550 8765 /live &
    local viz_pid=$!
    trap 'kill '"$viz_pid"' 2>/dev/null || true; swarm_cleanup' EXIT INT TERM

    publish_waypoints_to_live "$tmp_wp_dir/waypoints_enu.json" || true

    # Resolve MAVLink host/port (NodePort under k8s).
    local mav_host="127.0.0.1" mav_base_port=5760 mav_port_step=10
    if [ "$backend" = "k8s" ]; then
        mav_host=$(kubectl get nodes -o jsonpath='{.items[0].status.addresses[?(@.type=="InternalIP")].address}')
        mav_base_port=$(kubectl get svc "${K8S_STS_NAME}-mavlink-1" -n "$K8S_NAMESPACE" \
            -o jsonpath='{.spec.ports[0].nodePort}' 2>/dev/null || echo 30760)
    fi

    local timeout_flag=()
    [ -n "$timeout" ] && timeout_flag=(--timeout "$timeout")

    # The orchestrator forwards every received MAVLink frame (with its
    # original system_id) to UDP 14550. MAVLinkLiveSource demuxes by
    # system_id, so the live view sees N drones automatically.
    python "$SIM_DIR/sitl_orchestrator.py" swarm-formation \
        --n "$n" \
        --host "$mav_host" \
        --base-port "$mav_base_port" \
        --port-step "$mav_port_step" \
        --mission-dir "$tmp_wp_dir" \
        --telemetry-forward "udpout:127.0.0.1:14550" \
        "${timeout_flag[@]}" || true

    info "Swarm mission finished — waiting for helm uninstall..."
    swarm_cleanup
    wait_helm_uninstalled 60 || warn "Helm release still present after 60s"

    kill "$viz_pid" 2>/dev/null || true
    wait "$viz_pid" 2>/dev/null || true
    info "Live view stopped"
}

# Run a Python physics simulation and stream the results through the
# live Run-time View in real time — no Docker, no SITL container needed.
# The pipeline: run_simulation() → MAVLinkBridge.run_replay() → UDP
# → MAVLinkLiveSource → FastAPI /ws/telemetry → Three.js browser.
run_physics_live() {
    local extra_args=("$@")
    info "Launching physics simulation with the live Run-time View"
    (
        cd "$SIM_DIR"
        python -m physics_live_replay "${extra_args[@]}"
    )
}

# ── Pre-flight cleanup ───────────────────────────────────────────────────────
# Reclaim resources from a previous run before starting a new simulation.
# Without this, a leftover python process holds UDP 14550 / TCP 8765 and
# the next launch dies with "OSError: [Errno 48] Address already in use".

LIVE_HTTP_PORT=8765
LIVE_MAV_PORT=14550

_kill_pids() {
    local pids=("$@")
    [ ${#pids[@]} -eq 0 ] && return 0
    kill "${pids[@]}" 2>/dev/null || true
    local i pid
    for i in 1 2 3; do
        sleep 1
        local alive=()
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive+=("$pid")
            fi
        done
        [ ${#alive[@]} -eq 0 ] && return 0
        pids=("${alive[@]}")
    done
    kill -9 "${pids[@]}" 2>/dev/null || true
}

# pgrep/lsof return non-zero on "no match" — that is the common case here,
# so every pipeline below ends with `|| true` to keep `set -euo pipefail`
# from killing the whole script on a clean machine.
_kill_pattern() {
    local pattern="$1"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null | tr '\n' ' ' || true)
    [ -z "${pids// /}" ] && return 0
    info "  Killing stale: $pattern (pids: $pids)"
    # shellcheck disable=SC2086
    _kill_pids $pids
}

_free_port() {
    local kind="$1" port="$2"  # kind: tcp|udp
    if ! command -v lsof &>/dev/null; then
        return 0
    fi
    local pids
    if [ "$kind" = "tcp" ]; then
        pids=$(lsof -ti "tcp:${port}" -sTCP:LISTEN 2>/dev/null | tr '\n' ' ' || true)
    else
        pids=$(lsof -ti "udp:${port}" 2>/dev/null | tr '\n' ' ' || true)
    fi
    [ -z "${pids// /}" ] && return 0
    info "  Freeing ${kind}/${port} (pids: $pids)"
    # shellcheck disable=SC2086
    _kill_pids $pids
}

# pre_start_cleanup [--with-stack]
#   Always: kill stale runtime_view / physics_live_replay / sitl_orchestrator
#           processes and free the live-view ports.
#   --with-stack: also tear down a leftover helm release / docker compose
#           stack so simulation pods don't double-bind GPS or accumulate.
pre_start_cleanup() {
    local with_stack=false
    [ "${1:-}" = "--with-stack" ] && with_stack=true

    info "Pre-flight cleanup"
    _kill_pattern "python.*runtime_view\\.server"
    _kill_pattern "python.*physics_live_replay"
    _kill_pattern "python.*sitl_orchestrator"
    _free_port tcp "$LIVE_HTTP_PORT"
    _free_port udp "$LIVE_MAV_PORT"

    if [ "$with_stack" = true ]; then
        local backend
        backend=$(detect_backend)
        if [ "$backend" = "k8s" ]; then
            if helm status "$HELM_RELEASE" -n "$K8S_NAMESPACE" &>/dev/null; then
                info "  Uninstalling stale helm release '$HELM_RELEASE'..."
                helm uninstall "$HELM_RELEASE" -n "$K8S_NAMESPACE" 2>/dev/null || true
                # Wait briefly for pods to terminate so the next deploy isn't
                # racing the old StatefulSet.
                local elapsed=0
                while kubectl get pods -n "$K8S_NAMESPACE" \
                        -l "app.kubernetes.io/instance=$HELM_RELEASE" \
                        --no-headers 2>/dev/null | grep -q .; do
                    [ "$elapsed" -ge 30 ] && break
                    sleep 2; elapsed=$((elapsed + 2))
                done
            fi
        else
            if $COMPOSE_CMD --profile "$COMPOSE_PROFILE" ps -q 2>/dev/null | grep -q .; then
                info "  Stopping stale docker compose stack..."
                $COMPOSE_CMD --profile "$COMPOSE_PROFILE" down --timeout 15 2>/dev/null || true
            fi
        fi
    fi
    ok "Cleanup done"
}

# Wait until the helm release is gone (best-effort, bounded).
wait_helm_uninstalled() {
    local timeout="${1:-60}"
    local backend
    backend=$(detect_backend)
    [ "$backend" != "k8s" ] && return 0
    local elapsed=0
    while helm status "$HELM_RELEASE" -n "$K8S_NAMESPACE" &>/dev/null; do
        [ "$elapsed" -ge "$timeout" ] && return 1
        sleep 2; elapsed=$((elapsed + 2))
    done
    return 0
}

# POST a waypoints_enu.json sidecar to the live view.
# Tries up to ~10s to give uvicorn time to come up.
publish_waypoints_to_live() {
    local sidecar_path="$1"
    [ -z "$sidecar_path" ] || [ ! -f "$sidecar_path" ] && return 0
    local url="http://127.0.0.1:${LIVE_HTTP_PORT}/api/waypoints"
    local i
    for i in 1 2 3 4 5; do
        if curl -fsS -X POST -H 'Content-Type: application/json' \
                --data-binary "@${sidecar_path}" "$url" >/dev/null 2>&1; then
            ok "  Waypoints published to live view"
            return 0
        fi
        sleep 1
    done
    warn "  Could not publish waypoints to $url (continuing without)"
    return 1
}

# ── Parse args ───────────────────────────────────────────────────────────────
PYTEST_TIMEOUT=""
BACKEND=""
POSITIONAL=()

for arg in "$@"; do
    case "$arg" in
        --timeout=*)
            PYTEST_TIMEOUT="${arg#--timeout=}"
            ;;
        --backend=*)
            BACKEND="${arg#--backend=}"
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
    # `--single` / `--sitl` / `--default` all run the SITL stack with the
    # live web Run-time View as the default visualizer (migrated roadmap
    # follow-up — flipping the default away from matplotlib).
    --single|--sitl|--single-live)
        NEED_PYMAVLINK=1 NEED_RUNTIME_VIEW=1 ensure_venv
        pre_start_cleanup --with-stack
        run_single_mission_live 300
        ;;
    --single-static)
        # Backwards-compat path for the old post-flight matplotlib viewer.
        NEED_PYMAVLINK=1 ensure_venv
        pre_start_cleanup --with-stack
        run_single_mission 300
        run_single_viz
        ;;
    --swarm|--sitl-swarm)
        NEED_PYMAVLINK=1 NEED_RUNTIME_VIEW=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        pre_start_cleanup --with-stack
        run_swarm_mission_live "$SWARM_DRONES" "${PYTEST_TIMEOUT:-}"
        ;;
    --swarm-static)
        # Legacy path: swarm + post-flight matplotlib (no live view).
        NEED_PYMAVLINK=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        pre_start_cleanup --with-stack
        run_swarm_mission "$SWARM_DRONES" "${PYTEST_TIMEOUT:-}"
        run_viz "$SIM_DIR/swarm_data.npz"
        ;;
    --sim-only)
        NEED_PYMAVLINK=1 ensure_venv
        pre_start_cleanup --with-stack
        run_single_mission 300
        ;;
    --swarm-only)
        NEED_PYMAVLINK=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-6]$ ]]; then
            SWARM_DRONES=6
        fi
        pre_start_cleanup --with-stack
        run_swarm_mission "$SWARM_DRONES" "${PYTEST_TIMEOUT:-}"
        ;;
    --viz-only)
        ensure_venv
        run_single_viz
        ;;
    --sitl-viz)
        ensure_venv
        run_sitl_viz "${2:-}"
        ;;
    --viz-live)
        NEED_RUNTIME_VIEW=1 ensure_venv
        pre_start_cleanup
        run_live_viz "${POSITIONAL[1]:-14550}" "${POSITIONAL[2]:-8765}" /live
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

    # ── Physics + live viewer (no Docker needed) ────────────────────────────
    --physics-live)
        NEED_RUNTIME_VIEW=1 ensure_venv
        pre_start_cleanup
        local extra=()
        [[ "${POSITIONAL[1]:-}" == "--loop" ]] && extra+=(--loop)
        run_physics_live "${extra[@]}"
        ;;
    --physics-swarm-live)
        NEED_RUNTIME_VIEW=1 ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
            SWARM_DRONES=6
        fi
        pre_start_cleanup
        run_physics_live --swarm --drones "$SWARM_DRONES" --loop
        ;;
    --replay-live)
        NEED_RUNTIME_VIEW=1 ensure_venv
        local replay_file="${POSITIONAL[1]:-}"
        if [ -z "$replay_file" ]; then
            # Default: look for existing scenario_data.npz
            replay_file="$SIM_DIR/scenario_data.npz"
        fi
        if [ ! -f "$replay_file" ]; then
            fail "File not found: $replay_file — run a simulation first or specify a path"
        fi
        pre_start_cleanup
        run_physics_live --replay "$replay_file" --loop
        ;;

    # ── Phase 6: full-system K8s validation matrix (Python pipeline) ──────
    --acceptance-matrix)
        ensure_venv
        local subset="${POSITIONAL[1]:-ci}"
        local out="${POSITIONAL[2]:-$ROOT_DIR/reports}"
        info "Running acceptance matrix subset='$subset' → $out/"
        ( cd "$SIM_DIR" && python -m simulation.acceptance_matrix \
              --subset "$subset" --output "$out" )
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
        echo "Usage: $0 [MODE] [OPTIONS]"
        echo ""
        echo "Orchestration backend (default: k8s, uses current kubectl context):"
        echo "  --backend=k8s      Use Kubernetes + Helm (default)"
        echo "  --backend=docker   Use Docker Compose"
        echo ""
        echo "Per-drone stack modes (Docker or K8s):"
        echo "  (default)        Run a physics simulation and stream it to the live"
        echo "                   Three.js viewer at http://127.0.0.1:8765/live (looping)."
        echo "                   No Docker or SITL needed."
        echo "  --single         Run the single-drone SITL stack and open the live HUD"
        echo "  --single-live    Same as --single — explicit live-view alias"
        echo "  --single-static  Run the single-drone SITL stack and open the legacy"
        echo "                   matplotlib post-flight replayer (no live view)"
        echo "  --swarm [N]      Run N-drone formation flight (default: 6) with the"
        echo "                   live multi-drone HUD. Drones fly in ring formation,"
        echo "                   offboard-controlled via Zenoh, telemetry forwarded to"
        echo "                   the live view."
        echo "  --swarm-static [N] Legacy: swarm + post-flight matplotlib replayer."
        echo "  --sim-only       Run single-drone stack only (no GUI)"
        echo "  --swarm-only [N] Run N-drone stack only (no GUI)"
        echo "  --status [N]     Show health status of running containers/pods"
        echo "  --down           Tear down all swarm containers/pods"
        echo "  --viz-only       Open the matplotlib replayer for an existing scenario"
        echo "  --sitl-viz [F]   Visualize newest SITL .BIN log (or specify path)"
        echo "  --viz-live [MAV_PORT] [HTTP_PORT]"
        echo "                   Same as the (default) mode — start the Run-time View"
        echo "                   web app standalone at http://127.0.0.1:HTTP_PORT/live"
        echo "                   (default 8765), listening for MAVLink on UDP MAV_PORT"
        echo "                   (default 14550)."
        echo ""
        echo "Legacy physics simulation (no Docker/K8s needed):"
        echo "  --physics-single       Run Python physics scenario + matplotlib post-flight"
        echo "  --physics-swarm [N]    Run Python physics swarm (default: 6) + matplotlib"
        echo "  --physics-sim-only     Run Python physics scenario only (no GUI)"
        echo "  --physics-live [--loop]"
        echo "                         Run Python physics single-drone scenario and stream"
        echo "                         to the live Three.js viewer in real time. No Docker"
        echo "                         or SITL needed. Pass --loop to replay indefinitely."
        echo "  --physics-swarm-live [N]"
        echo "                         Run Python physics swarm (default: 6) and stream all"
        echo "                         drones to the live viewer (looping). Each drone gets"
        echo "                         its own mesh, trail, and colour."
        echo "  --replay-live [FILE]"
        echo "                         Replay an existing .npz or .BIN flight data file"
        echo "                         in the live Three.js viewer (looping). Defaults to"
        echo "                         simulation/scenario_data.npz if no file is given."
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
        echo "Real-time Telemetry (Run-time View):"
        echo "  The default mode (no args) runs a looping physics simulation and"
        echo "  streams it to the live Three.js viewer. Use --viz-live for a bare"
        echo "  server waiting for external MAVLink, --single-live for SITL + live"
        echo "  HUD, or --single-static for the legacy matplotlib viewer."
        echo ""
        echo "Docker backend — per-drone stack (6 drones x 4 services = 24 containers):"
        echo "  sitl_drone_N      — Pixhawk sim + Micro-XRCE-DDS Agent (ROS_DOMAIN_ID=N)"
        echo "  swarm_node_N      — Rust swarm control (restart: always, healthcheck)"
        echo "  perception_node_N — Python vision pipeline (restart: always, healthcheck)"
        echo "  zenoh_bridge_N    — Mesh networking router (restart: unless-stopped)"
        echo ""
        echo "K8s backend — StatefulSet with 4-container pods:"
        echo "  sitl          — ArduPilot SITL (ardupilot-sitl:latest)"
        echo "  swarm-node    — Rust swarm control (swarm_companion:latest)"
        echo "  perception    — Python vision pipeline (swarm_companion:latest)"
        echo "  zenoh-bridge  — Zenoh-ROS2-DDS bridge (eclipse/zenoh-bridge-ros2dds:latest)"
        echo "  Point-to-point Zenoh peer mesh (no router, pod-0 = seed peer)"
        ;;

    # ── Default (no args) ────────────────────────────────────────────────────
    # Default flow runs a physics simulation and streams it to the live
    # Run-time View (FastAPI + Three.js).  The drone flies the default
    # waypoint pattern and loops so the viewer always has something to
    # show.  Use --viz-live for a bare server waiting for external
    # telemetry, or --single-live for SITL + live HUD.
    --default)
        NEED_RUNTIME_VIEW=1 ensure_venv
        pre_start_cleanup
        run_physics_live --loop
        ;;
    *)
        fail "Unknown option: $MODE (use --help)"
        ;;
esac
