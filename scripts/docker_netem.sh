#!/usr/bin/env bash
# Apply network emulation (tc netem) to Docker swarm containers.
# Simulates 100ms latency + 5% packet loss on inter-drone traffic.
#
# Usage:
#   ./docker_netem.sh apply   — apply netem to all swarm_node containers
#   ./docker_netem.sh clear   — remove netem from all swarm_node containers
#   ./docker_netem.sh status  — show current netem config
#
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
set -euo pipefail

MODE="${1:-apply}"
CONTAINERS=(swarm_node_1 swarm_node_2 swarm_node_3 swarm_node_4 swarm_node_5 swarm_node_6)
DELAY_MS=100
LOSS_PCT=5

apply_netem() {
    local container="$1"
    echo "[netem] Applying to ${container}: ${DELAY_MS}ms delay, ${LOSS_PCT}% loss"
    docker exec --privileged "${container}" \
        tc qdisc replace dev eth0 root netem \
        delay "${DELAY_MS}ms" 25ms distribution normal \
        loss "${LOSS_PCT}%" \
        || echo "[netem] WARN: failed on ${container} (container may not be running)"
}

clear_netem() {
    local container="$1"
    echo "[netem] Clearing netem from ${container}"
    docker exec --privileged "${container}" \
        tc qdisc del dev eth0 root 2>/dev/null \
        || echo "[netem] WARN: no netem to clear on ${container}"
}

show_status() {
    local container="$1"
    echo "=== ${container} ==="
    docker exec "${container}" tc qdisc show dev eth0 2>/dev/null \
        || echo "  (not running)"
}

case "$MODE" in
    apply)
        echo "Applying netem: ${DELAY_MS}ms latency, ${LOSS_PCT}% packet loss"
        for c in "${CONTAINERS[@]}"; do
            apply_netem "$c"
        done
        echo "Done. Verify with: $0 status"
        ;;
    clear)
        echo "Clearing netem from all containers"
        for c in "${CONTAINERS[@]}"; do
            clear_netem "$c"
        done
        echo "Done."
        ;;
    status)
        for c in "${CONTAINERS[@]}"; do
            show_status "$c"
        done
        ;;
    *)
        echo "Usage: $0 {apply|clear|status}"
        exit 1
        ;;
esac
