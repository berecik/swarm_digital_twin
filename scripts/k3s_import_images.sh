#!/usr/bin/env bash
# Import Docker images into k3s containerd.
# Run this ON the k3s node (paul), not remotely.
#
# Usage:
#   sudo ./scripts/k3s_import_images.sh
#   # or from another machine:
#   ssh paul 'cd /path/to/swarm_digital_twin && sudo ./scripts/k3s_import_images.sh'

set -euo pipefail

GREEN='\033[0;32m'; CYAN='\033[0;36m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

# Verify we're running with permissions to access containerd
if ! k3s ctr version &>/dev/null; then
    fail "Cannot access k3s containerd. Run with: sudo $0"
fi

IMAGES=(
    "ardupilot-sitl:latest"
    "swarm_companion:latest"
    "eclipse/zenoh-bridge-ros2dds:latest"
)

for img in "${IMAGES[@]}"; do
    # Check if already in containerd
    if k3s ctr images check "name==${img}" 2>/dev/null | grep -q "${img}"; then
        ok "$img already in k3s"
        continue
    fi

    # Try import from Docker
    if docker image inspect "$img" &>/dev/null; then
        info "Importing $img from Docker into k3s..."
        docker save "$img" | k3s ctr images import -
        ok "$img imported"
    else
        info "$img not in Docker, pulling..."
        k3s ctr images pull "docker.io/library/$img" 2>/dev/null || \
        k3s ctr images pull "docker.io/$img" 2>/dev/null || \
        fail "Cannot find $img in Docker or pull it"
        ok "$img pulled"
    fi
done

echo ""
ok "All images available in k3s"
k3s crictl images | grep -E "ardupilot|swarm_companion|zenoh" || true
