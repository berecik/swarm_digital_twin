#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Build, tag, and push Docker images to a public container registry.
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
#
# Usage:
#   ./scripts/push_images.sh                    # build + push all images
#   ./scripts/push_images.sh --sitl             # build + push ardupilot-sitl only
#   ./scripts/push_images.sh --companion        # build + push swarm_companion only
#   ./scripts/push_images.sh --tag v0.2.0       # use specific tag instead of latest
#   ./scripts/push_images.sh --registry ghcr.io/myorg  # override registry
#   ./scripts/push_images.sh --dry-run          # show commands without executing
#
# Prerequisites:
#   docker login ghcr.io   (or your registry)
# ──────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ── Defaults ──────────────────────────────────────────────────────────────────
REGISTRY="${SWARM_REGISTRY:-ghcr.io/berecik}"
TAG="${SWARM_IMAGE_TAG:-latest}"
BUILD_SITL=true
BUILD_COMPANION=true
DRY_RUN=false

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*" >&2; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*" >&2; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*" >&2; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*" >&2; exit 1; }

run() {
    if [ "$DRY_RUN" = true ]; then
        echo -e "${YELLOW}[DRY-RUN]${NC} $*" >&2
    else
        "$@"
    fi
}

# ── Parse args ────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sitl)       BUILD_COMPANION=false; shift ;;
        --companion)  BUILD_SITL=false; shift ;;
        --tag)        TAG="$2"; shift 2 ;;
        --registry)   REGISTRY="$2"; shift 2 ;;
        --dry-run)    DRY_RUN=true; shift ;;
        --help|-h)
            head -16 "$0" | tail -13
            exit 0
            ;;
        *) fail "Unknown option: $1" ;;
    esac
done

# ── Image names ───────────────────────────────────────────────────────────────
SITL_LOCAL="ardupilot-sitl:latest"
SITL_REMOTE="${REGISTRY}/ardupilot-sitl:${TAG}"

COMPANION_LOCAL="swarm_companion:latest"
COMPANION_REMOTE="${REGISTRY}/swarm_companion:${TAG}"

info "Registry: $REGISTRY"
info "Tag:      $TAG"
echo "" >&2

# ── Build + push ardupilot-sitl ───────────────────────────────────────────────
if [ "$BUILD_SITL" = true ]; then
    info "Building ardupilot-sitl..."
    run docker build --platform linux/amd64 -f "$ROOT_DIR/Dockerfile.sitl" \
        -t "$SITL_LOCAL" -t "$SITL_REMOTE" "$ROOT_DIR"
    ok "Built: $SITL_REMOTE"

    info "Pushing ardupilot-sitl..."
    run docker push "$SITL_REMOTE"
    ok "Pushed: $SITL_REMOTE"

    # Also push :latest if tag is not already latest
    if [ "$TAG" != "latest" ]; then
        local_latest="${REGISTRY}/ardupilot-sitl:latest"
        run docker tag "$SITL_REMOTE" "$local_latest"
        run docker push "$local_latest"
        ok "Pushed: $local_latest"
    fi
    echo "" >&2
fi

# ── Build + push swarm_companion ──────────────────────────────────────────────
if [ "$BUILD_COMPANION" = true ]; then
    info "Building swarm_companion..."
    run docker build -f "$ROOT_DIR/Dockerfile" \
        -t "$COMPANION_LOCAL" -t "$COMPANION_REMOTE" "$ROOT_DIR"
    ok "Built: $COMPANION_REMOTE"

    info "Pushing swarm_companion..."
    run docker push "$COMPANION_REMOTE"
    ok "Pushed: $COMPANION_REMOTE"

    if [ "$TAG" != "latest" ]; then
        local_latest="${REGISTRY}/swarm_companion:latest"
        run docker tag "$COMPANION_REMOTE" "$local_latest"
        run docker push "$local_latest"
        ok "Pushed: $local_latest"
    fi
    echo "" >&2
fi

# ── Summary ───────────────────────────────────────────────────────────────────
ok "All images pushed to $REGISTRY"
echo "" >&2
echo "Deploy with:" >&2
echo "  helm upgrade --install swarm ./helm/swarm-digital-twin \\" >&2
echo "    --set images.sitl.repository=${REGISTRY}/ardupilot-sitl \\" >&2
echo "    --set images.companion.repository=${REGISTRY}/swarm_companion \\" >&2
echo "    --set images.sitl.tag=${TAG} \\" >&2
echo "    --set images.companion.tag=${TAG} \\" >&2
echo "    -n swarm --create-namespace" >&2
echo "" >&2
echo "Or simply:" >&2
echo "  ./run_scenario.sh --swarm 6" >&2
