#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Usage:
#   ./run_scenario.sh              # run scenario then open 3D visualization
#   ./run_scenario.sh --sim-only   # run scenario only (no GUI)
#   ./run_scenario.sh --viz-only   # open visualization (using existing data)
#   ./run_scenario.sh --test       # run physics tests
#   ./run_scenario.sh --all        # run tests, scenario, and visualization
# ──────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_DIR="$ROOT_DIR/sar_swarm_ws/src/sar_simulation"
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

    ok "Virtual environment ready"
}

# ── Actions ──────────────────────────────────────────────────────────────────
run_tests() {
    info "Running drone physics tests..."
    python -m pytest "$SIM_DIR/test_drone_physics.py" -v
    ok "All physics tests passed"
}

run_scenario() {
    info "Running drone flight scenario..."
    python "$SIM_DIR/drone_scenario.py"
    ok "Scenario complete — data saved"
}

run_viz() {
    local data_file="$SIM_DIR/scenario_data.npz"
    if [ ! -f "$data_file" ]; then
        warn "No scenario data found. Running scenario first..."
        run_scenario
    fi
    info "Launching 3D visualization..."
    python "$SIM_DIR/visualize_drone_3d.py"
}

# ── Parse args ───────────────────────────────────────────────────────────────
MODE="${1:---default}"

case "$MODE" in
    --sim-only)
        ensure_venv
        run_scenario
        ;;
    --viz-only)
        ensure_venv
        run_viz
        ;;
    --test)
        RUN_TESTS=1 ensure_venv
        run_tests
        ;;
    --all)
        RUN_TESTS=1 ensure_venv
        run_tests
        run_scenario
        run_viz
        ;;
    --help|-h)
        echo "Usage: $0 [--sim-only|--viz-only|--test|--all|--help]"
        echo ""
        echo "  (default)    Run scenario then open 3D visualization"
        echo "  --sim-only   Run scenario only (no GUI)"
        echo "  --viz-only   Open visualization (uses existing data)"
        echo "  --test       Run physics tests"
        echo "  --all        Run tests, then scenario, then visualization"
        echo "  --help       Show this help"
        ;;
    --default)
        ensure_venv
        run_scenario
        run_viz
        ;;
    *)
        fail "Unknown option: $MODE (use --help)"
        ;;
esac
