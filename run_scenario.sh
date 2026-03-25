#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Usage:
#   ./run_scenario.sh              # run swarm scenario then open 3D visualization
#   ./run_scenario.sh --single     # run single-drone scenario then open 3D visualization
#   ./run_scenario.sh --sim-only   # run swarm scenario only (no GUI)
#   ./run_scenario.sh --viz-only   # open visualization (using existing swarm data)
#   ./run_scenario.sh --test       # run physics tests
#   ./run_scenario.sh --benchmark  # run deterministic benchmark validation gates
#   ./run_scenario.sh --all        # run tests, benchmark, scenario, and visualization
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

    ok "Virtual environment ready"
}

# ── Actions ──────────────────────────────────────────────────────────────────
run_tests() {
    info "Running drone physics tests..."
    python -m pytest "$SIM_DIR/test_drone_physics.py" -v
    ok "All physics tests passed"
}

run_scenario() {
    info "Running swarm flight scenario..."
    python "$SIM_DIR/swarm_scenario.py"
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
    local profile
    for profile in "${profiles[@]}"; do
        info "Running deterministic benchmark: $profile"
        python "$SIM_DIR/drone_scenario.py" --benchmark "$profile"
    done

    ok "Deterministic benchmark validation passed"
}

run_viz() {
    local data_file="$SIM_DIR/swarm_data.npz"
    if [ ! -f "$data_file" ]; then
        warn "No swarm data found. Running swarm scenario first..."
        run_scenario
    fi
    info "Launching 3D visualization..."
    python "$SIM_DIR/visualize_drone_3d.py" "$data_file"
}

run_single_viz() {
    local data_file="$SIM_DIR/scenario_data.npz"
    if [ ! -f "$data_file" ]; then
        warn "No single-drone data found. Running single-drone scenario first..."
        run_single_scenario
    fi
    info "Launching 3D visualization..."
    python "$SIM_DIR/visualize_drone_3d.py" "$data_file"
}

# ── Parse args ───────────────────────────────────────────────────────────────
MODE="${1:---default}"

case "$MODE" in
    --single)
        ensure_venv
        run_single_scenario
        run_single_viz
        ;;
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
        run_benchmark
        run_scenario
        run_viz
        ;;
    --benchmark)
        ensure_venv
        run_benchmark
        ;;
    --help|-h)
        echo "Usage: $0 [--single|--sim-only|--viz-only|--test|--benchmark|--all|--help]"
        echo ""
        echo "  (default)    Run swarm scenario then open 3D visualization"
        echo "  --single     Run single-drone scenario then open 3D visualization"
        echo "  --sim-only   Run swarm scenario only (no GUI)"
        echo "  --viz-only   Open visualization (uses existing swarm data)"
        echo "  --test       Run physics tests"
        echo "  --benchmark  Run deterministic benchmark validation gates"
        echo "  --all        Run tests, benchmark, then swarm scenario and visualization"
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
