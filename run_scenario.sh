#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Drone Digital Twin — Run Scenario & Visualization
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
# Website: https://marysia.app
#
# Usage:
#   ./run_scenario.sh              # run single-drone scenario then open 3D visualization
#   ./run_scenario.sh --single     # run single-drone scenario then open 3D visualization
#   ./run_scenario.sh --swarm [N]  # run swarm scenario for N drones (default: 6) + visualization
#   ./run_scenario.sh --ci-local   # run local CI/CD-equivalent pipeline (tests + all benchmarks)
#   ./run_scenario.sh --sim-only   # run single-drone scenario only (no GUI)
#   ./run_scenario.sh --viz-only   # open visualization (using existing single-drone data)
#   ./run_scenario.sh --test       # run physics tests
#   ./run_scenario.sh --benchmark  # run deterministic benchmark validation gates
#   ./run_scenario.sh --real-log   # run real-log validation against paper Table 5
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
        if [ "$data_file" = "$SIM_DIR/swarm_data.npz" ]; then
            warn "No swarm data found. Running swarm scenario first..."
            run_scenario
        else
            warn "No single-drone data found. Running single-drone scenario first..."
            run_single_scenario
        fi
    fi
    info "Launching 3D visualization..."
    python "$SIM_DIR/visualize_drone_3d.py" "$data_file"
}

run_single_viz() {
    run_viz "$SIM_DIR/scenario_data.npz"
}

# ── Parse args ───────────────────────────────────────────────────────────────
MODE="${1:---default}"
SWARM_DRONES="${2:-6}"

case "$MODE" in
    --single)
        ensure_venv
        run_single_scenario
        run_single_viz
        ;;
    --sim-only)
        ensure_venv
        run_single_scenario
        ;;
    --swarm)
        ensure_venv
        if ! [[ "$SWARM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
            fail "Invalid drone count: $SWARM_DRONES (must be a positive integer)"
        fi
        run_scenario "$SWARM_DRONES"
        run_viz "$SIM_DIR/swarm_data.npz"
        ;;
    --viz-only)
        ensure_venv
        run_single_viz
        ;;
    --test)
        RUN_TESTS=1 ensure_venv
        run_tests
        ;;
    --ci-local)
        RUN_TESTS=1 ensure_venv
        run_ci_local
        ;;
    --all)
        RUN_TESTS=1 ensure_venv
        run_tests
        run_benchmark
        run_real_log
        run_single_scenario
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
    --help|-h)
        echo "Usage: $0 [--single|--swarm [N]|--sim-only|--viz-only|--test|--benchmark|--real-log|--ci-local|--all|--help]"
        echo ""
        echo "  (default)    Run single-drone scenario then open 3D visualization"
        echo "  --single     Run single-drone scenario then open 3D visualization"
        echo "  --swarm [N]  Run swarm scenario for N drones (default: 6) then visualize"
        echo "  --sim-only   Run single-drone scenario only (no GUI)"
        echo "  --viz-only   Open visualization (uses existing single-drone data)"
        echo "  --test       Run physics tests"
        echo "  --benchmark  Run deterministic benchmark validation gates"
        echo "  --real-log   Run real-log validation against paper Table 5"
        echo "  --ci-local   Run local CI/CD-equivalent pipeline (tests + all CI benchmarks)"
        echo "  --all        Run tests, benchmark, then swarm scenario and visualization"
        echo "  --help       Show this help"
        ;;
    --default)
        ensure_venv
        run_single_scenario
        run_single_viz
        ;;
    *)
        fail "Unknown option: $MODE (use --help)"
        ;;
esac
