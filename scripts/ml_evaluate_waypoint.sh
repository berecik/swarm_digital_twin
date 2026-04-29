#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Evaluate a registered waypoint policy against the acceptance gate.
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
#
# Loads a policy by version (or the registry's best-by-completion if no
# version is given), re-runs evaluate_policy() on the requested mission(s),
# and prints the verdict + per-mission breakdown.
#
# Usage:
#   ./scripts/ml_evaluate_waypoint.sh                                     # best policy, patrol
#   ./scripts/ml_evaluate_waypoint.sh --version pid_v2                    # specific version
#   ./scripts/ml_evaluate_waypoint.sh --registry /tmp/wp_run1/policy_registry.json
#   ./scripts/ml_evaluate_waypoint.sh --missions patrol,lawnmower,escort,heavy_lift
# ──────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SIM_DIR="$ROOT_DIR/simulation"
VENV_DIR="$ROOT_DIR/.venv"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC}  $*" >&2; }
ok()   { echo -e "${GREEN}[OK]${NC}    $*" >&2; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*" >&2; }
fail() { echo -e "${RED}[FAIL]${NC}  $*" >&2; exit 1; }

VERSION=""
REGISTRY=""
MISSIONS="patrol"
MAX_TIME=60

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)  VERSION="$2"; shift 2 ;;
    --registry) REGISTRY="$2"; shift 2 ;;
    --missions) MISSIONS="$2"; shift 2 ;;
    --max-time) MAX_TIME="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,17p' "$0" | sed 's/^# //; s/^#//'
      exit 0 ;;
    *) fail "Unknown flag: $1" ;;
  esac
done

[[ -d "$VENV_DIR" ]] || fail "venv not found at $VENV_DIR — run ./run_scenario.sh once to bootstrap."

# Resolve registry path: --registry overrides; otherwise pick the most
# recent reports/ml_waypoint/<ts>/policy_registry.json.
if [[ -z "$REGISTRY" ]]; then
  latest=$(ls -1d "$ROOT_DIR/reports/ml_waypoint/"*/ 2>/dev/null | tail -1 || true)
  if [[ -n "$latest" && -f "${latest}policy_registry.json" ]]; then
    REGISTRY="${latest}policy_registry.json"
  else
    fail "No --registry given and no reports/ml_waypoint/*/policy_registry.json found. Run ml_train_waypoint.sh first."
  fi
fi

[[ -f "$REGISTRY" ]] || fail "Registry not found: $REGISTRY"
info "Registry: $REGISTRY"
info "Missions: $MISSIONS"
[[ -n "$VERSION" ]] && info "Version: $VERSION" || info "Version: (best-by-completion)"

"$VENV_DIR/bin/python" - "$REGISTRY" "$VERSION" "$MISSIONS" "$MAX_TIME" <<'PYEOF'
import json
import sys
from pathlib import Path

sys.path.insert(0, "simulation")

from ml.model_registry import ModelRegistry
from ml.waypoint_kpi import evaluate_waypoint_kpis
from ml.waypoint_optimizer import PolicyGains, evaluate_policy, run_episode

registry_path = Path(sys.argv[1])
version = sys.argv[2] or None
missions = [m.strip() for m in sys.argv[3].split(",") if m.strip()]
max_time = float(sys.argv[4])

reg = ModelRegistry(registry_path)
if version:
    entry = reg.get(version)
else:
    entry = reg.best_by_kpi("completion_ratio")
    if entry is None:
        sys.exit("Registry contains no policies with completion_ratio.")

print(f"Evaluating policy {entry.version!r} ({entry.notes})")
gain_keys = {f for f in PolicyGains.from_baseline().to_dict()}
gain_dict = {k: v for k, v in entry.kpis.items() if k in gain_keys}
if len(gain_dict) != 18:
    # Fall back to the weights_path file if the gains weren't bundled
    # into kpis (older policies stored them only in the JSON sidecar).
    weights_path = Path(entry.weights_path)
    if weights_path.is_file():
        gain_dict = json.loads(weights_path.read_text(encoding="utf-8"))
    else:
        sys.exit(f"Could not recover gain vector for {entry.version}.")
gains = PolicyGains.from_dict(gain_dict)

print()
print(f"Per-mission breakdown (max_time={max_time}s):")
print(f"  {'mission':<14}  {'completion':>10}  {'rmse_m':>7}  "
      f"{'t_first_s':>9}  {'overshoot_m':>11}  {'energy_J':>9}")
all_metrics = []
for kind in missions:
    m = run_episode(gains, mission_kind=kind, max_time=max_time)
    print(f"  {kind:<14}  {m.completion_ratio:>10.3f}  {m.rmse_xyz_m:>7.3f}  "
          f"{m.time_to_first_wp_s:>9.2f}  {m.max_overshoot_m:>11.3f}  "
          f"{m.energy_proxy_j:>9.1f}")
    all_metrics.append(m)

# Aggregated verdict.
agg = evaluate_policy(gains, mission_kinds_to_run=missions, max_time=max_time)
verdict = evaluate_waypoint_kpis(agg)
print()
print(f"Aggregated verdict: {verdict.verdict}")
if verdict.failures:
    for f in verdict.failures:
        print(f"  - {f}")
else:
    print(f"  All gates pass on {len(missions)} mission(s).")
sys.exit(0 if verdict.verdict == "PASS" else 1)
PYEOF
