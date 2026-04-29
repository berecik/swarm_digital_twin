#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# Single-drone waypoint-achievement PID tuner.
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
#
# Runs bounded random search over the cascaded position+attitude PID
# gains for a single drone, then registers the best policy via
# ml.model_registry. The "model" here is an 18-float gain vector
# (six PIDs × kp/ki/kd) — see simulation/ml/waypoint_optimizer.py for
# the schema.
#
# Promotion uses ml.waypoint_kpi.compare_waypoint_policies (acceptance
# gate + completion/RMSE delta + energy non-regression).
#
# Usage:
#   ./scripts/ml_train_waypoint.sh                                   # 8 trials, patrol mission
#   ./scripts/ml_train_waypoint.sh --trials 32 --mission lawnmower
#   ./scripts/ml_train_waypoint.sh --missions patrol,lawnmower --seed 7
#   ./scripts/ml_train_waypoint.sh --out /tmp/wp_run1 --max-time 90
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

TRIALS=8
MISSIONS="patrol"
SEED=0
MAX_TIME=60
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$ROOT_DIR/reports/ml_waypoint/$TIMESTAMP"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --trials)   TRIALS="$2"; shift 2 ;;
    --mission)  MISSIONS="$2"; shift 2 ;;
    --missions) MISSIONS="$2"; shift 2 ;;
    --seed)     SEED="$2"; shift 2 ;;
    --max-time) MAX_TIME="$2"; shift 2 ;;
    --out)      OUT_DIR="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,22p' "$0" | sed 's/^# //; s/^#//'
      exit 0 ;;
    *) fail "Unknown flag: $1" ;;
  esac
done

[[ -d "$VENV_DIR" ]] || fail "venv not found at $VENV_DIR — run ./run_scenario.sh once to bootstrap."

mkdir -p "$OUT_DIR"
info "Output directory: $OUT_DIR"
info "Trials: $TRIALS | Missions: $MISSIONS | Seed: $SEED | Episode max: ${MAX_TIME}s"

"$VENV_DIR/bin/python" - "$OUT_DIR" "$TRIALS" "$MISSIONS" "$SEED" "$MAX_TIME" <<'PYEOF'
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, "simulation")

from ml.model_registry import ModelEntry, ModelRegistry
from ml.waypoint_kpi import compare_waypoint_policies, evaluate_waypoint_kpis
from ml.waypoint_optimizer import (
    PolicyGains, SearchBounds, evaluate_policy, random_search,
)

out_dir = Path(sys.argv[1])
n_trials = int(sys.argv[2])
missions = [m.strip() for m in sys.argv[3].split(",") if m.strip()]
seed = int(sys.argv[4])
max_time = float(sys.argv[5])

print(f"[1/4] Random search over {n_trials} trials on missions={missions}…")
t0 = time.time()
result = random_search(
    n_trials=n_trials, seed=seed,
    mission_kinds_to_run=missions,
    max_time=max_time,
)
elapsed = time.time() - t0
print(f"    {len(result.trials)} trials evaluated in {elapsed:.1f}s")
print(f"    best objective: {result.best_objective:.3f}")
print(f"    best metrics: completion={result.best_metrics['completion_ratio']:.3f}"
      f" rmse={result.best_metrics['rmse_xyz_m']:.3f}m"
      f" t_first={result.best_metrics['time_to_first_wp_s']:.2f}s"
      f" overshoot={result.best_metrics['max_overshoot_m']:.2f}m"
      f" energy={result.best_metrics['energy_proxy_j']:.1f}J")

print(f"[2/4] Acceptance gate…")
verdict = evaluate_waypoint_kpis(result.best_metrics)
print(f"    verdict: {verdict.verdict}")
if verdict.failures:
    print(f"    failures: {verdict.failures}")

print(f"[3/4] Registering policy…")
reg_path = out_dir / "policy_registry.json"
reg = ModelRegistry(reg_path)
incumbent = reg.best_by_kpi("completion_ratio")
incumbent_kpis = incumbent.kpis if incumbent is not None else None
promote, reason = compare_waypoint_policies(result.best_metrics, incumbent_kpis)
print(f"    promote: {promote} — {reason}")

if verdict.verdict == "PASS" and (incumbent is None or promote):
    next_idx = len(reg.all()) + 1
    version = f"pid_v{next_idx}"
    parent = incumbent.version if incumbent else None
    kpis = {**result.best_metrics, **result.best_gains.to_dict()}
    reg.register(ModelEntry(
        version=version, backend="pid",
        weights_path=str(out_dir / f"{version}_gains.json"),
        parent_version=parent,
        kpis=kpis,
        notes=(f"random_search trials={n_trials} seed={seed} "
               f"missions={missions} obj={result.best_objective:.3f}"),
    ))
    (out_dir / f"{version}_gains.json").write_text(
        json.dumps(result.best_gains.to_dict(), indent=2), encoding="utf-8")
    print(f"    registered {version} → {reg_path}")
elif verdict.verdict != "PASS":
    print(f"    NOT registered: candidate failed acceptance gate.")
else:
    print(f"    NOT registered: {reason}")

print(f"[4/4] Writing trial history…")
trial_log = []
for gains, metrics, obj in result.trials:
    trial_log.append({
        "objective": obj,
        "metrics": metrics,
        "gains": gains.to_dict(),
    })
(out_dir / "trial_history.json").write_text(
    json.dumps(trial_log, indent=2), encoding="utf-8")

(out_dir / "summary.json").write_text(json.dumps({
    "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "trials_run": len(result.trials),
    "elapsed_s": elapsed,
    "missions": missions,
    "seed": seed,
    "best_objective": result.best_objective,
    "best_metrics": result.best_metrics,
    "verdict": verdict.verdict,
    "verdict_failures": verdict.failures,
    "promoted": (verdict.verdict == "PASS" and (incumbent is None or promote)),
}, indent=2), encoding="utf-8")
print(f"\nArtefacts written to {out_dir}/")
PYEOF

ok "Training complete. Artefacts in: $OUT_DIR"
ls -la "$OUT_DIR" >&2
