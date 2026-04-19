"""
CLI entry point for the Phase 6 acceptance matrix.

Usage::

    python -m simulation.acceptance_matrix --subset ci --output reports/

When a single scenario is wanted::

    python -m simulation.acceptance_matrix \\
        --drones 3 --terrain synthetic_rolling --wind crosswind \\
        --mission lawnmower --fault none --output reports/

Per-scenario report tree::

    reports/<scenario_id>/
        kpis.json
        summary.md
        config.toml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow `python -m simulation.acceptance_matrix` from project root or
# `python acceptance_matrix.py` from inside simulation/.
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from acceptance_report import run_scenario, write_report  # noqa: E402
from scenario_matrix import (  # noqa: E402
    DRONE_COUNTS, FAULTS, MISSIONS, ScenarioConfig, TERRAINS,
    WINDS, select,
)


def _parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="acceptance_matrix",
        description="Run the Phase 6 K8s validation matrix (Python-only path).",
    )
    p.add_argument("--subset", choices=["ci", "full"],
                   help="Run a named scenario subset.")
    p.add_argument("--output", type=Path, default=Path("reports"),
                   help="Report root directory (default: reports/).")
    p.add_argument("--max-time", type=float, default=180.0,
                   help="Per-scenario max sim time in seconds (default: 180).")
    p.add_argument("--drones", type=int, choices=DRONE_COUNTS,
                   help="Single-scenario: drone count.")
    p.add_argument("--terrain", choices=TERRAINS,
                   help="Single-scenario: terrain name.")
    p.add_argument("--wind", choices=WINDS,
                   help="Single-scenario: wind profile.")
    p.add_argument("--mission", choices=MISSIONS,
                   help="Single-scenario: mission kind.")
    p.add_argument("--fault", choices=FAULTS, default="none",
                   help="Single-scenario: fault kind (recorded only).")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)

    if args.subset:
        scenarios = select(args.subset)
    else:
        required = ("drones", "terrain", "wind", "mission")
        missing = [k for k in required if getattr(args, k) is None]
        if missing:
            print(f"Single-scenario mode requires {missing}; or pass --subset.",
                  file=sys.stderr)
            return 2
        scenarios = [ScenarioConfig(
            drones=args.drones, terrain=args.terrain, wind=args.wind,
            mission=args.mission, fault=args.fault)]

    args.output.mkdir(parents=True, exist_ok=True)
    n_pass = 0
    n_total = len(scenarios)
    print(f"Running {n_total} scenario(s) → {args.output}/", flush=True)
    for i, cfg in enumerate(scenarios, start=1):
        t0 = time.time()
        kpis = run_scenario(cfg, max_time=args.max_time)
        out = write_report(cfg, kpis, args.output)
        elapsed = time.time() - t0
        marker = "PASS" if kpis.verdict == "PASS" else "FAIL"
        print(f"  [{i:3d}/{n_total}] {marker} {cfg.scenario_id} "
              f"({elapsed:.1f}s) → {out.relative_to(args.output)}/",
              flush=True)
        if kpis.verdict == "PASS":
            n_pass += 1

    print(f"\n{n_pass}/{n_total} PASS", flush=True)
    return 0 if n_pass == n_total else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
