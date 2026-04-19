"""
Shared imports + helpers for the split test files.

This module centralises the parametrize-time helpers and module-level
constants that were originally inlined in `test_drone_physics.py` so the
per-domain test files (`test_terrain.py`, `test_runtime_view.py`,
`test_acceptance_matrix.py`, etc.) can import them without going through
the legacy mega-file.

Nothing here is collected by pytest itself — the underscore prefix on
the module name keeps it out of the discovery list and the file
contains no `test_*` symbols.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


# ── Path constants ───────────────────────────────────────────────────────────
# Tests originally lived at the simulation/ root and used
# `Path(__file__).resolve().parent` to find sibling files. After the
# directory split they live one level deeper, so import these constants
# instead of recomputing the relative path.

SIM_DIR = Path(__file__).resolve().parent      # .../simulation
PROJECT_ROOT = SIM_DIR.parent                  # repo root


# ── Live-view static-inspection helpers (Phase 7 invariant gate) ─────────────

_LIVE_JS = SIM_DIR / "runtime_view" / "web" / "live.js"


def live_js_source() -> str:
    """Return the raw `live.js` source — used by the static DOM gates."""
    return _LIVE_JS.read_text(encoding="utf-8")


# ── Terrain manifest helpers (Phase 3 parity tests) ──────────────────────────

PARITY_MAX_DELTA_M = 0.5
PARITY_SAMPLE_COUNT = 100
PARITY_RNG_SEED = 42


def parity_entry_names() -> list:
    """Manifest entries to exercise; safe to call at pytest collection time."""
    from terrain import manifest_entries
    try:
        return manifest_entries()
    except FileNotFoundError:  # pragma: no cover - manifest is in-tree
        return []


# ── Terrain regression helpers (Phase 3 sub-2) ───────────────────────────────

CRUISE_ALTITUDE_M = 25.0
CRUISE_AGL_FLOOR_M = 5.0
CLIMB_TIMEOUT_S = 6.0


def ramp_terrain():
    """Steep east-facing ramp (10° slope) bounded ±50 m around origin."""
    from terrain import TerrainMap
    return TerrainMap.from_function(
        lambda x, y: 0.176 * x,           # tan(10°) ≈ 0.176
        x_range=(-50.0, 50.0),
        y_range=(-50.0, 50.0),
        resolution=2.0,
    )


def regression_mission(terrain) -> list:
    """Three-waypoint patrol at CRUISE_ALTITUDE_M above the highest peak."""
    z_peak = float(np.max(terrain.elevations))
    z = z_peak + CRUISE_ALTITUDE_M
    return [
        np.array([0.0, 0.0, z]),
        np.array([20.0, 0.0, z]),
        np.array([20.0, 20.0, z]),
        np.array([0.0, 0.0, z]),
    ]


# ── Wind manifest helpers (Phase 5 stress envelopes) ─────────────────────────

def wind_profile_names_safe() -> list:
    """Manifest profiles to exercise; safe to call at pytest collection time."""
    from wind_model import wind_profile_names
    try:
        return wind_profile_names()
    except FileNotFoundError:  # pragma: no cover
        return []


# Per-profile expected wind-velocity magnitude (steady-state base wind, no
# turbulence component). Sourced from todo/wind_simulation.md.
PROFILE_BASE_SPEED_MS = {
    "calm": 0.5,
    "crosswind": 5.0,
    "gusty": 6.0,
    "storm": 12.0,
}


def stress_mission(z: float = 10.0) -> list:
    """A 3-waypoint there-and-back hop used by wind stress tests."""
    return [
        np.array([0.0, 0.0, z]),
        np.array([15.0, 0.0, z]),
        np.array([0.0, 0.0, z]),
    ]
