"""
pytest configuration for the swarm-digital-twin test surface.

Tests live both at the simulation/ root and inside per-domain
subdirectories (test_drone_physics/, test_runtime_view/). The simulation
modules they exercise (`drone_physics`, `terrain`, `wind_model`, …) are
imported via plain bare names like::

    from drone_physics import DroneParams

That works for tests at the simulation/ root because pytest auto-adds
the test file's directory to sys.path. For tests in subdirectories the
file's parent (e.g. simulation/test_drone_physics/) is added instead,
which doesn't contain the modules. This conftest puts simulation/ on
sys.path explicitly so both levels resolve.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parent
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))
