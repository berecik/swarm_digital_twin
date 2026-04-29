"""
Mission factory for the full-system acceptance matrix.

Maps mission kinds to per-drone waypoint lists. The terrain argument
lets each mission keep a safe AGL clearance over rolling/steep terrain
without rewriting the controller.

Mission kinds:
  patrol     — ring formation tracing a square patrol path.
  lawnmower  — area-search lawnmower scan; drones share lanes.
  escort     — single VIP path with N-1 escorts in offset slots.
  heavy_lift — N drones in a tight formation circling a pickup point
               (proxy for the heavy-lift Distributed Lift System).
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np


CRUISE_AGL_M = 25.0


def _cruise_z(terrain) -> float:
    """Pick a cruise altitude clear of the terrain's highest peak."""
    if terrain is None:
        return CRUISE_AGL_M
    return float(np.max(terrain.elevations) + CRUISE_AGL_M)


def _ring_offsets(n: int, radius: float = 8.0) -> List[np.ndarray]:
    return [
        radius * np.array([np.cos(2 * np.pi * i / n),
                           np.sin(2 * np.pi * i / n),
                           0.0])
        for i in range(n)
    ]


def patrol_mission(n: int, terrain=None) -> Dict[int, List[np.ndarray]]:
    """N drones in a ring tracing a small square patrol.

    Ring radius scales generously with *n* so the controller's transient
    overshoot still leaves adjacent drones above the 1.5 m separation
    floor (chord ≈ 2·r·sin(π/n)). For 12 drones, r=20 m → chord ≈ 10 m.
    """
    z = _cruise_z(terrain)
    offsets = _ring_offsets(n, radius=max(10.0, n * 1.5))
    centers = [
        np.array([0.0,  0.0, z]),
        np.array([15.0, 0.0, z]),
        np.array([15.0, 15.0, z]),
        np.array([0.0,  15.0, z]),
        np.array([0.0,  0.0, z]),
    ]
    return {
        i + 1: [c + offsets[i] for c in centers]
        for i in range(n)
    }


def lawnmower_mission(n: int, terrain=None) -> Dict[int, List[np.ndarray]]:
    """N drones share a lawnmower scan, splitting the area into N lanes."""
    z = _cruise_z(terrain)
    width = 30.0
    length = 40.0
    lane = width / max(n, 1)
    waypoints: Dict[int, List[np.ndarray]] = {}
    for i in range(n):
        x_lane = -width / 2 + lane * (i + 0.5)
        waypoints[i + 1] = [
            np.array([x_lane, -length / 2, z]),
            np.array([x_lane,  length / 2, z]),
            np.array([x_lane, -length / 2, z]),
        ]
    return waypoints


def escort_mission(n: int, terrain=None) -> Dict[int, List[np.ndarray]]:
    """One VIP plus N-1 escorts at fixed slot offsets around it."""
    z = _cruise_z(terrain)
    vip_path = [
        np.array([0.0,  0.0, z]),
        np.array([20.0, 0.0, z]),
        np.array([20.0, 20.0, z]),
        np.array([0.0,  0.0, z]),
    ]
    waypoints: Dict[int, List[np.ndarray]] = {1: list(vip_path)}
    if n == 1:
        return waypoints
    # Generous escort ring: 8 m base + 1 m per drone keeps adjacent
    # escorts above the 1.5 m floor even during transient overshoot.
    offsets = _ring_offsets(n - 1, radius=max(8.0, (n - 1) * 1.5))
    for i, off in enumerate(offsets, start=2):
        waypoints[i] = [p + off for p in vip_path]
    return waypoints


def heavy_lift_mission(n: int, terrain=None) -> Dict[int, List[np.ndarray]]:
    """N drones circle a pickup point (DLS proxy).

    Ring radius scales with *n* so adjacent drones stay above the
    1.5 m separation floor even with controller transient overshoot.
    """
    z = _cruise_z(terrain)
    radius = max(5.0, n * 1.2)
    offsets = _ring_offsets(n, radius=radius)
    pickup = np.array([0.0, 0.0, z])
    dropoff = np.array([10.0, 0.0, z])
    return {
        i + 1: [
            pickup + offsets[i],
            dropoff + offsets[i],
            pickup + offsets[i],
        ]
        for i in range(n)
    }


_MISSION_FNS = {
    "patrol": patrol_mission,
    "lawnmower": lawnmower_mission,
    "escort": escort_mission,
    "heavy_lift": heavy_lift_mission,
}


def build_mission(kind: str, n: int,
                  terrain=None) -> Dict[int, List[np.ndarray]]:
    """Resolve a mission kind to a per-drone waypoint dict."""
    if kind not in _MISSION_FNS:
        raise ValueError(
            f"unknown mission '{kind}'; available: {sorted(_MISSION_FNS)}"
        )
    return _MISSION_FNS[kind](n, terrain=terrain)


def mission_kinds() -> List[str]:
    return sorted(_MISSION_FNS)
