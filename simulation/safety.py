"""
Safety monitoring for swarm simulations.

Provides inter-drone separation checks, terrain collision detection,
and safety KPI tracking. Designed to run at the control loop rate
(50 Hz) on both standalone simulations and K8s/Gazebo deployments.

Usage::

    from safety import SeparationMonitor, TerrainMonitor, SafetyReport

    sep = SeparationMonitor(min_separation=1.5, near_miss_threshold=3.0)
    ter = TerrainMonitor(terrain, min_agl=5.0)

    for record in swarm_records:
        positions = {i: record.positions[i] for i in range(n)}
        sep.check(positions, record.t)
        for drone_id, pos in positions.items():
            ter.check(drone_id, pos, record.t)

    report = SafetyReport.from_monitors(sep, ter)
    print(report.summary())
    assert report.is_safe()
"""

from dataclasses import dataclass, field
from itertools import combinations
from typing import Dict, List, Optional

import numpy as np


# ── Safety events ────────────────────────────────────────────────────────


@dataclass
class NearMissEvent:
    """Two drones came within the near-miss threshold."""
    t: float
    drone_a: int
    drone_b: int
    distance: float
    severity: str = "warning"


@dataclass
class CollisionEvent:
    """Two drones breached the minimum separation distance."""
    t: float
    drone_a: int
    drone_b: int
    distance: float
    severity: str = "critical"


@dataclass
class ClearanceViolationEvent:
    """A drone dropped below minimum AGL clearance."""
    t: float
    drone_id: int
    position: np.ndarray
    agl: float
    severity: str = "warning"


@dataclass
class TerrainCollisionEvent:
    """A drone penetrated the terrain surface (AGL < 0)."""
    t: float
    drone_id: int
    position: np.ndarray
    agl: float
    severity: str = "critical"


# Union type for all safety events.
SafetyEvent = (NearMissEvent | CollisionEvent
               | ClearanceViolationEvent | TerrainCollisionEvent)


# ── Separation monitor ──────────────────────────────────────────────────


class SeparationMonitor:
    """Check pairwise inter-drone distances at each timestep.

    Produces :class:`CollisionEvent` (distance < ``min_separation``)
    and :class:`NearMissEvent` (distance < ``near_miss_threshold``).
    """

    def __init__(self, min_separation: float = 1.5,
                 near_miss_threshold: float = 3.0) -> None:
        self.min_separation = min_separation
        self.near_miss_threshold = near_miss_threshold
        self.events: List[SafetyEvent] = []
        self.min_distance: float = float("inf")
        self._samples: int = 0

    def check(self, positions: Dict[int, np.ndarray], t: float) -> None:
        """Check all pairwise distances at time *t*."""
        self._samples += 1
        for (i, pi), (j, pj) in combinations(positions.items(), 2):
            dist = float(np.linalg.norm(pi - pj))
            self.min_distance = min(self.min_distance, dist)
            if dist < self.min_separation:
                self.events.append(CollisionEvent(
                    t=t, drone_a=i, drone_b=j, distance=dist))
            elif dist < self.near_miss_threshold:
                self.events.append(NearMissEvent(
                    t=t, drone_a=i, drone_b=j, distance=dist))

    def check_swarm_record(self, record) -> None:
        """Check a :class:`SwarmRecord` (positions array indexed by drone)."""
        positions = {i: record.positions[i]
                     for i in range(len(record.positions))}
        self.check(positions, record.t)

    @property
    def collision_count(self) -> int:
        return sum(1 for e in self.events if isinstance(e, CollisionEvent))

    @property
    def near_miss_count(self) -> int:
        return sum(1 for e in self.events if isinstance(e, NearMissEvent))


# ── Terrain monitor ─────────────────────────────────────────────────────


class TerrainMonitor:
    """Check drone altitude against terrain at each timestep.

    Produces :class:`TerrainCollisionEvent` (AGL < 0) and
    :class:`ClearanceViolationEvent` (AGL < ``min_agl``).
    """

    def __init__(self, terrain, min_agl: float = 5.0) -> None:
        self.terrain = terrain
        self.min_agl = min_agl
        self.events: List[SafetyEvent] = []
        self.min_agl_observed: float = float("inf")
        self._samples: int = 0

    def check(self, drone_id: int, position: np.ndarray, t: float) -> None:
        """Check one drone's AGL at time *t*."""
        self._samples += 1
        ground = self.terrain.get_elevation(position[0], position[1])
        agl = position[2] - ground
        self.min_agl_observed = min(self.min_agl_observed, agl)
        if agl < 0:
            self.events.append(TerrainCollisionEvent(
                t=t, drone_id=drone_id,
                position=position.copy(), agl=agl))
        elif agl < self.min_agl:
            self.events.append(ClearanceViolationEvent(
                t=t, drone_id=drone_id,
                position=position.copy(), agl=agl))

    @property
    def terrain_collision_count(self) -> int:
        return sum(1 for e in self.events
                   if isinstance(e, TerrainCollisionEvent))

    @property
    def clearance_violation_count(self) -> int:
        return sum(1 for e in self.events
                   if isinstance(e, ClearanceViolationEvent))


# ── Safety report ────────────────────────────────────────────────────────


@dataclass
class SafetyReport:
    """Aggregated safety KPIs for a simulation run."""
    collision_count: int = 0
    near_miss_count: int = 0
    min_separation: float = float("inf")
    terrain_collision_count: int = 0
    clearance_violation_count: int = 0
    min_agl: float = float("inf")
    total_events: int = 0
    all_events: List[SafetyEvent] = field(default_factory=list)

    @classmethod
    def from_monitors(cls, sep: SeparationMonitor,
                      ter: Optional[TerrainMonitor] = None) -> "SafetyReport":
        events = list(sep.events)
        report = cls(
            collision_count=sep.collision_count,
            near_miss_count=sep.near_miss_count,
            min_separation=sep.min_distance,
        )
        if ter is not None:
            events.extend(ter.events)
            report.terrain_collision_count = ter.terrain_collision_count
            report.clearance_violation_count = ter.clearance_violation_count
            report.min_agl = ter.min_agl_observed
        report.total_events = len(events)
        report.all_events = sorted(events, key=lambda e: e.t)
        return report

    def is_safe(self) -> bool:
        """True if no critical events occurred."""
        return (self.collision_count == 0
                and self.terrain_collision_count == 0)

    def summary(self) -> str:
        verdict = "SAFE" if self.is_safe() else "UNSAFE"
        lines = [
            f"Safety Report: {verdict}",
            f"  Collisions:             {self.collision_count}",
            f"  Near misses:            {self.near_miss_count}",
            f"  Min separation:         {self.min_separation:.3f} m",
            f"  Terrain collisions:     {self.terrain_collision_count}",
            f"  Clearance violations:   {self.clearance_violation_count}",
            f"  Min AGL:                {self.min_agl:.3f} m",
            f"  Total events:           {self.total_events}",
        ]
        return "\n".join(lines)

    def to_dict(self) -> dict:
        return {
            "verdict": "SAFE" if self.is_safe() else "UNSAFE",
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            "min_separation_m": self.min_separation,
            "terrain_collision_count": self.terrain_collision_count,
            "clearance_violation_count": self.clearance_violation_count,
            "min_agl_m": self.min_agl,
            "total_events": self.total_events,
        }
