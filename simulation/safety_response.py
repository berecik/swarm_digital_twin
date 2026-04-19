"""
Safety Response Controller (Phase 4 close-out).

A pure-Python state machine that consumes the :class:`safety.SafetyEvent`
stream and decides what mode the autopilot should be in. The actual
PX4/MAVLink command emission lives behind a callback so this module
stays testable in CI without any flight-stack dependency.

States:

    NORMAL          — no incidents in the recent window
    WARNING         — at least one warning-severity event recently
    HOVER           — collision threshold breached; freeze in place
    RTL             — repeated breaches or terrain collision; return-to-launch
    EMERGENCY_STOP  — unrecoverable: cut motors / land immediately

Transitions are driven by the (severity, kind, count, dwell-time)
quadruple of recent safety events. The thresholds in
:class:`SafetyResponseThresholds` are configurable so the K8s lane can
tighten them without touching the state machine.

Example::

    cb = lambda old, new, reason: print(f"{old.value} -> {new.value} ({reason})")
    ctl = SafetyResponseController(on_transition=cb)
    for evt in safety_events:
        ctl.observe(evt, t=evt.t)
    ctl.tick(t=now)
    print(ctl.state, ctl.incident_log)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional

from safety import (
    ClearanceViolationEvent,
    CollisionEvent,
    NearMissEvent,
    SafetyEvent,
    TerrainCollisionEvent,
)


class SafetyMode(str, Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    HOVER = "HOVER"
    RTL = "RTL"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class SafetyResponseThresholds:
    """All knobs the responder hard-codes against. K8s tightens these."""
    near_miss_for_warning: int = 1            # any near-miss → WARNING
    collision_for_hover: int = 1              # any collision → HOVER
    collisions_for_rtl: int = 3               # repeated collisions → RTL
    terrain_collision_triggers_rtl: bool = True
    clearance_violations_for_warning: int = 5
    warning_dwell_s: float = 5.0              # min time in WARNING before NORMAL
    hover_dwell_s: float = 3.0                # min time in HOVER before downgrade
    rtl_locks_until_emergency: bool = True    # RTL never auto-clears in flight
    emergency_on_repeated_terrain: int = 2    # 2+ terrain hits → EMERGENCY


@dataclass
class IncidentRecord:
    """One transition entry — `to_dict()` is JSON-friendly."""
    t: float
    from_mode: SafetyMode
    to_mode: SafetyMode
    reason: str
    triggering_event_kind: str

    def to_dict(self) -> dict:
        return {
            "t": float(self.t),
            "from_mode": self.from_mode.value,
            "to_mode": self.to_mode.value,
            "reason": self.reason,
            "triggering_event_kind": self.triggering_event_kind,
        }


# ── Controller ───────────────────────────────────────────────────────────────


@dataclass
class SafetyResponseController:
    """State machine + incident log.

    *on_transition* is called as ``on_transition(old_mode, new_mode, reason)``
    on every transition; the K8s lane wires it to the PX4 command emitter.
    """
    thresholds: SafetyResponseThresholds = field(
        default_factory=SafetyResponseThresholds)
    on_transition: Optional[Callable[[SafetyMode, SafetyMode, str], None]] = None
    state: SafetyMode = SafetyMode.NORMAL
    incident_log: List[IncidentRecord] = field(default_factory=list)
    _state_entered_at: float = 0.0
    _collision_count: int = 0
    _terrain_collision_count: int = 0
    _near_miss_count: int = 0
    _clearance_count: int = 0

    # ── Inputs ──

    def observe(self, event: SafetyEvent, t: float) -> None:
        """Process one safety event at sim time *t*."""
        kind = type(event).__name__
        if isinstance(event, CollisionEvent):
            self._collision_count += 1
            if self._collision_count >= self.thresholds.collisions_for_rtl:
                self._transition(SafetyMode.RTL, t,
                                  f"{self._collision_count} collisions", kind)
            elif self._collision_count >= self.thresholds.collision_for_hover:
                self._transition(SafetyMode.HOVER, t, "collision", kind)
        elif isinstance(event, TerrainCollisionEvent):
            self._terrain_collision_count += 1
            if self._terrain_collision_count >= self.thresholds.emergency_on_repeated_terrain:
                self._transition(SafetyMode.EMERGENCY_STOP, t,
                                  f"{self._terrain_collision_count} terrain collisions",
                                  kind)
            elif self.thresholds.terrain_collision_triggers_rtl:
                self._transition(SafetyMode.RTL, t, "terrain collision", kind)
        elif isinstance(event, NearMissEvent):
            self._near_miss_count += 1
            if (self.state == SafetyMode.NORMAL
                    and self._near_miss_count >= self.thresholds.near_miss_for_warning):
                self._transition(SafetyMode.WARNING, t, "near miss", kind)
        elif isinstance(event, ClearanceViolationEvent):
            self._clearance_count += 1
            if (self.state == SafetyMode.NORMAL
                    and self._clearance_count >= self.thresholds.clearance_violations_for_warning):
                self._transition(
                    SafetyMode.WARNING, t,
                    f"{self._clearance_count} clearance violations", kind)

    def tick(self, t: float) -> None:
        """Time-driven transitions (downgrades after dwell)."""
        dwell = t - self._state_entered_at
        if (self.state == SafetyMode.WARNING
                and dwell >= self.thresholds.warning_dwell_s):
            self._transition(SafetyMode.NORMAL, t,
                              f"warning cleared after {dwell:.1f}s",
                              "tick")
        elif (self.state == SafetyMode.HOVER
                and dwell >= self.thresholds.hover_dwell_s
                and self._collision_count < self.thresholds.collisions_for_rtl):
            self._transition(SafetyMode.WARNING, t,
                              f"hover dwell elapsed ({dwell:.1f}s)",
                              "tick")
        # RTL stays put when rtl_locks_until_emergency=True; otherwise it
        # would also downgrade after a longer dwell. Keep the flight-safe
        # default — the operator clears RTL manually, not the controller.

    # ── Internals ──

    def _transition(self, new_mode: SafetyMode, t: float, reason: str,
                    kind: str) -> None:
        if new_mode == self.state:
            return
        if (self.state == SafetyMode.RTL
                and self.thresholds.rtl_locks_until_emergency
                and new_mode != SafetyMode.EMERGENCY_STOP):
            # Once in RTL we only escalate to EMERGENCY; never auto-downgrade.
            return
        old = self.state
        self.state = new_mode
        self._state_entered_at = t
        self.incident_log.append(IncidentRecord(
            t=t, from_mode=old, to_mode=new_mode,
            reason=reason, triggering_event_kind=kind,
        ))
        if self.on_transition is not None:
            try:
                self.on_transition(old, new_mode, reason)
            except Exception:  # pragma: no cover - callback must not kill us
                pass

    def replay(self, events: List[SafetyEvent]) -> None:
        """Convenience: replay an existing event list in time order."""
        for e in sorted(events, key=lambda x: x.t):
            self.observe(e, e.t)
        if events:
            self.tick(t=max(e.t for e in events))

    def to_dict(self) -> dict:
        return {
            "state": self.state.value,
            "incidents": [r.to_dict() for r in self.incident_log],
            "collision_count": self._collision_count,
            "terrain_collision_count": self._terrain_collision_count,
            "near_miss_count": self._near_miss_count,
            "clearance_violation_count": self._clearance_count,
        }
