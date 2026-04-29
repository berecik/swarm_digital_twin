"""
Waypoint-achievement optimisation for a single drone.

The detection pipeline (`coco_annotator`, `model_zoo`, …) optimises the
*perception* model. This module is the symmetric scaffolding for the
*control* model: given the existing rigid-body `drone_physics` engine
and the four mission kinds in `missions`, find PID-gain vectors that
maximise waypoint-achievement quality.

It is deliberately the same shape as `model_zoo` + `model_registry`:

* `PolicyGains`  — the trained "weights" (18 floats; six PIDs × kp/ki/kd).
* `EpisodeMetrics` — the equivalent of inference output: RMSE,
  completion ratio, time-to-first-waypoint, energy proxy, max overshoot.
* `run_episode()` — single deterministic forward pass through one
  mission with a given gain vector.
* `evaluate_policy()` — multi-episode aggregator with per-seed wind
  perturbation.
* `random_search()` — bounded random search over gain space; the
  CI-deliverable trainer that ships today. Real RL (PPO/SAC) plugs in
  later via the same `PolicyGains` interface — see
  `docs/nightly_lane.md`.

Optimisation goal: hit every waypoint, fast, with low overshoot and
modest control effort. Aggregated through `waypoint_kpi.evaluate_waypoint_kpis`.
"""

from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from drone_physics import (
    DroneParams, DroneState, PositionController, physics_step,
)
from missions import build_mission, mission_kinds


# ── Gain vector ──────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class PolicyGains:
    """PID gains for the cascaded position + attitude controller.

    Field naming mirrors `PositionController`: ``pid_<axis> = (kp, ki, kd)``.
    The factory `from_baseline()` returns the production defaults so a
    "trained" policy starts from a known-good point.
    """

    pos_x_kp: float; pos_x_ki: float; pos_x_kd: float
    pos_y_kp: float; pos_y_ki: float; pos_y_kd: float
    pos_z_kp: float; pos_z_ki: float; pos_z_kd: float
    att_roll_kp: float;  att_roll_ki: float;  att_roll_kd: float
    att_pitch_kp: float; att_pitch_ki: float; att_pitch_kd: float
    att_yaw_kp: float;   att_yaw_ki: float;   att_yaw_kd: float

    @staticmethod
    def from_baseline() -> "PolicyGains":
        """Production defaults from `PositionController.__init__`."""
        return PolicyGains(
            pos_x_kp=4.0, pos_x_ki=0.5, pos_x_kd=3.0,
            pos_y_kp=4.0, pos_y_ki=0.5, pos_y_kd=3.0,
            pos_z_kp=6.0, pos_z_ki=1.0, pos_z_kd=4.0,
            att_roll_kp=8.0,  att_roll_ki=0.1,  att_roll_kd=2.0,
            att_pitch_kp=8.0, att_pitch_ki=0.1, att_pitch_kd=2.0,
            att_yaw_kp=4.0,   att_yaw_ki=0.05,  att_yaw_kd=1.0,
        )

    def to_dict(self) -> Dict[str, float]:
        return {k: float(v) for k, v in asdict(self).items()}

    @classmethod
    def from_dict(cls, d: Dict[str, float]) -> "PolicyGains":
        return cls(**{k: float(v) for k, v in d.items()})

    # Maps gain-vector field prefixes to the matching PositionController
    # PID attribute names. Drives `apply_to` so the per-PID assignment
    # block doesn't need 18 lines of boilerplate.
    _PID_MAP = {
        "pos_x":     "pid_x",
        "pos_y":     "pid_y",
        "pos_z":     "pid_z",
        "att_roll":  "pid_roll",
        "att_pitch": "pid_pitch",
        "att_yaw":   "pid_yaw",
    }

    def apply_to(self, controller: PositionController) -> None:
        """Mutate an existing controller's PIDs in place."""
        for prefix, pid_attr in self._PID_MAP.items():
            pid = getattr(controller, pid_attr)
            pid.kp = getattr(self, f"{prefix}_kp")
            pid.ki = getattr(self, f"{prefix}_ki")
            pid.kd = getattr(self, f"{prefix}_kd")
        controller.reset()


# ── Episode runner ───────────────────────────────────────────────────────────

@dataclass
class EpisodeMetrics:
    """One-episode summary used both for promotion gates and tuning.

    Tracking error is measured only during the **settled** phase — i.e.
    when the drone is inside the waypoint capture radius. This isolates
    "how tightly does the controller hold a waypoint" from the
    necessarily-large transit error between distant waypoints.
    """

    waypoints_reached: int
    waypoint_count: int
    rmse_xyz_m: float           # RMSE during settled (in-capture-radius) windows
    time_to_first_wp_s: float   # +inf if never reached
    total_time_s: float
    energy_proxy_j: float       # ∫ thrust dt — surrogate for battery use
    max_overshoot_m: float      # worst overshoot past a waypoint plane during capture
    finite: bool = True         # False = simulation went non-finite (gain vector unstable)

    @property
    def completion_ratio(self) -> float:
        return (self.waypoints_reached / self.waypoint_count
                if self.waypoint_count > 0 else 0.0)

    def as_kpi_dict(self) -> Dict[str, float]:
        return {
            "waypoints_reached":    float(self.waypoints_reached),
            "waypoint_count":       float(self.waypoint_count),
            "completion_ratio":     float(self.completion_ratio),
            "rmse_xyz_m":           float(self.rmse_xyz_m),
            "time_to_first_wp_s":   float(self.time_to_first_wp_s),
            "total_time_s":         float(self.total_time_s),
            "energy_proxy_j":       float(self.energy_proxy_j),
            "max_overshoot_m":      float(self.max_overshoot_m),
        }


def _failed_metrics(waypoint_count: int, total_time_s: float) -> EpisodeMetrics:
    """Sentinel returned when the simulation diverges (NaN/Inf)."""
    return EpisodeMetrics(
        waypoints_reached=0,
        waypoint_count=waypoint_count,
        rmse_xyz_m=float("inf"),
        time_to_first_wp_s=float("inf"),
        total_time_s=total_time_s,
        energy_proxy_j=float("inf"),
        max_overshoot_m=float("inf"),
        finite=False,
    )


def run_episode(gains: PolicyGains,
                mission_kind: str = "patrol",
                params: Optional[DroneParams] = None,
                dt: float = 0.01,
                waypoint_radius: float = 0.5,
                hover_time: float = 1.0,
                max_time: float = 60.0,
                wind=None,
                terrain=None) -> EpisodeMetrics:
    """Run one single-drone episode with the given gains.

    The mission is built via `missions.build_mission(kind, n=1)` so the
    drone sees the same ground-truth waypoint list it would in a live
    scenario. Returns aggregate metrics for promotion / tuning.
    """
    if mission_kind not in mission_kinds():
        raise ValueError(
            f"unknown mission '{mission_kind}'; available: {mission_kinds()}"
        )
    if params is None:
        params = DroneParams()

    waypoints = build_mission(mission_kind, n=1, terrain=terrain)[1]
    state = DroneState()
    controller = PositionController(params)
    gains.apply_to(controller)

    wp_idx = 0
    hover_timer = 0.0
    t = 0.0
    settled_sq_err = 0.0
    settled_samples = 0
    energy = 0.0
    time_to_first_wp = float("inf")
    max_overshoot = 0.0
    capture_radius = max(waypoint_radius * 3.0, 1.0)  # range used for overshoot tracking
    in_capture_zone = False

    while t < max_time and wp_idx < len(waypoints):
        target = waypoints[wp_idx]
        cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)

        if not (np.isfinite(cmd.thrust) and np.all(np.isfinite(cmd.torque))):
            return _failed_metrics(len(waypoints), t)

        state = physics_step(state, cmd, params, dt, wind=wind, t=t,
                             terrain=terrain)
        if not np.all(np.isfinite(state.position)):
            return _failed_metrics(len(waypoints), t)

        err_vec = target - state.position
        dist = float(np.linalg.norm(err_vec))
        energy += cmd.thrust * dt

        if dist <= capture_radius:
            in_capture_zone = True
        if dist <= waypoint_radius:
            settled_sq_err += float(np.dot(err_vec, err_vec))
            settled_samples += 1
            hover_timer += dt
            if hover_timer >= hover_time:
                if wp_idx == 0:
                    time_to_first_wp = t
                wp_idx += 1
                hover_timer = 0.0
                in_capture_zone = False
                controller.reset()
        else:
            hover_timer = 0.0
            # Overshoot: when inside the capture zone but outside the
            # accept radius, track the worst excursion. This catches
            # over/undershoot during the approach without counting the
            # huge transit distance between distant waypoints.
            if in_capture_zone:
                max_overshoot = max(max_overshoot, dist - waypoint_radius)

        t += dt

    # If the loop hit max_time without finishing, the LAST visited
    # waypoint counts as "reached" only if we were inside the radius
    # at exit; otherwise the count stays at wp_idx.
    waypoints_reached = wp_idx
    rmse = (math.sqrt(settled_sq_err / settled_samples)
            if settled_samples > 0 else float("inf"))
    if not (math.isfinite(rmse) and math.isfinite(energy)):
        return _failed_metrics(len(waypoints), t)

    return EpisodeMetrics(
        waypoints_reached=waypoints_reached,
        waypoint_count=len(waypoints),
        rmse_xyz_m=rmse,
        time_to_first_wp_s=time_to_first_wp,
        total_time_s=t,
        energy_proxy_j=energy,
        max_overshoot_m=max_overshoot,
        finite=True,
    )


# ── Multi-episode evaluation ─────────────────────────────────────────────────


def evaluate_policy(gains: PolicyGains, *,
                    mission_kinds_to_run: Optional[List[str]] = None,
                    n_episodes_per_mission: int = 1,
                    seed: int = 0,
                    dt: float = 0.01,
                    max_time: float = 60.0) -> Dict[str, float]:
    """Average episode metrics across one or more missions.

    Returned dict matches the schema `waypoint_kpi.evaluate_waypoint_kpis`
    expects: completion_ratio, rmse_xyz_m, time_to_first_wp_s,
    energy_proxy_j, max_overshoot_m.

    *seed* is reserved for future Dryden-wind scenarios; today the
    default-no-wind episode is fully deterministic so seeding is a
    no-op. We still consume it so the public API is stable.
    """
    del seed  # reserved (no stochastic wind in this CI build)
    kinds = mission_kinds_to_run or ["patrol"]
    completions: List[float] = []
    rmses: List[float] = []
    first_wp_times: List[float] = []
    energies: List[float] = []
    overshoots: List[float] = []
    finite_ratio = 0
    n_total = 0
    for kind in kinds:
        for _ in range(max(1, n_episodes_per_mission)):
            m = run_episode(gains, mission_kind=kind, dt=dt, max_time=max_time)
            n_total += 1
            if m.finite:
                finite_ratio += 1
            completions.append(m.completion_ratio)
            rmses.append(m.rmse_xyz_m)
            first_wp_times.append(m.time_to_first_wp_s)
            energies.append(m.energy_proxy_j)
            overshoots.append(m.max_overshoot_m)

    def _mean_finite(xs: List[float]) -> float:
        finite = [x for x in xs if math.isfinite(x)]
        return float(np.mean(finite)) if finite else float("inf")

    return {
        "completion_ratio":     float(np.mean(completions)) if completions else 0.0,
        "rmse_xyz_m":           _mean_finite(rmses),
        "time_to_first_wp_s":   _mean_finite(first_wp_times),
        "energy_proxy_j":       _mean_finite(energies),
        "max_overshoot_m":      _mean_finite(overshoots),
        "finite_ratio":         float(finite_ratio / n_total) if n_total else 0.0,
    }


# ── Random search trainer ────────────────────────────────────────────────────


@dataclass(frozen=True)
class SearchBounds:
    """Multiplicative bounds around the baseline gain vector.

    `(lo, hi)` means a trial samples each gain uniformly in
    `[baseline*lo, baseline*hi]`. Conservative defaults (0.5x..1.5x)
    keep the search inside the regime where the controller doesn't
    diverge — bigger swings are usually catastrophic for the cascaded
    PD-style attitude loop.
    """
    pos_lo: float = 0.5
    pos_hi: float = 1.5
    att_lo: float = 0.5
    att_hi: float = 1.5


def _sample_gains(rng: random.Random,
                  baseline: PolicyGains,
                  bounds: SearchBounds) -> PolicyGains:
    """Sample each gain uniformly within multiplicative bounds.

    Position gains use ``(pos_lo, pos_hi)`` and attitude gains use
    ``(att_lo, att_hi)`` — picked by field-name prefix.
    """
    def _scaled(name: str, value: float) -> float:
        lo, hi = ((bounds.pos_lo, bounds.pos_hi)
                  if name.startswith("pos_") else
                  (bounds.att_lo, bounds.att_hi))
        return value * rng.uniform(lo, hi)

    return PolicyGains(**{
        name: _scaled(name, value)
        for name, value in baseline.to_dict().items()
    })


def _objective(metrics: Dict[str, float]) -> float:
    """Higher = better. Completion dominates; rmse + energy are tie-breakers.

    Encoded so a divergent trial (rmse=+inf, completion=0) returns
    -inf and is always rejected.
    """
    completion = metrics.get("completion_ratio", 0.0)
    rmse = metrics.get("rmse_xyz_m", float("inf"))
    energy = metrics.get("energy_proxy_j", float("inf"))
    if not (math.isfinite(rmse) and math.isfinite(energy)):
        return float("-inf")
    return 100.0 * completion - rmse - 0.001 * energy


@dataclass
class SearchResult:
    best_gains: PolicyGains
    best_metrics: Dict[str, float]
    best_objective: float
    trials: List[Tuple[PolicyGains, Dict[str, float], float]] = field(default_factory=list)


def random_search(*, n_trials: int = 16,
                  seed: int = 0,
                  baseline: Optional[PolicyGains] = None,
                  bounds: Optional[SearchBounds] = None,
                  mission_kinds_to_run: Optional[List[str]] = None,
                  dt: float = 0.01,
                  max_time: float = 60.0) -> SearchResult:
    """Bounded random search over PID gains.

    Always evaluates the baseline first — the returned policy is
    guaranteed to be at least as good as the baseline. Cheap enough to
    run end-to-end in CI (`n_trials=4` finishes well under a minute on
    a laptop with the default `patrol` mission).
    """
    base = baseline or PolicyGains.from_baseline()
    bnds = bounds or SearchBounds()
    rng = random.Random(seed)
    kinds = mission_kinds_to_run or ["patrol"]

    trials: List[Tuple[PolicyGains, Dict[str, float], float]] = []

    base_metrics = evaluate_policy(base, mission_kinds_to_run=kinds,
                                   dt=dt, max_time=max_time)
    base_obj = _objective(base_metrics)
    trials.append((base, base_metrics, base_obj))

    best_gains = base
    best_metrics = base_metrics
    best_obj = base_obj

    for _ in range(n_trials):
        cand = _sample_gains(rng, base, bnds)
        m = evaluate_policy(cand, mission_kinds_to_run=kinds,
                            dt=dt, max_time=max_time)
        obj = _objective(m)
        trials.append((cand, m, obj))
        if obj > best_obj:
            best_obj = obj
            best_metrics = m
            best_gains = cand

    return SearchResult(
        best_gains=best_gains,
        best_metrics=best_metrics,
        best_objective=best_obj,
        trials=trials,
    )
