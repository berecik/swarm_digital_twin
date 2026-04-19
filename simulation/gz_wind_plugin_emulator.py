"""
Gazebo-equivalent wind-plugin math (Phase 5 close-out).

A pure-Python port of the algorithm `libgazebo_wind_plugin` runs against
its `<wind>` element. Lets CI assert that
`wind_model.WindField.get_wind_velocity(t, position)` matches what the
Gazebo wind plugin would compute at the same time/place — without
spinning up a Gazebo container.

The contract we mimic:

* `<windDirectionMean>` (3-vec) and `<windVelocityMean>` (scalar) define
  a constant wind vector ``v_mean = mean_speed * normalize(direction)``.
* `<windGustDirection>` and `<windGustForceMean>` add a sinusoidal gust
  modulated by `<windGustStart>` / `<windGustDuration>`. The plugin
  source uses ``gust_speed * sin(2*pi*t/period)`` while the gust window
  is active.
* The plugin does NOT model spatial gradients itself, but the SDF lets
  the world author multiply by an attenuation per height; the emulator
  reproduces this with an optional ``gradient`` callback so we can
  cross-check `WindField(spatial_gradient=...)`.

The Dryden turbulence path in `WindField` is its own animal — Gazebo
delegates that to a separate plugin. The emulator therefore only
covers the constant + gust + gradient path, which is what every Helm
profile in `helm/swarm-digital-twin/values-wind-*.yaml` exercises.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np


@dataclass
class GazeboWindParams:
    """Subset of the `<wind>` element a Helm profile cares about."""
    mean_direction: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    mean_speed_ms: float = 0.0
    gust_direction: np.ndarray = field(
        default_factory=lambda: np.zeros(3))
    gust_speed_ms: float = 0.0
    gust_start_s: float = 0.0
    gust_duration_s: float = 0.0
    gust_period_s: float = 5.0
    spatial_gradient: Optional[np.ndarray] = None  # (3, 3) ENU


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3)
    return v / n


def gz_wind_at(params: GazeboWindParams, t: float,
               position: np.ndarray) -> np.ndarray:
    """Return the wind vector the Gazebo plugin would publish at (t, pos)."""
    base = params.mean_speed_ms * _unit(np.asarray(params.mean_direction,
                                                   dtype=float))
    if (params.gust_duration_s > 0.0
            and params.gust_start_s <= t <= params.gust_start_s + params.gust_duration_s
            and params.gust_period_s > 0.0):
        phase = 2.0 * np.pi * (t - params.gust_start_s) / params.gust_period_s
        gust = (params.gust_speed_ms * np.sin(phase)
                * _unit(np.asarray(params.gust_direction, dtype=float)))
        base = base + gust
    if params.spatial_gradient is not None:
        base = base + np.asarray(params.spatial_gradient) @ np.asarray(position)
    return base


def from_wind_field(field):
    """Build a :class:`GazeboWindParams` matching a constant ``WindField``.

    Used by the parity test: take a ``WindField(turbulence_type="constant",
    ...)`` constructed from a manifest profile and reproduce its math
    via the Gazebo emulator. Dryden profiles are handled by a separate
    parity helper because Gazebo doesn't ship a built-in Dryden source.
    """
    return GazeboWindParams(
        mean_direction=np.asarray(field.wind_direction, dtype=float),
        mean_speed_ms=float(field.wind_speed),
        spatial_gradient=field.spatial_gradient,
    )


def parity_samples(field, n: int = 100, seed: int = 7,
                   t_max: float = 5.0,
                   pos_extent: float = 50.0) -> tuple:
    """Return ``(deltas, max_delta_ms, rmse_ms)`` between WindField and
    the Gazebo emulator across ``n`` deterministic (t, position) samples.

    Constant profiles only — Dryden has its own RNG so it isn't
    bit-comparable to a Gazebo plugin that uses a different generator.
    """
    if field.turbulence_type != "constant":
        raise ValueError(
            "Gazebo wind-plugin parity is defined for the constant + "
            "gradient profile only; Dryden uses its own RNG path"
        )
    params = from_wind_field(field)
    rng = np.random.default_rng(seed)
    ts = rng.uniform(0.0, t_max, n)
    positions = rng.uniform(-pos_extent, pos_extent, (n, 3))
    deltas = np.array([
        np.linalg.norm(
            field.get_wind_velocity(float(t), pos)
            - gz_wind_at(params, float(t), pos)
        )
        for t, pos in zip(ts, positions)
    ])
    return deltas, float(deltas.max()), float(np.sqrt(np.mean(deltas ** 2)))
