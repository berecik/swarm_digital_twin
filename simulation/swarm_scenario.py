#!/usr/bin/env python3
"""
Swarm Scenario - Standalone multi-drone digital twin.

Demonstrates Phase C standalone swarm simulation with shared wind/terrain,
boids-style coupling, and collision-avoidance constraints.
"""

import os
import sys
import numpy as np
from dataclasses import dataclass
from typing import Dict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import make_holybro_x500, run_swarm_simulation
from drone_scenario import make_terrain
from wind_model import WindField


@dataclass(frozen=True)
class SwarmBenchmarkEnvelope:
    min_separation_min: float
    mean_tracking_error_max: float
    p75_tracking_error_max: float
    max_tracking_error_max: float


@dataclass(frozen=True)
class SwarmBenchmarkProfile:
    name: str
    seed: int
    wind_speed: float
    wind_direction: np.ndarray
    tolerance: float
    envelope: SwarmBenchmarkEnvelope


SWARM_BENCHMARK_PROFILES: Dict[str, SwarmBenchmarkProfile] = {
    "baseline": SwarmBenchmarkProfile(
        name="baseline",
        seed=20260325,
        wind_speed=1.05,
        wind_direction=np.array([1.0, 0.3, 0.0]),
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=2.0,
            mean_tracking_error_max=14.0,
            p75_tracking_error_max=16.5,
            max_tracking_error_max=23.0,
        ),
    ),
    "crosswind": SwarmBenchmarkProfile(
        name="crosswind",
        seed=20260326,
        wind_speed=2.5,
        wind_direction=np.array([0.0, 1.0, 0.0]),
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=1.9,
            mean_tracking_error_max=12.5,
            p75_tracking_error_max=14.5,
            max_tracking_error_max=18.0,
        ),
    ),
    "gusty": SwarmBenchmarkProfile(
        name="gusty",
        seed=20260327,
        wind_speed=4.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=1.8,
            mean_tracking_error_max=16.5,
            p75_tracking_error_max=22.0,
            max_tracking_error_max=26.0,
        ),
    ),
}


def get_swarm_benchmark_profile(profile_name: str) -> SwarmBenchmarkProfile:
    if profile_name not in SWARM_BENCHMARK_PROFILES:
        available = ", ".join(sorted(SWARM_BENCHMARK_PROFILES.keys()))
        raise ValueError(f"Unknown swarm benchmark profile '{profile_name}'. Available: {available}")
    return SWARM_BENCHMARK_PROFILES[profile_name]


def build_six_agent_ring_waypoints(radius: float = 10.0, altitude: float = 8.0):
    """Build a deterministic 6-agent ring pattern mission."""
    waypoints = {}
    for i in range(6):
        angle = i * np.pi / 3.0
        waypoints[f"drone_{i + 1}"] = [
            np.array([radius * np.cos(angle), radius * np.sin(angle), altitude]),
            np.array([
                radius * np.cos(angle + np.pi / 6.0),
                radius * np.sin(angle + np.pi / 6.0),
                altitude,
            ]),
            np.array([
                radius * np.cos(angle + np.pi / 3.0),
                radius * np.sin(angle + np.pi / 3.0),
                altitude,
            ]),
        ]
    return waypoints


def _compute_swarm_tracking_metrics(records, drone_waypoints):
    drone_ids = sorted(drone_waypoints.keys())
    final_targets = {
        drone_id: np.array(drone_waypoints[drone_id][-1], dtype=float)
        for drone_id in drone_ids
    }

    min_separation = float("inf")
    final_errors = []
    for rec in records:
        positions = rec.positions
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                d = np.linalg.norm(positions[i] - positions[j])
                min_separation = min(min_separation, d)

    final_positions = records[-1].positions
    for idx, drone_id in enumerate(drone_ids):
        final_errors.append(float(np.linalg.norm(final_positions[idx] - final_targets[drone_id])))

    return {
        "min_separation": float(min_separation),
        "mean_tracking_error": float(np.mean(final_errors)),
        "p75_tracking_error": float(np.percentile(final_errors, 75)),
        "max_tracking_error": float(np.max(final_errors)),
    }


def run_swarm_benchmark(profile_name: str):
    profile = get_swarm_benchmark_profile(profile_name)
    np.random.seed(profile.seed)

    params = make_holybro_x500()
    terrain = make_terrain()
    wind = WindField(
        wind_speed=profile.wind_speed,
        wind_direction=profile.wind_direction,
        turbulence_type="constant",
    )
    drone_waypoints = build_six_agent_ring_waypoints()
    records = run_swarm_simulation(
        drone_waypoints,
        params=params,
        dt=0.02,
        hover_time=0.5,
        max_time=20.0,
        wind=wind,
        terrain=terrain,
        min_separation=1.5,
    )

    metrics = _compute_swarm_tracking_metrics(records, drone_waypoints)
    envelope = profile.envelope
    failed = []
    if metrics["min_separation"] < envelope.min_separation_min:
        failed.append(
            f"min_separation={metrics['min_separation']:.4f} < {envelope.min_separation_min:.4f}"
        )
    if metrics["mean_tracking_error"] > envelope.mean_tracking_error_max:
        failed.append(
            f"mean_tracking_error={metrics['mean_tracking_error']:.4f} > {envelope.mean_tracking_error_max:.4f}"
        )
    if metrics["p75_tracking_error"] > envelope.p75_tracking_error_max:
        failed.append(
            f"p75_tracking_error={metrics['p75_tracking_error']:.4f} > {envelope.p75_tracking_error_max:.4f}"
        )
    if metrics["max_tracking_error"] > envelope.max_tracking_error_max:
        failed.append(
            f"max_tracking_error={metrics['max_tracking_error']:.4f} > {envelope.max_tracking_error_max:.4f}"
        )

    if failed:
        details = "; ".join(failed)
        raise AssertionError(f"Swarm benchmark '{profile.name}' failed: {details}")

    print(f"Swarm benchmark '{profile.name}' PASS (seed={profile.seed})")
    for key in ["min_separation", "mean_tracking_error", "p75_tracking_error", "max_tracking_error"]:
        print(f"  {key}: {metrics[key]:.4f}")

    return metrics


def run_demo():
    np.random.seed(42)
    params = make_holybro_x500()
    terrain = make_terrain()
    wind = WindField(
        wind_speed=1.05,
        wind_direction=np.array([1.0, 0.3, 0.0]),
        turbulence_type="constant",
    )

    drone_waypoints = build_six_agent_ring_waypoints()
    records = run_swarm_simulation(
        drone_waypoints,
        params=params,
        dt=0.02,
        hover_time=0.5,
        max_time=20.0,
        wind=wind,
        terrain=terrain,
        min_separation=1.5,
    )

    drone_ids = sorted(drone_waypoints.keys())
    waypoints = np.array([drone_waypoints[d] for d in drone_ids])
    np.savez(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "swarm_data.npz"),
        t=np.array([r.t for r in records]),
        positions=np.array([r.positions for r in records]),
        velocities=np.array([r.velocities for r in records]),
        drone_ids=np.array(drone_ids, dtype=object),
        waypoints=waypoints,
    )

    min_dist = float("inf")
    for rec in records:
        positions = rec.positions
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                d = np.linalg.norm(positions[i] - positions[j])
                min_dist = min(min_dist, d)

    print(f"Swarm records: {len(records)}")
    print(f"Minimum pairwise separation: {min_dist:.3f} m")
    print("Saved swarm_data.npz")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--benchmark":
        run_swarm_benchmark(sys.argv[2])
    else:
        run_demo()
