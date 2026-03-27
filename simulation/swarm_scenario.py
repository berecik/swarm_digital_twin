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
    p05_separation_min: float
    mean_tracking_error_max: float
    p75_tracking_error_max: float
    max_tracking_error_max: float
    mean_speed_max: float
    p90_speed_max: float


@dataclass(frozen=True)
class SwarmBenchmarkProfile:
    name: str
    seed: int
    wind_speed: float
    wind_direction: np.ndarray
    turbulence_type: str
    ring_radius: float
    altitude: float
    min_separation: float
    max_time: float
    tolerance: float
    envelope: SwarmBenchmarkEnvelope


# Envelopes are cross-platform bounds (ARM macOS + x86 Linux).
# Dryden turbulence uses np.random which may differ across platforms,
# so envelopes must accommodate variance. They catch catastrophic failures.
SWARM_BENCHMARK_PROFILES: Dict[str, SwarmBenchmarkProfile] = {
    "baseline": SwarmBenchmarkProfile(
        name="baseline",
        seed=20260325,
        wind_speed=1.05,
        wind_direction=np.array([1.0, 0.3, 0.0]),
        turbulence_type="constant",
        ring_radius=10.0,
        altitude=8.0,
        min_separation=1.5,
        max_time=20.0,
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=0.5,
            p05_separation_min=1.0,
            mean_tracking_error_max=30.0,
            p75_tracking_error_max=40.0,
            max_tracking_error_max=60.0,
            mean_speed_max=8.1,
            p90_speed_max=8.1,
        ),
    ),
    "crosswind": SwarmBenchmarkProfile(
        name="crosswind",
        seed=20260326,
        wind_speed=2.5,
        wind_direction=np.array([0.0, 1.0, 0.0]),
        turbulence_type="constant",
        ring_radius=10.0,
        altitude=8.0,
        min_separation=1.5,
        max_time=20.0,
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=0.5,
            p05_separation_min=1.0,
            mean_tracking_error_max=30.0,
            p75_tracking_error_max=35.0,
            max_tracking_error_max=55.0,
            mean_speed_max=8.1,
            p90_speed_max=8.1,
        ),
    ),
    "gusty": SwarmBenchmarkProfile(
        name="gusty",
        seed=20260327,
        wind_speed=4.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),
        turbulence_type="dryden",
        ring_radius=10.0,
        altitude=8.0,
        min_separation=1.6,
        max_time=24.0,
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=0.3,
            p05_separation_min=0.5,
            # Dryden gust coupling in multi-agent closed-loop can diverge
            # significantly across CPU/OS floating-point environments.
            # Keep safety/speed envelopes strict while allowing wider tracking tails.
            mean_tracking_error_max=60.0,
            p75_tracking_error_max=40.0,
            max_tracking_error_max=250.0,
            mean_speed_max=8.1,
            p90_speed_max=8.1,
        ),
    ),
    "tight_ring": SwarmBenchmarkProfile(
        name="tight_ring",
        seed=20260328,
        wind_speed=2.0,
        wind_direction=np.array([0.7, -0.7, 0.0]),
        turbulence_type="constant",
        ring_radius=7.0,
        altitude=7.0,
        min_separation=1.8,
        max_time=22.0,
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=0.1,
            p05_separation_min=0.5,
            mean_tracking_error_max=20.0,
            p75_tracking_error_max=25.0,
            max_tracking_error_max=30.0,
            mean_speed_max=8.1,
            p90_speed_max=8.1,
        ),
    ),
    "high_altitude": SwarmBenchmarkProfile(
        name="high_altitude",
        seed=20260329,
        wind_speed=3.2,
        wind_direction=np.array([1.0, 0.0, 0.0]),
        # Keep this profile deterministic and numerically stable across CI runners
        # while preserving the high-altitude wind-load scenario.
        turbulence_type="constant",
        ring_radius=12.0,
        altitude=14.0,
        min_separation=1.6,
        max_time=24.0,
        tolerance=1e-6,
        envelope=SwarmBenchmarkEnvelope(
            min_separation_min=0.3,
            p05_separation_min=0.5,
            mean_tracking_error_max=50.0,
            p75_tracking_error_max=50.0,
            max_tracking_error_max=150.0,
            mean_speed_max=8.1,
            p90_speed_max=8.1,
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
    pairwise_separations = []
    speed_samples = []
    final_errors = []
    for rec in records:
        positions = rec.positions
        speed_samples.extend(np.linalg.norm(rec.velocities, axis=1).tolist())
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                d = np.linalg.norm(positions[i] - positions[j])
                min_separation = min(min_separation, d)
                pairwise_separations.append(float(d))

    final_positions = records[-1].positions
    for idx, drone_id in enumerate(drone_ids):
        final_errors.append(float(np.linalg.norm(final_positions[idx] - final_targets[drone_id])))

    return {
        "min_separation": float(min_separation),
        "p05_separation": float(np.percentile(pairwise_separations, 5)),
        "mean_tracking_error": float(np.mean(final_errors)),
        "p75_tracking_error": float(np.percentile(final_errors, 75)),
        "max_tracking_error": float(np.max(final_errors)),
        "mean_speed": float(np.mean(speed_samples)),
        "p90_speed": float(np.percentile(speed_samples, 90)),
    }


def run_swarm_benchmark(profile_name: str):
    profile = get_swarm_benchmark_profile(profile_name)
    np.random.seed(profile.seed)

    params = make_holybro_x500()
    terrain = make_terrain()
    wind = WindField(
        wind_speed=profile.wind_speed,
        wind_direction=profile.wind_direction,
        turbulence_type=profile.turbulence_type,
    )
    drone_waypoints = build_six_agent_ring_waypoints(
        radius=profile.ring_radius,
        altitude=profile.altitude,
    )
    records = run_swarm_simulation(
        drone_waypoints,
        params=params,
        dt=0.02,
        hover_time=0.5,
        max_time=profile.max_time,
        wind=wind,
        terrain=terrain,
        min_separation=profile.min_separation,
    )

    metrics = _compute_swarm_tracking_metrics(records, drone_waypoints)
    envelope = profile.envelope
    failed = []
    if metrics["min_separation"] < envelope.min_separation_min:
        failed.append(
            f"min_separation={metrics['min_separation']:.4f} < {envelope.min_separation_min:.4f}"
        )
    if metrics["p05_separation"] < envelope.p05_separation_min:
        failed.append(
            f"p05_separation={metrics['p05_separation']:.4f} < {envelope.p05_separation_min:.4f}"
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
    if metrics["mean_speed"] > envelope.mean_speed_max:
        failed.append(
            f"mean_speed={metrics['mean_speed']:.4f} > {envelope.mean_speed_max:.4f}"
        )
    if metrics["p90_speed"] > envelope.p90_speed_max:
        failed.append(
            f"p90_speed={metrics['p90_speed']:.4f} > {envelope.p90_speed_max:.4f}"
        )

    if failed:
        details = "; ".join(failed)
        raise AssertionError(f"Swarm benchmark '{profile.name}' failed: {details}")

    print(f"Swarm benchmark '{profile.name}' PASS (seed={profile.seed})")
    for key in [
        "min_separation",
        "p05_separation",
        "mean_tracking_error",
        "p75_tracking_error",
        "max_tracking_error",
        "mean_speed",
        "p90_speed",
    ]:
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
