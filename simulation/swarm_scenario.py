#!/usr/bin/env python3
"""
Swarm Scenario - Standalone multi-drone digital twin.

Demonstrates Phase C standalone swarm simulation with shared wind/terrain,
boids-style coupling, and collision-avoidance constraints.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import make_holybro_x500, run_swarm_simulation
from drone_scenario import make_terrain
from wind_model import WindField


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
    run_demo()
