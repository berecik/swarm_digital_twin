#!/usr/bin/env python3
"""
Drone Flight Scenario - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Simple scenario: takeoff → fly to waypoints → return → land.
All physics simulated — no ROS 2 required.
"""

import os
import sys
import numpy as np

# Ensure imports work when run from any directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import DroneParams, run_simulation


def main():
    params = DroneParams(
        mass=1.5,
        drag_coeff=0.1,
        max_thrust=25.0,
    )

    # Waypoints: takeoff → cruise → waypoint → return → land
    waypoints = [
        np.array([0.0,  0.0, 10.0]),   # 1. Take off to 10 m
        np.array([20.0, 0.0, 10.0]),   # 2. Fly east 20 m
        np.array([20.0, 15.0, 12.0]),  # 3. Fly north and climb
        np.array([10.0, 10.0,  8.0]),  # 4. Cut diagonally, descend
        np.array([0.0,  0.0, 10.0]),   # 5. Return above origin
        np.array([0.0,  0.0,  0.5]),   # 6. Land (just above ground)
    ]

    print("Running drone flight scenario...")
    print(f"  Mass:       {params.mass} kg")
    print(f"  Max thrust: {params.max_thrust} N")
    print(f"  Drag coeff: {params.drag_coeff}")
    print(f"  Waypoints:  {len(waypoints)}")
    print()

    records = run_simulation(
        waypoints=waypoints,
        params=params,
        dt=0.005,
        waypoint_radius=0.5,
        hover_time=2.0,
        max_time=120.0,
    )

    print(f"Simulation complete: {len(records)} steps, {records[-1].t:.1f}s total")
    print()

    # Print summary per waypoint
    wp_idx = 0
    prev_dist = float('inf')
    for rec in records:
        if wp_idx >= len(waypoints):
            break
        dist = np.linalg.norm(rec.position - waypoints[wp_idx])
        if dist < 0.5 and prev_dist >= 0.5:
            print(f"  Waypoint {wp_idx+1} reached at t={rec.t:.2f}s  "
                  f"pos=[{rec.position[0]:.1f}, {rec.position[1]:.1f}, {rec.position[2]:.1f}]  "
                  f"speed={np.linalg.norm(rec.velocity):.2f} m/s")
            wp_idx += 1
        prev_dist = dist

    final = records[-1]
    print(f"\n  Final position: [{final.position[0]:.2f}, {final.position[1]:.2f}, {final.position[2]:.2f}]")
    print(f"  Final speed:    {np.linalg.norm(final.velocity):.3f} m/s")

    # Save data for visualization (next to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'scenario_data.npz')
    np.savez(
        output_path,
        t=np.array([r.t for r in records]),
        pos=np.array([r.position for r in records]),
        vel=np.array([r.velocity for r in records]),
        euler=np.array([r.euler for r in records]),
        thrust=np.array([r.thrust for r in records]),
        ang_vel=np.array([r.angular_velocity for r in records]),
        waypoints=np.array(waypoints),
    )
    print(f"\n  Data saved to {output_path}")


if __name__ == '__main__':
    main()
