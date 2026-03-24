#!/usr/bin/env python3
"""
Drone Flight Scenario - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Full-featured scenario: takeoff -> fly waypoints over terrain with wind -> land.
Demonstrates: quadratic drag, ISA atmosphere, wind perturbation, terrain collision.
All physics simulated -- no ROS 2 required.
"""

import os
import sys
import numpy as np

# Ensure imports work when run from any directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import (
    DroneParams, AeroCoefficients, Atmosphere, run_simulation,
)
from wind_model import WindField
from terrain import TerrainMap


MIN_AGL = 5.0   # minimum clearance above ground [m]


def make_terrain() -> TerrainMap:
    """Create terrain with prominent hills and a valley."""
    def hills(x, y):
        z = (
            8.0 * np.exp(-((x - 15)**2 + (y - 5)**2) / 80)    # hill east
            + 6.0 * np.exp(-((x - 30)**2 + (y - 18)**2) / 60)  # hill NE
            + 10.0 * np.exp(-((x - 10)**2 + (y - 20)**2) / 50) # tall ridge
            + 4.0 * np.sin(x * 0.06) * np.cos(y * 0.08)        # rolling base
            + 2.0
        )
        return np.maximum(z, 0.0)

    return TerrainMap.from_function(
        hills,
        x_range=(-15, 55),
        y_range=(-15, 40),
        resolution=0.5,
    )


def waypoints_above_terrain(xy_z_agl, terrain: TerrainMap):
    """Convert (x, y, agl) tuples to absolute waypoints above terrain.

    Each waypoint's z is set to terrain_elevation(x,y) + agl.
    Also checks that straight-line paths between consecutive waypoints
    clear the terrain by at least MIN_AGL.
    """
    wps = []
    for x, y, agl in xy_z_agl:
        gz = terrain.get_elevation(x, y)
        wps.append(np.array([x, y, gz + agl]))

    # Verify clearance along each leg
    for i in range(len(wps) - 1):
        p0, p1 = wps[i], wps[i + 1]
        for s in np.linspace(0, 1, 200):
            p = p0 + s * (p1 - p0)
            gz = terrain.get_elevation(p[0], p[1])
            clearance = p[2] - gz
            if clearance < MIN_AGL * 0.5:  # warn if less than half min AGL
                mid_gz = gz
                # Raise both endpoints so the midpoint clears
                needed = mid_gz + MIN_AGL - p[2]
                if needed > 0:
                    wps[i][2] += needed / 2
                    wps[i + 1][2] += needed / 2

    return wps


def main():
    # Full-featured drone parameters
    params = DroneParams(
        mass=1.5,
        arm_length=0.25,
        drag_coeff=0.1,       # legacy fallback (not used when aero is set)
        ang_drag_coeff=0.02,
        max_thrust=25.0,
        max_torque=5.0,
        aero=AeroCoefficients(
            reference_area=0.04,
            C_D=1.0,
            C_L=0.0,
        ),
        atmo=Atmosphere(
            rho_sea_level=1.225,
            altitude_msl=0.0,   # sea level
        ),
    )

    # Wind: gentle constant breeze
    wind = WindField(
        wind_speed=3.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),  # NE wind
        turbulence_type="constant",
    )

    # Terrain with prominent hills
    terrain = make_terrain()

    # Waypoints defined as (x, y, AGL) — z is computed relative to terrain
    wp_agl = [
        ( 0.0,  0.0, 8.0),    # 1. Take off
        (20.0,  0.0, 8.0),    # 2. Fly east over hill
        (20.0, 15.0, 8.0),    # 3. North, over ridge
        (35.0, 20.0, 8.0),    # 4. Continue NE
        (10.0, 10.0, 8.0),    # 5. Cut back over valley
        ( 0.0,  0.0, 8.0),    # 6. Return above origin
        ( 0.0,  0.0, 1.0),    # 7. Land
    ]
    waypoints = waypoints_above_terrain(wp_agl, terrain)

    print("Running drone flight scenario (full physics)...")
    print(f"  Mass:       {params.mass} kg")
    print(f"  Aero:       C_D={params.aero.C_D}, A={params.aero.reference_area} m^2")
    print(f"  Atmosphere: rho={params.atmo.rho:.3f} kg/m^3 (MSL={params.atmo.altitude_msl}m)")
    print(f"  Wind:       {wind.wind_speed} m/s, type={wind.turbulence_type}")
    print(f"  Terrain:    {terrain.elevations.shape[1]}x{terrain.elevations.shape[0]} grid, "
          f"res={terrain.resolution}m, "
          f"elev=[{terrain.elevations.min():.1f}, {terrain.elevations.max():.1f}]m")
    print(f"  Min AGL:    {MIN_AGL} m")
    print(f"  Waypoints:  {len(waypoints)}")
    for i, (wp, (_, _, agl)) in enumerate(zip(waypoints, wp_agl)):
        gz = terrain.get_elevation(wp[0], wp[1])
        print(f"    WP{i+1}: pos=[{wp[0]:5.1f}, {wp[1]:5.1f}, {wp[2]:5.1f}]  "
              f"ground={gz:.1f}m  AGL={agl:.0f}m")
    print()

    records = run_simulation(
        waypoints=waypoints,
        params=params,
        dt=0.005,
        waypoint_radius=0.5,
        hover_time=2.0,
        max_time=180.0,
        wind=wind,
        terrain=terrain,
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
            ground = terrain.get_elevation(rec.position[0], rec.position[1])
            print(f"  WP{wp_idx+1} reached at t={rec.t:.2f}s  "
                  f"pos=[{rec.position[0]:.1f}, {rec.position[1]:.1f}, {rec.position[2]:.1f}]  "
                  f"AGL={rec.position[2] - ground:.1f}m  "
                  f"speed={np.linalg.norm(rec.velocity):.2f} m/s")
            wp_idx += 1
        prev_dist = dist

    final = records[-1]
    print(f"\n  Final position: [{final.position[0]:.2f}, {final.position[1]:.2f}, {final.position[2]:.2f}]")
    print(f"  Final speed:    {np.linalg.norm(final.velocity):.3f} m/s")

    # Save data for visualization (next to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'scenario_data.npz')

    # Terrain grid for visualization
    bounds = terrain.bounds
    tx = np.arange(bounds[0], bounds[2] + terrain.resolution, terrain.resolution)
    ty = np.arange(bounds[1], bounds[3] + terrain.resolution, terrain.resolution)
    # Ensure shapes match the stored elevations
    tx = tx[:terrain.elevations.shape[1]]
    ty = ty[:terrain.elevations.shape[0]]

    np.savez(
        output_path,
        t=np.array([r.t for r in records]),
        pos=np.array([r.position for r in records]),
        vel=np.array([r.velocity for r in records]),
        euler=np.array([r.euler for r in records]),
        thrust=np.array([r.thrust for r in records]),
        ang_vel=np.array([r.angular_velocity for r in records]),
        waypoints=np.array(waypoints),
        # Phase 2 additions
        terrain_x=tx,
        terrain_y=ty,
        terrain_z=terrain.elevations,
        wind_speed=wind.wind_speed,
        wind_direction=wind.wind_direction / np.linalg.norm(wind.wind_direction),
        atmo_rho=params.atmo.rho,
        atmo_altitude_msl=params.atmo.altitude_msl,
        aero_cd=params.aero.C_D,
        aero_area=params.aero.reference_area,
    )
    print(f"\n  Data saved to {output_path}")


if __name__ == '__main__':
    main()
