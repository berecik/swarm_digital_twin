#!/usr/bin/env python3
"""
SITL Waypoint Generator — GPS ring-formation missions for ArduPilot SITL.

Converts ENU ring waypoints (matching swarm_scenario.py patterns) to
QGC WPL 110 format with GPS coordinates for SITL upload.

Usage:
    python sitl_waypoints.py --n 6 --output-dir missions/swarm/
    python sitl_waypoints.py --n 3 --radius 15 --altitude 30
"""

import math
import os
import sys
import argparse
import numpy as np


# Default GPS origin — Antisana region (used throughout the project)
DEFAULT_LAT = -0.508333
DEFAULT_LON = -78.141667
DEFAULT_ALT = 4500.0


def enu_to_gps(east: float, north: float, up: float,
               ref_lat: float, ref_lon: float, ref_alt: float):
    """Convert ENU offset [m] to GPS (lat, lon, alt_msl) in degrees.

    Uses the same formula as mavlink_bridge.py:356.
    """
    lat_m_per_deg = 111320.0
    lon_m_per_deg = 111320.0 * math.cos(math.radians(ref_lat))
    lat = ref_lat + north / lat_m_per_deg
    lon = ref_lon + east / lon_m_per_deg
    alt = ref_alt + up
    return lat, lon, alt


def build_ring_waypoints_enu(n_drones: int, radius: float = 10.0,
                             altitude: float = 20.0):
    """Generate ENU ring-formation waypoints for N drones.

    Matches the pattern from swarm_scenario.py build_six_agent_ring_waypoints:
    each drone starts at its ring position, then rotates 30° twice.

    Returns:
        dict mapping drone index (0-based) to list of ENU np.arrays.
    """
    waypoints = {}
    for i in range(n_drones):
        angle = i * 2.0 * math.pi / n_drones
        waypoints[i] = [
            np.array([radius * math.cos(angle),
                       radius * math.sin(angle),
                       altitude]),
            np.array([radius * math.cos(angle + math.pi / 6.0),
                       radius * math.sin(angle + math.pi / 6.0),
                       altitude]),
            np.array([radius * math.cos(angle + math.pi / 3.0),
                       radius * math.sin(angle + math.pi / 3.0),
                       altitude]),
        ]
    return waypoints


def waypoints_to_qgc_wpl(enu_waypoints, ref_lat: float, ref_lon: float,
                          ref_alt: float, takeoff_alt_agl: float = None):
    """Convert a drone's ENU waypoint list to QGC WPL 110 format string.

    Args:
        enu_waypoints: list of np.array([east, north, up]) in meters.
        ref_lat, ref_lon, ref_alt: GPS reference origin.
        takeoff_alt_agl: Takeoff altitude AGL. Defaults to first waypoint's Z.

    Returns:
        QGC WPL 110 format string.
    """
    if takeoff_alt_agl is None:
        takeoff_alt_agl = float(enu_waypoints[0][2])

    # Home position — drone's first waypoint converted to GPS
    home_lat, home_lon, home_alt = enu_to_gps(
        float(enu_waypoints[0][0]),
        float(enu_waypoints[0][1]),
        0.0,  # home is on ground
        ref_lat, ref_lon, ref_alt)

    lines = ["QGC WPL 110"]

    # Seq 0: Home (frame 0 = GLOBAL, cmd 16 = WAYPOINT)
    lines.append(
        f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.6f}\t{home_lon:.6f}\t"
        f"{ref_alt:.0f}\t1")

    # Seq 1: Takeoff (frame 3 = GLOBAL_RELATIVE_ALT, cmd 22 = TAKEOFF)
    lines.append(
        f"1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t{takeoff_alt_agl:.0f}\t1")

    # Waypoints (frame 3, cmd 16 = WAYPOINT)
    for seq, wp in enumerate(enu_waypoints, start=2):
        wp_lat, wp_lon, wp_alt = enu_to_gps(
            float(wp[0]), float(wp[1]), 0.0,
            ref_lat, ref_lon, ref_alt)
        agl = float(wp[2])
        lines.append(
            f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{wp_lat:.6f}\t{wp_lon:.6f}\t"
            f"{agl:.0f}\t1")

    # Land (frame 3, cmd 21 = LAND)
    land_seq = len(enu_waypoints) + 2
    lines.append(
        f"{land_seq}\t0\t3\t21\t0\t0\t0\t0\t{home_lat:.6f}\t{home_lon:.6f}\t"
        f"0\t1")

    return "\n".join(lines) + "\n"


def build_ring_missions(n_drones: int, ref_lat: float = DEFAULT_LAT,
                        ref_lon: float = DEFAULT_LON,
                        ref_alt: float = DEFAULT_ALT,
                        radius: float = 10.0, altitude: float = 20.0):
    """Generate QGC WPL 110 missions for N drones in ring formation.

    Returns:
        Tuple of:
            missions: dict[int, str] — drone index to WPL string
            waypoints_enu: dict[int, list[np.array]] — drone index to ENU waypoints
            home_gps: dict[int, tuple] — drone index to (lat, lon, alt) home
    """
    enu_wps = build_ring_waypoints_enu(n_drones, radius, altitude)
    missions = {}
    home_gps = {}

    for drone_id, wps in enu_wps.items():
        missions[drone_id] = waypoints_to_qgc_wpl(
            wps, ref_lat, ref_lon, ref_alt, takeoff_alt_agl=altitude)
        # Home GPS is the drone's initial ring position (on ground)
        lat, lon, alt = enu_to_gps(
            float(wps[0][0]), float(wps[0][1]), 0.0,
            ref_lat, ref_lon, ref_alt)
        home_gps[drone_id] = (lat, lon, ref_alt)

    return missions, enu_wps, home_gps


def write_mission_files(missions, output_dir: str):
    """Write mission WPL strings to files in output_dir."""
    os.makedirs(output_dir, exist_ok=True)
    paths = {}
    for drone_id, wpl in missions.items():
        path = os.path.join(output_dir, f"drone_{drone_id}.waypoints")
        with open(path, "w") as f:
            f.write(wpl)
        paths[drone_id] = path
    return paths


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate SITL ring-formation missions")
    parser.add_argument("--n", type=int, default=6, help="Number of drones")
    parser.add_argument("--radius", type=float, default=10.0, help="Ring radius [m]")
    parser.add_argument("--altitude", type=float, default=20.0, help="Flight altitude AGL [m]")
    parser.add_argument("--ref-lat", type=float, default=DEFAULT_LAT)
    parser.add_argument("--ref-lon", type=float, default=DEFAULT_LON)
    parser.add_argument("--ref-alt", type=float, default=DEFAULT_ALT)
    parser.add_argument("--output-dir", default="missions/swarm")
    args = parser.parse_args()

    missions, enu_wps, home_gps = build_ring_missions(
        args.n, args.ref_lat, args.ref_lon, args.ref_alt,
        args.radius, args.altitude)

    paths = write_mission_files(missions, args.output_dir)

    for drone_id in sorted(missions.keys()):
        lat, lon, alt = home_gps[drone_id]
        n_wps = len(enu_wps[drone_id])
        print(f"  drone_{drone_id}: {n_wps} waypoints, "
              f"home=({lat:.6f}, {lon:.6f}), file={paths[drone_id]}")
    print(f"Generated {len(missions)} mission files in {args.output_dir}/")
