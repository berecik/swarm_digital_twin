#!/usr/bin/env python3
"""
SITL Log Merger — Convert ArduPilot .BIN flight logs to NPZ visualization format.

Single mode: one .BIN → scenario_data.npz (keys: t, pos, vel, euler, thrust, waypoints)
Swarm mode:  N .BIN → swarm_data.npz (keys: t, positions, velocities, drone_ids, waypoints)

Usage:
    python sitl_log_merger.py single path/to/flight.BIN
    python sitl_log_merger.py swarm logs/drone_0/*.BIN logs/drone_1/*.BIN ...
    python sitl_log_merger.py swarm --log-dirs logs/drone_0 logs/drone_1 logs/drone_2
"""

import os
import sys
import glob
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flight_log import FlightLog


def _ned_to_enu(positions_ned: np.ndarray) -> np.ndarray:
    """Convert NED positions (N,3) to ENU."""
    return np.column_stack([
        positions_ned[:, 1],   # East
        positions_ned[:, 0],   # North
        -positions_ned[:, 2],  # Up
    ])


def _compute_velocity(positions: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    """Compute velocity via central differences."""
    vel = np.zeros_like(positions)
    if len(timestamps) < 2:
        return vel
    dt = np.diff(timestamps)
    dt = np.clip(dt, 1e-6, None)
    dp = np.diff(positions, axis=0)
    vel[1:-1] = (dp[:-1] / dt[:-1, None] + dp[1:] / dt[1:, None]) / 2.0
    vel[0] = dp[0] / dt[0]
    vel[-1] = dp[-1] / dt[-1]
    return vel


def _extract_waypoints_enu(log: FlightLog) -> np.ndarray:
    """Extract waypoints from flight log, convert NED to ENU."""
    wps_ned = log.extract_waypoints()
    if len(wps_ned) == 0:
        # Fallback: start and end positions
        wps_ned = [log.positions[0], log.positions[-1]]
    wps = np.array(wps_ned)
    return _ned_to_enu(wps)


def _find_newest_bin(directory: str) -> str:
    """Find the newest .BIN file in a directory."""
    bins = glob.glob(os.path.join(directory, "*.BIN"))
    if not bins:
        raise FileNotFoundError(f"No .BIN files found in {directory}")
    return max(bins, key=os.path.getmtime)


def convert_single_log(bin_path: str, output_path: str = None) -> str:
    """Convert a single .BIN log to scenario_data.npz format.

    Returns the output file path.
    """
    if output_path is None:
        output_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "scenario_data.npz")

    log = FlightLog.from_bin(bin_path)
    t = log.timestamps - log.timestamps[0]
    pos_enu = _ned_to_enu(log.positions)
    vel_enu = _compute_velocity(pos_enu, t)
    waypoints = _extract_waypoints_enu(log)

    # Euler angles from ATT messages (already in radians)
    euler = log.attitudes if len(log.attitudes) == len(t) else np.zeros((len(t), 3))
    thrust = log.throttle if len(log.throttle) == len(t) else np.zeros(len(t))

    np.savez(
        output_path,
        t=t,
        pos=pos_enu,
        vel=vel_enu,
        euler=euler,
        thrust=thrust,
        waypoints=waypoints,
        source="sitl",
        origin_lat=log.origin_lat,
        origin_lon=log.origin_lon,
        origin_alt=log.origin_alt,
    )

    n_pts = len(t)
    duration = t[-1] if len(t) > 0 else 0
    print(f"  Single log: {n_pts} points, {duration:.1f}s → {output_path}")
    return output_path


def merge_swarm_logs(bin_paths: list, drone_ids: list = None,
                     waypoints_enu: dict = None,
                     output_path: str = None) -> str:
    """Merge N .BIN logs into swarm_data.npz format.

    Args:
        bin_paths: list of .BIN file paths (one per drone, ordered by drone index)
        drone_ids: list of drone ID strings (default: drone_0, drone_1, ...)
        waypoints_enu: dict mapping drone index to list of ENU waypoint arrays.
            If None, waypoints are extracted from each log via dwell detection.
        output_path: output file path (default: simulation/swarm_data.npz)

    Returns the output file path.
    """
    n_drones = len(bin_paths)
    if n_drones == 0:
        raise ValueError("No .BIN files provided")

    if output_path is None:
        output_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "swarm_data.npz")

    if drone_ids is None:
        drone_ids = [f"drone_{i}" for i in range(n_drones)]

    # Parse all logs
    logs = []
    for path in bin_paths:
        log = FlightLog.from_bin(path)
        logs.append(log)
        print(f"  Loaded {path}: {len(log.timestamps)} GPS points, "
              f"{log.timestamps[-1] - log.timestamps[0]:.1f}s")

    # Find common time window (overlap of all logs)
    t_starts = [log.timestamps[0] for log in logs]
    t_ends = [log.timestamps[-1] for log in logs]
    t_common_start = max(t_starts)
    t_common_end = min(t_ends)

    if t_common_end <= t_common_start:
        # No overlap — use the widest window and zero-pad missing data
        print("  WARNING: Logs don't overlap in time — using widest window")
        t_common_start = min(t_starts)
        t_common_end = max(t_ends)

    # Create common time axis at ~5 Hz (matching typical GPS rate)
    dt = 0.2
    t_common = np.arange(0, t_common_end - t_common_start, dt)
    n_steps = len(t_common)

    # Interpolate each log to common time grid
    positions = np.zeros((n_steps, n_drones, 3))
    velocities = np.zeros((n_steps, n_drones, 3))

    for i, log in enumerate(logs):
        t_rel = log.timestamps - t_common_start
        pos_enu = _ned_to_enu(log.positions)

        # Interpolate each axis
        for axis in range(3):
            positions[:, i, axis] = np.interp(
                t_common, t_rel, pos_enu[:, axis],
                left=pos_enu[0, axis], right=pos_enu[-1, axis])

        # Compute velocity from interpolated positions
        vel = _compute_velocity(positions[:, i, :], t_common)
        velocities[:, i, :] = vel

    # Waypoints per drone
    if waypoints_enu is not None:
        # Use provided ENU waypoints
        max_wps = max(len(wps) for wps in waypoints_enu.values())
        wp_array = np.zeros((n_drones, max_wps, 3))
        for i in range(n_drones):
            wps = waypoints_enu.get(i, [])
            for j, wp in enumerate(wps):
                wp_array[i, j, :] = wp
    else:
        # Extract from logs via dwell detection
        all_wps = []
        for log in logs:
            all_wps.append(_extract_waypoints_enu(log))
        max_wps = max(len(w) for w in all_wps)
        wp_array = np.zeros((n_drones, max_wps, 3))
        for i, wps in enumerate(all_wps):
            wp_array[i, :len(wps), :] = wps

    np.savez(
        output_path,
        t=t_common,
        positions=positions,
        velocities=velocities,
        drone_ids=np.array(drone_ids, dtype=object),
        waypoints=wp_array,
    )

    print(f"  Merged {n_drones} logs: {n_steps} steps, "
          f"{t_common[-1]:.1f}s → {output_path}")
    return output_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SITL log converter")
    sub = parser.add_subparsers(dest="mode", required=True)

    # Single
    sp = sub.add_parser("single", help="Convert single .BIN to scenario_data.npz")
    sp.add_argument("bin_file", help="Path to .BIN flight log")
    sp.add_argument("--output", default=None)

    # Swarm
    sp = sub.add_parser("swarm", help="Merge N .BIN logs to swarm_data.npz")
    sp.add_argument("--log-dirs", nargs="+",
                    help="Directories containing .BIN files (one per drone)")
    sp.add_argument("--bin-files", nargs="+",
                    help="Explicit .BIN file paths (one per drone)")
    sp.add_argument("--output", default=None)

    args = parser.parse_args()

    if args.mode == "single":
        convert_single_log(args.bin_file, args.output)
    elif args.mode == "swarm":
        if args.bin_files:
            bin_paths = args.bin_files
        elif args.log_dirs:
            bin_paths = [_find_newest_bin(d) for d in sorted(args.log_dirs)]
        else:
            parser.error("--log-dirs or --bin-files required for swarm mode")
        merge_swarm_logs(bin_paths, output_path=args.output)
