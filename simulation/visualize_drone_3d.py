#!/usr/bin/env python3
"""
3D Drone Flight Visualization - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Reads scenario_data.npz (produced by drone_scenario.py), swarm_data.npz,
or ArduPilot DataFlash .BIN logs (from SITL missions) and renders:
  - 3D terrain surface with color-mapped elevation
  - Animated drone with body-frame axes (attitude indicator)
  - Velocity vector and ground shadow projected onto terrain
  - Wind direction indicator
  - Altitude (MSL + AGL), speed, and thrust timeline panels
"""

import os
import sys
from typing import Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from pathlib import Path

# Ensure imports work when run from any directory
SCRIPT_DIR = Path(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, str(SCRIPT_DIR))

# Allow running from repo root or from simulation/
DATA_CANDIDATES = [
    SCRIPT_DIR / 'swarm_data.npz',
    SCRIPT_DIR / 'scenario_data.npz',
    Path('scenario_data.npz'),
    Path('simulation/swarm_data.npz'),
    Path('simulation/scenario_data.npz'),
]

# SITL log search paths (newest first)
SITL_LOG_GLOBS = [
    SCRIPT_DIR.parent / 'logs' / 'sitl_*' / 'flight_logs' / '*.BIN',
    Path('logs') / 'sitl_*' / 'flight_logs' / '*.BIN',
]


def _load_bin(path: str) -> dict:
    """Load an ArduPilot DataFlash .BIN log and convert to visualization format.

    Converts NED (ArduPilot) -> ENU (visualization) coordinate frame.
    Computes velocity from position differences.
    Extracts waypoints from dwell-point detection.
    """
    from flight_log import FlightLog

    log = FlightLog.from_bin(path)
    if len(log.timestamps) < 2:
        raise ValueError(f"Flight log {path} contains insufficient data")

    # Relative timestamps
    t = log.timestamps - log.timestamps[0]

    # NED -> ENU: X_enu=East(ned_e), Y_enu=North(ned_n), Z_enu=Up(-ned_d)
    pos_enu = np.column_stack([
        log.positions[:, 1],   # East
        log.positions[:, 0],   # North
        -log.positions[:, 2],  # Up
    ])

    # Velocity via central differences (ENU)
    dt = np.diff(t)
    dt = np.maximum(dt, 1e-6)
    vel_diff = np.diff(pos_enu, axis=0) / dt[:, np.newaxis]
    vel_enu = np.vstack([vel_diff, vel_diff[-1:]])

    # Attitudes (roll, pitch, yaw) already in radians from FlightLog
    euler = log.attitudes

    # Throttle percentage (0-100) — displayed as-is on the throttle panel
    if len(log.throttle) == len(t):
        thrust = log.throttle.copy()
    else:
        thrust = np.full(len(t), 0.0)

    # Extract waypoints via dwell detection, convert NED -> ENU
    waypoints_ned = log.extract_waypoints()
    if waypoints_ned:
        waypoints = np.array([[wp[1], wp[0], -wp[2]] for wp in waypoints_ned])
    else:
        waypoints = np.array([pos_enu[0], pos_enu[-1]])

    origin_str = f"{log.origin_lat:.6f}, {log.origin_lon:.6f}, {log.origin_alt:.0f}m MSL"
    print(f"Loaded SITL log: {path}")
    print(f"  GPS origin: {origin_str}")
    print(f"  Duration: {t[-1]:.1f}s, {len(t)} samples")
    print(f"  Waypoints detected: {len(waypoints)}")

    return dict(
        t=t,
        pos=pos_enu,
        vel=vel_enu,
        euler=euler,
        thrust=thrust,
        waypoints=waypoints,
        source='sitl',
        origin_lat=np.float64(log.origin_lat),
        origin_lon=np.float64(log.origin_lon),
        origin_alt=np.float64(log.origin_alt),
    )


def _find_newest_sitl_log() -> Optional[Path]:
    """Find the most recently modified .BIN file in SITL log directories."""
    bins = []
    for pattern in SITL_LOG_GLOBS:
        bins.extend(pattern.parent.parent.parent.glob(
            str(Path(*pattern.parts[-3:]))))
    if not bins:
        return None
    return max(bins, key=lambda p: p.stat().st_mtime)


def load_data(path: str = None):
    if path:
        if path.lower().endswith('.bin'):
            return _load_bin(path)
        return dict(np.load(path, allow_pickle=True))

    # Try standard npz candidates first
    for p in DATA_CANDIDATES:
        if p.exists():
            return dict(np.load(str(p), allow_pickle=True))

    # Try SITL logs as fallback
    sitl_log = _find_newest_sitl_log()
    if sitl_log:
        print(f"No .npz data found. Using newest SITL log: {sitl_log}")
        return _load_bin(str(sitl_log))

    # No file found -- run the scenario inline
    print("No scenario_data.npz found. Running simulation inline...")
    from drone_physics import DroneParams, AeroCoefficients, Atmosphere, run_simulation
    from wind_model import WindField
    from terrain import TerrainMap

    params = DroneParams(
        mass=1.5, max_thrust=25.0,
        aero=AeroCoefficients(reference_area=0.04, C_D=1.0),
        atmo=Atmosphere(),
    )
    wind = WindField(wind_speed=3.0,
                     wind_direction=np.array([0.6, 0.8, 0.0]),
                     turbulence_type="constant")

    def hills(x, y):
        z = 3.0 * np.sin(x * 0.08) * np.cos(y * 0.06) + 1.5 * np.sin(x * 0.15 + 1.0) * np.sin(y * 0.12)
        return np.maximum(z, 0.0)

    terrain = TerrainMap.from_function(hills, x_range=(-10, 50), y_range=(-10, 35), resolution=0.5)

    waypoints = [
        np.array([0.0, 0.0, 12.0]), np.array([20.0, 0.0, 14.0]),
        np.array([20.0, 15.0, 16.0]), np.array([35.0, 20.0, 12.0]),
        np.array([10.0, 10.0, 10.0]), np.array([0.0, 0.0, 12.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    records = run_simulation(waypoints, params=params, dt=0.005,
                             waypoint_radius=0.5, hover_time=2.0,
                             max_time=180.0, wind=wind, terrain=terrain)

    bounds = terrain.bounds
    tx = np.arange(bounds[0], bounds[2] + terrain.resolution, terrain.resolution)[:terrain.elevations.shape[1]]
    ty = np.arange(bounds[1], bounds[3] + terrain.resolution, terrain.resolution)[:terrain.elevations.shape[0]]

    return dict(
        t=np.array([r.t for r in records]),
        pos=np.array([r.position for r in records]),
        vel=np.array([r.velocity for r in records]),
        euler=np.array([r.euler for r in records]),
        thrust=np.array([r.thrust for r in records]),
        ang_vel=np.array([r.angular_velocity for r in records]),
        waypoints=np.array(waypoints),
        terrain_x=tx, terrain_y=ty, terrain_z=terrain.elevations,
        wind_speed=np.float64(wind.wind_speed),
        wind_direction=wind.wind_direction / np.linalg.norm(wind.wind_direction),
    )


def rotation_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr   ],
    ])


def get_terrain_z(data, x, y):
    """Look up terrain elevation at (x, y) from the saved grid."""
    if 'terrain_x' not in data:
        return 0.0
    tx = data['terrain_x']
    ty = data['terrain_y']
    tz = data['terrain_z']
    # Bilinear interpolation
    fx = np.clip((x - tx[0]) / (tx[1] - tx[0]) if len(tx) > 1 else 0, 0, len(tx) - 1)
    fy = np.clip((y - ty[0]) / (ty[1] - ty[0]) if len(ty) > 1 else 0, 0, len(ty) - 1)
    ix0 = int(np.floor(fx))
    iy0 = int(np.floor(fy))
    ix1 = min(ix0 + 1, len(tx) - 1)
    iy1 = min(iy0 + 1, len(ty) - 1)
    sx = fx - ix0
    sy = fy - iy0
    return float(
        tz[iy0, ix0] * (1-sx) * (1-sy) + tz[iy0, ix1] * sx * (1-sy)
        + tz[iy1, ix0] * (1-sx) * sy + tz[iy1, ix1] * sx * sy
    )


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    data = load_data(path)

    is_swarm = 'positions' in data and 'pos' not in data
    t = data['t']

    if is_swarm:
        positions = data['positions']
        velocities = data['velocities']
        waypoints = data['waypoints']
        drone_ids = data.get('drone_ids', np.array([f'drone_{i+1}' for i in range(positions.shape[1])]))
        drone_ids = [str(x) for x in drone_ids]
    else:
        pos = data['pos']
        vel = data['vel']
        euler = data['euler']
        thrust = data['thrust']
        waypoints = data['waypoints']

    has_terrain = 'terrain_x' in data
    has_wind = 'wind_speed' in data and float(data['wind_speed']) > 0

    n = len(t)
    # Increase subsampling if in headless mode to speed up GIF creation
    is_interactive = plt.get_backend().lower() not in ['agg', 'svg', 'pdf', 'ps', 'template']
    if not is_interactive:
        step = max(2, n // 200) # Only ~200 frames for GIF
    else:
        step = max(1, n // 2000) # Full detail for interactive
    idx = np.arange(0, n, step)

    if is_swarm:
        center = positions.mean(axis=1)
        center_speed = np.linalg.norm(velocities.mean(axis=1), axis=1)
        sep = []
        for frame in positions:
            min_d = np.inf
            for i in range(frame.shape[0]):
                for j in range(i + 1, frame.shape[0]):
                    min_d = min(min_d, np.linalg.norm(frame[i] - frame[j]))
            sep.append(min_d)
        min_sep_series = np.array(sep)
    else:
        speed = np.linalg.norm(vel, axis=1)

    # Compute AGL (above ground level) for each position
    if has_terrain and not is_swarm:
        agl = np.array([pos[i, 2] - get_terrain_z(data, pos[i, 0], pos[i, 1]) for i in range(n)])
    elif is_swarm:
        agl = np.array([
            center[i, 2] - get_terrain_z(data, center[i, 0], center[i, 1]) if has_terrain else center[i, 2]
            for i in range(n)
        ])
    else:
        agl = pos[:, 2].copy()

    # ── Figure layout ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 11), facecolor='#1a1a2e')
    is_sitl = data.get('source') == 'sitl'
    if is_sitl:
        origin_lat = float(data.get('origin_lat', 0))
        origin_lon = float(data.get('origin_lon', 0))
        origin_alt = float(data.get('origin_alt', 0))
        title = (f'Swarm Digital Twin — SITL Flight Replay  '
                 f'({origin_lat:.4f}, {origin_lon:.4f}, {origin_alt:.0f}m MSL)')
    else:
        title = 'Swarm Digital Twin — Full Physics Visualization'
    fig.suptitle(title, color='white', fontsize=14, fontweight='bold')

    # 3D trajectory (main, left column)
    ax3d = fig.add_subplot(2, 2, (1, 3), projection='3d', facecolor='#16213e')

    # Timeline panels (right column)
    ax_alt = fig.add_subplot(3, 2, 2, facecolor='#16213e')
    ax_agl = fig.add_subplot(3, 2, 4, facecolor='#16213e')
    ax_thr = fig.add_subplot(3, 2, 6, facecolor='#16213e')

    # ── Style all axes ───────────────────────────────────────────────────
    for ax in [ax_alt, ax_agl, ax_thr]:
        ax.tick_params(colors='white', labelsize=8)
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        for spine in ax.spines.values():
            spine.set_color('#333')

    ax3d.tick_params(colors='white', labelsize=8)
    ax3d.xaxis.label.set_color('white')
    ax3d.yaxis.label.set_color('white')
    ax3d.zaxis.label.set_color('white')
    ax3d.xaxis.pane.fill = False
    ax3d.yaxis.pane.fill = False
    ax3d.zaxis.pane.fill = False

    # ── Terrain surface ──────────────────────────────────────────────────
    if has_terrain:
        tx = data['terrain_x']
        ty = data['terrain_y']
        tz = data['terrain_z']
        # Subsample terrain for rendering performance
        t_step = max(1, len(tx) // 80)
        tx_s = tx[::t_step]
        ty_s = ty[::t_step]
        tz_s = tz[::t_step, ::t_step]
        TX, TY = np.meshgrid(tx_s, ty_s)

        # Color map: green lowlands -> brown hills
        terrain_norm = mcolors.Normalize(vmin=tz.min(), vmax=max(tz.max(), 0.1))
        terrain_cmap = plt.cm.terrain

        ax3d.plot_surface(TX, TY, tz_s, alpha=0.6,
                          cmap=terrain_cmap, norm=terrain_norm,
                          linewidth=0, antialiased=True, zorder=0)
    else:
        # Flat ground
        pos_ref = positions.reshape(-1, 3) if is_swarm else pos
        gx = np.linspace(pos_ref[:, 0].min() - 2, pos_ref[:, 0].max() + 2, 10)
        gy = np.linspace(pos_ref[:, 1].min() - 2, pos_ref[:, 1].max() + 2, 10)
        GX, GY = np.meshgrid(gx, gy)
        ax3d.plot_surface(GX, GY, np.zeros_like(GX), alpha=0.08, color='green')

    # ── Full trajectory (faded) ──────────────────────────────────────────
    if is_swarm:
        for d in range(positions.shape[1]):
            ax3d.plot(positions[:, d, 0], positions[:, d, 1], positions[:, d, 2],
                      color='cyan', alpha=0.08, linewidth=0.4)
    else:
        ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2],
                  color='cyan', alpha=0.15, linewidth=0.5)

    # ── Waypoints ────────────────────────────────────────────────────────
    if is_swarm:
        for d in range(waypoints.shape[0]):
            wp_d = waypoints[d]
            ax3d.scatter(wp_d[:, 0], wp_d[:, 1], wp_d[:, 2],
                         color='red', s=40, marker='D', zorder=5)
    else:
        ax3d.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                     color='red', s=80, marker='D', zorder=5, label='Waypoints')
        for i, wp in enumerate(waypoints):
            ax3d.text(wp[0], wp[1], wp[2] + 0.8, f'WP{i+1}',
                      color='red', fontsize=8, ha='center')

    # ── Waypoint ground posts (vertical lines to terrain) ────────────────
    if has_terrain:
        if is_swarm:
            for d in range(waypoints.shape[0]):
                for wp in waypoints[d]:
                    gz = get_terrain_z(data, wp[0], wp[1])
                    ax3d.plot([wp[0], wp[0]], [wp[1], wp[1]], [gz, wp[2]],
                              color='red', alpha=0.2, linewidth=0.6, linestyle=':')
        else:
            for wp in waypoints:
                gz = get_terrain_z(data, wp[0], wp[1])
                ax3d.plot([wp[0], wp[0]], [wp[1], wp[1]], [gz, wp[2]],
                          color='red', alpha=0.3, linewidth=0.8, linestyle=':')

    # ── Wind indicator ───────────────────────────────────────────────────
    if has_wind:
        wind_dir = data['wind_direction']
        wind_spd = float(data['wind_speed'])
        # Place wind arrow at top corner of the flight area
        pos_ref = positions.reshape(-1, 3) if is_swarm else pos
        wx0 = pos_ref[:, 0].max() + 2
        wy0 = pos_ref[:, 1].max() + 2
        wz0 = pos_ref[:, 2].max() * 0.8
        scale = 3.0
        ax3d.quiver(wx0, wy0, wz0,
                     wind_dir[0] * scale, wind_dir[1] * scale, wind_dir[2] * scale,
                     color='#ffaa00', arrow_length_ratio=0.3, linewidth=2.5)
        ax3d.text(wx0, wy0, wz0 + 1.5,
                  f'Wind {wind_spd:.1f} m/s', color='#ffaa00', fontsize=9,
                  ha='center', fontweight='bold')

    # ── Axis limits ──────────────────────────────────────────────────────
    margin = 5
    pos_ref = positions.reshape(-1, 3) if is_swarm else pos
    wp_ref = waypoints.reshape(-1, 3) if is_swarm else waypoints
    z_max = max(pos_ref[:, 2].max(), wp_ref[:, 2].max())
    z_min = tz.min() if has_terrain else 0
    ax3d.set_xlim(pos_ref[:, 0].min() - margin, pos_ref[:, 0].max() + margin)
    ax3d.set_ylim(pos_ref[:, 1].min() - margin, pos_ref[:, 1].max() + margin)
    ax3d.set_zlim(z_min, z_max + margin)
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')

    # ── Timeline: Altitude MSL + Speed ───────────────────────────────────
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(0, (center[:, 2].max() if is_swarm else pos[:, 2].max()) * 1.2)
    ax_alt.set_ylabel('Altitude MSL (m)', fontsize=9)
    ax_alt.set_title('Altitude & Speed', fontsize=10)
    ax_alt.grid(True, alpha=0.2, color='white')
    ax_alt.set_xticklabels([])

    ax_alt2 = ax_alt.twinx()
    ax_alt2.set_ylim(0, max((center_speed.max() if is_swarm else speed.max()) * 1.3, 0.1))
    ax_alt2.set_ylabel('Speed (m/s)', color='#ff6b6b', fontsize=9)
    ax_alt2.tick_params(colors='#ff6b6b', labelsize=8)

    # ── Timeline: AGL ────────────────────────────────────────────────────
    ax_agl.set_xlim(t[0], t[-1])
    ax_agl.set_ylim(0, max(agl.max() * 1.2, 1.0))
    ax_agl.set_ylabel('AGL (m)', fontsize=9)
    ax_agl.set_title('Above Ground Level', fontsize=10, color='white')
    ax_agl.grid(True, alpha=0.2, color='white')
    ax_agl.set_xticklabels([])

    # ── Timeline: Thrust ─────────────────────────────────────────────────
    ax_thr.set_xlim(t[0], t[-1])
    if is_swarm:
        ax_thr.set_ylim(0, max(min_sep_series.max() * 1.2, 1.0))
        ax_thr.set_ylabel('Min separation (m)', fontsize=9)
        ax_thr.set_title('Swarm Separation', fontsize=10, color='white')
    else:
        ax_thr.set_ylim(0, max(thrust.max() * 1.2, 1.0) if not is_sitl else 105)
    ax_thr.set_xlabel('Time (s)', fontsize=9)
    if not is_swarm:
        if is_sitl:
            ax_thr.set_ylabel('Throttle (%)', fontsize=9)
            ax_thr.set_title('Throttle', fontsize=10, color='white')
        else:
            ax_thr.set_ylabel('Thrust (N)', fontsize=9)
            ax_thr.set_title('Thrust', fontsize=10, color='white')
    ax_thr.grid(True, alpha=0.2, color='white')
    if not is_swarm:
        if is_sitl:
            ax_thr.axhline(y=50, color='yellow', linestyle='--', alpha=0.5,
                            label='Hover (~50%)')
        else:
            hover_thrust = 1.5 * 9.81
            ax_thr.axhline(y=hover_thrust, color='yellow', linestyle='--', alpha=0.5,
                            label=f'Hover ({hover_thrust:.1f} N)')
        ax_thr.legend(loc='upper right', fontsize=7)

    # ── Animated elements ────────────────────────────────────────────────

    # Trail
    if is_swarm:
        swarm_colors = plt.cm.tab10(np.linspace(0, 1, max(positions.shape[1], 3)))
        trail_lines = [ax3d.plot([], [], [], color=swarm_colors[i], linewidth=1.2, alpha=0.5)[0]
                       for i in range(positions.shape[1])]
        drone_dots = [ax3d.plot([], [], [], 'o', color=swarm_colors[i], markersize=6, zorder=10)[0]
                      for i in range(positions.shape[1])]
    else:
        trail_line, = ax3d.plot([], [], [], color='cyan', linewidth=1.5, alpha=0.6)
        drone_dot, = ax3d.plot([], [], [], 'o', color='#00ff88', markersize=8, zorder=10)

    # Body axes (attitude)
    ARM = 1.5
    body_x_line, = ax3d.plot([], [], [], color='red', linewidth=2)
    body_y_line, = ax3d.plot([], [], [], color='green', linewidth=2)
    body_z_line, = ax3d.plot([], [], [], color='blue', linewidth=2)

    # Velocity vector
    vel_line, = ax3d.plot([], [], [], color='yellow', linewidth=1.5, alpha=0.7)

    # Shadow on terrain
    shadow_dot, = ax3d.plot([], [], [], 'o', color='white', markersize=4, alpha=0.4)

    # Vertical line from drone to ground shadow
    drop_line, = ax3d.plot([], [], [], color='white', linewidth=0.5, alpha=0.2,
                            linestyle=':')

    # Timeline traces
    alt_line, = ax_alt.plot([], [], color='cyan', linewidth=1.2)
    speed_line, = ax_alt2.plot([], [], color='#ff6b6b', linewidth=1.2)
    agl_line, = ax_agl.plot([], [], color='#44ff88', linewidth=1.2)
    thr_line, = ax_thr.plot([], [], color='#ffd93d', linewidth=1.2)

    # Text overlays
    time_text = fig.text(0.02, 0.02, '', color='white', fontsize=10,
                         fontfamily='monospace', transform=fig.transFigure)
    info_text = fig.text(0.02, 0.96, '', color='#00ff88', fontsize=9,
                         fontfamily='monospace', transform=fig.transFigure)
    phys_text = fig.text(0.52, 0.02, '', color='#aaaaaa', fontsize=8,
                         fontfamily='monospace', transform=fig.transFigure)

    # Static physics info
    phys_parts = []
    if is_sitl:
        phys_parts.append("source=SITL")
        phys_parts.append(f"samples={len(t)}")
        phys_parts.append(f"duration={t[-1]:.1f}s")
    if 'aero_cd' in data:
        phys_parts.append(f"C_D={float(data['aero_cd']):.1f}")
    if 'aero_area' in data:
        phys_parts.append(f"A={float(data['aero_area']):.3f}m^2")
    if 'atmo_rho' in data:
        phys_parts.append(f"rho={float(data['atmo_rho']):.3f}kg/m^3")
    if has_wind:
        phys_parts.append(f"wind={float(data['wind_speed']):.1f}m/s")
    if has_terrain:
        phys_parts.append(f"terrain={data['terrain_z'].shape[1]}x{data['terrain_z'].shape[0]}")
    phys_text.set_text('  '.join(phys_parts))

    def update(frame_i):
        i = idx[frame_i]

        trail_start = max(0, i - 400)

        if is_swarm:
            frame = positions[i]
            p = center[i]
            for d in range(frame.shape[0]):
                trail_lines[d].set_data(positions[trail_start:i, d, 0], positions[trail_start:i, d, 1])
                trail_lines[d].set_3d_properties(positions[trail_start:i, d, 2])
                drone_dots[d].set_data([frame[d, 0]], [frame[d, 1]])
                drone_dots[d].set_3d_properties([frame[d, 2]])
            if frame_i == 0:
                for d in range(frame.shape[0]):
                    ax3d.text(frame[d, 0], frame[d, 1], frame[d, 2] + 0.5, drone_ids[d],
                              color='white', fontsize=7, ha='center')
        else:
            trail_line.set_data(pos[trail_start:i, 0], pos[trail_start:i, 1])
            trail_line.set_3d_properties(pos[trail_start:i, 2])
            p = pos[i]
            drone_dot.set_data([p[0]], [p[1]])
            drone_dot.set_3d_properties([p[2]])

        # Ground shadow + drop line
        gz = get_terrain_z(data, p[0], p[1]) if has_terrain else 0.0
        shadow_dot.set_data([p[0]], [p[1]])
        shadow_dot.set_3d_properties([gz])
        drop_line.set_data([p[0], p[0]], [p[1], p[1]])
        drop_line.set_3d_properties([gz, p[2]])

        # Body axes
        if not is_swarm:
            r, pt, yw = euler[i]
            R = rotation_from_euler(r, pt, yw)
            for line, col_idx in [(body_x_line, 0), (body_y_line, 1), (body_z_line, 2)]:
                axis = R[:, col_idx] * ARM
                line.set_data([p[0], p[0] + axis[0]], [p[1], p[1] + axis[1]])
                line.set_3d_properties([p[2], p[2] + axis[2]])

        # Velocity vector
        v = velocities.mean(axis=1)[i] if is_swarm else vel[i]
        v_scale = 0.5
        vel_line.set_data([p[0], p[0] + v[0]*v_scale],
                          [p[1], p[1] + v[1]*v_scale])
        vel_line.set_3d_properties([p[2], p[2] + v[2]*v_scale])

        # Timeline traces
        alt_line.set_data(t[:i], center[:i, 2] if is_swarm else pos[:i, 2])
        speed_line.set_data(t[:i], center_speed[:i] if is_swarm else speed[:i])
        agl_line.set_data(t[:i], agl[:i])
        thr_line.set_data(t[:i], min_sep_series[:i] if is_swarm else thrust[:i])

        # Text
        if is_swarm:
            time_text.set_text(
                f't={t[i]:.2f}s  center=[{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}]  '
                f'v={center_speed[i]:.2f}m/s  min_sep={min_sep_series[i]:.2f}m  AGL={agl[i]:.1f}m'
            )
            info_text.set_text(f'swarm_size={positions.shape[1]}  mode=swarm')
            return tuple(trail_lines + drone_dots + [shadow_dot, drop_line, vel_line,
                                                     alt_line, speed_line, agl_line, thr_line,
                                                     time_text, info_text])

        if is_sitl:
            time_text.set_text(
                f't={t[i]:.2f}s  pos=[{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}]  '
                f'v={speed[i]:.2f}m/s  throttle={thrust[i]:.0f}%  AGL={agl[i]:.1f}m'
            )
        else:
            time_text.set_text(
                f't={t[i]:.2f}s  pos=[{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}]  '
                f'v={speed[i]:.2f}m/s  T={thrust[i]:.1f}N  AGL={agl[i]:.1f}m'
            )
        info_text.set_text(
            f'roll={np.degrees(r):+.1f} deg  pitch={np.degrees(pt):+.1f} deg  '
            f'yaw={np.degrees(yw):+.1f} deg'
        )

        return (trail_line, drone_dot, shadow_dot, drop_line,
                body_x_line, body_y_line, body_z_line, vel_line,
                alt_line, speed_line, agl_line, thr_line,
                time_text, info_text)

    # Detect headless mode
    is_interactive = plt.get_backend().lower() not in ['agg', 'svg', 'pdf', 'ps', 'template']
    if not is_interactive:
        output_file = 'simulation/flight_visualization.gif'
        print(f"INFO: Non-interactive backend ({plt.get_backend()}).")
        print(f"      Saving visualization to {output_file}...")
        try:
            # We already have is_interactive check above, but for clarity:
            ani = FuncAnimation(fig, update, frames=len(idx),
                                interval=33, blit=False, repeat=False)
            # Use faster settings for headless mode
            # pillow writer is standard but slow; lower fps helps
            ani.save(output_file, writer='pillow', fps=10)
            print(f"OK: Visualization saved to {output_file}")
            plt.close(fig)
            return # Exit after saving
        except Exception as e:
            print(f"WARN: Could not save visualization: {e}")
            plt.close(fig)
            return

    ani = FuncAnimation(fig, update, frames=len(idx),
                        interval=33, blit=False, repeat=True)

    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.06, top=0.92,
                        wspace=0.25, hspace=0.28)
    plt.show()


if __name__ == '__main__':
    main()
