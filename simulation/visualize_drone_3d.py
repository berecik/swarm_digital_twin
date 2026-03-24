#!/usr/bin/env python3
"""
3D Drone Flight Visualization - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Reads scenario_data.npz (produced by drone_scenario.py) and renders:
  - 3D terrain surface with color-mapped elevation
  - Animated drone with body-frame axes (attitude indicator)
  - Velocity vector and ground shadow projected onto terrain
  - Wind direction indicator
  - Altitude (MSL + AGL), speed, and thrust timeline panels
"""

import os
import sys
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
    SCRIPT_DIR / 'scenario_data.npz',
    Path('scenario_data.npz'),
    Path('simulation/scenario_data.npz'),
]


def load_data(path: str = None):
    if path:
        return dict(np.load(path, allow_pickle=True))
    for p in DATA_CANDIDATES:
        if p.exists():
            return dict(np.load(str(p), allow_pickle=True))

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

    t = data['t']
    pos = data['pos']
    vel = data['vel']
    euler = data['euler']
    thrust = data['thrust']
    waypoints = data['waypoints']

    has_terrain = 'terrain_x' in data
    has_wind = 'wind_speed' in data and float(data['wind_speed']) > 0

    n = len(t)
    step = max(1, n // 2000)
    idx = np.arange(0, n, step)

    speed = np.linalg.norm(vel, axis=1)

    # Compute AGL (above ground level) for each position
    if has_terrain:
        agl = np.array([pos[i, 2] - get_terrain_z(data, pos[i, 0], pos[i, 1])
                         for i in range(n)])
    else:
        agl = pos[:, 2].copy()

    # ── Figure layout ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 11), facecolor='#1a1a2e')
    fig.suptitle('Swarm Digital Twin — Full Physics Visualization',
                 color='white', fontsize=14, fontweight='bold')

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
        gx = np.linspace(pos[:, 0].min() - 2, pos[:, 0].max() + 2, 10)
        gy = np.linspace(pos[:, 1].min() - 2, pos[:, 1].max() + 2, 10)
        GX, GY = np.meshgrid(gx, gy)
        ax3d.plot_surface(GX, GY, np.zeros_like(GX), alpha=0.08, color='green')

    # ── Full trajectory (faded) ──────────────────────────────────────────
    ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2],
              color='cyan', alpha=0.15, linewidth=0.5)

    # ── Waypoints ────────────────────────────────────────────────────────
    ax3d.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                 color='red', s=80, marker='D', zorder=5, label='Waypoints')
    for i, wp in enumerate(waypoints):
        ax3d.text(wp[0], wp[1], wp[2] + 0.8, f'WP{i+1}',
                  color='red', fontsize=8, ha='center')

    # ── Waypoint ground posts (vertical lines to terrain) ────────────────
    if has_terrain:
        for wp in waypoints:
            gz = get_terrain_z(data, wp[0], wp[1])
            ax3d.plot([wp[0], wp[0]], [wp[1], wp[1]], [gz, wp[2]],
                      color='red', alpha=0.3, linewidth=0.8, linestyle=':')

    # ── Wind indicator ───────────────────────────────────────────────────
    if has_wind:
        wind_dir = data['wind_direction']
        wind_spd = float(data['wind_speed'])
        # Place wind arrow at top corner of the flight area
        wx0 = pos[:, 0].max() + 2
        wy0 = pos[:, 1].max() + 2
        wz0 = pos[:, 2].max() * 0.8
        scale = 3.0
        ax3d.quiver(wx0, wy0, wz0,
                     wind_dir[0] * scale, wind_dir[1] * scale, wind_dir[2] * scale,
                     color='#ffaa00', arrow_length_ratio=0.3, linewidth=2.5)
        ax3d.text(wx0, wy0, wz0 + 1.5,
                  f'Wind {wind_spd:.1f} m/s', color='#ffaa00', fontsize=9,
                  ha='center', fontweight='bold')

    # ── Axis limits ──────────────────────────────────────────────────────
    margin = 5
    z_max = max(pos[:, 2].max(), waypoints[:, 2].max())
    z_min = tz.min() if has_terrain else 0
    ax3d.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
    ax3d.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
    ax3d.set_zlim(z_min, z_max + margin)
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')

    # ── Timeline: Altitude MSL + Speed ───────────────────────────────────
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(0, pos[:, 2].max() * 1.2)
    ax_alt.set_ylabel('Altitude MSL (m)', fontsize=9)
    ax_alt.set_title('Altitude & Speed', fontsize=10)
    ax_alt.grid(True, alpha=0.2, color='white')
    ax_alt.set_xticklabels([])

    ax_alt2 = ax_alt.twinx()
    ax_alt2.set_ylim(0, max(speed.max() * 1.3, 0.1))
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
    ax_thr.set_ylim(0, thrust.max() * 1.2)
    ax_thr.set_xlabel('Time (s)', fontsize=9)
    ax_thr.set_ylabel('Thrust (N)', fontsize=9)
    ax_thr.set_title('Thrust', fontsize=10, color='white')
    ax_thr.grid(True, alpha=0.2, color='white')
    hover_thrust = 1.5 * 9.81
    ax_thr.axhline(y=hover_thrust, color='yellow', linestyle='--', alpha=0.5,
                    label=f'Hover ({hover_thrust:.1f} N)')
    ax_thr.legend(loc='upper right', fontsize=7)

    # ── Animated elements ────────────────────────────────────────────────

    # Trail
    trail_line, = ax3d.plot([], [], [], color='cyan', linewidth=1.5, alpha=0.6)

    # Drone marker
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

        # Trail
        trail_start = max(0, i - 400)
        trail_line.set_data(pos[trail_start:i, 0], pos[trail_start:i, 1])
        trail_line.set_3d_properties(pos[trail_start:i, 2])

        # Drone position
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
        r, pt, yw = euler[i]
        R = rotation_from_euler(r, pt, yw)
        for line, col_idx in [(body_x_line, 0), (body_y_line, 1), (body_z_line, 2)]:
            axis = R[:, col_idx] * ARM
            line.set_data([p[0], p[0] + axis[0]], [p[1], p[1] + axis[1]])
            line.set_3d_properties([p[2], p[2] + axis[2]])

        # Velocity vector
        v = vel[i]
        v_scale = 0.5
        vel_line.set_data([p[0], p[0] + v[0]*v_scale],
                          [p[1], p[1] + v[1]*v_scale])
        vel_line.set_3d_properties([p[2], p[2] + v[2]*v_scale])

        # Timeline traces
        alt_line.set_data(t[:i], pos[:i, 2])
        speed_line.set_data(t[:i], speed[:i])
        agl_line.set_data(t[:i], agl[:i])
        thr_line.set_data(t[:i], thrust[:i])

        # Text
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

    ani = FuncAnimation(fig, update, frames=len(idx),
                        interval=33, blit=False, repeat=True)

    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.show()


if __name__ == '__main__':
    main()
