#!/usr/bin/env python3
"""
3D Drone Flight Visualization - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Reads scenario_data.npz (produced by drone_scenario.py) and renders:
  - 3D trajectory with animated drone
  - Drone body-frame axes (attitude indicator)
  - Velocity vector
  - Altitude, speed, and thrust timeline panels
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3D
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
        return np.load(path)
    for p in DATA_CANDIDATES:
        if p.exists():
            return np.load(str(p))

    # No file found — run the scenario inline
    print("No scenario_data.npz found. Running simulation inline...")
    from drone_physics import DroneParams, run_simulation
    params = DroneParams(mass=1.5, drag_coeff=0.1, max_thrust=25.0)
    waypoints = [
        np.array([0.0,  0.0, 10.0]),
        np.array([20.0, 0.0, 10.0]),
        np.array([20.0, 15.0, 12.0]),
        np.array([10.0, 10.0,  8.0]),
        np.array([0.0,  0.0, 10.0]),
        np.array([0.0,  0.0,  0.5]),
    ]
    records = run_simulation(waypoints, params=params, dt=0.005,
                             waypoint_radius=0.5, hover_time=2.0, max_time=120.0)
    data = dict(
        t=np.array([r.t for r in records]),
        pos=np.array([r.position for r in records]),
        vel=np.array([r.velocity for r in records]),
        euler=np.array([r.euler for r in records]),
        thrust=np.array([r.thrust for r in records]),
        ang_vel=np.array([r.angular_velocity for r in records]),
        waypoints=np.array(waypoints),
    )
    return data


def rotation_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr   ],
    ])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    data = load_data(path)

    t = data['t']
    pos = data['pos']
    vel = data['vel']
    euler = data['euler']
    thrust = data['thrust']
    waypoints = data['waypoints']

    n = len(t)
    # Subsample for animation speed (target ~30 FPS visual)
    step = max(1, n // 2000)
    idx = np.arange(0, n, step)

    speed = np.linalg.norm(vel, axis=1)

    # ── Figure layout ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 10), facecolor='#1a1a2e')
    fig.suptitle('Drone Digital Twin — Flight Visualization',
                 color='white', fontsize=14, fontweight='bold')

    # 3D trajectory (main)
    ax3d = fig.add_subplot(2, 2, (1, 3), projection='3d', facecolor='#16213e')

    # Timeline panels
    ax_alt = fig.add_subplot(2, 2, 2, facecolor='#16213e')
    ax_thr = fig.add_subplot(2, 2, 4, facecolor='#16213e')

    # ── Style all axes ───────────────────────────────────────────────────
    for ax in [ax_alt, ax_thr]:
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        for spine in ax.spines.values():
            spine.set_color('#333')

    ax3d.tick_params(colors='white')
    ax3d.xaxis.label.set_color('white')
    ax3d.yaxis.label.set_color('white')
    ax3d.zaxis.label.set_color('white')
    ax3d.xaxis.pane.fill = False
    ax3d.yaxis.pane.fill = False
    ax3d.zaxis.pane.fill = False

    # ── Static elements ──────────────────────────────────────────────────

    # Full trajectory (faded)
    ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2],
              color='cyan', alpha=0.15, linewidth=0.5)

    # Waypoints
    ax3d.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                 color='red', s=80, marker='D', zorder=5, label='Waypoints')
    for i, wp in enumerate(waypoints):
        ax3d.text(wp[0], wp[1], wp[2] + 0.8, f'WP{i+1}',
                  color='red', fontsize=8, ha='center')

    # Ground grid
    gx = np.linspace(pos[:, 0].min() - 2, pos[:, 0].max() + 2, 10)
    gy = np.linspace(pos[:, 1].min() - 2, pos[:, 1].max() + 2, 10)
    GX, GY = np.meshgrid(gx, gy)
    ax3d.plot_surface(GX, GY, np.zeros_like(GX), alpha=0.08, color='green')

    # Axis limits
    margin = 3
    ax3d.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
    ax3d.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
    ax3d.set_zlim(0, pos[:, 2].max() + margin)
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')

    # Timeline axes setup
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(0, pos[:, 2].max() * 1.2)
    ax_alt.set_ylabel('Altitude (m)')
    ax_alt.set_title('Altitude & Speed')
    ax_alt.grid(True, alpha=0.2, color='white')

    ax_alt2 = ax_alt.twinx()
    ax_alt2.set_ylim(0, speed.max() * 1.3)
    ax_alt2.set_ylabel('Speed (m/s)', color='#ff6b6b')
    ax_alt2.tick_params(colors='#ff6b6b')

    ax_thr.set_xlim(t[0], t[-1])
    ax_thr.set_ylim(0, thrust.max() * 1.2)
    ax_thr.set_xlabel('Time (s)')
    ax_thr.set_ylabel('Thrust (N)')
    ax_thr.set_title('Thrust')
    ax_thr.grid(True, alpha=0.2, color='white')
    ax_thr.axhline(y=1.5 * 9.81, color='yellow', linestyle='--', alpha=0.5, label='Hover thrust')
    ax_thr.legend(loc='upper right', fontsize=8)

    # ── Animated elements ────────────────────────────────────────────────

    # Trail
    trail_line, = ax3d.plot([], [], [], color='cyan', linewidth=1.5, alpha=0.6)

    # Drone marker
    drone_dot, = ax3d.plot([], [], [], 'o', color='#00ff88', markersize=8, zorder=10)

    # Body axes (attitude)
    ARM = 1.5  # visual arm length
    body_x_line, = ax3d.plot([], [], [], color='red', linewidth=2)
    body_y_line, = ax3d.plot([], [], [], color='green', linewidth=2)
    body_z_line, = ax3d.plot([], [], [], color='blue', linewidth=2)

    # Velocity vector
    vel_line, = ax3d.plot([], [], [], color='yellow', linewidth=1.5, alpha=0.7)

    # Shadow on ground
    shadow_dot, = ax3d.plot([], [], [], 'o', color='white', markersize=4, alpha=0.3)

    # Timeline traces
    alt_line, = ax_alt.plot([], [], color='cyan', linewidth=1.2)
    speed_line, = ax_alt2.plot([], [], color='#ff6b6b', linewidth=1.2)
    thr_line, = ax_thr.plot([], [], color='#ffd93d', linewidth=1.2)

    # Time marker
    time_text = fig.text(0.02, 0.02, '', color='white', fontsize=11,
                         fontfamily='monospace', transform=fig.transFigure)
    info_text = fig.text(0.02, 0.95, '', color='#00ff88', fontsize=10,
                         fontfamily='monospace', transform=fig.transFigure)

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

        # Shadow
        shadow_dot.set_data([p[0]], [p[1]])
        shadow_dot.set_3d_properties([0.0])

        # Body axes
        r, pt, yw = euler[i]
        R = rotation_from_euler(r, pt, yw)
        for line, col_idx, color in [(body_x_line, 0, 'red'),
                                      (body_y_line, 1, 'green'),
                                      (body_z_line, 2, 'blue')]:
            axis = R[:, col_idx] * ARM
            line.set_data([p[0], p[0] + axis[0]], [p[1], p[1] + axis[1]])
            line.set_3d_properties([p[2], p[2] + axis[2]])

        # Velocity vector
        v = vel[i]
        v_scale = 0.5
        vel_line.set_data([p[0], p[0] + v[0]*v_scale], [p[1], p[1] + v[1]*v_scale])
        vel_line.set_3d_properties([p[2], p[2] + v[2]*v_scale])

        # Timeline traces
        alt_line.set_data(t[:i], pos[:i, 2])
        speed_line.set_data(t[:i], speed[:i])
        thr_line.set_data(t[:i], thrust[:i])

        # Text
        time_text.set_text(
            f't={t[i]:.2f}s  pos=[{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}]  '
            f'v={speed[i]:.2f} m/s  thrust={thrust[i]:.1f} N'
        )
        info_text.set_text(
            f'roll={np.degrees(r):+.1f}°  pitch={np.degrees(pt):+.1f}°  '
            f'yaw={np.degrees(yw):+.1f}°'
        )

        return (trail_line, drone_dot, shadow_dot,
                body_x_line, body_y_line, body_z_line, vel_line,
                alt_line, speed_line, thr_line, time_text, info_text)

    ani = FuncAnimation(fig, update, frames=len(idx),
                        interval=33, blit=False, repeat=True)

    plt.tight_layout(rect=[0, 0.04, 1, 0.96])
    plt.show()


if __name__ == '__main__':
    main()
