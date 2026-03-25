#!/usr/bin/env python3
"""
Paper-Style Validation Visualization - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Generates validation plots matching Valencia et al. (2025) Fig. 9/10/12/13:
  - Altitude comparison (sim vs reference) per mission type
  - Per-axis error boxplots (Fig. 13 style)
  - IRS-4 quadrotor altitude tracking at multiple AGL heights
  - Table 5 RMSE summary bar chart
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from drone_physics import (
    make_valencia_fixed_wing, make_irs4_quadrotor,
    run_simulation,
)
from wind_model import WindField
from validation import compute_rmse


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Paper Table 5 reference data
TABLE5 = {
    "FW-185":       {"rmse_z": 1.885, "rmse_x": 0.865, "rmse_y": 0.373, "type": "fw"},
    "FW-178":       {"rmse_z": 1.994, "rmse_x": 1.729, "rmse_y": 1.248, "type": "fw"},
    "FW-158":       {"rmse_z": 2.001, "rmse_x": 0.661, "rmse_y": 0.247, "type": "fw"},
    "Quad-Car40":   {"rmse_z": 0.07,  "rmse_x": 0.043, "rmse_y": 0.039, "type": "quad"},
    "Quad-Car20":   {"rmse_z": 0.054, "rmse_x": 0.037, "rmse_y": 0.027, "type": "quad"},
    "Quad-EPN30":   {"rmse_z": 0.07,  "rmse_x": 0.062, "rmse_y": 0.055, "type": "quad"},
    "Quad-EPN20":   {"rmse_z": 0.10,  "rmse_x": 0.071, "rmse_y": 0.036, "type": "quad"},
}


def run_comparison(params, waypoints, wind_speed, wind_dir, max_time=60.0):
    """Run ref (no wind) and sim (with wind), return times + positions."""
    ref_wind = WindField(wind_speed=0.0, wind_direction=wind_dir,
                         turbulence_type="none")
    sim_wind = WindField(wind_speed=wind_speed, wind_direction=wind_dir,
                         turbulence_type="constant")

    ref = run_simulation(waypoints=waypoints, params=params, dt=0.005,
                         waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
                         wind=ref_wind)
    sim = run_simulation(waypoints=waypoints, params=params, dt=0.005,
                         waypoint_radius=0.5, hover_time=1.5, max_time=max_time,
                         wind=sim_wind)
    n = min(len(ref), len(sim))
    return {
        "ref_t": np.array([r.t for r in ref[:n]]),
        "ref_pos": np.array([r.position for r in ref[:n]]),
        "sim_t": np.array([r.t for r in sim[:n]]),
        "sim_pos": np.array([r.position for r in sim[:n]]),
    }


def plot_altitude_comparison(results_dict, output_path):
    """Paper Fig. 9/10/12 style: altitude vs time for each mission."""
    n = len(results_dict)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3.5 * n), sharex=False)
    if n == 1:
        axes = [axes]

    for ax, (name, data) in zip(axes, results_dict.items()):
        ax.plot(data["ref_t"], data["ref_pos"][:, 2], "b-", linewidth=1.2,
                label="Sim (no wind)")
        ax.plot(data["sim_t"], data["sim_pos"][:, 2], "r-", linewidth=1.2,
                label="Sim (with wind)")
        rmse = compute_rmse(data["sim_pos"], data["ref_pos"])
        ax.set_title(f"{name}  (RMSE_Z={rmse.rmse_z:.3f}m)", fontsize=11)
        ax.set_ylabel("Altitude [m]")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_error_boxplots(results_dict, output_path):
    """Paper Fig. 13 style: error boxplots per mission."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Error Statistics (Paper Fig. 13 style)", fontsize=13)

    # Separate FW and Quad
    fw_data, fw_labels = [], []
    quad_data, quad_labels = [], []

    for name, data in results_dict.items():
        errors = data["sim_pos"][:, 2] - data["ref_pos"][:, 2]
        if "FW" in name or "fw" in name:
            fw_data.append(errors)
            fw_labels.append(name)
        else:
            quad_data.append(errors)
            quad_labels.append(name)

    if fw_data:
        bp1 = ax1.boxplot(fw_data, tick_labels=fw_labels, patch_artist=True)
        for patch in bp1['boxes']:
            patch.set_facecolor('#FFB3B3')
        ax1.set_title("(a) Fixed-wing Antisana missions")
        ax1.set_ylabel("Altitude error [m]")
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)

    if quad_data:
        bp2 = ax2.boxplot(quad_data, tick_labels=quad_labels, patch_artist=True)
        for patch in bp2['boxes']:
            patch.set_facecolor('#B3D9FF')
        ax2.set_title("(b) Quadrotor urban missions")
        ax2.set_ylabel("Altitude error [m]")
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_table5_bar_chart(output_path):
    """Bar chart of paper Table 5 RMSE values for reference."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Paper Table 5: RMSE by Mission (Valencia et al. 2025)", fontsize=13)

    # Fixed-wing
    fw_names = [k for k, v in TABLE5.items() if v["type"] == "fw"]
    fw_z = [TABLE5[k]["rmse_z"] for k in fw_names]
    fw_x = [TABLE5[k]["rmse_x"] for k in fw_names]
    fw_y = [TABLE5[k]["rmse_y"] for k in fw_names]

    x = np.arange(len(fw_names))
    w = 0.25
    ax1.bar(x - w, fw_z, w, label="RMSE_Z", color="#E74C3C")
    ax1.bar(x, fw_x, w, label="RMSE_X", color="#3498DB")
    ax1.bar(x + w, fw_y, w, label="RMSE_Y", color="#2ECC71")
    ax1.set_xticks(x)
    ax1.set_xticklabels(fw_names, fontsize=9)
    ax1.set_ylabel("RMSE [m]")
    ax1.set_title("(a) Fixed-wing (Antisana, 4500m)")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3, axis='y')

    # Quadrotor
    q_names = [k for k, v in TABLE5.items() if v["type"] == "quad"]
    q_z = [TABLE5[k]["rmse_z"] for k in q_names]
    q_x = [TABLE5[k]["rmse_x"] for k in q_names]
    q_y = [TABLE5[k]["rmse_y"] for k in q_names]

    x2 = np.arange(len(q_names))
    ax2.bar(x2 - w, q_z, w, label="RMSE_Z", color="#E74C3C")
    ax2.bar(x2, q_x, w, label="RMSE_X", color="#3498DB")
    ax2.bar(x2 + w, q_y, w, label="RMSE_Y", color="#2ECC71")
    ax2.set_xticks(x2)
    ax2.set_xticklabels(q_names, fontsize=8, rotation=15)
    ax2.set_ylabel("RMSE [m]")
    ax2.set_title("(b) Quadrotor (Quito, 2800m)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_3d_trajectory_comparison(results_dict, output_path):
    """3D trajectory comparison plot for all missions."""
    fig = plt.figure(figsize=(14, 6))

    # Fixed-wing 3D
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    for name, data in results_dict.items():
        if "FW" in name or "fw" in name:
            ax1.plot(data["ref_pos"][:, 0], data["ref_pos"][:, 1], data["ref_pos"][:, 2],
                     'b-', linewidth=0.8, alpha=0.6)
            ax1.plot(data["sim_pos"][:, 0], data["sim_pos"][:, 1], data["sim_pos"][:, 2],
                     'r-', linewidth=0.8, alpha=0.8, label=name)
    ax1.set_xlabel("N [m]")
    ax1.set_ylabel("E [m]")
    ax1.set_zlabel("Alt [m]")
    ax1.set_title("Fixed-wing trajectories")
    ax1.legend(fontsize=8)

    # Quadrotor 3D
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    for name, data in results_dict.items():
        if "Quad" in name or "quad" in name:
            ax2.plot(data["ref_pos"][:, 0], data["ref_pos"][:, 1], data["ref_pos"][:, 2],
                     'b-', linewidth=0.8, alpha=0.6)
            ax2.plot(data["sim_pos"][:, 0], data["sim_pos"][:, 1], data["sim_pos"][:, 2],
                     'r-', linewidth=0.8, alpha=0.8, label=name)
    ax2.set_xlabel("N [m]")
    ax2.set_ylabel("E [m]")
    ax2.set_zlabel("Alt [m]")
    ax2.set_title("Quadrotor trajectories")
    ax2.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def main():
    print("Generating paper-style validation plots...")

    wind_dir = np.array([0.6, 0.8, 0.2])
    results = {}

    # Fixed-wing simulations
    fw_params = make_valencia_fixed_wing()
    fw_waypoints = [
        np.array([0, 0, 100]),
        np.array([50, 0, 100]),
        np.array([50, 30, 100]),
        np.array([0, 30, 100]),
        np.array([0, 0, 100]),
    ]
    print("  Running FW simulation...")
    results["FW-sim"] = run_comparison(fw_params, fw_waypoints, 3.0, wind_dir, 120.0)

    # Quadrotor simulations at different altitudes
    quad_params = make_irs4_quadrotor()
    for alt, name in [(20, "Quad-20m"), (30, "Quad-30m"), (40, "Quad-40m")]:
        print(f"  Running {name} simulation...")
        quad_wps = [
            np.array([0, 0, alt]),
            np.array([10, 0, alt]),
            np.array([10, 10, alt]),
            np.array([0, 10, alt]),
            np.array([0, 0, alt]),
        ]
        results[name] = run_comparison(quad_params, quad_wps, 2.0, wind_dir, 60.0)

    # Generate plots
    output_dir = os.path.join(SCRIPT_DIR, "validation_plots")
    os.makedirs(output_dir, exist_ok=True)

    plot_altitude_comparison(results, os.path.join(output_dir, "altitude_comparison.png"))
    plot_error_boxplots(results, os.path.join(output_dir, "error_boxplots.png"))
    plot_table5_bar_chart(os.path.join(output_dir, "table5_rmse.png"))
    plot_3d_trajectory_comparison(results, os.path.join(output_dir, "trajectory_3d.png"))

    # Print summary table
    print("\nRMSE Summary:")
    print(f"  {'Mission':<15} {'RMSE_X':>8} {'RMSE_Y':>8} {'RMSE_Z':>8} {'Total':>8}")
    print(f"  {'-'*15} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
    for name, data in results.items():
        r = compute_rmse(data["sim_pos"], data["ref_pos"])
        print(f"  {name:<15} {r.rmse_x:8.4f} {r.rmse_y:8.4f} {r.rmse_z:8.4f} {r.rmse_total:8.4f}")

    print(f"\nAll plots saved to {output_dir}/")


if __name__ == '__main__':
    main()
