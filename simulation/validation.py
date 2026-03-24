"""
Validation Metrics - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Compare simulation output against reference data (flight log or another sim).
Metrics follow Valencia et al. (2025), Table 5, Fig. 13.
"""

import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass


@dataclass
class ValidationResult:
    """RMSE and error statistics per axis and total."""
    rmse_x: float
    rmse_y: float
    rmse_z: float
    rmse_total: float
    median_error: float
    p25_error: float
    p75_error: float
    max_error: float
    n_points: int


def compute_rmse(sim_trajectory: np.ndarray,
                 ref_trajectory: np.ndarray) -> ValidationResult:
    """Compute RMSE per axis and total, matching paper's Table 5.

    Args:
        sim_trajectory: Nx3 simulation positions.
        ref_trajectory: Nx3 reference positions (same length, time-aligned).

    Returns:
        ValidationResult with per-axis and total RMSE plus error percentiles.
    """
    assert sim_trajectory.shape == ref_trajectory.shape, (
        f"Shape mismatch: sim {sim_trajectory.shape} vs ref {ref_trajectory.shape}"
    )
    assert sim_trajectory.shape[1] == 3, "Expected Nx3 arrays"

    errors = sim_trajectory - ref_trajectory
    per_point = np.linalg.norm(errors, axis=1)

    rmse_x = np.sqrt(np.mean(errors[:, 0] ** 2))
    rmse_y = np.sqrt(np.mean(errors[:, 1] ** 2))
    rmse_z = np.sqrt(np.mean(errors[:, 2] ** 2))
    rmse_total = np.sqrt(np.mean(per_point ** 2))

    return ValidationResult(
        rmse_x=rmse_x,
        rmse_y=rmse_y,
        rmse_z=rmse_z,
        rmse_total=rmse_total,
        median_error=float(np.median(per_point)),
        p25_error=float(np.percentile(per_point, 25)),
        p75_error=float(np.percentile(per_point, 75)),
        max_error=float(np.max(per_point)),
        n_points=len(per_point),
    )


def interpolate_to_common_times(sim_times: np.ndarray, sim_positions: np.ndarray,
                                 ref_times: np.ndarray, ref_positions: np.ndarray):
    """Interpolate both trajectories to common timestamps.

    Returns (common_times, sim_interp, ref_interp) all Nx3.
    """
    t_start = max(sim_times[0], ref_times[0])
    t_end = min(sim_times[-1], ref_times[-1])
    dt = max(np.median(np.diff(sim_times)), np.median(np.diff(ref_times)))
    common_times = np.arange(t_start, t_end, dt)

    sim_interp = np.column_stack([
        np.interp(common_times, sim_times, sim_positions[:, i])
        for i in range(3)
    ])
    ref_interp = np.column_stack([
        np.interp(common_times, ref_times, ref_positions[:, i])
        for i in range(3)
    ])

    return common_times, sim_interp, ref_interp


def plot_comparison(sim_times: np.ndarray, sim_positions: np.ndarray,
                    ref_times: np.ndarray, ref_positions: np.ndarray,
                    output_path: str, title: str = "DT Validation"):
    """Generate paper-style comparison plots.

    Creates:
    - Altitude vs time (sim vs ref)
    - 3D trajectory comparison
    - Error boxplots per axis
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    common_t, sim_interp, ref_interp = interpolate_to_common_times(
        sim_times, sim_positions, ref_times, ref_positions,
    )
    result = compute_rmse(sim_interp, ref_interp)
    errors = sim_interp - ref_interp

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"{title}  (RMSE: {result.rmse_total:.3f} m)", fontsize=14)

    # Altitude vs time
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(common_t, -sim_interp[:, 2], label="Sim", linewidth=1.5)
    ax1.plot(common_t, -ref_interp[:, 2], label="Ref", linewidth=1.5, linestyle="--")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Altitude [m]")
    ax1.set_title("Altitude vs Time")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 3D trajectory
    ax2 = fig.add_subplot(2, 2, 2, projection="3d")
    ax2.plot(sim_interp[:, 0], sim_interp[:, 1], -sim_interp[:, 2],
             label="Sim", linewidth=1.5)
    ax2.plot(ref_interp[:, 0], ref_interp[:, 1], -ref_interp[:, 2],
             label="Ref", linewidth=1.5, linestyle="--")
    ax2.set_xlabel("N [m]")
    ax2.set_ylabel("E [m]")
    ax2.set_zlabel("Alt [m]")
    ax2.set_title("3D Trajectory")
    ax2.legend()

    # Per-axis error over time
    ax3 = fig.add_subplot(2, 2, 3)
    for i, label in enumerate(["X", "Y", "Z"]):
        ax3.plot(common_t, errors[:, i], label=label, linewidth=1)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Error [m]")
    ax3.set_title("Per-axis Error")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Error boxplot
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.boxplot([errors[:, 0], errors[:, 1], errors[:, 2],
                 np.linalg.norm(errors, axis=1)],
                labels=["X", "Y", "Z", "Total"])
    ax4.set_ylabel("Error [m]")
    ax4.set_title(f"Error Distribution (n={result.n_points})")
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    return result
