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
from typing import Dict, Optional
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


@dataclass(frozen=True)
class ValidationEnvelope:
    """Acceptance envelope for deterministic benchmark validation."""
    rmse_x_max: float
    rmse_y_max: float
    rmse_z_max: float
    rmse_total_max: float
    median_error_max: float
    p75_error_max: float
    max_error_max: float


@dataclass(frozen=True)
class BenchmarkProfile:
    """Canonical benchmark profile definition (Phase A1)."""
    name: str
    seed: int
    wind_speed: float
    wind_direction: np.ndarray
    tolerance: float
    envelope: ValidationEnvelope


# Envelopes are cross-platform bounds (ARM macOS + x86 Linux).
# Floating-point differences cause trajectory divergence over long sims,
# so envelopes must be generous. They catch catastrophic failures, not
# platform-specific regressions — use per-platform determinism tests for that.
BENCHMARK_PROFILES: Dict[str, BenchmarkProfile] = {
    "moderate": BenchmarkProfile(
        name="moderate",
        seed=20260324,
        wind_speed=3.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=16.0,
            rmse_y_max=12.0,
            rmse_z_max=8.0,
            rmse_total_max=20.0,
            median_error_max=14.0,
            p75_error_max=22.0,
            max_error_max=55.0,
        ),
    ),
    "strong_wind": BenchmarkProfile(
        name="strong_wind",
        seed=20260325,
        wind_speed=8.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=25.0,
            rmse_y_max=25.0,
            rmse_z_max=12.0,
            rmse_total_max=30.0,
            median_error_max=25.0,
            p75_error_max=30.0,
            max_error_max=80.0,
        ),
    ),
    "crosswind": BenchmarkProfile(
        name="crosswind",
        seed=20260326,
        wind_speed=5.0,
        wind_direction=np.array([1.0, 0.0, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=20.0,
            rmse_y_max=20.0,
            rmse_z_max=10.0,
            rmse_total_max=25.0,
            median_error_max=20.0,
            p75_error_max=25.0,
            max_error_max=65.0,
        ),
    ),
    "storm": BenchmarkProfile(
        name="storm",
        seed=20260327,
        wind_speed=10.0,
        wind_direction=np.array([0.0, 1.0, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=30.0,
            rmse_y_max=30.0,
            rmse_z_max=15.0,
            rmse_total_max=35.0,
            median_error_max=30.0,
            p75_error_max=35.0,
            max_error_max=90.0,
        ),
    ),
    "irs4_carolina": BenchmarkProfile(
        name="irs4_carolina",
        seed=20260328,
        wind_speed=2.0,
        wind_direction=np.array([0.5, 0.5, 0.3]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=15.0,
            rmse_y_max=15.0,
            rmse_z_max=10.0,
            rmse_total_max=20.0,
            median_error_max=15.0,
            p75_error_max=20.0,
            max_error_max=50.0,
        ),
    ),
    "irs4_epn": BenchmarkProfile(
        name="irs4_epn",
        seed=20260329,
        wind_speed=3.0,
        wind_direction=np.array([0.8, 0.6, 0.2]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=15.0,
            rmse_y_max=15.0,
            rmse_z_max=10.0,
            rmse_total_max=20.0,
            median_error_max=15.0,
            p75_error_max=20.0,
            max_error_max=50.0,
        ),
    ),
}


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


def assert_validation_pass(result: ValidationResult,
                           envelope: ValidationEnvelope,
                           profile_name: str = "") -> None:
    """Raise AssertionError when validation metrics exceed acceptance envelope."""
    checks = {
        "rmse_x": (result.rmse_x, envelope.rmse_x_max),
        "rmse_y": (result.rmse_y, envelope.rmse_y_max),
        "rmse_z": (result.rmse_z, envelope.rmse_z_max),
        "rmse_total": (result.rmse_total, envelope.rmse_total_max),
        "median_error": (result.median_error, envelope.median_error_max),
        "p75_error": (result.p75_error, envelope.p75_error_max),
        "max_error": (result.max_error, envelope.max_error_max),
    }

    failures = []
    for metric, (actual, max_allowed) in checks.items():
        if actual > max_allowed:
            failures.append(f"{metric}={actual:.4f} > {max_allowed:.4f}")

    if failures:
        prefix = f"[{profile_name}] " if profile_name else ""
        raise AssertionError(
            f"{prefix}Validation failed: " + "; ".join(failures)
        )


def get_benchmark_profile(profile_name: str) -> BenchmarkProfile:
    """Return canonical benchmark profile by name."""
    if profile_name not in BENCHMARK_PROFILES:
        available = ", ".join(sorted(BENCHMARK_PROFILES.keys()))
        raise KeyError(f"Unknown benchmark profile '{profile_name}'. Available: {available}")
    return BENCHMARK_PROFILES[profile_name]


def summarize_validation(result: ValidationResult) -> Dict[str, float]:
    """Return plain dict summary for logging/CLI output."""
    return {
        "rmse_x": result.rmse_x,
        "rmse_y": result.rmse_y,
        "rmse_z": result.rmse_z,
        "rmse_total": result.rmse_total,
        "median_error": result.median_error,
        "p75_error": result.p75_error,
        "max_error": result.max_error,
        "n_points": float(result.n_points),
    }


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


def compare_sim_real(sim_times: np.ndarray, sim_positions: np.ndarray,
                     ref_times: np.ndarray, ref_positions: np.ndarray) -> Dict[str, float]:
    """Compare simulation against real flight data (paper Table 5 format).

    Returns dict with per-axis RMSE, median, percentiles — matching Table 5 columns.
    """
    common_t, sim_interp, ref_interp = interpolate_to_common_times(
        sim_times, sim_positions, ref_times, ref_positions,
    )
    result = compute_rmse(sim_interp, ref_interp)
    errors = sim_interp - ref_interp

    return {
        "rmse_z": result.rmse_z,
        "rmse_x": result.rmse_x,
        "rmse_y": result.rmse_y,
        "median": float(np.median(errors[:, 2])),
        "p75": float(np.percentile(errors[:, 2], 75)),
        "p25": float(np.percentile(errors[:, 2], 25)),
        "n_points": result.n_points,
        "rmse_total": result.rmse_total,
    }


def compare_signals(sim_times: np.ndarray, sim_signal: np.ndarray,
                    ref_times: np.ndarray, ref_signal: np.ndarray) -> Dict[str, float]:
    """Compare two time-series signals (e.g. throttle, elevator).

    Returns cross-correlation, RMSE, and time-aligned metrics.
    """
    # Interpolate to common timestamps
    t_start = max(sim_times[0], ref_times[0])
    t_end = min(sim_times[-1], ref_times[-1])
    dt = max(np.median(np.diff(sim_times)), np.median(np.diff(ref_times)))
    common_t = np.arange(t_start, t_end, dt)

    if len(common_t) < 2:
        return {"cross_correlation": 0.0, "rmse": float("inf"), "n_points": 0}

    sim_interp = np.interp(common_t, sim_times, sim_signal)
    ref_interp = np.interp(common_t, ref_times, ref_signal)

    # Normalize for cross-correlation
    sim_norm = sim_interp - np.mean(sim_interp)
    ref_norm = ref_interp - np.mean(ref_interp)
    sim_std = np.std(sim_interp)
    ref_std = np.std(ref_interp)

    if sim_std < 1e-12 or ref_std < 1e-12:
        xcorr = 0.0
    else:
        xcorr = float(np.mean(sim_norm * ref_norm) / (sim_std * ref_std))

    rmse = float(np.sqrt(np.mean((sim_interp - ref_interp) ** 2)))

    return {
        "cross_correlation": xcorr,
        "rmse": rmse,
        "n_points": len(common_t),
    }


def plot_signal_comparison(sim_times: np.ndarray, sim_signal: np.ndarray,
                           ref_times: np.ndarray, ref_signal: np.ndarray,
                           output_path: str, title: str = "Signal Comparison",
                           ylabel: str = "Signal") -> Dict[str, float]:
    """Generate paper-style signal comparison plot (Fig. 9/10 lower panels)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    metrics = compare_signals(sim_times, sim_signal, ref_times, ref_signal)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle(f"{title}  (r={metrics['cross_correlation']:.3f})")

    ax1.plot(sim_times, sim_signal, label="Simulation", linewidth=1, color="red")
    ax1.plot(ref_times, ref_signal, label="Real data", linewidth=1, color="black")
    ax1.set_ylabel(ylabel)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Error over time (interpolated)
    t_start = max(sim_times[0], ref_times[0])
    t_end = min(sim_times[-1], ref_times[-1])
    dt = max(np.median(np.diff(sim_times)), np.median(np.diff(ref_times)))
    common_t = np.arange(t_start, t_end, dt)
    sim_interp = np.interp(common_t, sim_times, sim_signal)
    ref_interp = np.interp(common_t, ref_times, ref_signal)

    ax2.plot(common_t, sim_interp - ref_interp, linewidth=1, color="blue")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error")
    ax2.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    return metrics
