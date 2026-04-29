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
from typing import Dict, Callable, Tuple
from dataclasses import dataclass
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import urlretrieve


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


@dataclass(frozen=True)
class RealLogMission:
    """Real-flight-data mission profile mapped to paper Table 5 RMSE values."""
    name: str
    source_filename: str
    segment_start_s: float
    segment_end_s: float
    paper_rmse_z: float
    paper_rmse_x: float
    paper_rmse_y: float


@dataclass(frozen=True)
class WindAutoTuneResult:
    """Result of wind-force scale tuning against altitude RMSE."""
    best_scale: float
    best_rmse_z: float
    iterations: int
    converged: bool
    history: Tuple[Tuple[float, float], ...]


OSSITLQUAD_FLIGHT_LOGS_RAW_BASE = (
    "https://raw.githubusercontent.com/estebanvt/OSSITLQUAD/master/Flight_logs"
)

OSSITLQUAD_FLIGHT_LOGS_RAW_BASE_CANDIDATES: Tuple[str, ...] = (
    # Current upstream layout (verified via GitHub API):
    # Flight_logs/Quadrotor_logs/<file>.bin on main.
    "https://raw.githubusercontent.com/estebanvt/OSSITLQUAD/main/Flight_logs/Quadrotor_logs",
    # Legacy branch path used in older docs/scripts.
    OSSITLQUAD_FLIGHT_LOGS_RAW_BASE,
    # Default branch path for repositories migrated to `main`.
    "https://raw.githubusercontent.com/estebanvt/OSSITLQUAD/main/Flight_logs",
)

REAL_LOG_MISSIONS: Dict[str, RealLogMission] = {
    # Carolina 40m + 20m log: two distinct flight phases separated by landing.
    # Leg 1 (40 m): takeoff at ~t=10 s, steady at 38-42 m, descent at ~t=155 s.
    # Leg 2 (20 m): takeoff at ~t=340 s, steady at 18-20 m, descent at ~t=480 s.
    "quad_carolina_40": RealLogMission(
        name="quad_carolina_40",
        source_filename="Carolina_quad_40m_plus_20m.bin",
        segment_start_s=15.0,
        segment_end_s=155.0,
        paper_rmse_z=0.07,
        paper_rmse_x=0.043,
        paper_rmse_y=0.039,
    ),
    "quad_carolina_20": RealLogMission(
        name="quad_carolina_20",
        source_filename="Carolina_quad_40m_plus_20m.bin",
        segment_start_s=340.0,
        segment_end_s=478.0,
        paper_rmse_z=0.054,
        paper_rmse_x=0.037,
        paper_rmse_y=0.027,
    ),
    # EPN 30m + 20m log: multiple flights after ~t=276s gap.
    # Phase at 20m altitude: ~t=340-440s (EPN campus, 20m AGL).
    # Phase at 30m altitude: ~t=620-725s (EPN campus, 30m AGL).
    "quad_epn_30": RealLogMission(
        name="quad_epn_30",
        source_filename="EPN_quad_30m_plus_20m.bin",
        segment_start_s=620.0,
        segment_end_s=725.0,
        paper_rmse_z=0.07,
        paper_rmse_x=0.062,
        paper_rmse_y=0.055,
    ),
    "quad_epn_20": RealLogMission(
        name="quad_epn_20",
        source_filename="EPN_quad_30m_plus_20m.bin",
        segment_start_s=340.0,
        segment_end_s=440.0,
        paper_rmse_z=0.10,
        paper_rmse_x=0.071,
        paper_rmse_y=0.036,
    ),
}


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
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
    "strong_wind": BenchmarkProfile(
        name="strong_wind",
        seed=20260325,
        wind_speed=8.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
    "crosswind": BenchmarkProfile(
        name="crosswind",
        seed=20260326,
        wind_speed=5.0,
        wind_direction=np.array([1.0, 0.0, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
    "storm": BenchmarkProfile(
        name="storm",
        seed=20260327,
        wind_speed=10.0,
        wind_direction=np.array([0.0, 1.0, 0.0]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
    "irs4_carolina": BenchmarkProfile(
        name="irs4_carolina",
        seed=20260328,
        wind_speed=2.0,
        wind_direction=np.array([0.5, 0.5, 0.3]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
    "irs4_epn": BenchmarkProfile(
        name="irs4_epn",
        seed=20260329,
        wind_speed=3.0,
        wind_direction=np.array([0.8, 0.6, 0.2]),
        tolerance=1e-6,
        envelope=ValidationEnvelope(
            rmse_x_max=50.0,
            rmse_y_max=50.0,
            rmse_z_max=50.0,
            rmse_total_max=80.0,
            median_error_max=50.0,
            p75_error_max=60.0,
            max_error_max=200.0,
        ),
    ),
}


def get_real_log_mission(name: str) -> RealLogMission:
    """Return real-log mission profile by name (paper Table 5)."""
    if name not in REAL_LOG_MISSIONS:
        available = ", ".join(sorted(REAL_LOG_MISSIONS.keys()))
        raise KeyError(f"Unknown real-log mission '{name}'. Available: {available}")
    return REAL_LOG_MISSIONS[name]


def ensure_real_log_logs(data_dir: str = "data/flight_logs") -> Dict[str, str]:
    """Ensure required OSSITLQUAD `.bin` logs are present, downloading when missing."""
    dest = Path(data_dir)
    dest.mkdir(parents=True, exist_ok=True)

    local_paths: Dict[str, str] = {}
    needed = sorted({mission.source_filename for mission in REAL_LOG_MISSIONS.values()})
    for filename in needed:
        local = dest / filename
        if not local.exists():
            errors = []
            for raw_base in OSSITLQUAD_FLIGHT_LOGS_RAW_BASE_CANDIDATES:
                remote_url = f"{raw_base}/{filename}"
                try:
                    urlretrieve(remote_url, str(local))
                except (HTTPError, URLError) as exc:
                    errors.append(f"{remote_url} -> {exc}")
                    continue
                break
            else:
                attempted = "\n  - ".join(errors) if errors else "(no URL attempts)"
                raise RuntimeError(
                    f"Failed to download required real-log file '{filename}'.\n"
                    f"Tried URLs:\n  - {attempted}\n"
                    f"Place the file manually in '{dest}' and retry."
                )
        local_paths[filename] = str(local)
    return local_paths


def assert_real_log_validation_pass(result: ValidationResult | Dict[str, float],
                                    mission: RealLogMission,
                                    multiplier: float = 2.0) -> None:
    """Gate real-log RMSE against paper Table 5 values with configurable multiplier."""
    rmse_z = result.rmse_z if isinstance(result, ValidationResult) else float(result["rmse_z"])
    rmse_x = result.rmse_x if isinstance(result, ValidationResult) else float(result["rmse_x"])
    rmse_y = result.rmse_y if isinstance(result, ValidationResult) else float(result["rmse_y"])
    checks = {
        "rmse_z": (rmse_z, mission.paper_rmse_z * multiplier),
        "rmse_x": (rmse_x, mission.paper_rmse_x * multiplier),
        "rmse_y": (rmse_y, mission.paper_rmse_y * multiplier),
    }
    failures = []
    for metric, (actual, allowed) in checks.items():
        if actual > allowed:
            failures.append(f"{metric}={actual:.4f} > {allowed:.4f}")
    if failures:
        raise AssertionError(
            f"[{mission.name}] Real-log validation failed: " + "; ".join(failures)
        )


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


def auto_tune_wind_force_scale(
    ref_times: np.ndarray,
    ref_positions: np.ndarray,
    simulate_with_scale: Callable[[float], Tuple[np.ndarray, np.ndarray]],
    initial_scale: float = 1.0,
    initial_step: float = 0.5,
    max_iterations: int = 30,
    convergence_tol: float = 0.01,
    min_scale: float = 0.0,
    max_scale: float = 8.0,
) -> WindAutoTuneResult:
    """Iteratively tune wind-force scale to minimize altitude RMSE.

    The optimizer uses deterministic coordinate search with adaptive step decay.
    Convergence criterion: |ΔRMSE_z| < convergence_tol (wind auto-tuning).
    """
    if len(ref_times) < 2 or len(ref_positions) < 2:
        raise ValueError("Reference trajectory must contain at least 2 samples")
    if initial_step <= 0.0:
        raise ValueError("initial_step must be > 0")
    if max_scale <= min_scale:
        raise ValueError("max_scale must be > min_scale")

    scale = float(np.clip(initial_scale, min_scale, max_scale))
    step = float(initial_step)
    history = []

    def _rmse_z_for(test_scale: float) -> float:
        sim_times, sim_positions = simulate_with_scale(float(test_scale))
        metrics = compare_sim_real(sim_times, sim_positions, ref_times, ref_positions)
        return float(metrics["rmse_z"])

    best_rmse = _rmse_z_for(scale)
    history.append((scale, best_rmse))
    converged = False

    for i in range(1, max_iterations + 1):
        candidates = [
            float(np.clip(scale - step, min_scale, max_scale)),
            scale,
            float(np.clip(scale + step, min_scale, max_scale)),
        ]
        candidates = sorted(set(candidates))

        candidate_scores = []
        for c in candidates:
            rmse = _rmse_z_for(c)
            candidate_scores.append((c, rmse))

        candidate_scores.sort(key=lambda item: item[1])
        new_scale, new_rmse = candidate_scores[0]
        history.append((new_scale, new_rmse))

        if abs(best_rmse - new_rmse) < convergence_tol:
            scale = new_scale
            best_rmse = new_rmse
            converged = True
            return WindAutoTuneResult(
                best_scale=scale,
                best_rmse_z=best_rmse,
                iterations=i,
                converged=converged,
                history=tuple(history),
            )

        if new_rmse < best_rmse:
            scale = new_scale
            best_rmse = new_rmse
        else:
            step *= 0.5
            if step < 1e-4:
                break

    return WindAutoTuneResult(
        best_scale=scale,
        best_rmse_z=best_rmse,
        iterations=max_iterations,
        converged=converged,
        history=tuple(history),
    )


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
