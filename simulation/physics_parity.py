"""
Physics Parity Contract — verify that Gazebo/K8s and standalone physics
produce equivalent results for the same airframe and mission.

This module provides:
- ``ParityContract``: parameter-level comparison between DroneParams and SDF
- ``compare_trajectories()``: RMSE-based trajectory comparison
- ``check_timing_determinism()``: control loop jitter analysis
- ``ParityReport``: aggregated pass/fail verdict

Usage::

    # Compare parameters
    contract = ParityContract.from_sdf("gazebo/models/x500/model.sdf")
    mismatches = contract.check()

    # Compare trajectories
    result = compare_trajectories(standalone_pos, k8s_pos)
    assert result.passed

    # Check timing
    timing = check_timing_determinism(timestamps, expected_dt=0.02)
    assert timing.passed
"""

import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_SIM_DIR = Path(__file__).resolve().parent
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from drone_physics import (
    GRAVITY,
    Atmosphere,
    DroneParams,
    make_holybro_x500,
    make_irs4_quadrotor,
)
from validation import ValidationResult, compute_rmse


# ── Parity thresholds ────────────────────────────────────────────────────

POSITION_RMSE_XY_MAX = 2.0    # metres
POSITION_RMSE_Z_MAX = 1.0     # metres
ATTITUDE_RMSE_MAX = 5.0        # degrees
ENERGY_DELTA_MAX = 0.15        # 15% relative
TIMING_JITTER_MAX_MS = 5.0     # milliseconds at 50 Hz


# ── Parameter parity ────────────────────────────────────────────────────


@dataclass
class ParamMismatch:
    """One parameter that differs between standalone and Gazebo."""
    name: str
    standalone_value: float
    gazebo_value: float
    tolerance: float

    @property
    def delta(self) -> float:
        return abs(self.standalone_value - self.gazebo_value)

    @property
    def passed(self) -> bool:
        return self.delta <= self.tolerance


@dataclass
class ParityContract:
    """Parameter-level comparison between DroneParams and Gazebo SDF."""
    params: DroneParams
    sdf_values: Dict[str, float]
    mismatches: List[ParamMismatch] = field(default_factory=list)

    @classmethod
    def from_sdf(cls, sdf_path: str,
                 params: Optional[DroneParams] = None) -> "ParityContract":
        """Parse a Gazebo SDF model and compare against DroneParams."""
        if params is None:
            params = make_holybro_x500()

        sdf_vals = _parse_sdf_physics(sdf_path)
        contract = cls(params=params, sdf_values=sdf_vals)
        contract._check()
        return contract

    @classmethod
    def from_preset(cls, preset: str = "x500") -> "ParityContract":
        """Build contract from a known preset + its SDF file."""
        project_root = _SIM_DIR.parent
        if preset == "x500":
            params = make_holybro_x500()
            sdf = str(project_root / "gazebo" / "models" / "x500" / "model.sdf")
        elif preset == "irs4":
            params = make_irs4_quadrotor()
            sdf = str(project_root / "gazebo" / "models" / "x500" / "model.sdf")
        else:
            raise ValueError(f"Unknown preset: {preset}")
        return cls.from_sdf(sdf, params)

    def _check(self) -> None:
        """Compare each physics parameter."""
        p = self.params
        s = self.sdf_values

        I = p.inertia
        aero = p.aero
        atmo = p.atmo
        checks = [
            ("mass", p.mass, s.get("mass", 0.0), 0.01),
            ("Ixx", float(I[0, 0]), s.get("ixx", 0.0), 0.001),
            ("Iyy", float(I[1, 1]), s.get("iyy", 0.0), 0.001),
            ("Izz", float(I[2, 2]), s.get("izz", 0.0), 0.001),
        ]
        if aero is not None:
            checks.extend([
                ("C_D", aero.C_D, s.get("cda", 0.0), 0.01),
                ("reference_area", aero.reference_area, s.get("area", 0.0), 0.001),
            ])
        if atmo is not None:
            checks.append(
                ("air_density", atmo.rho, s.get("air_density", 1.225), 0.01))
        checks.append(
            ("gravity", GRAVITY, s.get("gravity", 9.81), 0.01))
        self.mismatches = []
        for name, standalone, gazebo, tol in checks:
            m = ParamMismatch(name, standalone, gazebo, tol)
            if not m.passed:
                self.mismatches.append(m)

    @property
    def passed(self) -> bool:
        return len(self.mismatches) == 0

    def summary(self) -> str:
        if self.passed:
            return "ParityContract: PASS — all parameters match"
        lines = ["ParityContract: FAIL — mismatches:"]
        for m in self.mismatches:
            lines.append(
                f"  {m.name}: standalone={m.standalone_value:.6f} "
                f"gazebo={m.gazebo_value:.6f} delta={m.delta:.6f} "
                f"(tol={m.tolerance})")
        return "\n".join(lines)


def _parse_sdf_physics(sdf_path: str) -> Dict[str, float]:
    """Extract physics parameters from a Gazebo SDF model file."""
    vals: Dict[str, float] = {}
    if not os.path.isfile(sdf_path):
        return vals

    with open(sdf_path, "r", encoding="utf-8") as f:
        content = f.read()

    def _extract(tag: str) -> Optional[float]:
        m = re.search(rf"<{tag}>\s*([-\d.eE+]+)\s*</{tag}>", content)
        return float(m.group(1)) if m else None

    for key in ("mass", "ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
        v = _extract(key)
        if v is not None:
            vals[key] = v

    # LiftDragPlugin parameters
    for key in ("cda", "cla", "cma", "area", "air_density", "a0", "alpha_stall"):
        v = _extract(key)
        if v is not None:
            vals[key] = v

    # Gravity (from <gravity> element)
    grav_match = re.search(
        r"<gravity>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*</gravity>",
        content)
    if grav_match:
        vals["gravity"] = abs(float(grav_match.group(3)))  # Z component
    else:
        vals["gravity"] = 9.81  # default

    return vals


# ── Trajectory parity ────────────────────────────────────────────────────


@dataclass
class TrajectoryParityResult:
    """Result of comparing two trajectory datasets."""
    rmse: ValidationResult
    position_rmse_xy: float
    position_rmse_z: float
    passed: bool
    thresholds: Dict[str, float] = field(default_factory=dict)

    def summary(self) -> str:
        verdict = "PASS" if self.passed else "FAIL"
        return (
            f"TrajectoryParity: {verdict}\n"
            f"  RMSE XY: {self.position_rmse_xy:.4f} m "
            f"(max {self.thresholds.get('xy', POSITION_RMSE_XY_MAX)})\n"
            f"  RMSE Z:  {self.position_rmse_z:.4f} m "
            f"(max {self.thresholds.get('z', POSITION_RMSE_Z_MAX)})\n"
            f"  Total:   {self.rmse.rmse_total:.4f} m\n"
            f"  Points:  {self.rmse.n_points}")


def compare_trajectories(
    standalone_positions: np.ndarray,
    k8s_positions: np.ndarray,
    max_rmse_xy: float = POSITION_RMSE_XY_MAX,
    max_rmse_z: float = POSITION_RMSE_Z_MAX,
) -> TrajectoryParityResult:
    """Compare two Nx3 position arrays and check against thresholds."""
    rmse = compute_rmse(standalone_positions, k8s_positions)
    rmse_xy = float(np.sqrt(rmse.rmse_x**2 + rmse.rmse_y**2))
    passed = (rmse_xy <= max_rmse_xy and rmse.rmse_z <= max_rmse_z)
    return TrajectoryParityResult(
        rmse=rmse,
        position_rmse_xy=rmse_xy,
        position_rmse_z=rmse.rmse_z,
        passed=passed,
        thresholds={"xy": max_rmse_xy, "z": max_rmse_z},
    )


# ── Timing determinism ──────────────────────────────────────────────────


@dataclass
class TimingResult:
    """Result of control loop timing analysis."""
    expected_dt: float
    mean_dt: float
    std_dt: float
    max_jitter_ms: float
    p99_jitter_ms: float
    passed: bool

    def summary(self) -> str:
        verdict = "PASS" if self.passed else "FAIL"
        return (
            f"TimingDeterminism: {verdict}\n"
            f"  Expected dt: {self.expected_dt*1000:.1f} ms\n"
            f"  Mean dt:     {self.mean_dt*1000:.2f} ms\n"
            f"  Std dt:      {self.std_dt*1000:.2f} ms\n"
            f"  Max jitter:  {self.max_jitter_ms:.2f} ms\n"
            f"  P99 jitter:  {self.p99_jitter_ms:.2f} ms")


def check_timing_determinism(
    timestamps: np.ndarray,
    expected_dt: float = 0.02,
    max_jitter_ms: float = TIMING_JITTER_MAX_MS,
) -> TimingResult:
    """Analyze control loop timing from a sequence of timestamps."""
    if len(timestamps) < 2:
        return TimingResult(
            expected_dt=expected_dt, mean_dt=0.0, std_dt=0.0,
            max_jitter_ms=0.0, p99_jitter_ms=0.0, passed=True)

    dts = np.diff(timestamps)
    jitters_ms = np.abs(dts - expected_dt) * 1000.0
    max_j = float(np.max(jitters_ms))
    p99_j = float(np.percentile(jitters_ms, 99))

    return TimingResult(
        expected_dt=expected_dt,
        mean_dt=float(np.mean(dts)),
        std_dt=float(np.std(dts)),
        max_jitter_ms=max_j,
        p99_jitter_ms=p99_j,
        passed=(p99_j <= max_jitter_ms),
    )


# ── Telemetry truth pipeline ────────────────────────────────────────────


@dataclass
class TelemetryTruthRecord:
    """One synchronized truth record for post-run validation."""
    t: float
    position: np.ndarray       # ENU [m]
    velocity: np.ndarray       # ENU [m/s]
    euler: Tuple[float, float, float]  # (roll, pitch, yaw) [rad]
    thrust_pct: float
    battery_voltage: float
    battery_soc: float
    source: str                # "standalone" or "k8s"


def extract_truth_from_records(records, source: str = "standalone"
                               ) -> List[TelemetryTruthRecord]:
    """Convert SimRecord list to truth records for parity comparison."""
    truth = []
    for r in records:
        truth.append(TelemetryTruthRecord(
            t=r.t,
            position=r.position.copy(),
            velocity=r.velocity.copy(),
            euler=tuple(r.euler),
            thrust_pct=min(100.0, 100.0 * r.thrust / 40.0),
            battery_voltage=getattr(r, "battery_voltage_v", 12.6),
            battery_soc=getattr(r, "battery_soc", 1.0),
            source=source,
        ))
    return truth


def save_truth_csv(records: List[TelemetryTruthRecord], path: str) -> None:
    """Write truth records to CSV for post-run comparison."""
    import csv
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "vx", "vy", "vz",
                     "roll", "pitch", "yaw", "thrust_pct",
                     "voltage", "soc", "source"])
        for r in records:
            w.writerow([
                f"{r.t:.6f}",
                f"{r.position[0]:.6f}", f"{r.position[1]:.6f}",
                f"{r.position[2]:.6f}",
                f"{r.velocity[0]:.6f}", f"{r.velocity[1]:.6f}",
                f"{r.velocity[2]:.6f}",
                f"{r.euler[0]:.9f}", f"{r.euler[1]:.9f}",
                f"{r.euler[2]:.9f}",
                f"{r.thrust_pct:.3f}",
                f"{r.battery_voltage:.4f}", f"{r.battery_soc:.4f}",
                r.source,
            ])
