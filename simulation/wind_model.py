"""
Wind Perturbation Model - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Wind perturbation model following Valencia et al. (2025), Eq. 5-7.
Supports: no wind, constant wind, Dryden turbulence, from-log replay,
spatially varying gradient, deterministic seeded turbulence, and a
manifest-driven profile loader.
"""

from pathlib import Path
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class WindField:
    """Wind perturbation model (paper Eq. 5-7).

    Wind types:
      - "none": no wind (baseline)
      - "constant": uniform wind vector
      - "dryden": MIL-F-8785C Dryden turbulence model
      - "from_log": replay altitude-derived perturbation from real data (1D)
      - "from_log_3d": replay 3D wind profile from lateral deviation analysis

    Spatial gradient: when ``spatial_gradient`` is provided
    as a (3, 3) matrix, ``get_wind_velocity`` adds ``spatial_gradient @
    position`` to the base wind, giving multi-drone simulations spatially
    varying disturbances. Rows index the wind component (E, N, U); columns
    index the position component the partial derivative is taken with
    respect to. Units: (m/s) per metre.

    Determinism: pass ``seed`` to make stochastic profiles
    (currently Dryden) reproducible. Two ``WindField`` instances with the
    same parameters and seed produce bit-identical timeseries.
    """
    wind_speed: float = 0.0
    wind_direction: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0])
    )
    gust_intensity: float = 0.0
    turbulence_type: str = "none"
    altitude_profile: Optional[np.ndarray] = None  # Nx2 array: [time, wind_speed]
    wind_profile_3d: Optional[np.ndarray] = None  # Nx4 array: [time, wx, wy, wz]
    force_scale: float = 1.0
    seed: Optional[int] = None
    spatial_gradient: Optional[np.ndarray] = None  # (3, 3) matrix
    _dryden_state: np.ndarray = field(
        default_factory=lambda: np.zeros(3), repr=False
    )
    _rng: Optional[np.random.Generator] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        # A per-instance RNG keeps stochastic profiles independent of the
        # global numpy RNG; without this, two `WindField`s would share state
        # and reproducibility would only hold when no other code touches
        # `np.random` between samples.
        self._rng = np.random.default_rng(self.seed)

    def get_wind_velocity(self, t: float, position: np.ndarray) -> np.ndarray:
        """Return wind velocity vector [m/s] at given time and position."""
        base = self._base_wind(t, position)
        if self.spatial_gradient is not None:
            base = base + np.asarray(self.spatial_gradient) @ np.asarray(position)
        return base

    def _base_wind(self, t: float, position: np.ndarray) -> np.ndarray:
        if self.turbulence_type == "none":
            return np.zeros(3)

        if self.turbulence_type == "constant":
            d = self.wind_direction
            norm = np.linalg.norm(d)
            if norm < 1e-8:
                return np.zeros(3)
            return self.wind_speed * (d / norm)

        if self.turbulence_type == "dryden":
            return self._dryden_wind(t, position)

        if self.turbulence_type == "from_log":
            return self._from_log_wind(t)

        if self.turbulence_type == "from_log_3d":
            return self._from_log_3d_wind(t)

        return np.zeros(3)

    def get_force(self, t: float, position: np.ndarray,
                  aero, rho: float) -> np.ndarray:
        """Compute wind perturbation force in world frame (paper Eq. 5-7).

        Eq. 5: Delta_F_D = 0.5 * rho * A * C_D(alpha) * V_wind^2
        Eq. 6: Delta_F_L = 0.5 * rho * A * C_L(alpha) * V_wind^2
        Eq. 7: Delta_F_W = F_D + F_L (vector sum)

        For FixedWingAero, C_D and C_L are evaluated at alpha=0 (wind-relative).
        Lift from wind is perpendicular to wind velocity in the world XZ plane.
        """
        V_wind = self.get_wind_velocity(t, position)
        V_mag = np.linalg.norm(V_wind)
        if V_mag < 1e-8:
            return np.zeros(3)

        if aero is not None:
            A = aero.reference_area
            C_D = aero.get_CD(0.0)  # wind-relative AoA ~ 0
            C_L = aero.get_CL(0.0)
        else:
            A = 0.04
            C_D = 1.0
            C_L = 0.0

        V_hat = V_wind / V_mag
        q = 0.5 * rho * A * V_mag**2

        # Eq. 5: drag along wind direction
        F_drag = q * C_D * V_hat

        # Eq. 6: lift perpendicular to wind in XZ plane (world frame)
        F_lift = np.zeros(3)
        if abs(C_L) > 1e-12:
            L_hat = np.array([-V_hat[2], 0.0, V_hat[0]])
            L_norm = np.linalg.norm(L_hat)
            if L_norm > 1e-8:
                L_hat = L_hat / L_norm
                F_lift = q * C_L * L_hat

        # Eq. 7: total wind force with mission-level tuning scale
        return float(self.force_scale) * (F_drag + F_lift)

    def _dryden_wind(self, t: float, position: np.ndarray) -> np.ndarray:
        """Simplified Dryden turbulence (MIL-F-8785C).

        Uses a first-order Markov process driven by white noise.
        Turbulence intensity scales with gust_intensity. Reproducible
        across runs when ``seed`` was set on construction.
        """
        h = max(position[2], 1.0)  # altitude above ground (min 1m)
        # Dryden scale lengths (low altitude, light turbulence)
        L_u = h / (0.177 + 0.000823 * h) ** 1.2
        L_w = h

        # Band-limited white noise from the per-instance generator (seeded
        # via __post_init__ so two WindFields with the same seed are
        # bit-identical regardless of the global numpy RNG state).
        dt = 0.005  # assumed timestep
        noise = self._rng.standard_normal(3) * self.gust_intensity

        # First-order filter: dx/dt = -V/L * x + sqrt(2*V/L) * noise
        V = max(self.wind_speed, 1.0)
        tau = np.array([L_u / V, L_u / V, L_w / V])
        alpha = dt / (tau + dt)
        self._dryden_state = (1 - alpha) * self._dryden_state + alpha * noise

        # Base wind + turbulence
        d = self.wind_direction
        norm = np.linalg.norm(d)
        base = self.wind_speed * (d / norm) if norm > 1e-8 else np.zeros(3)
        return base + self._dryden_state

    def _from_log_wind(self, t: float) -> np.ndarray:
        """Replay wind from altitude profile data.

        altitude_profile is Nx2: [[t0, speed0], [t1, speed1], ...]
        Wind is applied along wind_direction.
        """
        if self.altitude_profile is None or len(self.altitude_profile) == 0:
            return np.zeros(3)

        times = self.altitude_profile[:, 0]
        speeds = self.altitude_profile[:, 1]
        speed = np.interp(t, times, speeds, left=speeds[0], right=speeds[-1])

        d = self.wind_direction
        norm = np.linalg.norm(d)
        if norm < 1e-8:
            return np.zeros(3)
        return speed * (d / norm)

    def _from_log_3d_wind(self, t: float) -> np.ndarray:
        """Replay 3D wind profile from lateral deviation analysis.

        wind_profile_3d is Nx4: [[t, wx, wy, wz], ...]
        Returns interpolated 3D wind velocity vector at time t.
        """
        if self.wind_profile_3d is None or len(self.wind_profile_3d) == 0:
            return np.zeros(3)

        times = self.wind_profile_3d[:, 0]
        wx = np.interp(t, times, self.wind_profile_3d[:, 1],
                        left=self.wind_profile_3d[0, 1], right=self.wind_profile_3d[-1, 1])
        wy = np.interp(t, times, self.wind_profile_3d[:, 2],
                        left=self.wind_profile_3d[0, 2], right=self.wind_profile_3d[-1, 2])
        wz = np.interp(t, times, self.wind_profile_3d[:, 3],
                        left=self.wind_profile_3d[0, 3], right=self.wind_profile_3d[-1, 3])
        return np.array([wx, wy, wz])


# ── Wind profile manifest ────────────────────────────────────────────────────
# Mirrors the terrain manifest pattern: a TOML file is
# the single source of truth for available profiles, plus a small loader
# that turns a named entry into a configured `WindField`.

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WIND_MANIFEST_PATH = (
    _PROJECT_ROOT / "gazebo" / "worlds" / "wind" / "manifest.toml"
)


def _read_wind_manifest(manifest_path: Optional[Path]) -> dict:
    import tomllib
    path = Path(manifest_path) if manifest_path else DEFAULT_WIND_MANIFEST_PATH
    if not path.is_file():
        raise FileNotFoundError(f"wind manifest not found: {path}")
    with open(path, "rb") as f:
        return tomllib.load(f)


def _wind_require(entry: dict, name: str, key: str):
    if key not in entry:
        raise ValueError(f"wind '{name}': missing required field '{key}'")
    return entry[key]


def _build_spatial_gradient(entry: dict) -> Optional[np.ndarray]:
    keys = [
        "gradient_e_e", "gradient_e_n", "gradient_e_u",
        "gradient_n_e", "gradient_n_n", "gradient_n_u",
        "gradient_u_e", "gradient_u_n", "gradient_u_u",
    ]
    if not any(k in entry for k in keys):
        return None
    g = np.zeros((3, 3))
    flat = [float(entry.get(k, 0.0)) for k in keys]
    g[:] = np.asarray(flat).reshape((3, 3))
    return g


def load_wind_profile(name: str,
                      manifest_path: Optional[Path] = None) -> "WindField":
    """Build a :class:`WindField` from a manifest entry.

    Args:
        name: Manifest entry name (top-level table in manifest.toml).
        manifest_path: Override the default manifest location.

    Raises:
        FileNotFoundError: manifest file missing.
        ValueError: unknown name, missing/unknown profile, or missing
            required field.
    """
    manifest = _read_wind_manifest(manifest_path)
    if name not in manifest:
        raise ValueError(
            f"unknown wind profile '{name}'; available: {sorted(manifest)}"
        )
    entry = manifest[name]
    profile = entry.get("profile")
    if profile is None:
        raise ValueError(f"wind '{name}': missing required field 'profile'")
    gradient = _build_spatial_gradient(entry)
    force_scale = float(entry.get("force_scale", 1.0))
    seed = entry.get("seed")
    seed = int(seed) if seed is not None else None

    if profile == "none":
        return WindField(turbulence_type="none", spatial_gradient=gradient,
                         force_scale=force_scale, seed=seed)
    if profile == "constant":
        direction = np.asarray(_wind_require(entry, name, "direction"),
                               dtype=float)
        return WindField(
            wind_speed=float(_wind_require(entry, name, "speed_ms")),
            wind_direction=direction,
            turbulence_type="constant",
            spatial_gradient=gradient,
            force_scale=force_scale,
            seed=seed,
        )
    if profile == "dryden":
        direction = np.asarray(_wind_require(entry, name, "direction"),
                               dtype=float)
        return WindField(
            wind_speed=float(_wind_require(entry, name, "base_speed_ms")),
            wind_direction=direction,
            gust_intensity=float(entry.get("gust_intensity", 0.0)),
            turbulence_type="dryden",
            spatial_gradient=gradient,
            force_scale=force_scale,
            seed=seed,
        )
    if profile == "from_log":
        csv_path = _wind_require(entry, name, "csv_path")
        base = (Path(manifest_path) if manifest_path
                else DEFAULT_WIND_MANIFEST_PATH).parent
        full = (base / csv_path).resolve()
        if not full.is_file():
            raise FileNotFoundError(f"wind '{name}': csv not found: {full}")
        data = np.loadtxt(str(full), delimiter=",", skiprows=1)
        if data.ndim != 2 or data.shape[1] < 2:
            raise ValueError(
                f"wind '{name}': csv must have columns [time, speed]"
            )
        direction = np.asarray(
            entry.get("direction", [1.0, 0.0, 0.0]), dtype=float)
        return WindField(
            wind_direction=direction,
            turbulence_type="from_log",
            altitude_profile=data[:, :2],
            spatial_gradient=gradient,
            force_scale=force_scale,
            seed=seed,
        )
    raise ValueError(f"wind '{name}': unknown profile '{profile}'")


def wind_profile_names(manifest_path: Optional[Path] = None) -> list:
    """Return the list of wind profile names in the manifest (sorted)."""
    return sorted(_read_wind_manifest(manifest_path))
