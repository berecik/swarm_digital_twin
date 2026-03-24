"""
Wind Perturbation Model - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Wind perturbation model following Valencia et al. (2025), Eq. 5-7.
Supports: no wind, constant wind, Dryden turbulence, from-log replay.
"""

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
      - "from_log": replay altitude-derived perturbation from real data
    """
    wind_speed: float = 0.0
    wind_direction: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0])
    )
    gust_intensity: float = 0.0
    turbulence_type: str = "none"
    altitude_profile: Optional[np.ndarray] = None  # Nx2 array: [time, wind_speed]
    _dryden_state: np.ndarray = field(
        default_factory=lambda: np.zeros(3), repr=False
    )

    def get_wind_velocity(self, t: float, position: np.ndarray) -> np.ndarray:
        """Return wind velocity vector [m/s] at given time and position."""
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

        return np.zeros(3)

    def get_force(self, t: float, position: np.ndarray,
                  aero, rho: float) -> np.ndarray:
        """Compute wind perturbation force in world frame (paper Eq. 5-7).

        Delta_F_D = 0.5 * rho * A * C_D * V_wind^2, applied along wind direction.
        """
        V_wind = self.get_wind_velocity(t, position)
        V_mag = np.linalg.norm(V_wind)
        if V_mag < 1e-8:
            return np.zeros(3)

        if aero is not None:
            A = aero.reference_area
            C_D = aero.C_D
        else:
            A = 0.04
            C_D = 1.0

        # Eq. 5: Delta_F_D = 0.5 * rho * A * C_D * V_wind^2
        force_mag = 0.5 * rho * A * C_D * V_mag**2
        return force_mag * (V_wind / V_mag)

    def _dryden_wind(self, t: float, position: np.ndarray) -> np.ndarray:
        """Simplified Dryden turbulence (MIL-F-8785C).

        Uses a first-order Markov process driven by white noise.
        Turbulence intensity scales with gust_intensity.
        """
        h = max(position[2], 1.0)  # altitude above ground (min 1m)
        # Dryden scale lengths (low altitude, light turbulence)
        L_u = h / (0.177 + 0.000823 * h) ** 1.2
        L_w = h

        # Band-limited white noise
        dt = 0.005  # assumed timestep
        noise = np.random.randn(3) * self.gust_intensity

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
