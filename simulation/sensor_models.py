"""
Sensor Noise Models - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Paper-aligned sensor perturbation models (Valencia et al., Section 2.4):
- GPS: quantization + white noise + slow bias drift
- IMU: white noise density + bias random walk
- Barometer: quantization + low-pass lag + slow bias drift
"""

from dataclasses import dataclass, field

import numpy as np


EARTH_RADIUS_M = 6378137.0


@dataclass
class GPSNoise:
    """GPS noise model with white noise, drift, and quantization."""

    horizontal_sigma_m: float = 2.5
    vertical_sigma_m: float = 5.0
    quantization_deg: float = 1e-7
    drift_m_per_hour: float = 0.5
    rng_seed: int | None = None
    _rng: np.random.Generator = field(init=False, repr=False)
    _bias_m: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.rng_seed)

    def _update_bias(self, dt: float) -> None:
        if dt <= 0.0:
            return
        sigma_per_sec = self.drift_m_per_hour / 3600.0
        self._bias_m += self._rng.normal(0.0, sigma_per_sec * np.sqrt(dt), size=3)

    def apply(self, true_lat: float, true_lon: float, true_alt: float,
              dt: float = 0.02) -> tuple[float, float, float]:
        """Apply GPS noise to geodetic position.

        Args:
            true_lat: latitude [deg]
            true_lon: longitude [deg]
            true_alt: altitude [m]
            dt: sample period [s]
        """
        self._update_bias(dt)

        lat_rad = np.deg2rad(true_lat)
        meters_per_deg_lat = np.pi * EARTH_RADIUS_M / 180.0
        meters_per_deg_lon = meters_per_deg_lat * max(np.cos(lat_rad), 1e-6)

        white_xy = self._rng.normal(0.0, self.horizontal_sigma_m, size=2)
        white_z = self._rng.normal(0.0, self.vertical_sigma_m)

        noisy_lat = true_lat + (white_xy[0] + self._bias_m[0]) / meters_per_deg_lat
        noisy_lon = true_lon + (white_xy[1] + self._bias_m[1]) / meters_per_deg_lon
        noisy_alt = true_alt + white_z + self._bias_m[2]

        noisy_lat = np.round(noisy_lat / self.quantization_deg) * self.quantization_deg
        noisy_lon = np.round(noisy_lon / self.quantization_deg) * self.quantization_deg

        return float(noisy_lat), float(noisy_lon), float(noisy_alt)

    def apply_local(self, position_m: np.ndarray, dt: float = 0.02) -> np.ndarray:
        """Apply GPS noise in local frame (meters).

        For use in simulation bridges that work in local NED/ENU coordinates
        rather than geodetic (lat/lon/alt).  Applies the same white noise,
        bias drift, and sigma model as ``apply()``, but directly in meters.

        Args:
            position_m: True position as [x, y, z] in meters.
            dt: Sample period [s].

        Returns:
            Noisy position as [x, y, z] in meters.
        """
        self._update_bias(dt)
        pos = np.asarray(position_m, dtype=float).copy()
        pos[0] += self._rng.normal(0.0, self.horizontal_sigma_m) + self._bias_m[0]
        pos[1] += self._rng.normal(0.0, self.horizontal_sigma_m) + self._bias_m[1]
        pos[2] += self._rng.normal(0.0, self.vertical_sigma_m) + self._bias_m[2]
        return pos


@dataclass
class IMUNoise:
    """IMU noise model with density-based white noise and bias random walk."""

    accel_noise_density_ug: float = 400.0
    gyro_noise_density_dps: float = 0.01
    accel_bias_rw_mps2_per_sqrt_s: float = 0.001
    gyro_bias_rw_radps_per_sqrt_s: float = np.deg2rad(0.005)
    rng_seed: int | None = None
    _rng: np.random.Generator = field(init=False, repr=False)
    _accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)
    _gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.rng_seed)

    def _update_bias(self, dt: float) -> None:
        if dt <= 0.0:
            return
        self._accel_bias += self._rng.normal(
            0.0,
            self.accel_bias_rw_mps2_per_sqrt_s * np.sqrt(dt),
            size=3,
        )
        self._gyro_bias += self._rng.normal(
            0.0,
            self.gyro_bias_rw_radps_per_sqrt_s * np.sqrt(dt),
            size=3,
        )

    def apply_accel(self, true_accel: np.ndarray, dt: float) -> np.ndarray:
        """Apply accelerometer noise to true acceleration [m/s²]."""
        self._update_bias(dt)
        density_mps2 = self.accel_noise_density_ug * 1e-6 * 9.80665
        sigma = density_mps2 / np.sqrt(max(dt, 1e-12))
        white = self._rng.normal(0.0, sigma, size=3)
        return true_accel + self._accel_bias + white

    def apply_gyro(self, true_gyro: np.ndarray, dt: float) -> np.ndarray:
        """Apply gyroscope noise to true angular rate [rad/s]."""
        self._update_bias(dt)
        density_radps = np.deg2rad(self.gyro_noise_density_dps)
        sigma = density_radps / np.sqrt(max(dt, 1e-12))
        white = self._rng.normal(0.0, sigma, size=3)
        return true_gyro + self._gyro_bias + white


@dataclass
class BaroNoise:
    """Barometer pressure perturbation model with lag and drift."""

    quantization_hpa: float = 0.12
    drift_hpa_per_hour: float = 1.0
    lag_time_constant_s: float = 0.35
    white_sigma_hpa: float = 0.03
    rng_seed: int | None = None
    _rng: np.random.Generator = field(init=False, repr=False)
    _bias_hpa: float = field(default=0.0, init=False, repr=False)
    _filtered_hpa: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.rng_seed)

    def apply(self, true_pressure_hpa: float, dt: float = 0.02) -> float:
        """Apply barometer noise to pressure [hPa]."""
        if dt > 0.0:
            sigma_per_sec = self.drift_hpa_per_hour / 3600.0
            self._bias_hpa += float(self._rng.normal(0.0, sigma_per_sec * np.sqrt(dt)))

        raw_hpa = true_pressure_hpa + self._bias_hpa + float(
            self._rng.normal(0.0, self.white_sigma_hpa)
        )

        if self._filtered_hpa is None:
            self._filtered_hpa = raw_hpa
        else:
            alpha = dt / (self.lag_time_constant_s + dt) if dt > 0.0 else 1.0
            self._filtered_hpa = (1.0 - alpha) * self._filtered_hpa + alpha * raw_hpa

        quant = self.quantization_hpa
        return float(np.round(self._filtered_hpa / quant) * quant)
