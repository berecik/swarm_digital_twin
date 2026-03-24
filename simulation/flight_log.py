"""
Flight Log Parser - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Parse Ardupilot flight logs (.bin, .csv) for digital twin validation.
Based on Valencia et al. (2025) validation methodology.
"""

import numpy as np
import csv
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from pathlib import Path


@dataclass
class FlightLog:
    """Parsed flight log data in local NED frame."""
    timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    positions: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    attitudes: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    airspeeds: np.ndarray = field(default_factory=lambda: np.array([]))
    throttle: np.ndarray = field(default_factory=lambda: np.array([]))
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    origin_alt: float = 0.0

    @staticmethod
    def from_csv(path: str, lat_col: str = "lat", lon_col: str = "lon",
                 alt_col: str = "alt", time_col: str = "time",
                 roll_col: str = "roll", pitch_col: str = "pitch",
                 yaw_col: str = "yaw") -> 'FlightLog':
        """Parse exported CSV log.

        Expects columns for time, lat, lon, alt, and optionally
        roll, pitch, yaw, airspeed, throttle.
        """
        data: Dict[str, List[float]] = {
            "time": [], "lat": [], "lon": [], "alt": [],
            "roll": [], "pitch": [], "yaw": [],
            "airspeed": [], "throttle": [],
        }
        col_map = {
            time_col: "time", lat_col: "lat", lon_col: "lon", alt_col: "alt",
            roll_col: "roll", pitch_col: "pitch", yaw_col: "yaw",
        }

        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for csv_col, key in col_map.items():
                    if csv_col in row:
                        try:
                            data[key].append(float(row[csv_col]))
                        except (ValueError, TypeError):
                            data[key].append(0.0)
                for opt in ("airspeed", "throttle"):
                    if opt in row:
                        try:
                            data[opt].append(float(row[opt]))
                        except (ValueError, TypeError):
                            data[opt].append(0.0)

        if not data["lat"]:
            return FlightLog()

        origin_lat = data["lat"][0]
        origin_lon = data["lon"][0]
        origin_alt = data["alt"][0]

        positions = _gps_to_ned(
            np.array(data["lat"]), np.array(data["lon"]),
            np.array(data["alt"]), origin_lat, origin_lon, origin_alt,
        )

        attitudes = np.column_stack([
            np.deg2rad(data["roll"]) if data["roll"] else np.zeros(len(data["lat"])),
            np.deg2rad(data["pitch"]) if data["pitch"] else np.zeros(len(data["lat"])),
            np.deg2rad(data["yaw"]) if data["yaw"] else np.zeros(len(data["lat"])),
        ])

        return FlightLog(
            timestamps=np.array(data["time"]),
            positions=positions,
            attitudes=attitudes,
            airspeeds=np.array(data["airspeed"]) if data["airspeed"] else np.array([]),
            throttle=np.array(data["throttle"]) if data["throttle"] else np.array([]),
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            origin_alt=origin_alt,
        )

    def get_trajectory(self) -> np.ndarray:
        """Return Nx3 position array in local NED."""
        return self.positions.copy()

    def get_wind_profile(self) -> np.ndarray:
        """Estimate wind perturbation from altitude deviations (paper method).

        Returns Nx2 array: [[t0, wind_estimate_0], [t1, wind_estimate_1], ...]
        Wind is estimated from altitude rate deviations from expected climb/descent.
        """
        if len(self.timestamps) < 2:
            return np.array([[0.0, 0.0]])

        dt = np.diff(self.timestamps)
        alt = self.positions[:, 2]  # down axis in NED, so negate for altitude
        alt_rate = np.diff(-alt) / dt

        # Smooth altitude rate
        kernel_size = min(11, len(alt_rate))
        if kernel_size > 1:
            kernel = np.ones(kernel_size) / kernel_size
            alt_rate_smooth = np.convolve(alt_rate, kernel, mode="same")
        else:
            alt_rate_smooth = alt_rate

        # Wind estimate = deviation from smooth trend
        wind_estimate = np.abs(alt_rate - alt_rate_smooth)

        return np.column_stack([
            self.timestamps[:-1],
            wind_estimate,
        ])


def _gps_to_ned(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray,
                origin_lat: float, origin_lon: float,
                origin_alt: float) -> np.ndarray:
    """Convert GPS coordinates to local NED frame."""
    R_EARTH = 6371000.0  # m
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    origin_lat_rad = np.deg2rad(origin_lat)
    origin_lon_rad = np.deg2rad(origin_lon)

    north = R_EARTH * (lat_rad - origin_lat_rad)
    east = R_EARTH * np.cos(origin_lat_rad) * (lon_rad - origin_lon_rad)
    down = -(alt - origin_alt)

    return np.column_stack([north, east, down])
