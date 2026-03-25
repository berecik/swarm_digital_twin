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


    def get_wind_profile_3d(self, window: int = 11) -> np.ndarray:
        """Estimate 3D wind perturbation from lateral and vertical trajectory deviations.

        Extends the Z-only `get_wind_profile()` by analyzing X/Y deviations
        between consecutive waypoint segments to estimate horizontal wind
        components. Paper Section 3.4 notes this as future work.

        Returns Nx4 array: [[t, wind_x, wind_y, wind_z], ...]
        Wind components are in local NED frame (m/s).
        """
        if len(self.timestamps) < 3:
            return np.array([[0.0, 0.0, 0.0, 0.0]])

        dt = np.diff(self.timestamps)
        dt = np.maximum(dt, 1e-6)  # avoid division by zero

        # Compute velocity from positions
        velocities = np.diff(self.positions, axis=0) / dt[:, np.newaxis]

        # Smooth velocities to get expected (commanded) trajectory
        kernel_size = min(window, len(velocities))
        if kernel_size > 1 and kernel_size % 2 == 0:
            kernel_size -= 1
        if kernel_size > 1:
            kernel = np.ones(kernel_size) / kernel_size
            vel_smooth = np.column_stack([
                np.convolve(velocities[:, i], kernel, mode="same")
                for i in range(3)
            ])
        else:
            vel_smooth = velocities.copy()

        # Wind estimate = deviation of actual velocity from smoothed trend
        # Positive deviation in a direction implies wind pushing the drone that way
        wind_deviation = velocities - vel_smooth

        # Scale by a damping factor: the controller partially compensates for wind,
        # so raw velocity deviation underestimates true wind. Use controller
        # bandwidth heuristic (factor ~2-3x for typical PID response).
        wind_estimate = wind_deviation * 2.0

        return np.column_stack([
            self.timestamps[:-1],
            wind_estimate,
        ])

    @staticmethod
    def from_bin(path: str) -> 'FlightLog':
        """Parse ArduPilot DataFlash binary log (.bin).

        Extracts GPS, ATT, RCOU, CTUN message types for positions,
        attitudes, throttle, and airspeed.
        """
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Log file not found: {path}")

        data = p.read_bytes()
        messages = _parse_dataflash_messages(data)

        gps_msgs = messages.get("GPS", [])
        att_msgs = messages.get("ATT", [])
        rcou_msgs = messages.get("RCOU", [])
        ctun_msgs = messages.get("CTUN", [])

        if not gps_msgs:
            return FlightLog()

        # Extract GPS data
        gps_times = np.array([m.get("TimeUS", 0) / 1e6 for m in gps_msgs])
        gps_lat = np.array([m.get("Lat", 0.0) for m in gps_msgs])
        gps_lon = np.array([m.get("Lng", 0.0) for m in gps_msgs])
        gps_alt = np.array([m.get("Alt", 0.0) for m in gps_msgs])

        origin_lat = float(gps_lat[0])
        origin_lon = float(gps_lon[0])
        origin_alt = float(gps_alt[0])

        positions = _gps_to_ned(gps_lat, gps_lon, gps_alt,
                                origin_lat, origin_lon, origin_alt)

        # Extract attitude data (interpolate to GPS timestamps)
        attitudes = np.zeros((len(gps_times), 3))
        if att_msgs:
            att_times = np.array([m.get("TimeUS", 0) / 1e6 for m in att_msgs])
            roll = np.array([m.get("Roll", 0.0) for m in att_msgs])
            pitch = np.array([m.get("Pitch", 0.0) for m in att_msgs])
            yaw = np.array([m.get("Yaw", 0.0) for m in att_msgs])
            attitudes[:, 0] = np.interp(gps_times, att_times, np.deg2rad(roll))
            attitudes[:, 1] = np.interp(gps_times, att_times, np.deg2rad(pitch))
            attitudes[:, 2] = np.interp(gps_times, att_times, np.deg2rad(yaw))

        # Extract throttle (RCOU C1 = throttle channel)
        throttle_arr = np.array([])
        if rcou_msgs:
            rcou_times = np.array([m.get("TimeUS", 0) / 1e6 for m in rcou_msgs])
            rcou_c3 = np.array([m.get("C3", 0.0) for m in rcou_msgs])
            # Normalize PWM (1000-2000) to percentage (0-100)
            throttle_pct = np.clip((rcou_c3 - 1000.0) / 10.0, 0.0, 100.0)
            throttle_arr = np.interp(gps_times, rcou_times, throttle_pct)

        # Extract airspeed from CTUN
        airspeeds = np.array([])
        if ctun_msgs:
            ctun_times = np.array([m.get("TimeUS", 0) / 1e6 for m in ctun_msgs])
            aspd = np.array([m.get("As", m.get("Aspd", 0.0)) for m in ctun_msgs])
            airspeeds = np.interp(gps_times, ctun_times, aspd)

        return FlightLog(
            timestamps=gps_times,
            positions=positions,
            attitudes=attitudes,
            airspeeds=airspeeds,
            throttle=throttle_arr,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            origin_alt=origin_alt,
        )

    def get_elevator(self) -> np.ndarray:
        """Extract elevator deflection estimate from pitch attitude rate."""
        if len(self.timestamps) < 2:
            return np.array([])
        dt = np.diff(self.timestamps)
        pitch = self.attitudes[:, 1]
        pitch_rate = np.diff(pitch) / np.maximum(dt, 1e-6)
        return np.append(pitch_rate, pitch_rate[-1])

    def extract_waypoints(self, speed_threshold: float = 0.5,
                          min_dwell: float = 2.0) -> List[np.ndarray]:
        """Extract waypoints from trajectory by detecting dwell points.

        A waypoint is where the drone hovers (speed < threshold) for at least min_dwell seconds.
        """
        if len(self.timestamps) < 2:
            return []
        dt = np.diff(self.timestamps)
        velocities = np.diff(self.positions, axis=0) / dt[:, np.newaxis]
        speeds = np.linalg.norm(velocities, axis=1)

        waypoints: List[np.ndarray] = []
        dwell_start = None
        for i in range(len(speeds)):
            if speeds[i] < speed_threshold:
                if dwell_start is None:
                    dwell_start = i
            else:
                if dwell_start is not None:
                    dwell_time = self.timestamps[i] - self.timestamps[dwell_start]
                    if dwell_time >= min_dwell:
                        mid = (dwell_start + i) // 2
                        waypoints.append(self.positions[mid].copy())
                    dwell_start = None
        # Final dwell check
        if dwell_start is not None:
            dwell_time = self.timestamps[-1] - self.timestamps[dwell_start]
            if dwell_time >= min_dwell:
                mid = (dwell_start + len(speeds)) // 2
                waypoints.append(self.positions[min(mid, len(self.positions) - 1)].copy())

        return waypoints


def _parse_dataflash_messages(data: bytes) -> Dict[str, list]:
    """Parse ArduPilot DataFlash binary format.

    DataFlash format:
    - Header: 0xA3 0x95
    - Message ID: 1 byte
    - FMT messages (id=128) define message formats
    - Data messages follow their FMT definitions
    """
    messages: Dict[str, list] = {}
    formats: Dict[int, dict] = {}
    pos = 0
    length = len(data)

    while pos < length - 2:
        # Find header
        if data[pos] != 0xA3 or data[pos + 1] != 0x95:
            pos += 1
            continue

        if pos + 3 > length:
            break

        msg_id = data[pos + 2]

        if msg_id == 128:
            # FMT message: defines format for other message types
            if pos + 89 > length:
                break
            fmt_type = data[pos + 3]
            fmt_length = data[pos + 4]
            fmt_name = data[pos + 5:pos + 9].decode("ascii", errors="ignore").rstrip("\x00")
            fmt_format = data[pos + 9:pos + 25].decode("ascii", errors="ignore").rstrip("\x00")
            fmt_labels = data[pos + 25:pos + 89].decode("ascii", errors="ignore").rstrip("\x00")
            labels = [l.strip() for l in fmt_labels.split(",")]
            formats[fmt_type] = {
                "name": fmt_name,
                "length": fmt_length,
                "format": fmt_format,
                "labels": labels,
            }
            pos += 89
            continue

        if msg_id not in formats:
            pos += 3
            continue

        fmt = formats[msg_id]
        msg_len = fmt["length"]
        if pos + msg_len > length:
            break

        # Parse message fields
        payload = data[pos + 3:pos + msg_len]
        msg_data = _decode_dataflash_payload(payload, fmt["format"], fmt["labels"])
        if msg_data:
            name = fmt["name"]
            if name not in messages:
                messages[name] = []
            messages[name].append(msg_data)

        pos += msg_len

    return messages


def _decode_dataflash_payload(payload: bytes, fmt_str: str,
                               labels: List[str]) -> Optional[Dict]:
    """Decode a DataFlash message payload using format string.

    Format chars: B=uint8, H=uint16, I=uint32, Q=uint64,
    b=int8, h=int16, i=int32, q=int64, f=float, d=double,
    n=char[4], N=char[16], Z=char[64], c=int16*100, C=uint16*100,
    e=int32*100, E=uint32*100, L=int32 (lat/lng 1e-7)
    """
    struct_map = {
        'b': ('b', 1), 'B': ('B', 1), 'h': ('<h', 2), 'H': ('<H', 2),
        'i': ('<i', 4), 'I': ('<I', 4), 'q': ('<q', 8), 'Q': ('<Q', 8),
        'f': ('<f', 4), 'd': ('<d', 8),
        'c': ('<h', 2), 'C': ('<H', 2), 'e': ('<i', 4), 'E': ('<I', 4),
        'L': ('<i', 4), 'M': ('B', 1), 'n': ('4s', 4), 'N': ('16s', 16),
        'Z': ('64s', 64),
    }
    scale_map = {'c': 0.01, 'C': 0.01, 'e': 0.01, 'E': 0.01, 'L': 1e-7}

    result = {}
    offset = 0
    label_idx = 0

    for ch in fmt_str:
        if ch not in struct_map:
            continue
        fmt_code, size = struct_map[ch]
        if offset + size > len(payload):
            break
        try:
            val = struct.unpack_from(fmt_code, payload, offset)[0]
        except struct.error:
            break
        if isinstance(val, bytes):
            val = val.decode("ascii", errors="ignore").rstrip("\x00")
        elif ch in scale_map:
            val = val * scale_map[ch]
        if label_idx < len(labels):
            result[labels[label_idx]] = val
        label_idx += 1
        offset += size

    return result if result else None


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
