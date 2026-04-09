"""
Live Telemetry Module - Swarm Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>

Handles MAVLink telemetry reception, buffering, and persistence for the
Run-time View.
"""

import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

import numpy as np

# Re-use constants and decoder from mavlink_bridge to avoid duplication where possible,
# but we follow the plan's instruction to keep this module standalone for tests.
from mavlink_bridge import (
    MAVLINK_MSG_ID_HEARTBEAT,
    MAVLINK_MSG_ID_SYS_STATUS,
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_VFR_HUD,
    decode_mavlink_v2,
    CUSTOM_MODE_STABILIZE,
    CUSTOM_MODE_GUIDED,
    CUSTOM_MODE_AUTO,
    CUSTOM_MODE_LAND,
    CUSTOM_MODE_RTL
)

@dataclass
class LiveTelemetrySample:
    t_wall: float = 0.0
    time_boot_ms: int = 0
    pos_enu: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel_enu: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw rad
    throttle_pct: float = 0.0
    airspeed: float = 0.0
    groundspeed: float = 0.0
    alt_msl: float = 0.0
    climb_rate: float = 0.0
    battery_voltage_v: float = 0.0
    battery_current_a: float = 0.0
    battery_remaining_pct: float = 0.0
    flight_mode: str = "UNKNOWN"
    armed: bool = False
    lat_deg: float = 0.0
    lon_deg: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert sample to a JSON-serializable dictionary."""
        return {
            "t_wall": self.t_wall,
            "time_boot_ms": self.time_boot_ms,
            "pos_enu": self.pos_enu.tolist(),
            "vel_enu": self.vel_enu.tolist(),
            "euler": list(self.euler),
            "throttle_pct": self.throttle_pct,
            "airspeed": self.airspeed,
            "groundspeed": self.groundspeed,
            "alt_msl": self.alt_msl,
            "climb_rate": self.climb_rate,
            "battery_voltage_v": self.battery_voltage_v,
            "battery_current_a": self.battery_current_a,
            "battery_remaining_pct": self.battery_remaining_pct,
            "flight_mode": self.flight_mode,
            "armed": self.armed,
            "lat_deg": self.lat_deg,
            "lon_deg": self.lon_deg
        }

class TelemetryQueue:
    """Bounded ring buffer for LiveTelemetrySamples."""
    def __init__(self, maxlen: int = 4096):
        self._buffer = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, sample: LiveTelemetrySample) -> None:
        with self._lock:
            # print(f"DEBUG: TelemetryQueue.push. Sample: {sample.time_boot_ms}")
            self._buffer.append(sample)
            # Verify it's actually there
            if not self._buffer:
                 print("DEBUG ERROR: Deque still empty after append!")

    def latest(self) -> Optional[LiveTelemetrySample]:
        with self._lock:
            # print(f"DEBUG: TelemetryQueue.latest. Buffer len: {len(self._buffer)}")
            return self._buffer[-1] if self._buffer else None

    def snapshot(self, n: Optional[int] = None) -> List[LiveTelemetrySample]:
        with self._lock:
            if n is None or n >= len(self._buffer):
                return list(self._buffer)
            # Return last n elements
            return list(self._buffer)[-n:]

    def __len__(self) -> int:
        with self._lock:
            return len(self._buffer)

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()

def _gps_to_enu(lat, lon, alt_msl, ref_lat, ref_lon, ref_alt_msl) -> np.ndarray:
    """Convert GPS (WGS84) to ENU (Local) coordinates.

    Uses the same linear approximation as mavlink_bridge._enu_to_gps.
    """
    lat_m_per_deg = 111320.0
    lon_m_per_deg = 111320.0 * math.cos(math.radians(ref_lat))

    x = (lon - ref_lon) * lon_m_per_deg
    y = (lat - ref_lat) * lat_m_per_deg
    z = alt_msl - ref_alt_msl
    return np.array([x, y, z])

import math # Ensure math is available for _gps_to_enu

def parse_telemetry_payload(msg_id: int, payload: bytes) -> Dict[str, Any]:
    """Parse MAVLink v2 telemetry payload bytes into a dictionary."""
    if msg_id == MAVLINK_MSG_ID_HEARTBEAT:
        # custom_mode (uint32), type (uint8), autopilot (uint8),
        # base_mode (uint8), system_status (uint8), mavlink_version (uint8)
        # Standard HEARTBEAT is 9 bytes.
        custom_mode = struct.unpack_from('<I', payload, 0)[0]
        base_mode = struct.unpack_from('<B', payload, 6)[0]
        armed = bool(base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED

        mode_map = {
            CUSTOM_MODE_STABILIZE: "STABILIZE",
            CUSTOM_MODE_GUIDED: "GUIDED",
            CUSTOM_MODE_AUTO: "AUTO",
            CUSTOM_MODE_LAND: "LAND",
            CUSTOM_MODE_RTL: "RTL"
        }
        mode_str = mode_map.get(custom_mode, f"MODE_{custom_mode}")
        return {"flight_mode": mode_str, "armed": armed}

    elif msg_id == MAVLINK_MSG_ID_ATTITUDE:
        # time_boot_ms (uint32), roll, pitch, yaw, rollspeed, pitchspeed, yawspeed (float)
        fmt = '<Iffffff'
        data = struct.unpack_from(fmt, payload)
        return {
            "time_boot_ms": data[0],
            "euler": (data[1], data[2], data[3])
        }

    elif msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        # time_boot_ms (uint32), lat (int32), lon (int32), alt (int32),
        # relative_alt (int32), vx (int16), vy (int16), vz (int16), hdg (uint16)
        time_boot_ms = struct.unpack_from('<I', payload, 0)[0]
        lat = struct.unpack_from('<i', payload, 4)[0]
        lon = struct.unpack_from('<i', payload, 8)[0]
        alt = struct.unpack_from('<i', payload, 12)[0]
        rel_alt = struct.unpack_from('<i', payload, 16)[0]
        vx = struct.unpack_from('<h', payload, 20)[0]
        vy = struct.unpack_from('<h', payload, 22)[0]
        vz = struct.unpack_from('<h', payload, 24)[0]
        hdg = struct.unpack_from('<H', payload, 26)[0]
        return {
            "time_boot_ms": time_boot_ms,
            "lat_deg": lat / 1e7,
            "lon_deg": lon / 1e7,
            "alt_msl": alt / 1000.0,
            "vel_enu": np.array([vy/100.0, vx/100.0, -vz/100.0]) # vx, vy, vz are in NED? Actually mavlink_bridge says vx_cm_s, vy_cm_s, vz_cm_s. Usually vx is North, vy is East.
        }

    elif msg_id == MAVLINK_MSG_ID_VFR_HUD:
        # airspeed (float), groundspeed (float), heading (int16),
        # throttle (uint16), alt (float), climb (float)
        # float(4), float(4), int16(2), uint16(2), float(4), float(4) = 20 bytes
        airspeed = struct.unpack_from('<f', payload, 0)[0]
        groundspeed = struct.unpack_from('<f', payload, 4)[0]
        heading = struct.unpack_from('<h', payload, 8)[0]
        throttle = struct.unpack_from('<H', payload, 10)[0]
        alt = struct.unpack_from('<f', payload, 12)[0]
        climb = struct.unpack_from('<f', payload, 16)[0]
        return {
            "airspeed": airspeed,
            "groundspeed": groundspeed,
            "throttle_pct": float(throttle),
            "climb_rate": climb
        }

    elif msg_id == MAVLINK_MSG_ID_SYS_STATUS:
        # onboard_control_sensors_present/enabled/health (uint32),
        # load (uint16), voltage_battery (uint16), current_battery (int16),
        # battery_remaining (int8), drop_rate_comm (uint16), errors_comm (uint16),
        # errors_count1/2/3/4 (uint16)
        # Note: we only pack up to battery_remaining in build_sys_status and pad to 31 bytes
        voltage_mv = struct.unpack_from('<H', payload, 12)[0]
        current_ca = struct.unpack_from('<h', payload, 14)[0]
        battery_pct = struct.unpack_from('<b', payload, 16)[0]
        return {
            "battery_voltage_v": voltage_mv / 1000.0,
            "battery_current_a": current_ca / 100.0,
            "battery_remaining_pct": float(battery_pct)
        }

    return {}

class TelemetryCSVRecorder:
    """Optional persistence sink for telemetry."""
    def __init__(self, path: str):
        self.path = path
        self.file = open(path, 'w')
        header = "t_wall,time_boot_ms,x,y,z,vx,vy,vz,roll,pitch,yaw,throttle_pct,airspeed,alt_msl,batt_v,batt_a,batt_pct,mode,armed\n"
        self.file.write(header)

    def record(self, sample: LiveTelemetrySample) -> None:
        line = (f"{sample.t_wall},{sample.time_boot_ms},"
                f"{sample.pos_enu[0]},{sample.pos_enu[1]},{sample.pos_enu[2]},"
                f"{sample.vel_enu[0]},{sample.vel_enu[1]},{sample.vel_enu[2]},"
                f"{sample.euler[0]},{sample.euler[1]},{sample.euler[2]},"
                f"{sample.throttle_pct},{sample.airspeed},{sample.alt_msl},"
                f"{sample.battery_voltage_v},{sample.battery_current_a},{sample.battery_remaining_pct},"
                f"{sample.flight_mode},{int(sample.armed)}\n")
        self.file.write(line)

    def close(self) -> None:
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None

class MAVLinkLiveSource:
    """UDP receiver thread for MAVLink telemetry."""
    def __init__(self, listen_ip: str = "0.0.0.0", listen_port: int = 14550,
                 queue: TelemetryQueue | None = None,
                 ref_lat: float = 47.3769, ref_lon: float = 8.5417, ref_alt_msl: float = 408.0,
                 max_samples: int = 4096, recorder: TelemetryCSVRecorder | None = None):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.queue = queue or TelemetryQueue(maxlen=max_samples)
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt_msl = ref_alt_msl
        self.recorder = recorder
        self._running = False
        self._thread = None
        self._current_sample = LiveTelemetrySample()

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _receive_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.listen_ip, self.listen_port))
        except Exception as e:
            print(f"MAVLinkLiveSource bind error: {e}")
            return
        sock.settimeout(0.1)

        while self._running:
            try:
                data, addr = sock.recvfrom(1024)
                # print(f"DEBUG: Received {len(data)} bytes from {addr}")
                msg = decode_mavlink_v2(data)
                if msg is None:
                    # print("DEBUG: MAVLink decode failed")
                    continue

                msg_id, payload = msg
                # print(f"DEBUG: Decoded msg_id: {msg_id}")
                updates = parse_telemetry_payload(msg_id, payload)
                if not updates:
                    # print(f"DEBUG: Parse failed or skipped for msg_id: {msg_id}")
                    continue

                # print(f"DEBUG: Updates: {updates}")

                # Merge updates
                t_boot = updates.get("time_boot_ms")
                if t_boot is not None and t_boot != self._current_sample.time_boot_ms:
                    # Finalize previous sample if it was complete-ish
                    if self._current_sample.time_boot_ms != 0:
                        self._current_sample.t_wall = time.time()
                        if self.recorder:
                            self.recorder.record(self._current_sample)
                        self.queue.push(self._current_sample)
                        # print(f"DEBUG: Pushed sample for t_boot {self._current_sample.time_boot_ms}")

                    # Start new sample, inheriting persistent state from previous
                    prev_armed = self._current_sample.armed
                    prev_mode = self._current_sample.flight_mode
                    prev_lat = self._current_sample.lat_deg
                    prev_lon = self._current_sample.lon_deg
                    prev_alt = self._current_sample.alt_msl
                    prev_pos = self._current_sample.pos_enu.copy()
                    prev_euler = self._current_sample.euler

                    self._current_sample = LiveTelemetrySample(
                        time_boot_ms=t_boot,
                        armed=prev_armed,
                        flight_mode=prev_mode,
                        lat_deg=prev_lat,
                        lon_deg=prev_lon,
                        alt_msl=prev_alt,
                        pos_enu=prev_pos,
                        euler=prev_euler
                    )

                # Apply updates to current sample
                for k, v in updates.items():
                    setattr(self._current_sample, k, v)

                # Update ENU position if GPS was updated
                if "lat_deg" in updates or "lon_deg" in updates or "alt_msl" in updates:
                    self._current_sample.pos_enu = _gps_to_enu(
                        self._current_sample.lat_deg,
                        self._current_sample.lon_deg,
                        self._current_sample.alt_msl,
                        self.ref_lat, self.ref_lon, self.ref_alt_msl
                    )

                # Push sample on GPS update OR if it's the very first valid sample received
                if "lat_deg" in updates or (self._current_sample.time_boot_ms != 0 and len(self.queue) == 0):
                    self._current_sample.t_wall = time.time()
                    # print(f"DEBUG: Pushing sample to queue. Current len: {len(self.queue)}")
                    self.queue.push(self._current_sample)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"MAVLinkLiveSource error: {e}")
                continue
        sock.close()
