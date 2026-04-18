"""
Live Telemetry Module - Swarm Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Handles MAVLink telemetry reception, buffering, and persistence for the
web Run-time View (``simulation/runtime_view``). Mirrors the message
IDs produced by ``simulation/mavlink_bridge.py`` so the digital twin or
a real vehicle can push pose/HUD/battery telemetry into the same
pipeline.
"""

import csv
import math
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional

import numpy as np

# Re-use constants and decoder from mavlink_bridge to stay in lock-step
# with the sender's message IDs and wire format.
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
    CUSTOM_MODE_RTL,
)


# ── Sample type ───────────────────────────────────────────────────────────


@dataclass
class LiveTelemetrySample:
    """One telemetry snapshot produced by the receiver thread.

    All numeric fields default to zero so that partial messages can be
    pushed into the queue without losing schema compatibility with the
    JSON consumers (FastAPI ``/api/status`` and ``/ws/telemetry``).
    """

    t_wall: float = 0.0
    time_boot_ms: int = 0
    pos_enu: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel_enu: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: tuple = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) rad
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
        """Return a JSON-safe dict (numpy arrays become plain lists)."""
        return {
            "t_wall": float(self.t_wall),
            "time_boot_ms": int(self.time_boot_ms),
            "pos_enu": [float(x) for x in np.asarray(self.pos_enu).tolist()],
            "vel_enu": [float(x) for x in np.asarray(self.vel_enu).tolist()],
            "euler": [float(x) for x in self.euler],
            "throttle_pct": float(self.throttle_pct),
            "airspeed": float(self.airspeed),
            "groundspeed": float(self.groundspeed),
            "alt_msl": float(self.alt_msl),
            "climb_rate": float(self.climb_rate),
            "battery_voltage_v": float(self.battery_voltage_v),
            "battery_current_a": float(self.battery_current_a),
            "battery_remaining_pct": float(self.battery_remaining_pct),
            "flight_mode": str(self.flight_mode),
            "armed": bool(self.armed),
            "lat_deg": float(self.lat_deg),
            "lon_deg": float(self.lon_deg),
        }

    # Alias requested by the plan; the tests use ``to_dict`` so keep both.
    to_json = to_dict


# ── Queue ─────────────────────────────────────────────────────────────────


class TelemetryQueue:
    """Thread-safe bounded ring buffer of :class:`LiveTelemetrySample`."""

    def __init__(self, maxlen: int = 4096) -> None:
        self._buffer: Deque[LiveTelemetrySample] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, sample: LiveTelemetrySample) -> None:
        with self._lock:
            self._buffer.append(sample)

    def latest(self) -> Optional[LiveTelemetrySample]:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def snapshot(self, n: Optional[int] = None) -> List[LiveTelemetrySample]:
        with self._lock:
            if n is None or n >= len(self._buffer):
                return list(self._buffer)
            if n <= 0:
                return []
            return list(self._buffer)[-n:]

    def __len__(self) -> int:
        with self._lock:
            return len(self._buffer)

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()

    @property
    def maxlen(self) -> int:
        return self._buffer.maxlen or 0


# ── GPS ↔ ENU conversion (inverse of mavlink_bridge._enu_to_gps) ──────────


def _gps_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_msl_m: float,
    ref_lat: float,
    ref_lon: float,
    ref_alt_msl: float,
) -> np.ndarray:
    """Inverse of :func:`mavlink_bridge._enu_to_gps`.

    Uses the same small-angle equirectangular approximation so that the
    two directions round-trip within ~1 mm for bounded (≤1 km) offsets.
    """
    lat_m_per_deg = 111320.0
    lon_m_per_deg = 111320.0 * math.cos(math.radians(ref_lat))
    x = (lon_deg - ref_lon) * lon_m_per_deg  # East
    y = (lat_deg - ref_lat) * lat_m_per_deg  # North
    z = alt_msl_m - ref_alt_msl
    return np.array([x, y, z], dtype=np.float64)


# ── MAVLink telemetry payload parser ──────────────────────────────────────


def parse_telemetry_payload(msg_id: int, payload: bytes) -> Dict[str, Any]:
    """Parse a MAVLink v2 *inbound* telemetry payload into a dict.

    The dict contains only the fields that the given message ID
    carries; the receiver merges them into the running
    :class:`LiveTelemetrySample`. Unknown/short payloads return ``{}``.
    """
    if msg_id == MAVLINK_MSG_ID_ATTITUDE and len(payload) >= 28:
        fields = struct.unpack_from("<Iffffff", payload)
        return {
            "time_boot_ms": int(fields[0]),
            "euler": (float(fields[1]), float(fields[2]), float(fields[3])),
            "rollspeed": float(fields[4]),
            "pitchspeed": float(fields[5]),
            "yawspeed": float(fields[6]),
        }

    if msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT and len(payload) >= 28:
        fields = struct.unpack_from("<IiiiihhhH", payload)
        return {
            "time_boot_ms": int(fields[0]),
            "lat_deg": float(fields[1]) / 1e7,
            "lon_deg": float(fields[2]) / 1e7,
            "alt_msl": float(fields[3]) / 1000.0,
            "alt_rel_m": float(fields[4]) / 1000.0,
            # mavlink_bridge encodes NED (vx=North, vy=East, vz=Down),
            # so ENU = (East=vy, North=vx, Up=-vz).
            "vel_enu": np.array(
                [float(fields[6]) / 100.0,
                 float(fields[5]) / 100.0,
                 -float(fields[7]) / 100.0],
                dtype=np.float64,
            ),
            "hdg_deg": float(fields[8]) / 100.0,
        }

    if msg_id == MAVLINK_MSG_ID_VFR_HUD and len(payload) >= 20:
        fields = struct.unpack_from("<ffhHff", payload)
        return {
            "airspeed": float(fields[0]),
            "groundspeed": float(fields[1]),
            "heading_deg": int(fields[2]),
            "throttle_pct": float(int(fields[3])),
            "alt_msl_hud": float(fields[4]),
            "climb_rate": float(fields[5]),
        }

    if msg_id == MAVLINK_MSG_ID_SYS_STATUS and len(payload) >= 20:
        # Layout matches mavlink_bridge.build_sys_status(): three u32
        # sensor bitmaps, load(u16), voltage(u16), current(i16),
        # battery_remaining(i8), zero-padded to 31 bytes.
        fields = struct.unpack_from("<IIIHHhb", payload)
        return {
            "battery_voltage_v": float(fields[4]) / 1000.0,
            "battery_current_a": float(fields[5]) / 100.0,
            "battery_remaining_pct": float(fields[6]),
        }

    if msg_id == MAVLINK_MSG_ID_HEARTBEAT and len(payload) >= 9:
        fields = struct.unpack_from("<IBBBBB", payload)
        custom_mode = int(fields[0])
        base_mode = int(fields[3])
        mode_map = {
            CUSTOM_MODE_STABILIZE: "STABILIZE",
            CUSTOM_MODE_GUIDED: "GUIDED",
            CUSTOM_MODE_AUTO: "AUTO",
            CUSTOM_MODE_LAND: "LAND",
            CUSTOM_MODE_RTL: "RTL",
        }
        return {
            "flight_mode": mode_map.get(custom_mode, f"MODE_{custom_mode}"),
            "armed": bool(base_mode & 0x80),
        }

    return {}


# ── CSV recorder ──────────────────────────────────────────────────────────


CSV_HEADER = [
    "t_wall",
    "time_boot_ms",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "roll",
    "pitch",
    "yaw",
    "throttle_pct",
    "airspeed",
    "alt_msl",
    "batt_v",
    "batt_a",
    "batt_pct",
    "mode",
    "armed",
]


class TelemetryCSVRecorder:
    """Append :class:`LiveTelemetrySample` rows to a CSV file."""

    def __init__(self, path: str) -> None:
        self.path = str(path)
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
        self._lock = threading.Lock()
        self._closed = False

    def record(self, sample: LiveTelemetrySample) -> None:
        if self._closed:
            return
        pos = np.asarray(sample.pos_enu).tolist()
        vel = np.asarray(sample.vel_enu).tolist()
        row = [
            f"{float(sample.t_wall):.6f}",
            int(sample.time_boot_ms),
            f"{float(pos[0]):.6f}",
            f"{float(pos[1]):.6f}",
            f"{float(pos[2]):.6f}",
            f"{float(vel[0]):.6f}",
            f"{float(vel[1]):.6f}",
            f"{float(vel[2]):.6f}",
            f"{float(sample.euler[0]):.9f}",
            f"{float(sample.euler[1]):.9f}",
            f"{float(sample.euler[2]):.9f}",
            f"{float(sample.throttle_pct):.3f}",
            f"{float(sample.airspeed):.6f}",
            f"{float(sample.alt_msl):.6f}",
            f"{float(sample.battery_voltage_v):.4f}",
            f"{float(sample.battery_current_a):.4f}",
            f"{float(sample.battery_remaining_pct):.2f}",
            str(sample.flight_mode),
            "1" if sample.armed else "0",
        ]
        with self._lock:
            self._writer.writerow(row)
            self._fh.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._fh.flush()
            finally:
                self._fh.close()

    def __enter__(self) -> "TelemetryCSVRecorder":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


# ── UDP MAVLink receiver ──────────────────────────────────────────────────


class MAVLinkLiveSource:
    """Background thread that reads MAVLink v2 telemetry from a UDP socket.

    Fragments from the five supported message IDs are merged per
    ``time_boot_ms`` tick into one :class:`LiveTelemetrySample` and
    pushed to a :class:`TelemetryQueue`. Lifecycle mirrors
    :class:`mavlink_bridge.MAVLinkBridge`: :meth:`start` binds the
    socket synchronously and spawns the daemon receiver thread;
    :meth:`stop` joins it within ~2 s.
    """

    def __init__(
        self,
        listen_ip: str = "0.0.0.0",
        listen_port: int = 14550,
        queue: Optional[TelemetryQueue] = None,
        ref_lat: float = 47.3769,
        ref_lon: float = 8.5417,
        ref_alt_msl: float = 408.0,
        max_samples: int = 4096,
        recorder: Optional[TelemetryCSVRecorder] = None,
        sample_hook: Optional[Callable[[LiveTelemetrySample], None]] = None,
    ) -> None:
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.queue = queue if queue is not None else TelemetryQueue(maxlen=max_samples)
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt_msl = ref_alt_msl
        self.recorder = recorder
        self._sample_hook = sample_hook

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        # Sample being assembled from the current tick's fragments.
        self._current = LiveTelemetrySample()
        self._have_any = False

    # ── lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        """Bind the UDP socket and start the receiver thread."""
        if self._running:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.listen_ip, self.listen_port))
        self._sock.settimeout(0.1)
        # If the caller asked for an ephemeral port (``0``), record the
        # one the OS actually gave us so clients can discover it.
        self.listen_port = self._sock.getsockname()[1]
        self._running = True
        self._thread = threading.Thread(
            target=self._receive_loop, name="MAVLinkLiveSource", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self.recorder is not None:
            try:
                self.recorder.close()
            except Exception:  # pragma: no cover - defensive
                pass

    def __enter__(self) -> "MAVLinkLiveSource":
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.stop()

    # ── receiver loop ────────────────────────────────────────────────

    def _receive_loop(self) -> None:
        sock = self._sock
        assert sock is not None
        while self._running:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self._consume_datagram(data)

    def _consume_datagram(self, data: bytes) -> None:
        """Decode and apply a single UDP datagram."""
        decoded = decode_mavlink_v2(data)
        if decoded is None:
            return
        msg_id, payload = decoded
        updates = parse_telemetry_payload(msg_id, payload)
        if not updates:
            return
        self._apply_updates(updates)

    def _apply_updates(self, updates: Dict[str, Any]) -> None:
        # Heartbeat / sys_status carry no time_boot_ms — merge them into
        # the current sample without flushing.
        new_tick = updates.get("time_boot_ms")
        if new_tick is not None and self._have_any and new_tick != self._current.time_boot_ms:
            # Tick advanced → finalize the previous sample.
            self._finalize_current()

        # Apply updates.
        if "time_boot_ms" in updates:
            self._current.time_boot_ms = int(updates["time_boot_ms"])
            self._have_any = True
        if "euler" in updates:
            self._current.euler = updates["euler"]
            self._have_any = True
        if "lat_deg" in updates:
            self._current.lat_deg = updates["lat_deg"]
        if "lon_deg" in updates:
            self._current.lon_deg = updates["lon_deg"]
        if "alt_msl" in updates:
            self._current.alt_msl = updates["alt_msl"]
        if "vel_enu" in updates:
            self._current.vel_enu = updates["vel_enu"]
        if "airspeed" in updates:
            self._current.airspeed = updates["airspeed"]
        if "groundspeed" in updates:
            self._current.groundspeed = updates["groundspeed"]
        if "throttle_pct" in updates:
            self._current.throttle_pct = updates["throttle_pct"]
        if "climb_rate" in updates:
            self._current.climb_rate = updates["climb_rate"]
        if "alt_msl_hud" in updates and self._current.alt_msl == 0.0:
            self._current.alt_msl = updates["alt_msl_hud"]
        if "battery_voltage_v" in updates:
            self._current.battery_voltage_v = updates["battery_voltage_v"]
        if "battery_current_a" in updates:
            self._current.battery_current_a = updates["battery_current_a"]
        if "battery_remaining_pct" in updates:
            self._current.battery_remaining_pct = updates["battery_remaining_pct"]
        if "flight_mode" in updates:
            self._current.flight_mode = updates["flight_mode"]
        if "armed" in updates:
            self._current.armed = bool(updates["armed"])

        # Recompute ENU from GPS when GPS has changed.
        if ("lat_deg" in updates) or ("lon_deg" in updates) or ("alt_msl" in updates):
            self._current.pos_enu = _gps_to_enu(
                self._current.lat_deg,
                self._current.lon_deg,
                self._current.alt_msl,
                self.ref_lat,
                self.ref_lon,
                self.ref_alt_msl,
            )
            self._have_any = True

        # The sender's last message per tick is SYS_STATUS, but GPS comes
        # in third. Push immediately on GPS so consumers see fresh
        # position without waiting for the next tick boundary — the
        # finalize_current on the next tick is then a no-op because
        # _have_any is False.
        if "lat_deg" in updates and self._have_any:
            self._finalize_current()

    def _finalize_current(self) -> None:
        """Push the current sample into the queue and start a fresh one."""
        if not self._have_any:
            return
        self._current.t_wall = time.time()
        if self.recorder is not None:
            try:
                self.recorder.record(self._current)
            except Exception:  # pragma: no cover - defensive
                pass
        self.queue.push(self._current)
        if self._sample_hook is not None:
            try:
                self._sample_hook(self._current)
            except Exception:  # pragma: no cover - defensive
                pass
        # Start a fresh sample, inheriting "sticky" fields that don't
        # get re-sent every tick (battery, mode, last known pose).
        fresh = LiveTelemetrySample(
            time_boot_ms=self._current.time_boot_ms,
            flight_mode=self._current.flight_mode,
            armed=self._current.armed,
            battery_voltage_v=self._current.battery_voltage_v,
            battery_current_a=self._current.battery_current_a,
            battery_remaining_pct=self._current.battery_remaining_pct,
            lat_deg=self._current.lat_deg,
            lon_deg=self._current.lon_deg,
            alt_msl=self._current.alt_msl,
            pos_enu=self._current.pos_enu.copy(),
            euler=self._current.euler,
        )
        self._current = fresh
        self._have_any = False

    # ── test hooks ───────────────────────────────────────────────────

    def inject_frame(self, frame: bytes) -> None:
        """Feed one MAVLink v2 frame without opening a socket (tests only)."""
        self._consume_datagram(frame)

    def flush(self) -> None:
        """Force-push the pending sample regardless of tick advancement."""
        self._finalize_current()
