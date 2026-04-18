"""
DataFlash .BIN Recorder — write ArduPilot-compatible binary log files.

Converts ``LiveTelemetrySample`` data into the DataFlash binary format
used by ArduPilot (*.BIN / *.log). The output can be opened by
Mission Planner, MAVExplorer, or ``mavlogdump.py``.

Format reference: https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html

Record structure:
    [0xA3, 0x95, msg_type, ...payload...]

Every .BIN file starts with FMT records (msg_type=128) that describe
the schema of each subsequent message type.
"""

import struct
import threading
import time
from pathlib import Path
from typing import Optional

# ── Constants ─────────────────────────────────────────────────────────

HEAD1 = 0xA3
HEAD2 = 0x95

# Message type IDs (matching ArduPilot conventions).
MSG_FMT = 128
MSG_PARM = 129
MSG_GPS = 130
MSG_ATT = 137
MSG_BAT = 147
MSG_MODE = 15

# Format characters → struct format + size.
_FMT_CHAR = {
    'b': ('b', 1),   # int8
    'B': ('B', 1),   # uint8
    'h': ('h', 2),   # int16
    'H': ('H', 2),   # uint16
    'i': ('i', 4),   # int32
    'I': ('I', 4),   # uint32
    'f': ('f', 4),   # float32
    'n': ('4s', 4),  # char[4]
    'N': ('16s', 16),  # char[16]
    'Z': ('64s', 64),  # char[64]
    'q': ('q', 8),   # int64
    'Q': ('Q', 8),   # uint64
    'e': ('i', 4),   # float * 100 → int32 (centi-degrees etc.)
    'E': ('I', 4),   # float * 100 → uint32
    'L': ('i', 4),   # latitude/longitude * 1e7 → int32
    'M': ('B', 1),   # flight mode as uint8
    'c': ('h', 2),   # float * 100 → int16 (centi-X)
    'C': ('H', 2),   # float * 100 → uint16
}


def _format_size(fmt_str: str) -> int:
    """Total payload size for a DataFlash format string."""
    return sum(_FMT_CHAR[c][1] for c in fmt_str)


def _pack_payload(fmt_str: str, values: tuple) -> bytes:
    """Pack values according to a DataFlash format string."""
    parts = []
    for ch, val in zip(fmt_str, values):
        struct_fmt, _ = _FMT_CHAR[ch]
        if ch in ('n', 'N', 'Z'):
            if isinstance(val, str):
                val = val.encode('ascii', errors='replace')
            val = val[:_FMT_CHAR[ch][1]].ljust(_FMT_CHAR[ch][1], b'\x00')
            parts.append(struct.pack(f'<{struct_fmt}', val))
        elif ch == 'e':
            parts.append(struct.pack('<i', int(val * 100)))
        elif ch == 'E':
            parts.append(struct.pack('<I', int(val * 100)))
        elif ch == 'L':
            parts.append(struct.pack('<i', int(val * 1e7)))
        elif ch == 'c':
            parts.append(struct.pack('<h', int(val * 100)))
        elif ch == 'C':
            parts.append(struct.pack('<H', int(val * 100)))
        else:
            parts.append(struct.pack(f'<{struct_fmt}', val))
    return b''.join(parts)


# ── FMT record builder ──────────────────────────────────────────────

def _build_fmt_record(msg_type: int, length: int, name: str,
                      fmt: str, labels: str) -> bytes:
    """Build one FMT (type 128) record."""
    # FMT payload: B type, B length, 4s name, 16s format, 64s labels
    name_b = name.encode('ascii')[:4].ljust(4, b'\x00')
    fmt_b = fmt.encode('ascii')[:16].ljust(16, b'\x00')
    labels_b = labels.encode('ascii')[:64].ljust(64, b'\x00')
    payload = struct.pack('<BB4s16s64s', msg_type, length, name_b, fmt_b, labels_b)
    return struct.pack('BBB', HEAD1, HEAD2, MSG_FMT) + payload


# ── Message definitions ─────────────────────────────────────────────

# Each entry: (msg_type, name, format_string, comma-separated labels)
MESSAGES = {
    'FMT': (MSG_FMT, 'FMT', 'BB4s16s64s', 'Type,Length,Name,Format,Columns'),
    'ATT': (MSG_ATT, 'ATT', 'Qfffffff',
            'TimeUS,Roll,Pitch,Yaw,DesRoll,DesPitch,DesYaw,ErrRP'),
    'GPS': (MSG_GPS, 'GPS', 'QBILLfffffB',
            'TimeUS,Status,GMS,Lat,Lng,Alt,Spd,GCrs,VZ,SAcc,NSats'),
    'BAT': (MSG_BAT, 'BAT', 'QfffffffB',
            'TimeUS,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,Pct'),
    'MODE': (MSG_MODE, 'MODE', 'QMBB',
             'TimeUS,Mode,ModeNum,Rsn'),
}


class DataFlashRecorder:
    """Write ArduPilot-compatible DataFlash .BIN files.

    Usage::

        with DataFlashRecorder('flight.BIN') as rec:
            rec.write_att(time_us, roll, pitch, yaw)
            rec.write_gps(time_us, lat, lon, alt, ...)
            rec.write_bat(time_us, voltage, current, pct)
    """

    def __init__(self, path: str) -> None:
        self.path = str(path)
        self._fh = open(self.path, 'wb')
        self._lock = threading.Lock()
        self._closed = False
        self._t0: Optional[float] = None
        self._write_headers()

    def _write_headers(self) -> None:
        """Write FMT records for all supported message types."""
        # FMT record for the FMT type itself.
        fmt_len = 3 + 1 + 1 + 4 + 16 + 64  # head(2)+type(1) + payload
        self._fh.write(_build_fmt_record(MSG_FMT, fmt_len, 'FMT', 'BB4s16s64s',
                                         'Type,Length,Name,Format,Columns'))

        # FMT records for each data message type.
        for key in ('ATT', 'GPS', 'BAT', 'MODE'):
            msg_type, name, fmt, labels = MESSAGES[key]
            payload_size = _format_size(fmt)
            record_len = 3 + payload_size  # 2 head bytes + 1 type byte + payload
            self._fh.write(_build_fmt_record(msg_type, record_len, name, fmt, labels))

        self._fh.flush()

    def _write_record(self, msg_type: int, fmt_str: str, values: tuple) -> None:
        payload = _pack_payload(fmt_str, values)
        header = struct.pack('BBB', HEAD1, HEAD2, msg_type)
        with self._lock:
            if self._closed:
                return
            self._fh.write(header + payload)

    def _time_us(self, t_wall: float) -> int:
        """Convert wall time to microseconds since recording start."""
        if self._t0 is None:
            self._t0 = t_wall
        return int((t_wall - self._t0) * 1e6)

    # ── Public write methods ────────────────────────────────────────

    def write_att(self, t_wall: float, roll: float, pitch: float,
                  yaw: float) -> None:
        """Write an ATT (attitude) record. Angles in degrees."""
        t_us = self._time_us(t_wall)
        # DesRoll/DesPitch/DesYaw/ErrRP set to 0 — we don't have desired.
        self._write_record(MSG_ATT, 'Qfffffff',
                           (t_us, roll, pitch, yaw, 0.0, 0.0, 0.0, 0.0))

    def write_gps(self, t_wall: float, lat: float, lon: float,
                  alt: float, speed: float = 0.0, course: float = 0.0,
                  vz: float = 0.0, n_sats: int = 10) -> None:
        """Write a GPS record. lat/lon in degrees, alt in metres."""
        t_us = self._time_us(t_wall)
        gms = int((t_wall % 604800) * 1000)  # GPS ms of week (approx)
        self._write_record(MSG_GPS, 'QBILLfffffB',
                           (t_us, 3, gms, lat, lon, alt, speed,
                            course, vz, 0.5, n_sats))

    def write_bat(self, t_wall: float, voltage: float, current: float,
                  remaining_pct: float) -> None:
        """Write a BAT (battery) record."""
        t_us = self._time_us(t_wall)
        self._write_record(MSG_BAT, 'QfffffffB',
                           (t_us, voltage, voltage, current, 0.0, 0.0,
                            25.0, 0.0, int(remaining_pct)))

    def write_mode(self, t_wall: float, mode_num: int) -> None:
        """Write a MODE record."""
        t_us = self._time_us(t_wall)
        self._write_record(MSG_MODE, 'QMBB',
                           (t_us, mode_num, mode_num, 0))

    def record_sample(self, sample) -> None:
        """Write ATT + GPS + BAT records from a LiveTelemetrySample."""
        import math
        t = sample.t_wall
        r, p, y = sample.euler
        self.write_att(t, math.degrees(r), math.degrees(p), math.degrees(y))
        spd = float((sample.vel_enu[0]**2 + sample.vel_enu[1]**2 +
                      sample.vel_enu[2]**2) ** 0.5)
        self.write_gps(t, sample.lat_deg, sample.lon_deg,
                       sample.alt_msl, speed=spd,
                       vz=float(sample.vel_enu[2]))
        self.write_bat(t, sample.battery_voltage_v,
                       sample.battery_current_a,
                       sample.battery_remaining_pct)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._fh.flush()
            finally:
                self._fh.close()

    def __enter__(self) -> 'DataFlashRecorder':
        return self

    def __exit__(self, *exc) -> None:
        self.close()
