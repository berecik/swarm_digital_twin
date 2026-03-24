"""
MAVLink Bridge - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

MAVLink telemetry bridge connecting the standalone physics simulator
to QGroundControl or other MAVLink-compatible ground control stations.

Uses pymavlink to encode/decode MAVLink v2 messages over UDP.
Sends: HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS.
Receives: COMMAND_LONG (arm/disarm, mode change), SET_POSITION_TARGET_LOCAL_NED.

Usage:
    bridge = MAVLinkBridge(target_ip="127.0.0.1", target_port=14550)
    bridge.start()
    # In sim loop:
    bridge.send_state(sim_record, params)
    commands = bridge.receive_commands()
    bridge.stop()
"""

import time
import socket
import struct
import threading
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any

import numpy as np


# ── MAVLink constants ──────────────────────────────────────────────────────

# Message IDs
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_VFR_HUD = 74
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84

# MAVLink v2 header
MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN = 10
MAVLINK_CHECKSUM_LEN = 2

# Component/system IDs
MAV_TYPE_QUADROTOR = 2
MAV_TYPE_FIXED_WING = 1
MAV_AUTOPILOT_GENERIC = 0
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_STATE_ACTIVE = 4
MAV_STATE_STANDBY = 3

# MAV_CMD
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176

# Custom modes (ArduPilot-compatible)
CUSTOM_MODE_STABILIZE = 0
CUSTOM_MODE_GUIDED = 4
CUSTOM_MODE_AUTO = 3
CUSTOM_MODE_LAND = 9
CUSTOM_MODE_RTL = 6

# CRC extra bytes for MAVLink v2 message validation
_CRC_EXTRA = {
    MAVLINK_MSG_ID_HEARTBEAT: 50,
    MAVLINK_MSG_ID_SYS_STATUS: 124,
    MAVLINK_MSG_ID_ATTITUDE: 39,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 104,
    MAVLINK_MSG_ID_VFR_HUD: 20,
    MAVLINK_MSG_ID_COMMAND_LONG: 152,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: 143,
}


# ── CRC computation (MAVLink X.25) ────────────────────────────────────────

def _crc_accumulate(byte: int, crc: int) -> int:
    """Accumulate one byte into the X.25 CRC."""
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink_crc(data: bytes, crc_extra: int) -> int:
    """Compute MAVLink v2 checksum over data with CRC extra byte."""
    crc = 0xFFFF
    for b in data:
        crc = _crc_accumulate(b, crc)
    crc = _crc_accumulate(crc_extra, crc)
    return crc


# ── MAVLink v2 message encoding ───────────────────────────────────────────

def encode_mavlink_v2(msg_id: int, system_id: int, component_id: int,
                      payload: bytes, seq: int = 0) -> bytes:
    """Encode a MAVLink v2 message with header and CRC.

    Returns the complete message bytes ready for transmission.
    """
    payload_len = len(payload)
    # Header: STX, payload_len, incompat_flags, compat_flags, seq,
    #         system_id, component_id, msg_id (3 bytes LE)
    header = struct.pack('<BBBBBBB',
                         MAVLINK_STX_V2,
                         payload_len,
                         0,  # incompat_flags
                         0,  # compat_flags
                         seq & 0xFF,
                         system_id,
                         component_id)
    msg_id_bytes = struct.pack('<I', msg_id)[:3]  # 3-byte LE msg_id
    header += msg_id_bytes

    # CRC covers everything after STX: header[1:] + payload
    crc_data = header[1:] + payload
    crc_extra = _CRC_EXTRA.get(msg_id, 0)
    crc = mavlink_crc(crc_data, crc_extra)
    crc_bytes = struct.pack('<H', crc)

    return header + payload + crc_bytes


# ── Message builders ───────────────────────────────────────────────────────

def build_heartbeat(system_id: int = 1, component_id: int = 1,
                    mav_type: int = MAV_TYPE_QUADROTOR,
                    armed: bool = True, seq: int = 0) -> bytes:
    """Build a HEARTBEAT message."""
    custom_mode = CUSTOM_MODE_GUIDED
    base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (0x80 if armed else 0)
    system_status = MAV_STATE_ACTIVE if armed else MAV_STATE_STANDBY
    # Payload: custom_mode(u32), type(u8), autopilot(u8),
    #          base_mode(u8), system_status(u8), mavlink_version(u8)
    payload = struct.pack('<IBBBBB',
                          custom_mode,
                          mav_type,
                          MAV_AUTOPILOT_GENERIC,
                          base_mode,
                          system_status,
                          3)  # MAVLink version 2
    return encode_mavlink_v2(MAVLINK_MSG_ID_HEARTBEAT, system_id,
                             component_id, payload, seq)


def build_attitude(roll: float, pitch: float, yaw: float,
                   rollspeed: float = 0.0, pitchspeed: float = 0.0,
                   yawspeed: float = 0.0, time_boot_ms: int = 0,
                   system_id: int = 1, component_id: int = 1,
                   seq: int = 0) -> bytes:
    """Build an ATTITUDE message."""
    # Payload: time_boot_ms(u32), roll(f32), pitch(f32), yaw(f32),
    #          rollspeed(f32), pitchspeed(f32), yawspeed(f32)
    payload = struct.pack('<Iffffff',
                          time_boot_ms,
                          roll, pitch, yaw,
                          rollspeed, pitchspeed, yawspeed)
    return encode_mavlink_v2(MAVLINK_MSG_ID_ATTITUDE, system_id,
                             component_id, payload, seq)


def build_global_position_int(lat_deg: float, lon_deg: float,
                               alt_msl_m: float, alt_rel_m: float,
                               vx_cm_s: int = 0, vy_cm_s: int = 0,
                               vz_cm_s: int = 0, hdg_cdeg: int = 0,
                               time_boot_ms: int = 0,
                               system_id: int = 1, component_id: int = 1,
                               seq: int = 0) -> bytes:
    """Build a GLOBAL_POSITION_INT message."""
    lat_e7 = int(lat_deg * 1e7)
    lon_e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_msl_m * 1000)
    rel_alt_mm = int(alt_rel_m * 1000)
    # Payload: time_boot_ms(u32), lat(i32), lon(i32), alt(i32),
    #          relative_alt(i32), vx(i16), vy(i16), vz(i16), hdg(u16)
    payload = struct.pack('<IiiiihhhH',
                          time_boot_ms,
                          lat_e7, lon_e7, alt_mm, rel_alt_mm,
                          vx_cm_s, vy_cm_s, vz_cm_s, hdg_cdeg)
    return encode_mavlink_v2(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, system_id,
                             component_id, payload, seq)


def build_vfr_hud(airspeed: float, groundspeed: float,
                  heading_deg: int, throttle_pct: int,
                  alt_msl: float, climb_rate: float,
                  system_id: int = 1, component_id: int = 1,
                  seq: int = 0) -> bytes:
    """Build a VFR_HUD message."""
    # Payload: airspeed(f32), groundspeed(f32), heading(i16),
    #          throttle(u16), alt(f32), climb(f32)
    payload = struct.pack('<ffhHff',
                          airspeed, groundspeed,
                          heading_deg, throttle_pct,
                          alt_msl, climb_rate)
    return encode_mavlink_v2(MAVLINK_MSG_ID_VFR_HUD, system_id,
                             component_id, payload, seq)


def build_sys_status(voltage_mv: int = 12600, current_ca: int = 0,
                     battery_pct: int = 100,
                     system_id: int = 1, component_id: int = 1,
                     seq: int = 0) -> bytes:
    """Build a SYS_STATUS message."""
    # Simplified: only battery fields populated
    # Payload: onboard_control_sensors_present(u32),
    #          onboard_control_sensors_enabled(u32),
    #          onboard_control_sensors_health(u32),
    #          load(u16), voltage_battery(u16), current_battery(i16),
    #          battery_remaining(i8), ... (padded)
    sensors = 0xFFFF  # all sensors present/enabled/healthy
    payload = struct.pack('<IIIHHhb',
                          sensors, sensors, sensors,
                          0,  # load (0 = unknown)
                          voltage_mv,
                          current_ca,
                          battery_pct)
    # Pad remaining fields (drop_rate_comm, errors_count, etc.)
    payload += b'\x00' * (31 - len(payload))
    return encode_mavlink_v2(MAVLINK_MSG_ID_SYS_STATUS, system_id,
                             component_id, payload, seq)


# ── Command parsing ────────────────────────────────────────────────────────

@dataclass
class MAVCommand:
    """Parsed MAVLink command received from GCS."""
    command_id: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0
    param6: float = 0.0
    param7: float = 0.0


@dataclass
class PositionTarget:
    """Parsed SET_POSITION_TARGET_LOCAL_NED."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    type_mask: int = 0


def parse_mavlink_payload(msg_id: int, payload: bytes) -> Optional[Any]:
    """Parse a MAVLink v2 payload by message ID. Returns parsed object or None."""
    if msg_id == MAVLINK_MSG_ID_COMMAND_LONG and len(payload) >= 33:
        fields = struct.unpack_from('<fffffffHBBB', payload)
        return MAVCommand(
            command_id=fields[7],
            param1=fields[0], param2=fields[1], param3=fields[2],
            param4=fields[3], param5=fields[4], param6=fields[5],
            param7=fields[6],
        )
    if msg_id == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED and len(payload) >= 49:
        fields = struct.unpack_from('<IHffffffffffffffBBB', payload)
        return PositionTarget(
            x=fields[2], y=fields[3], z=fields[4],
            vx=fields[5], vy=fields[6], vz=fields[7],
            type_mask=fields[1],
        )
    return None


def decode_mavlink_v2(data: bytes) -> Optional[Tuple[int, bytes]]:
    """Decode one MAVLink v2 message from raw bytes.

    Returns (msg_id, payload) or None if invalid.
    """
    if len(data) < MAVLINK_HEADER_LEN + MAVLINK_CHECKSUM_LEN:
        return None
    if data[0] != MAVLINK_STX_V2:
        return None

    payload_len = data[1]
    msg_id = struct.unpack_from('<I', data[7:10] + b'\x00')[0]
    total_len = MAVLINK_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN

    if len(data) < total_len:
        return None

    payload = data[MAVLINK_HEADER_LEN:MAVLINK_HEADER_LEN + payload_len]

    # Verify CRC
    crc_data = data[1:MAVLINK_HEADER_LEN + payload_len]
    crc_extra = _CRC_EXTRA.get(msg_id, 0)
    expected_crc = mavlink_crc(crc_data, crc_extra)
    actual_crc = struct.unpack_from('<H', data, MAVLINK_HEADER_LEN + payload_len)[0]

    if actual_crc != expected_crc:
        return None

    return (msg_id, payload)


# ── Bridge ─────────────────────────────────────────────────────────────────

@dataclass
class SimState:
    """Simplified simulation state for MAVLink telemetry."""
    time_s: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # ENU [m]
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # ENU [m/s]
    roll: float = 0.0       # rad
    pitch: float = 0.0      # rad
    yaw: float = 0.0        # rad
    rollspeed: float = 0.0  # rad/s
    pitchspeed: float = 0.0 # rad/s
    yawspeed: float = 0.0   # rad/s
    thrust_pct: float = 0.0 # 0-100
    armed: bool = True

    # Reference point for GPS conversion (default: Zurich)
    ref_lat: float = 47.3769
    ref_lon: float = 8.5417
    ref_alt_msl: float = 408.0  # m


def sim_state_from_record(record, angular_velocity: Optional[np.ndarray] = None,
                          max_thrust: float = 25.0,
                          ref_lat: float = 47.3769, ref_lon: float = 8.5417,
                          ref_alt_msl: float = 408.0) -> SimState:
    """Convert a SimRecord (from drone_physics) to SimState for MAVLink."""
    roll, pitch, yaw = record.euler
    omega = angular_velocity if angular_velocity is not None else record.angular_velocity
    return SimState(
        time_s=record.t,
        position=record.position.copy(),
        velocity=record.velocity.copy(),
        roll=roll, pitch=pitch, yaw=yaw,
        rollspeed=omega[0], pitchspeed=omega[1], yawspeed=omega[2],
        thrust_pct=min(100.0, 100.0 * record.thrust / max_thrust),
        ref_lat=ref_lat, ref_lon=ref_lon, ref_alt_msl=ref_alt_msl,
    )


def _enu_to_gps(pos_enu: np.ndarray, ref_lat: float, ref_lon: float,
                ref_alt: float) -> Tuple[float, float, float]:
    """Convert ENU position [m] to GPS (lat, lon, alt_msl) degrees."""
    # Approximate conversion (good for small distances)
    lat_m_per_deg = 111320.0
    lon_m_per_deg = 111320.0 * math.cos(math.radians(ref_lat))
    lat = ref_lat + pos_enu[1] / lat_m_per_deg  # North = +Y in ENU
    lon = ref_lon + pos_enu[0] / lon_m_per_deg   # East = +X in ENU
    alt = ref_alt + pos_enu[2]                     # Up = +Z in ENU
    return lat, lon, alt


class MAVLinkBridge:
    """UDP MAVLink bridge between the standalone simulator and QGroundControl.

    Sends periodic telemetry (HEARTBEAT, ATTITUDE, GPS, HUD, SYS_STATUS)
    and receives commands (COMMAND_LONG, position targets).

    Usage:
        bridge = MAVLinkBridge()
        bridge.start()
        # In simulation loop:
        bridge.send_state(sim_state)
        cmds = bridge.get_received_commands()
        # ...
        bridge.stop()
    """

    def __init__(self, target_ip: str = "127.0.0.1", target_port: int = 14550,
                 listen_port: int = 14551, system_id: int = 1,
                 component_id: int = 1, mav_type: int = MAV_TYPE_QUADROTOR,
                 heartbeat_rate_hz: float = 1.0):
        self.target_ip = target_ip
        self.target_port = target_port
        self.listen_port = listen_port
        self.system_id = system_id
        self.component_id = component_id
        self.mav_type = mav_type
        self.heartbeat_rate_hz = heartbeat_rate_hz

        self._seq = 0
        self._sock: Optional[socket.socket] = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None
        self._commands: List[MAVCommand] = []
        self._position_targets: List[PositionTarget] = []
        self._lock = threading.Lock()
        self._last_state: Optional[SimState] = None

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def start(self):
        """Open UDP socket and start background threads."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self.listen_port))
        self._sock.settimeout(0.1)
        self._running = True

        self._rx_thread = threading.Thread(target=self._receive_loop,
                                            daemon=True)
        self._rx_thread.start()

        self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                            daemon=True)
        self._hb_thread.start()

    def stop(self):
        """Stop background threads and close socket."""
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def send_state(self, state: SimState):
        """Send full telemetry for current simulation state."""
        if self._sock is None:
            return

        self._last_state = state
        time_boot_ms = int(state.time_s * 1000)
        target = (self.target_ip, self.target_port)

        # ATTITUDE
        msg = build_attitude(
            roll=state.roll, pitch=state.pitch, yaw=state.yaw,
            rollspeed=state.rollspeed, pitchspeed=state.pitchspeed,
            yawspeed=state.yawspeed, time_boot_ms=time_boot_ms,
            system_id=self.system_id, component_id=self.component_id,
            seq=self._next_seq(),
        )
        self._sock.sendto(msg, target)

        # GLOBAL_POSITION_INT
        lat, lon, alt_msl = _enu_to_gps(state.position,
                                         state.ref_lat, state.ref_lon,
                                         state.ref_alt_msl)
        speed = np.linalg.norm(state.velocity)
        heading_deg = int(math.degrees(state.yaw)) % 360
        msg = build_global_position_int(
            lat_deg=lat, lon_deg=lon,
            alt_msl_m=alt_msl, alt_rel_m=state.position[2],
            vx_cm_s=int(state.velocity[1] * 100),  # ENU→NED: North=Y
            vy_cm_s=int(state.velocity[0] * 100),   # East=X
            vz_cm_s=int(-state.velocity[2] * 100),  # Down=-Z
            hdg_cdeg=heading_deg * 100,
            time_boot_ms=time_boot_ms,
            system_id=self.system_id, component_id=self.component_id,
            seq=self._next_seq(),
        )
        self._sock.sendto(msg, target)

        # VFR_HUD
        msg = build_vfr_hud(
            airspeed=speed, groundspeed=speed,
            heading_deg=heading_deg,
            throttle_pct=int(state.thrust_pct),
            alt_msl=alt_msl,
            climb_rate=state.velocity[2],
            system_id=self.system_id, component_id=self.component_id,
            seq=self._next_seq(),
        )
        self._sock.sendto(msg, target)

        # SYS_STATUS (periodic, simplified)
        msg = build_sys_status(
            system_id=self.system_id, component_id=self.component_id,
            seq=self._next_seq(),
        )
        self._sock.sendto(msg, target)

    def get_received_commands(self) -> List[MAVCommand]:
        """Return and clear any received COMMAND_LONG messages."""
        with self._lock:
            cmds = self._commands.copy()
            self._commands.clear()
        return cmds

    def get_received_position_targets(self) -> List[PositionTarget]:
        """Return and clear any received position target messages."""
        with self._lock:
            targets = self._position_targets.copy()
            self._position_targets.clear()
        return targets

    def _heartbeat_loop(self):
        """Send periodic heartbeat messages."""
        while self._running:
            if self._sock is not None:
                armed = self._last_state.armed if self._last_state else False
                msg = build_heartbeat(
                    system_id=self.system_id,
                    component_id=self.component_id,
                    mav_type=self.mav_type,
                    armed=armed,
                    seq=self._next_seq(),
                )
                try:
                    self._sock.sendto(msg, (self.target_ip, self.target_port))
                except OSError:
                    break
            time.sleep(1.0 / self.heartbeat_rate_hz)

    def _receive_loop(self):
        """Receive and parse incoming MAVLink messages."""
        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            result = decode_mavlink_v2(data)
            if result is None:
                continue

            msg_id, payload = result
            parsed = parse_mavlink_payload(msg_id, payload)
            if parsed is None:
                continue

            with self._lock:
                if isinstance(parsed, MAVCommand):
                    self._commands.append(parsed)
                elif isinstance(parsed, PositionTarget):
                    self._position_targets.append(parsed)
