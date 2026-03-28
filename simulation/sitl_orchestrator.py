#!/usr/bin/env python3
"""
SITL Orchestrator — Host-side pymavlink controller for 1..N ArduPilot SITL instances.

Connects to SITL containers via TCP, uploads missions, arms, monitors until
mission completion (disarm). Supports single-drone and swarm modes.

Usage:
    # Single drone
    python sitl_orchestrator.py --single --port 5760 --mission mission.waypoints

    # Swarm of 6
    python sitl_orchestrator.py --swarm --n 6 --base-port 5760 --mission-dir missions/swarm/
"""

import os
import sys
import time
import argparse
import glob

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class SITLDrone:
    """Manages a single MAVLink TCP connection to an ArduPilot SITL instance."""

    def __init__(self, drone_id: int, host: str = "127.0.0.1", port: int = 5760):
        self.drone_id = drone_id
        self.host = host
        self.port = port
        self.conn = None
        self.was_armed = False

    def connect(self, timeout: float = 30):
        from pymavlink import mavutil
        uri = f"tcp:{self.host}:{self.port}"
        print(f"  [drone_{self.drone_id}] Connecting to {uri}...")
        self.conn = mavutil.mavlink_connection(uri)
        self.conn.wait_heartbeat(timeout=timeout)
        print(f"  [drone_{self.drone_id}] Connected "
              f"(sys={self.conn.target_system})")

    def request_streams(self, rate: int = 4):
        for stream_id in range(13):
            self.conn.mav.request_data_stream_send(
                self.conn.target_system, self.conn.target_component,
                stream_id, rate, 1)

    def wait_gps_ekf(self, timeout: float = 90):
        """Wait for GPS 3D fix and EKF initialization."""
        from pymavlink import mavutil
        gps_ok = False
        ekf_ok = False
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "GPS_RAW_INT" and msg.fix_type >= 3 and not gps_ok:
                print(f"  [drone_{self.drone_id}] GPS 3D fix "
                      f"(fix={msg.fix_type}, sats={msg.satellites_visible})")
                gps_ok = True
            elif mtype == "EKF_STATUS_REPORT" and (msg.flags & 0x19) == 0x19 and not ekf_ok:
                print(f"  [drone_{self.drone_id}] EKF ready "
                      f"(flags=0x{msg.flags:04x})")
                ekf_ok = True
            elif mtype == "STATUSTEXT":
                text = getattr(msg, "text", "").rstrip("\x00")
                if text and any(k in text for k in ("Ready", "EKF3", "GPS", "Error")):
                    print(f"  [drone_{self.drone_id}] {text}")
            if gps_ok and ekf_ok:
                return True
            if time.time() - t0 > 60 and gps_ok:
                print(f"  [drone_{self.drone_id}] EKF timeout, proceeding with GPS")
                return True
        if not gps_ok:
            print(f"  [drone_{self.drone_id}] No GPS fix, "
                  "proceeding (SITL provides sim GPS)")
        return gps_ok

    def upload_mission_wpl(self, wpl_string: str) -> bool:
        """Upload a QGC WPL 110 mission string."""
        from pymavlink import mavwp
        import tempfile

        # mavwp.MAVWPLoader.load() requires a file path
        with tempfile.NamedTemporaryFile(mode="w", suffix=".waypoints",
                                         delete=False) as f:
            f.write(wpl_string)
            tmp_path = f.name

        try:
            loader = mavwp.MAVWPLoader()
            loader.load(tmp_path)
        finally:
            os.unlink(tmp_path)

        wp_count = loader.count()
        if wp_count == 0:
            print(f"  [drone_{self.drone_id}] FAIL: No waypoints in mission")
            return False

        # Clear existing mission
        self.conn.waypoint_clear_all_send()
        self.conn.recv_match(type="MISSION_ACK", blocking=True, timeout=10)

        # Send waypoint count
        self.conn.waypoint_count_send(wp_count)

        # Send each waypoint on request
        for i in range(wp_count):
            msg = self.conn.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                blocking=True, timeout=10)
            if msg is None:
                print(f"  [drone_{self.drone_id}] FAIL: Timeout waiting for wp {i}")
                return False
            self.conn.mav.send(loader.wp(msg.seq))

        # Wait for ACK
        ack = self.conn.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
        if ack is None or ack.type != 0:
            ack_type = ack.type if ack else "timeout"
            print(f"  [drone_{self.drone_id}] FAIL: Upload rejected "
                  f"(ack={ack_type})")
            return False

        print(f"  [drone_{self.drone_id}] Mission uploaded ({wp_count} waypoints)")
        return True

    def upload_mission_file(self, path: str) -> bool:
        with open(path) as f:
            return self.upload_mission_wpl(f.read())

    def arm_and_auto(self) -> bool:
        """Set GUIDED mode, arm, immediately switch to AUTO."""
        from pymavlink import mavutil

        # GUIDED mode
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)  # GUIDED = 4
        time.sleep(1)

        # Arm
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)

        # Quick wait for arm ACK
        for _ in range(20):
            msg = self.conn.recv_match(
                type=["COMMAND_ACK", "STATUSTEXT"],
                blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_ACK" and msg.command == 400:
                if msg.result == 0:
                    break
                else:
                    print(f"  [drone_{self.drone_id}] Arm rejected "
                          f"(result={msg.result}), retrying...")
                    time.sleep(2)
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 0, 0, 0, 0, 0, 0)

        # Switch to AUTO immediately
        self.conn.set_mode_auto()
        self.was_armed = False
        print(f"  [drone_{self.drone_id}] Armed + AUTO")
        return True

    def poll_once(self) -> str:
        """Non-blocking poll. Returns 'flying', 'complete', or 'timeout'."""
        from pymavlink import mavutil
        msg = self.conn.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            return "flying"
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            is_armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed:
                self.was_armed = True
            if self.was_armed and not is_armed:
                return "complete"
        elif mtype == "STATUSTEXT":
            text = getattr(msg, "text", "").rstrip("\x00")
            if text and ("Hit ground" in text or "Disarming" in text
                         or "Mission:" in text):
                print(f"  [drone_{self.drone_id}] {text}")
        return "flying"

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None


def run_single(host: str, port: int, mission_path: str,
               timeout: float = 300) -> bool:
    """Run a single SITL drone mission. Returns True on success."""
    drone = SITLDrone(0, host, port)
    try:
        drone.connect(timeout=30)
        drone.request_streams(rate=4)
        drone.wait_gps_ekf(timeout=90)

        if not drone.upload_mission_file(mission_path):
            return False
        drone.arm_and_auto()

        # Monitor until disarm
        start = time.time()
        last_status = 0
        while time.time() - start < timeout:
            result = drone.poll_once()
            if result == "complete":
                elapsed = int(time.time() - start)
                print(f"  [drone_0] Mission complete after {elapsed}s")
                return True
            elapsed = time.time() - start
            if elapsed - last_status >= 15:
                print(f"  [drone_0] [{int(elapsed)}s] flying...")
                last_status = elapsed

        print(f"  [drone_0] Timeout after {timeout}s")
        return False
    finally:
        drone.close()


def run_swarm(n_drones: int, base_port: int, port_step: int,
              mission_dir: str, timeout: float = 600,
              host: str = "127.0.0.1") -> dict:
    """Run N SITL drones with missions from mission_dir.

    Returns dict mapping drone_id to success (True/False).
    """
    drones = []
    results = {}

    try:
        # Phase 1: Connect all drones sequentially
        print(f"\n=== Connecting {n_drones} SITL drones ===")
        for i in range(n_drones):
            port = base_port + i * port_step
            drone = SITLDrone(i, host, port)
            drone.connect(timeout=30)
            drone.request_streams(rate=2)  # Lower rate for swarm
            drones.append(drone)

        # Phase 2: Wait for GPS/EKF on all
        print(f"\n=== Waiting for GPS/EKF ({n_drones} drones) ===")
        for drone in drones:
            drone.wait_gps_ekf(timeout=90)

        # Phase 3: Upload missions
        print(f"\n=== Uploading missions ===")
        for drone in drones:
            mission_path = os.path.join(
                mission_dir, f"drone_{drone.drone_id}.waypoints")
            if not os.path.exists(mission_path):
                print(f"  [drone_{drone.drone_id}] FAIL: Mission file not found: "
                      f"{mission_path}")
                results[drone.drone_id] = False
                continue
            if not drone.upload_mission_file(mission_path):
                results[drone.drone_id] = False
                continue

        # Phase 4: Arm all with stagger
        print(f"\n=== Arming {n_drones} drones (2s stagger) ===")
        for drone in drones:
            if drone.drone_id in results:
                continue  # Skip failed drones
            drone.arm_and_auto()
            if drone.drone_id < n_drones - 1:
                time.sleep(2)

        # Phase 5: Monitor all in round-robin
        print(f"\n=== Monitoring swarm (timeout={timeout}s) ===")
        start = time.time()
        active = {d.drone_id: d for d in drones if d.drone_id not in results}
        last_status = 0

        while active and (time.time() - start) < timeout:
            for drone_id in list(active.keys()):
                drone = active[drone_id]
                result = drone.poll_once()
                if result == "complete":
                    elapsed = int(time.time() - start)
                    print(f"  [drone_{drone_id}] Complete after {elapsed}s")
                    results[drone_id] = True
                    del active[drone_id]

            elapsed = time.time() - start
            if elapsed - last_status >= 15:
                n_active = len(active)
                n_done = len(results)
                print(f"  [{int(elapsed)}s] {n_active} active, {n_done} done")
                last_status = elapsed

        # Mark remaining as timed out
        for drone_id in active:
            print(f"  [drone_{drone_id}] Timed out")
            results[drone_id] = False

        return results

    finally:
        for drone in drones:
            drone.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SITL mission orchestrator")
    sub = parser.add_subparsers(dest="mode", required=True)

    # Single drone
    sp = sub.add_parser("single", help="Single drone SITL mission")
    sp.add_argument("--host", default="127.0.0.1")
    sp.add_argument("--port", type=int, default=5760)
    sp.add_argument("--mission", required=True, help="QGC WPL mission file")
    sp.add_argument("--timeout", type=int, default=300)

    # Swarm
    sp = sub.add_parser("swarm", help="Multi-drone SITL swarm")
    sp.add_argument("--host", default="127.0.0.1")
    sp.add_argument("--n", type=int, required=True, help="Number of drones")
    sp.add_argument("--base-port", type=int, default=5760)
    sp.add_argument("--port-step", type=int, default=10)
    sp.add_argument("--mission-dir", required=True, help="Dir with drone_N.waypoints")
    sp.add_argument("--timeout", type=int, default=600)

    args = parser.parse_args()

    if args.mode == "single":
        ok = run_single(args.host, args.port, args.mission, args.timeout)
        sys.exit(0 if ok else 4)
    elif args.mode == "swarm":
        results = run_swarm(args.n, args.base_port, args.port_step,
                            args.mission_dir, args.timeout, args.host)
        n_ok = sum(1 for v in results.values() if v)
        n_total = len(results)
        print(f"\n=== Swarm results: {n_ok}/{n_total} completed ===")
        sys.exit(0 if n_ok == n_total else 4)
