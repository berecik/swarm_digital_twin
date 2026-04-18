#!/usr/bin/env python3
"""
Physics Live Replay — run a Python physics simulation and stream results
to the Run-time View (FastAPI + Three.js) in real time.

Pipeline: run_simulation() → SimRecord[] → MAVLinkBridge.run_replay()
          → UDP → MAVLinkLiveSource → TelemetryQueue → /ws/telemetry → browser

Usage:
    # Single drone (default scenario)
    python -m simulation.physics_live_replay

    # Single drone, looping
    python -m simulation.physics_live_replay --loop

    # Swarm (6 drones, first drone streamed)
    python -m simulation.physics_live_replay --swarm --drones 6

    # Replay existing .npz file
    python -m simulation.physics_live_replay --replay simulation/scenario_data.npz

    # Custom FPS and ports
    python -m simulation.physics_live_replay --fps 30 --http-port 8765 --mav-port 14550
"""

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# Ensure sibling modules are importable regardless of cwd.
_SIM_DIR = Path(__file__).resolve().parent
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from drone_physics import (  # noqa: E402
    DroneParams,
    SimRecord,
    run_simulation,
)
from mavlink_bridge import MAVLinkBridge  # noqa: E402


def _default_waypoints() -> List[np.ndarray]:
    """Simple triangle pattern suitable for a quick live demo."""
    return [
        np.array([0.0, 0.0, 5.0]),
        np.array([15.0, 0.0, 7.0]),
        np.array([15.0, 15.0, 10.0]),
        np.array([0.0, 15.0, 7.0]),
        np.array([0.0, 0.0, 5.0]),
    ]


def run_physics_simulation(
    waypoints: Optional[List[np.ndarray]] = None,
    params: Optional[DroneParams] = None,
    dt: float = 0.02,
    max_time: float = 60.0,
) -> List[SimRecord]:
    """Run the Python physics engine and return the record list."""
    if waypoints is None:
        waypoints = _default_waypoints()
    if params is None:
        params = DroneParams(mass=1.5, max_thrust=25.0)
    return run_simulation(
        waypoints=waypoints,
        params=params,
        dt=dt,
        waypoint_radius=0.5,
        hover_time=1.0,
        max_time=max_time,
    )


def load_npz_records(path: str) -> List[SimRecord]:
    """Load SimRecord list from a scenario_data.npz file.

    Gracefully handles missing optional keys (euler, thrust, ang_vel,
    euler_rates) so older .npz files can still be replayed.
    """
    # allow_pickle needed for .npz files with object arrays (drone_ids)
    data = np.load(path, allow_pickle=True)  # noqa: S301
    t = data["t"]
    pos = data["pos"]
    vel = data["vel"]
    euler = data["euler"] if "euler" in data else np.zeros((len(t), 3))
    thrust = data["thrust"] if "thrust" in data else np.zeros(len(t))
    ang_vel = data["ang_vel"] if "ang_vel" in data else np.zeros((len(t), 3))
    euler_rates = data["euler_rates"] if "euler_rates" in data else None
    records = []
    for i in range(len(t)):
        er = euler_rates[i] if euler_rates is not None else None
        records.append(SimRecord(
            t=float(t[i]),
            position=pos[i].copy(),
            velocity=vel[i].copy(),
            euler=tuple(euler[i]),
            thrust=float(thrust[i]),
            angular_velocity=ang_vel[i].copy(),
            euler_rates=er,
        ))
    return records


def load_swarm_npz_records(path: str, drone_index: int = 0) -> List[SimRecord]:
    """Load one drone's trajectory from a swarm_data.npz file."""
    data = np.load(path, allow_pickle=True)
    t = data["t"]
    positions = data["positions"]  # shape: (timesteps, num_drones, 3)
    velocities = data["velocities"]
    records = []
    for i in range(len(t)):
        records.append(SimRecord(
            t=float(t[i]),
            position=positions[i][drone_index].copy(),
            velocity=velocities[i][drone_index].copy(),
            euler=(0.0, 0.0, 0.0),
            thrust=0.5,
            angular_velocity=np.zeros(3),
        ))
    return records


def run_physics_live(
    records: List[SimRecord],
    fps: float = 50.0,
    loop: bool = False,
    http_port: int = 8765,
    mav_port: int = 14550,
    open_browser: bool = True,
    waypoints: Optional[List[np.ndarray]] = None,
    swarm_records_per_drone: Optional[Dict[int, List[SimRecord]]] = None,
    waypoints_per_drone: Optional[Dict[int, List[np.ndarray]]] = None,
    record_bin: Optional[str] = None,
) -> None:
    """
    Stream pre-computed SimRecords to the live viewer.

    Starts the MAVLink receiver, then the bridge + replay thread, then
    uvicorn.  The receiver **must** bind before the replay starts so
    that no UDP packets are lost — without this ordering the drone mesh
    never moves in the browser (the packets arrive at an unbound port
    and are silently dropped by the OS).
    """
    import runtime_view.server as _srv  # noqa: E402

    print(f"Physics Live Replay: {len(records)} records at {fps} FPS "
          f"(loop={loop})")
    print(f"  MAVLink UDP: 127.0.0.1:{mav_port}")
    print(f"  HTTP:        http://127.0.0.1:{http_port}/live")

    # Publish waypoints so the live viewer can render them.
    # For multi-drone, waypoints_per_drone is a dict {drone_id: [wp, ...]}.
    if waypoints_per_drone is not None:
        _srv.mission_waypoints = {
            str(did): [wp.tolist() for wp in wps]
            for did, wps in waypoints_per_drone.items()
        }
    elif waypoints is not None:
        _srv.mission_waypoints = {"1": [wp.tolist() for wp in waypoints]}

    # 1. Bind the MAVLink UDP receiver FIRST so no replay packets are lost.
    # Optionally attach a DataFlash .BIN recorder via the sample hook.
    bin_recorder = None
    if record_bin:
        from dataflash_recorder import DataFlashRecorder
        bin_recorder = DataFlashRecorder(record_bin)
        print(f"  Recording to: {record_bin}")

        from live_telemetry import MAVLinkLiveSource  # noqa: E402
        _orig_start = _srv.start_telemetry

        def _start_with_hook(listen_port=14550, listen_ip="0.0.0.0"):
            _orig_start(listen_port=listen_port, listen_ip=listen_ip)
            if _srv.live_source is not None:
                _srv.live_source._sample_hook = bin_recorder.record_sample

        _start_with_hook(listen_port=mav_port)
    else:
        _srv.start_telemetry(listen_port=mav_port)

    # 2. Create bridge(s).  For multi-drone, one bridge per drone with
    #    a unique system_id so the receiver can demultiplex.
    bridges: List[MAVLinkBridge] = []
    if swarm_records_per_drone:
        drone_ids = sorted(swarm_records_per_drone.keys())
        for did in drone_ids:
            b = MAVLinkBridge(
                target_ip="127.0.0.1",
                target_port=mav_port,
                listen_port=0,
                system_id=did,
            )
            b.start()
            bridges.append(b)
        print(f"  Bridges: {len(bridges)} drones "
              f"(system_ids {drone_ids})")
    else:
        b = MAVLinkBridge(
            target_ip="127.0.0.1",
            target_port=mav_port,
            listen_port=0,
        )
        b.start()
        bridges.append(b)

    # 3. Open the browser after a short delay so uvicorn has time to boot.
    if open_browser:
        def _open_browser():
            time.sleep(1.5)
            url = f"http://127.0.0.1:{http_port}/live"
            try:
                import webbrowser
                webbrowser.open(url)
            except Exception:
                pass
        threading.Thread(target=_open_browser, daemon=True).start()

    # 4. Start replay thread(s).
    if swarm_records_per_drone:
        # Multi-drone: interleave all drones in a single thread by
        # stepping through timesteps and sending each drone's state.
        drone_ids = sorted(swarm_records_per_drone.keys())
        bridge_map = {did: br for did, br in zip(drone_ids, bridges)}

        def _replay_swarm():
            from mavlink_bridge import sim_state_from_record
            try:
                dt = 1.0 / fps
                while True:
                    # All record lists have the same length (from SwarmRecord).
                    n_steps = len(swarm_records_per_drone[drone_ids[0]])
                    for step in range(n_steps):
                        t0 = time.time()
                        for did in drone_ids:
                            rec = swarm_records_per_drone[did][step]
                            state = sim_state_from_record(rec)
                            bridge_map[did].send_state(state)
                        elapsed = time.time() - t0
                        if elapsed < dt:
                            time.sleep(dt - elapsed)
                    if not loop:
                        break
            except Exception as e:
                print(f"Swarm replay error: {e}")
            finally:
                if not loop:
                    print("Replay finished — server stays up. "
                          "Press Ctrl-C to quit.")

        threading.Thread(target=_replay_swarm, daemon=True).start()
    else:
        def _replay():
            try:
                bridges[0].run_replay(records, fps=fps, loop=loop)
            except Exception as e:
                print(f"Replay error: {e}")
            finally:
                if not loop:
                    print("Replay finished — server stays up. "
                          "Press Ctrl-C to quit.")
        threading.Thread(target=_replay, daemon=True).start()

    try:
        # 5. Start uvicorn (blocking).  Source is already running so
        #    pass start_source=False to avoid a double-bind.
        _srv.run_server(
            host="127.0.0.1",
            port=http_port,
            listen_port=mav_port,
            start_source=False,
        )
    except KeyboardInterrupt:
        pass
    finally:
        for b in bridges:
            b.stop()
        _srv.stop_telemetry()
        if bin_recorder is not None:
            bin_recorder.close()
            print(f"  .BIN saved: {record_bin}")
        print("Physics Live Replay: shutdown complete")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="physics_live_replay",
        description="Run a Python physics simulation and stream to the "
                    "live Three.js viewer in real time.",
    )
    parser.add_argument(
        "--replay", metavar="NPZ_FILE",
        help="Replay an existing .npz file instead of running a new sim",
    )
    parser.add_argument(
        "--swarm", action="store_true",
        help="Run a swarm simulation (streams first drone by default)",
    )
    parser.add_argument(
        "--drones", type=int, default=6,
        help="Number of drones for swarm mode (default: 6)",
    )
    parser.add_argument(
        "--drone-index", type=int, default=0,
        help="Which drone to stream in swarm mode (default: 0 = first)",
    )
    parser.add_argument(
        "--fps", type=float, default=50.0,
        help="Playback FPS (default: 50)",
    )
    parser.add_argument(
        "--loop", action="store_true",
        help="Loop the replay indefinitely",
    )
    parser.add_argument(
        "--http-port", type=int, default=8765,
        help="HTTP server port (default: 8765)",
    )
    parser.add_argument(
        "--mav-port", type=int, default=14550,
        help="MAVLink UDP port (default: 14550)",
    )
    parser.add_argument(
        "--no-browser", action="store_true",
        help="Don't open the browser automatically",
    )
    parser.add_argument(
        "--max-time", type=float, default=60.0,
        help="Max simulation time in seconds (default: 60)",
    )
    parser.add_argument(
        "--record-bin", metavar="FILE",
        help="Record telemetry to an ArduPilot-compatible .BIN file",
    )

    args = parser.parse_args(argv)

    # ── Build records ────────────────────────────────────────────────
    waypoints = None

    if args.replay:
        path = args.replay
        if not os.path.isfile(path):
            print(f"Error: file not found: {path}", file=sys.stderr)
            return 1
        # Detect swarm vs single by checking for 'positions' key
        data = np.load(path, allow_pickle=True)
        if "positions" in data:
            print(f"Loading swarm data from {path} "
                  f"(drone index {args.drone_index})")
            records = load_swarm_npz_records(path, args.drone_index)
            if "waypoints" in data:
                wps = data["waypoints"]
                idx = min(args.drone_index, len(wps) - 1)
                waypoints = [np.array(w) for w in wps[idx]]
        else:
            print(f"Loading single-drone data from {path}")
            records = load_npz_records(path)
            if "waypoints" in data:
                waypoints = [np.array(w) for w in data["waypoints"]]
    elif args.swarm:
        print(f"Running swarm simulation ({args.drones} drones)...")
        from swarm_scenario import (  # noqa: E402
            build_six_agent_ring_waypoints,
            make_holybro_x500,
            make_terrain,
        )
        from drone_physics import run_swarm_simulation  # noqa: E402
        from wind_model import WindField  # noqa: E402

        np.random.seed(42)
        drone_waypoints = build_six_agent_ring_waypoints()
        # Trim to requested drone count
        drone_ids = sorted(drone_waypoints.keys())[:args.drones]
        drone_waypoints = {k: drone_waypoints[k] for k in drone_ids}

        swarm_records = run_swarm_simulation(
            drone_waypoints,
            params=make_holybro_x500(),
            dt=0.02,
            hover_time=0.5,
            max_time=args.max_time,
            wind=WindField(
                wind_speed=1.05,
                wind_direction=np.array([1.0, 0.3, 0.0]),
                turbulence_type="constant",
            ),
            terrain=make_terrain(),
            min_separation=1.5,
        )
        # Build per-drone SimRecord lists with sequential system_ids
        # starting at 1 so the MAVLink bridges demultiplex correctly.
        swarm_records_per_drone: Dict[int, List[SimRecord]] = {}
        wp_per_drone: Dict[int, List[np.ndarray]] = {}
        for i, did in enumerate(drone_ids):
            sys_id = i + 1  # MAVLink system_id 1..N
            wp_per_drone[sys_id] = drone_waypoints[did]
            per_drone = []
            for sr in swarm_records:
                per_drone.append(SimRecord(
                    t=sr.t,
                    position=sr.positions[i].copy(),
                    velocity=sr.velocities[i].copy(),
                    euler=(0.0, 0.0, 0.0),
                    thrust=0.5,
                    angular_velocity=np.zeros(3),
                ))
            swarm_records_per_drone[sys_id] = per_drone
        print(f"Swarm simulation complete: {len(swarm_records)} steps, "
              f"{len(drone_ids)} drones")
    else:
        print("Running single-drone physics simulation...")
        waypoints = _default_waypoints()
        records = run_physics_simulation(waypoints=waypoints,
                                         max_time=args.max_time)
        print(f"Simulation complete: {len(records)} steps, "
              f"{records[-1].t:.1f}s")

    # ── Stream to live viewer ────────────────────────────────────────
    if args.swarm and swarm_records_per_drone:
        # Use the first drone's records as the fallback single-drone
        # record list (keeps run_physics_live's signature happy).
        first_id = sorted(swarm_records_per_drone.keys())[0]
        run_physics_live(
            swarm_records_per_drone[first_id],
            fps=args.fps,
            loop=args.loop,
            http_port=args.http_port,
            mav_port=args.mav_port,
            open_browser=not args.no_browser,
            swarm_records_per_drone=swarm_records_per_drone,
            waypoints_per_drone=wp_per_drone,
            record_bin=args.record_bin,
        )
    else:
        if not records:
            print("Error: no records produced", file=sys.stderr)
            return 1
        run_physics_live(
            records,
            fps=args.fps,
            loop=args.loop,
            http_port=args.http_port,
            mav_port=args.mav_port,
            open_browser=not args.no_browser,
            waypoints=waypoints,
            record_bin=args.record_bin,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
