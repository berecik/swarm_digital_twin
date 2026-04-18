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
from typing import List, Optional

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
    """Load SimRecord list from a scenario_data.npz file."""
    data = np.load(path, allow_pickle=True)
    t = data["t"]
    pos = data["pos"]
    vel = data["vel"]
    euler = data["euler"]
    thrust = data["thrust"]
    ang_vel = data["ang_vel"]
    euler_rates = data.get("euler_rates")
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
    if waypoints is not None:
        _srv.mission_waypoints = [wp.tolist() for wp in waypoints]

    # 1. Bind the MAVLink UDP receiver FIRST so no replay packets are lost.
    _srv.start_telemetry(listen_port=mav_port)

    # 2. Start the bridge that sends MAVLink to that receiver.
    bridge = MAVLinkBridge(
        target_ip="127.0.0.1",
        target_port=mav_port,
        listen_port=0,  # ephemeral — we don't receive commands
    )
    bridge.start()

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

    # 4. Start the replay in a background thread.  The receiver is
    #    already listening so every packet will be captured.
    def _replay():
        try:
            bridge.run_replay(records, fps=fps, loop=loop)
        except Exception as e:
            print(f"Replay error: {e}")
        finally:
            if not loop:
                print("Replay finished — server stays up for inspection. "
                      "Press Ctrl-C to quit.")

    replay_thread = threading.Thread(target=_replay, daemon=True)
    replay_thread.start()

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
        bridge.stop()
        _srv.stop_telemetry()
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
        # Convert swarm records to single-drone SimRecords for the bridge
        idx = min(args.drone_index, len(drone_ids) - 1)
        waypoints = drone_waypoints[drone_ids[idx]]
        records = []
        for sr in swarm_records:
            records.append(SimRecord(
                t=sr.t,
                position=sr.positions[idx].copy(),
                velocity=sr.velocities[idx].copy(),
                euler=(0.0, 0.0, 0.0),
                thrust=0.5,
                angular_velocity=np.zeros(3),
            ))
        print(f"Swarm simulation complete: {len(records)} steps, "
              f"streaming drone {idx}")
    else:
        print("Running single-drone physics simulation...")
        waypoints = _default_waypoints()
        records = run_physics_simulation(waypoints=waypoints,
                                         max_time=args.max_time)
        print(f"Simulation complete: {len(records)} steps, "
              f"{records[-1].t:.1f}s")

    if not records:
        print("Error: no records produced", file=sys.stderr)
        return 1

    # ── Stream to live viewer ────────────────────────────────────────
    run_physics_live(
        records,
        fps=args.fps,
        loop=args.loop,
        http_port=args.http_port,
        mav_port=args.mav_port,
        open_browser=not args.no_browser,
        waypoints=waypoints,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
