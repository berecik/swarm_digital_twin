#!/usr/bin/env python3
"""
Simulation Bridge - UDP server bridging Rust DriverCore to Python physics.

Listens for JSON-encoded DriverAction messages from the Rust
sim_realtime_driver binary, feeds them into the physics engine,
and returns DriverStatus + position as JSON.

Message contract:
  Action (Rust → Python):  [{"type":"RequestOffboard"}, {"type":"PublishSetpoint","x":0,"y":0,"z":5}]
  Status (Python → Rust):  {"nav_state":14,"arming_state":2,"position":[0.0,0.0,4.5]}

Usage:
    python sim_bridge.py [--port 9100] [--hz 50]
"""

import json
import socket
import sys
import os
import time
import argparse

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import (
    DroneParams,
    DroneState,
    DroneCommand,
    PositionController,
    physics_step,
    make_holybro_x500,
)
from wind_model import WindField
from sensor_models import GPSNoise


class SimBridge:
    """UDP server that bridges Rust driver actions to Python physics."""

    def __init__(self, port: int = 9100, dt: float = 0.02, params: DroneParams = None,
                 sensor_noise: bool = True):
        self.port = port
        self.dt = dt
        self.params = params or make_holybro_x500()

        # Physics state
        self.state = DroneState(
            position=np.array([0.0, 0.0, 0.0]),
            velocity=np.zeros(3),
            rotation=np.eye(3),
            angular_velocity=np.zeros(3),
        )
        self.controller = PositionController(self.params)
        self.wind = WindField(wind_speed=0.0, wind_direction=np.array([1.0, 0.0, 0.0]))

        # Sensor noise models (GPS perturbation on reported position)
        self.gps_noise = GPSNoise() if sensor_noise else None

        # Vehicle status FSM (mirrors PX4 nav/arming states)
        self.nav_state = 0
        self.arming_state = 0
        self.target_position = np.array([0.0, 0.0, 0.0])

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(1.0)
        self.client_addr = None

    def process_actions(self, actions: list):
        """Process a batch of DriverAction messages."""
        for action in actions:
            action_type = action.get("type")
            if action_type == "RequestOffboard":
                self.nav_state = 14  # PX4 NAVIGATION_STATE_OFFBOARD
            elif action_type == "RequestArm":
                self.arming_state = 2  # ARMING_STATE_ARMED
            elif action_type == "PublishSetpoint":
                self.target_position = np.array([
                    action.get("x", 0.0),
                    action.get("y", 0.0),
                    action.get("z", 0.0),
                ])

    def step_physics(self):
        """Advance physics by one timestep."""
        if self.arming_state == 2:
            cmd = self.controller.compute(
                self.state, self.target_position, 0.0, self.dt
            )
        else:
            cmd = DroneCommand(thrust=0.0, torque=np.zeros(3))

        self.state = physics_step(self.state, cmd, self.params, self.dt, wind=self.wind)

    def make_status_message(self) -> dict:
        """Build the status JSON to send back to Rust.

        If sensor noise is enabled, the reported position is routed through
        the GPSNoise.apply_local API so that white noise, bias drift, and
        sigma parameters stay consistent with the rest of the sensor pipeline.
        """
        pos = self.state.position.copy()
        if self.gps_noise is not None:
            pos = self.gps_noise.apply_local(pos, dt=self.dt)
        return {
            "nav_state": self.nav_state,
            "arming_state": self.arming_state,
            "position": pos.tolist(),
        }

    def run(self, max_steps: int = 0):
        """Main server loop. Runs until interrupted or max_steps reached."""
        print(f"[sim_bridge] listening on 127.0.0.1:{self.port}")
        step = 0

        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                self.client_addr = addr

                actions = json.loads(data.decode("utf-8"))
                self.process_actions(actions)

                # Run multiple physics substeps per control step (50Hz physics / 10Hz control)
                substeps = max(1, int(0.1 / self.dt))
                for _ in range(substeps):
                    self.step_physics()

                response = json.dumps(self.make_status_message()).encode("utf-8")
                self.sock.sendto(response, addr)

                step += 1
                if max_steps > 0 and step >= max_steps:
                    print(f"[sim_bridge] reached {max_steps} steps, stopping")
                    break

            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break
            except json.JSONDecodeError as e:
                print(f"[sim_bridge] bad JSON: {e}")
                continue

        self.sock.close()
        print("[sim_bridge] stopped")


def main():
    parser = argparse.ArgumentParser(description="Simulation bridge (UDP)")
    parser.add_argument("--port", type=int, default=9100, help="UDP port (default: 9100)")
    parser.add_argument("--hz", type=float, default=50.0, help="Physics rate Hz (default: 50)")
    parser.add_argument("--max-steps", type=int, default=0, help="Max control steps (0=unlimited)")
    args = parser.parse_args()

    bridge = SimBridge(port=args.port, dt=1.0 / args.hz)
    bridge.run(max_steps=args.max_steps)


if __name__ == "__main__":
    main()
