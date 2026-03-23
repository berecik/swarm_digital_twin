"""
Standalone Drone Physics Engine - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Quadrotor rigid-body simulation with:
  - Gravity, thrust, aerodynamic drag
  - Attitude via rotation matrix (roll/pitch/yaw)
  - Angular velocity and torque dynamics
  - Simple PID position controller
  - No ROS 2 dependency — pure numpy
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


# ── Constants ────────────────────────────────────────────────────────────────

GRAVITY = 9.81  # m/s^2


# ── Data classes ─────────────────────────────────────────────────────────────

@dataclass
class DroneParams:
    mass: float = 1.5            # kg
    arm_length: float = 0.25     # m  (motor-to-center)
    drag_coeff: float = 0.1      # linear drag  N/(m/s)
    ang_drag_coeff: float = 0.02 # angular drag  N·m/(rad/s)
    max_thrust: float = 25.0     # N  (total, all motors)
    max_torque: float = 5.0      # N·m per axis
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([0.02, 0.02, 0.04])  # kg·m²
    )


@dataclass
class DroneState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))     # body→world
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body frame


@dataclass
class DroneCommand:
    thrust: float = 0.0              # N  (total, along body-Z up)
    torque: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body [roll, pitch, yaw]


# ── Rotation helpers ─────────────────────────────────────────────────────────

def euler_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr   ],
    ])


def rotation_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0],
    ])


# ── Physics step ─────────────────────────────────────────────────────────────

def physics_step(state: DroneState, cmd: DroneCommand,
                 params: DroneParams, dt: float) -> DroneState:
    """Advance the drone state by dt seconds."""

    # Clamp commands
    thrust = np.clip(cmd.thrust, 0.0, params.max_thrust)
    torque = np.clip(cmd.torque, -params.max_torque, params.max_torque)

    R = state.rotation
    omega = state.angular_velocity

    # ── Forces in world frame ────────────────────────────────────────────
    gravity_force = np.array([0.0, 0.0, -params.mass * GRAVITY])

    # Thrust along body z-axis (third column of R)
    thrust_world = R @ np.array([0.0, 0.0, thrust])

    # Aerodynamic drag (world frame, opposes velocity)
    drag_force = -params.drag_coeff * state.velocity

    total_force = gravity_force + thrust_world + drag_force

    # ── Linear dynamics ──────────────────────────────────────────────────
    accel = total_force / params.mass
    new_vel = state.velocity + accel * dt
    new_pos = state.position + state.velocity * dt + 0.5 * accel * dt**2

    # Ground constraint
    if new_pos[2] < 0.0:
        new_pos[2] = 0.0
        new_vel[2] = max(new_vel[2], 0.0)

    # ── Angular dynamics (body frame) ────────────────────────────────────
    ang_drag = -params.ang_drag_coeff * omega
    I = params.inertia
    I_inv = np.linalg.inv(I)

    # Euler's rotation equation: I·alpha = torque - omega x (I·omega) - drag
    alpha = I_inv @ (torque - np.cross(omega, I @ omega) + ang_drag)
    new_omega = omega + alpha * dt

    # ── Rotation integration (first-order) ───────────────────────────────
    dR = skew(new_omega * dt)
    new_R = R @ (np.eye(3) + dR)

    # Re-orthogonalize via SVD to prevent drift
    U, _, Vt = np.linalg.svd(new_R)
    new_R = U @ Vt

    return DroneState(
        position=new_pos,
        velocity=new_vel,
        rotation=new_R,
        angular_velocity=new_omega,
    )


# ── PID Controller ───────────────────────────────────────────────────────────

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float,
                 limit: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.limit, self.limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, -self.limit, self.limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class PositionController:
    """Cascaded position → attitude → thrust controller."""

    def __init__(self, params: DroneParams):
        self.params = params

        # Position PID (outputs desired acceleration)
        self.pid_x = PIDController(kp=4.0, ki=0.5, kd=3.0, limit=8.0)
        self.pid_y = PIDController(kp=4.0, ki=0.5, kd=3.0, limit=8.0)
        self.pid_z = PIDController(kp=6.0, ki=1.0, kd=4.0, limit=15.0)

        # Attitude PID (outputs torque)
        self.pid_roll = PIDController(kp=8.0, ki=0.1, kd=2.0, limit=params.max_torque)
        self.pid_pitch = PIDController(kp=8.0, ki=0.1, kd=2.0, limit=params.max_torque)
        self.pid_yaw = PIDController(kp=4.0, ki=0.05, kd=1.0, limit=params.max_torque)

    def compute(self, state: DroneState, target_pos: np.ndarray,
                target_yaw: float, dt: float) -> DroneCommand:
        # Position error
        err = target_pos - state.position

        # Desired acceleration (world frame)
        ax = self.pid_x.update(err[0], dt)
        ay = self.pid_y.update(err[1], dt)
        az = self.pid_z.update(err[2], dt)

        # Add gravity compensation
        desired_accel = np.array([ax, ay, az + GRAVITY])

        # Thrust magnitude = mass * |desired_accel|
        thrust = self.params.mass * np.linalg.norm(desired_accel)
        thrust = np.clip(thrust, 0.0, self.params.max_thrust)

        # Desired body-z direction (unit vector)
        if np.linalg.norm(desired_accel) > 1e-6:
            z_des = desired_accel / np.linalg.norm(desired_accel)
        else:
            z_des = np.array([0.0, 0.0, 1.0])

        # Construct desired rotation from z_des and target_yaw
        x_c = np.array([np.cos(target_yaw), np.sin(target_yaw), 0.0])
        y_des = np.cross(z_des, x_c)
        y_norm = np.linalg.norm(y_des)
        if y_norm > 1e-6:
            y_des /= y_norm
        else:
            y_des = np.array([0.0, 1.0, 0.0])
        x_des = np.cross(y_des, z_des)

        R_des = np.column_stack([x_des, y_des, z_des])

        # Attitude error (rotation from current to desired)
        R_err = state.rotation.T @ R_des
        # Extract error angles (small-angle: use skew-symmetric part)
        err_roll = R_err[2, 1] - R_err[1, 2]
        err_pitch = R_err[0, 2] - R_err[2, 0]
        err_yaw = R_err[1, 0] - R_err[0, 1]

        torque = np.array([
            self.pid_roll.update(err_roll, dt),
            self.pid_pitch.update(err_pitch, dt),
            self.pid_yaw.update(err_yaw, dt),
        ])

        return DroneCommand(thrust=thrust, torque=torque)

    def reset(self):
        for pid in [self.pid_x, self.pid_y, self.pid_z,
                    self.pid_roll, self.pid_pitch, self.pid_yaw]:
            pid.reset()


# ── Simulation runner ────────────────────────────────────────────────────────

@dataclass
class SimRecord:
    t: float
    position: np.ndarray
    velocity: np.ndarray
    euler: Tuple[float, float, float]
    thrust: float
    angular_velocity: np.ndarray


def run_simulation(waypoints: List[np.ndarray],
                   params: Optional[DroneParams] = None,
                   dt: float = 0.005,
                   waypoint_radius: float = 0.5,
                   hover_time: float = 2.0,
                   max_time: float = 120.0) -> List[SimRecord]:
    """
    Fly through waypoints in order. Hover at each for hover_time seconds.
    Returns a list of SimRecord for every timestep.
    """
    if params is None:
        params = DroneParams()

    state = DroneState()
    controller = PositionController(params)
    records: List[SimRecord] = []

    wp_idx = 0
    hover_timer = 0.0
    t = 0.0

    while t < max_time and wp_idx < len(waypoints):
        target = waypoints[wp_idx]
        cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)
        state = physics_step(state, cmd, params, dt)

        roll, pitch, yaw = rotation_to_euler(state.rotation)
        records.append(SimRecord(
            t=t,
            position=state.position.copy(),
            velocity=state.velocity.copy(),
            euler=(roll, pitch, yaw),
            thrust=cmd.thrust,
            angular_velocity=state.angular_velocity.copy(),
        ))

        # Check waypoint reached
        dist = np.linalg.norm(state.position - target)
        speed = np.linalg.norm(state.velocity)
        if dist < waypoint_radius and speed < 1.0:
            hover_timer += dt
            if hover_timer >= hover_time:
                wp_idx += 1
                hover_timer = 0.0
                controller.reset()
        else:
            hover_timer = 0.0

        t += dt

    return records
