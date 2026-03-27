"""
Standalone Drone Physics Engine - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

UAV rigid-body simulation with:
  - Gravity, thrust, aerodynamic drag (linear or quadratic)
  - Attitude via rotation matrix (roll/pitch/yaw)
  - Angular velocity and torque dynamics
  - Body-frame force decomposition (Valencia et al. 2025, Eq. 3)
  - Quadratic aerodynamic drag with ISA atmosphere model (Eq. 5)
  - Aerodynamic lift with AoA-dependent coefficients and stall (Eq. 6)
  - Fixed-wing support: FixedWingAero with pre/post-stall CL/CD curves
  - Wind perturbation support (Eq. 5-7)
  - Simple PID position controller
  - No ROS 2 dependency — pure numpy
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import warnings


# ── Constants ────────────────────────────────────────────────────────────────

GRAVITY = 9.81  # m/s^2


# ── Atmosphere & Aerodynamics ────────────────────────────────────────────────

@dataclass
class Atmosphere:
    """ISA-based atmosphere model (troposphere)."""
    rho_sea_level: float = 1.225      # kg/m³
    altitude_msl: float = 0.0         # m above mean sea level

    @property
    def rho(self) -> float:
        """Air density at configured altitude using ISA troposphere formula."""
        return self.rho_sea_level * (1 - 2.2558e-5 * self.altitude_msl) ** 4.2559


@dataclass
class AeroCoefficients:
    """Aerodynamic coefficients for quadratic drag model (paper Eq. 5)."""
    reference_area: float = 0.04      # m² (quadrotor frontal area)
    C_D: float = 1.0                  # drag coefficient
    C_L: float = 0.0                  # lift coefficient (0 for quadrotor)

    def get_CD(self, alpha: float) -> float:
        """Return drag coefficient at angle of attack alpha [rad]."""
        return self.C_D

    def get_CL(self, alpha: float) -> float:
        """Return lift coefficient at angle of attack alpha [rad]."""
        return self.C_L

    def get_CM(self, alpha: float) -> float:
        """Return pitching moment coefficient at angle of attack alpha [rad]."""
        return 0.0


@dataclass
class FixedWingAero(AeroCoefficients):
    """AoA-dependent aerodynamic model with stall (paper Table 3, Fig. 5).

    Pre-stall:  CL = C_La * (alpha - alpha_0),  CD = C_D0 + C_Da * alpha^2
    Post-stall: CL = C_La_stall * (alpha - alpha_0),  CD = C_D0 + C_Da_stall * alpha^2
    """
    alpha_0: float = 0.05236          # zero-lift AoA [rad] (~3 deg)
    alpha_stall: float = 0.26180      # stall angle [rad] (~15 deg)
    C_D0: float = 0.02                # parasitic drag at zero AoA
    C_La: float = 3.50141             # lift curve slope (pre-stall) [1/rad]
    C_Da: float = 0.63662             # induced drag slope (pre-stall) [1/rad^2]
    C_La_stall: float = -1.1459       # lift slope (post-stall) [1/rad]
    C_Da_stall: float = 2.29183       # drag slope (post-stall) [1/rad^2]
    C_Ma: float = -0.2040            # pitching moment slope (pre-stall) [1/rad] (Table 3)
    C_Ma_stall: float = -0.1313      # pitching moment slope (post-stall) [1/rad] (Table 3)
    chord: float = 0.235             # mean aerodynamic chord [m] (Table 2)

    def get_CL(self, alpha: float) -> float:
        """AoA-dependent lift coefficient with stall model."""
        if abs(alpha) < self.alpha_stall:
            return self.C_La * (alpha - self.alpha_0)
        else:
            return self.C_La_stall * (alpha - self.alpha_0)

    def get_CD(self, alpha: float) -> float:
        """AoA-dependent drag coefficient with stall model."""
        if abs(alpha) < self.alpha_stall:
            return self.C_D0 + self.C_Da * alpha**2
        else:
            return self.C_D0 + self.C_Da_stall * alpha**2

    def get_CM(self, alpha: float) -> float:
        """AoA-dependent pitching moment coefficient (paper Table 3)."""
        if abs(alpha) < self.alpha_stall:
            return self.C_Ma * alpha
        else:
            return self.C_Ma_stall * alpha


# ── Data classes ─────────────────────────────────────────────────────────────

@dataclass
class DroneParams:
    mass: float = 1.5            # kg
    arm_length: float = 0.25     # m  (motor-to-center)
    drag_coeff: float = 0.1      # linear drag  N/(m/s)  (used when aero=None)
    ang_drag_coeff: float = 0.02 # angular drag  N·m/(rad/s)
    max_thrust: float = 25.0     # N  (total, all motors)
    max_torque: float = 5.0      # N·m per axis
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([0.02, 0.02, 0.04])  # kg·m²
    )
    aero: Optional[AeroCoefficients] = None
    atmo: Optional[Atmosphere] = None
    _aero_ranges_checked: bool = field(default=False, repr=False, compare=False)


def _warn_if_aero_params_out_of_range(params: DroneParams) -> None:
    """Warn once when aerodynamic parameters are outside validated ranges."""
    if params._aero_ranges_checked or params.aero is None:
        return

    params._aero_ranges_checked = True
    aero = params.aero
    warnings_to_emit = []

    if not 0.5 <= params.mass <= 30.0:
        warnings_to_emit.append(
            f"mass={params.mass:.3f}kg is outside validated range [0.5, 30.0]kg"
        )

    if not 0.01 <= aero.reference_area <= 1.0:
        warnings_to_emit.append(
            "reference_area="
            f"{aero.reference_area:.4f}m^2 is outside validated range [0.01, 1.0]m^2"
        )

    if isinstance(aero, FixedWingAero):
        if not -0.15 <= aero.alpha_0 <= 0.20:
            warnings_to_emit.append(
                f"alpha_0={aero.alpha_0:.4f}rad is outside validated range [-0.15, 0.20]rad"
            )
        if not 0.10 <= aero.alpha_stall <= 0.60:
            warnings_to_emit.append(
                "alpha_stall="
                f"{aero.alpha_stall:.4f}rad is outside validated range [0.10, 0.60]rad"
            )
        if aero.C_La_stall >= 0.0:
            warnings_to_emit.append(
                f"C_La_stall={aero.C_La_stall:.4f} should be negative in post-stall model"
            )
    else:
        if not 0.2 <= aero.C_D <= 2.5:
            warnings_to_emit.append(
                f"C_D={aero.C_D:.4f} is outside validated range [0.2, 2.5] for multirotors"
            )

    for warning_text in warnings_to_emit:
        warnings.warn(
            f"Aerodynamic parameter warning: {warning_text}",
            RuntimeWarning,
            stacklevel=3,
        )


# ── Airframe presets ─────────────────────────────────────────────────────────

def make_generic_quad() -> DroneParams:
    """Generic 1.5 kg quadrotor with default parameters."""
    return DroneParams()


def make_fixed_wing() -> DroneParams:
    """Generic fixed-wing UAV with AoA-dependent aerodynamics and stall model.

    Based on paper's Table 3 parameters (Valencia et al. 2025).
    """
    return DroneParams(
        mass=3.0,
        arm_length=0.0,  # N/A for fixed-wing
        drag_coeff=0.1,
        ang_drag_coeff=0.05,
        max_thrust=30.0,  # propeller thrust
        max_torque=8.0,
        inertia=np.array([
            [0.10,  0.0,   0.005],
            [0.0,   0.15,  0.0  ],
            [0.005, 0.0,   0.20 ],
        ]),
        aero=FixedWingAero(
            reference_area=0.50,  # m² (wing area)
            C_D=0.03,             # base C_D (overridden by get_CD)
            C_L=0.0,              # base C_L (overridden by get_CL)
            alpha_0=0.05236,      # ~3 deg
            alpha_stall=0.26180,  # ~15 deg
            C_D0=0.02,
            C_La=3.50141,
            C_Da=0.63662,
            C_La_stall=-1.1459,
            C_Da_stall=2.29183,
        ),
        atmo=Atmosphere(),
    )


def make_valencia_fixed_wing() -> DroneParams:
    """Paper-exact fixed-wing preset (Valencia et al. 2025, Tables 2-3).

    Dimensions: wingspan 2.20m, chord 0.235m, wing_ref_area 0.3997m^2,
    total_ref_area 0.5125m^2, weight 2.5kg, cruise 12.0 m/s.
    Aero coefficients from CFD (OpenFOAM) per Table 3.
    """
    return DroneParams(
        mass=2.5,                   # Table 2: Weight 2.5 kg
        arm_length=0.0,             # N/A for fixed-wing
        drag_coeff=0.1,
        ang_drag_coeff=0.05,
        max_thrust=25.0,            # propeller thrust estimate
        max_torque=8.0,
        inertia=np.array([
            [0.08,  0.0,   0.004],  # Jx, estimated from wingspan/mass
            [0.0,   0.12,  0.0  ],  # Jy, estimated from chord/fuselage
            [0.004, 0.0,   0.16 ],  # Jz, Jxz product of inertia
        ]),
        aero=FixedWingAero(
            reference_area=0.3997,   # Table 2: Wing reference area [m^2]
            C_D=0.03,
            C_L=0.0,
            alpha_0=0.05236,         # Table 3: Zero-lift AoA [rad]
            alpha_stall=0.26180,     # Table 3: Stall angle [rad]
            C_D0=0.02,
            C_La=3.50141,            # Table 3: Lift slope pre-stall
            C_Da=0.63662,            # Table 3: Drag slope pre-stall
            C_La_stall=-1.1459,      # Table 3: Lift slope post-stall
            C_Da_stall=2.29183,      # Table 3: Drag slope post-stall
            C_Ma=-0.2040,            # Table 3: Moment slope pre-stall
            C_Ma_stall=-0.1313,      # Table 3: Moment slope post-stall
            chord=0.235,             # Table 2: Mean aerodynamic chord [m]
        ),
        atmo=Atmosphere(altitude_msl=4500.0),  # Antisana altitude
    )


def make_holybro_x500() -> DroneParams:
    """Holybro X500 V2 quadrotor (similar to paper's IRS-4 validation platform)."""
    return DroneParams(
        mass=2.0,
        arm_length=0.25,
        drag_coeff=0.1,
        ang_drag_coeff=0.02,
        max_thrust=40.0,
        max_torque=6.0,
        inertia=np.array([
            [ 0.030,  0.0,    0.0   ],
            [ 0.0,    0.030,  0.0   ],
            [ 0.0,    0.0,    0.050 ],
        ]),
        aero=AeroCoefficients(reference_area=0.06, C_D=1.1, C_L=0.0),
        atmo=Atmosphere(),
    )


def make_irs4_quadrotor(altitude_msl: float = 2800.0) -> DroneParams:
    """Paper-exact IRS-4 quadrotor preset (Valencia et al. 2025, Section 3.2).

    Used for urban validation experiments at Carolina Park and EPN campus
    in Quito, Ecuador (~2800m MSL). Matches default ArduPilot IRS-4 config.

    Paper Table 4 missions:
      - Carolina-40: 326m path, 40m AGL
      - Carolina-20: 326m path, 20m AGL
      - EPN-30: 226m path, 30m AGL
      - EPN-20: 226m path, 20m AGL

    Paper Table 5 RMSE targets: Z ≤ 0.10m, X ≤ 0.071m, Y ≤ 0.055m.
    """
    return DroneParams(
        mass=1.8,                       # IRS-4 platform (3D-printed, Li-Po, PixHawk)
        arm_length=0.22,                # motor-to-center distance
        drag_coeff=0.1,
        ang_drag_coeff=0.015,
        max_thrust=35.0,                # 4x ~8.75N per motor
        max_torque=5.0,
        inertia=np.array([
            [0.025,  0.0,    0.0  ],    # Jx — roll
            [0.0,    0.025,  0.0  ],    # Jy — pitch (symmetric X-frame)
            [0.0,    0.0,    0.042],    # Jz — yaw
        ]),
        aero=AeroCoefficients(
            reference_area=0.05,         # frontal area (compact quad frame)
            C_D=1.0,                     # bluff-body drag
            C_L=0.0,                     # no lift for quadrotor
        ),
        atmo=Atmosphere(altitude_msl=altitude_msl),  # Quito altitude
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


@dataclass
class FlockingParams:
    neighbor_radius: float = 10.0
    separation_radius: float = 2.0
    separation_weight: float = 2.0
    alignment_weight: float = 1.0
    cohesion_weight: float = 1.0


@dataclass
class SwarmRecord:
    t: float
    positions: np.ndarray
    velocities: np.ndarray


def calculate_flocking_vector(me_position: np.ndarray,
                              me_velocity: np.ndarray,
                              neighbor_positions: List[np.ndarray],
                              neighbor_velocities: List[np.ndarray],
                              params: Optional[FlockingParams] = None) -> np.ndarray:
    """Python mirror of `swarm_control/src/boids.rs::calculate_flocking_vector`."""
    if params is None:
        params = FlockingParams()

    if len(neighbor_positions) == 0:
        return np.zeros(3)

    separation = np.zeros(3)
    alignment = np.zeros(3)
    cohesion = np.zeros(3)
    neighbors_count = 0
    center_of_mass = np.zeros(3)

    for other_pos, other_vel in zip(neighbor_positions, neighbor_velocities):
        diff = me_position - other_pos
        dist = np.linalg.norm(diff)

        if dist < params.neighbor_radius and dist > 0.0:
            neighbors_count += 1

            if dist < params.separation_radius:
                separation += (diff / dist) / dist

            alignment += other_vel
            center_of_mass += other_pos

    if neighbors_count > 0:
        alignment /= neighbors_count
        center_of_mass /= neighbors_count
        cohesion = center_of_mass - me_position

    return (
        separation * params.separation_weight
        + (alignment - me_velocity) * params.alignment_weight
        + cohesion * params.cohesion_weight
    )


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


def euler_rates_from_body_rates(phi: float, theta: float,
                                p: float, q: float, r: float) -> np.ndarray:
    """Convert body angular rates to Euler angle rates (paper Eq. 2).

    [phi_dot, theta_dot, psi_dot]^T = E(phi, theta) @ [p, q, r]^T

    where E is the kinematic transformation matrix:
        | 1   sin(phi)*tan(theta)   cos(phi)*tan(theta) |
        | 0   cos(phi)              -sin(phi)           |
        | 0   sin(phi)/cos(theta)   cos(phi)/cos(theta) |

    Args:
        phi: Roll angle [rad].
        theta: Pitch angle [rad].
        p: Body roll rate [rad/s].
        q: Body pitch rate [rad/s].
        r: Body yaw rate [rad/s].

    Returns:
        Array [phi_dot, theta_dot, psi_dot] in rad/s.

    Raises:
        ValueError: If theta is at gimbal lock (±π/2).
    """
    ct = np.cos(theta)
    if abs(ct) < 1e-8:
        raise ValueError(f"Gimbal lock: theta={theta:.4f} rad (cos(theta)≈0)")

    sp, cp = np.sin(phi), np.cos(phi)
    tt = np.tan(theta)

    E = np.array([
        [1.0,  sp * tt,   cp * tt],
        [0.0,  cp,       -sp],
        [0.0,  sp / ct,   cp / ct],
    ])
    return E @ np.array([p, q, r])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0],
    ])


# ── Physics step ─────────────────────────────────────────────────────────────

def compute_aoa(V_body: np.ndarray) -> float:
    """Compute angle of attack from body-frame velocity.

    AoA = atan2(-V_z, V_x) where V_x is forward and V_z is up in body frame.
    For a quadrotor in hover, AoA is ill-defined (V~0); returns 0.
    """
    vx = V_body[0]  # forward
    vz = V_body[2]  # up (body frame)
    if abs(vx) < 1e-8 and abs(vz) < 1e-8:
        return 0.0
    return np.arctan2(-vz, vx)


def _compute_quadratic_drag(V_body: np.ndarray, aero: AeroCoefficients,
                            rho: float) -> np.ndarray:
    """Quadratic aerodynamic drag in body frame (paper Eq. 5).

    F_D = -0.5 * rho * A * C_D(alpha) * |V|^2 * V_hat

    For FixedWingAero, C_D is AoA-dependent.
    """
    V_mag = np.linalg.norm(V_body)
    if V_mag < 1e-8:
        return np.zeros(3)
    V_hat = V_body / V_mag
    alpha = compute_aoa(V_body)
    C_D = aero.get_CD(alpha)
    return -0.5 * rho * aero.reference_area * C_D * V_mag**2 * V_hat


def _compute_lift(V_body: np.ndarray, aero: AeroCoefficients,
                  rho: float) -> np.ndarray:
    """Aerodynamic lift in body frame (paper Eq. 6).

    F_L = 0.5 * rho * A * C_L(alpha) * |V|^2 * L_hat

    Lift is perpendicular to velocity in the body XZ-plane (up direction).
    For quadrotors C_L=0, so this returns zero.
    """
    V_mag = np.linalg.norm(V_body)
    if V_mag < 1e-8:
        return np.zeros(3)
    alpha = compute_aoa(V_body)
    C_L = aero.get_CL(alpha)
    if abs(C_L) < 1e-12:
        return np.zeros(3)

    # Lift direction: perpendicular to velocity in body XZ plane, pointing "up"
    # For body frame: velocity is V_body, lift is normal to V in the XZ plane
    V_hat = V_body / V_mag
    # Lift unit vector: rotate V_hat by +90 deg in XZ plane
    # L_hat = [-V_hat_z, 0, V_hat_x] (in body XZ plane)
    L_hat = np.array([-V_hat[2], 0.0, V_hat[0]])
    L_norm = np.linalg.norm(L_hat)
    if L_norm < 1e-8:
        # Velocity is purely in Y (sideslip) — no lift in XZ plane
        return np.zeros(3)
    L_hat = L_hat / L_norm

    return 0.5 * rho * aero.reference_area * C_L * V_mag**2 * L_hat


def physics_step(state: DroneState, cmd: DroneCommand,
                 params: DroneParams, dt: float,
                 wind=None, t: float = 0.0,
                 terrain=None) -> DroneState:
    """Advance the drone state by dt seconds.

    Args:
        state: Current drone state.
        cmd: Thrust/torque command.
        params: Drone parameters. When params.aero is set, uses quadratic
                drag in body frame (Eq. 3/5). Otherwise uses linear drag.
        dt: Timestep in seconds.
        wind: Optional WindField for wind perturbation (Eq. 5-7).
        t: Current simulation time (used by wind model).
        terrain: Optional TerrainMap for ground collision detection.
    """

    _warn_if_aero_params_out_of_range(params)

    # Clamp commands
    thrust = np.clip(cmd.thrust, 0.0, params.max_thrust)
    torque = np.clip(cmd.torque, -params.max_torque, params.max_torque)

    R = state.rotation
    omega = state.angular_velocity
    aero = params.aero
    atmo = params.atmo

    if aero is not None:
        # ── Body-frame dynamics (paper Eq. 3) ─────────────────────────────
        rho = atmo.rho if atmo is not None else 1.225

        # Velocity in body frame
        V_body = R.T @ state.velocity

        # Forces in body frame
        gravity_body = R.T @ np.array([0.0, 0.0, -params.mass * GRAVITY])
        thrust_body = np.array([0.0, 0.0, thrust])
        drag_body = _compute_quadratic_drag(V_body, aero, rho)
        lift_body = _compute_lift(V_body, aero, rho)

        # Wind perturbation (body frame)
        wind_body = np.zeros(3)
        if wind is not None:
            wind_world = wind.get_force(t, state.position, aero, rho)
            wind_body = R.T @ wind_world

        total_force_body = gravity_body + thrust_body + drag_body + lift_body + wind_body

        # Body-frame acceleration with Coriolis: a = F/m - omega x V_body
        accel_body = total_force_body / params.mass - np.cross(omega, V_body)

        # Integrate in body frame, then transform to world
        new_V_body = V_body + accel_body * dt
        new_vel = R @ new_V_body

        # Position update in world frame
        accel_world = R @ accel_body
        new_pos = state.position + state.velocity * dt + 0.5 * accel_world * dt**2
    else:
        # ── Legacy world-frame dynamics (linear drag) ─────────────────────
        gravity_force = np.array([0.0, 0.0, -params.mass * GRAVITY])
        thrust_world = R @ np.array([0.0, 0.0, thrust])
        drag_force = -params.drag_coeff * state.velocity

        # Wind perturbation (world frame, if provided)
        wind_force = np.zeros(3)
        if wind is not None:
            wind_force = wind.get_force(t, state.position, None, 1.225)

        total_force = gravity_force + thrust_world + drag_force + wind_force

        accel = total_force / params.mass
        new_vel = state.velocity + accel * dt
        new_pos = state.position + state.velocity * dt + 0.5 * accel * dt**2

    # Ground constraint (terrain-aware or flat z=0)
    if terrain is not None:
        ground_z = terrain.get_elevation(new_pos[0], new_pos[1])
    else:
        ground_z = 0.0
    if new_pos[2] < ground_z:
        new_pos[2] = ground_z
        new_vel[2] = max(new_vel[2], 0.0)

    # ── Angular dynamics (body frame) ────────────────────────────────────
    ang_drag = -params.ang_drag_coeff * omega
    I = params.inertia
    I_inv = np.linalg.inv(I)

    # Aerodynamic pitching moment: M_pitch = 0.5 * rho * A * c * V^2 * C_M(alpha)
    aero_torque = np.zeros(3)
    if aero is not None and isinstance(aero, FixedWingAero):
        rho_m = (atmo.rho if atmo is not None else 1.225)
        V_body_m = R.T @ state.velocity
        V_mag = np.linalg.norm(V_body_m)
        if V_mag > 1e-6:
            aoa = compute_aoa(V_body_m)
            q_dyn = 0.5 * rho_m * V_mag**2
            M_pitch = q_dyn * aero.reference_area * aero.chord * aero.get_CM(aoa)
            aero_torque[1] = M_pitch  # pitch axis is body Y

    # Euler's rotation equation: I·alpha = torque - omega x (I·omega) - drag + aero_moment
    alpha = I_inv @ (torque + aero_torque - np.cross(omega, I @ omega) + ang_drag)
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

        return DroneCommand(thrust=float(thrust), torque=torque)

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
    euler_rates: Optional[np.ndarray] = None  # [phi_dot, theta_dot, psi_dot] from Eq. 2


def run_simulation(waypoints: List[np.ndarray],
                   params: Optional[DroneParams] = None,
                   dt: float = 0.005,
                   waypoint_radius: float = 0.5,
                   hover_time: float = 2.0,
                   max_time: float = 120.0,
                   wind=None,
                   terrain=None) -> List[SimRecord]:
    """
    Fly through waypoints in order. Hover at each for hover_time seconds.
    Returns a list of SimRecord for every timestep.

    Args:
        wind: Optional WindField for wind perturbation during simulation.
        terrain: Optional TerrainMap for ground collision detection.
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
        state = physics_step(state, cmd, params, dt, wind=wind, t=t,
                             terrain=terrain)

        roll, pitch, yaw = rotation_to_euler(state.rotation)
        p, q, r = state.angular_velocity
        try:
            e_rates = euler_rates_from_body_rates(roll, pitch, p, q, r)
        except ValueError:
            e_rates = np.zeros(3)  # gimbal lock fallback
        records.append(SimRecord(
            t=t,
            position=state.position.copy(),
            velocity=state.velocity.copy(),
            euler=(roll, pitch, yaw),
            thrust=cmd.thrust,
            angular_velocity=state.angular_velocity.copy(),
            euler_rates=e_rates,
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


def run_swarm_simulation(drone_waypoints: Dict[str, List[np.ndarray]],
                         params: Optional[DroneParams] = None,
                         dt: float = 0.01,
                         waypoint_radius: float = 0.8,
                         hover_time: float = 1.5,
                         max_time: float = 120.0,
                         wind=None,
                         terrain=None,
                         flocking_params: Optional[FlockingParams] = None,
                         avoidance_gain: float = 0.8,
                         min_separation: float = 1.5,
                         max_speed: float = 8.0) -> List[SwarmRecord]:
    """Run N-drone standalone simulation with shared wind/terrain and avoidance."""
    if params is None:
        params = DroneParams()
    if flocking_params is None:
        flocking_params = FlockingParams()

    drone_ids = sorted(drone_waypoints.keys())
    if len(drone_ids) == 0:
        return []

    states: Dict[str, DroneState] = {
        drone_id: DroneState(position=np.array(drone_waypoints[drone_id][0], dtype=float))
        for drone_id in drone_ids
    }
    controllers: Dict[str, PositionController] = {
        drone_id: PositionController(params) for drone_id in drone_ids
    }
    wp_idx: Dict[str, int] = {drone_id: 0 for drone_id in drone_ids}
    hover_timer: Dict[str, float] = {drone_id: 0.0 for drone_id in drone_ids}
    records: List[SwarmRecord] = []

    t = 0.0
    while t < max_time:
        all_done = True
        next_states: Dict[str, DroneState] = {}

        for drone_id in drone_ids:
            waypoints = drone_waypoints[drone_id]
            current_wp_idx = wp_idx[drone_id]
            if current_wp_idx >= len(waypoints):
                next_states[drone_id] = states[drone_id]
                continue

            all_done = False
            state = states[drone_id]
            target = waypoints[current_wp_idx]

            neighbor_positions = [states[n].position for n in drone_ids if n != drone_id]
            neighbor_velocities = [states[n].velocity for n in drone_ids if n != drone_id]
            flock_vec = calculate_flocking_vector(
                me_position=state.position,
                me_velocity=state.velocity,
                neighbor_positions=neighbor_positions,
                neighbor_velocities=neighbor_velocities,
                params=flocking_params,
            )

            avoid_vec = np.zeros(3)
            for other_id in drone_ids:
                if other_id == drone_id:
                    continue
                diff = state.position - states[other_id].position
                dist = np.linalg.norm(diff)
                if 0.0 < dist < min_separation:
                    avoid_vec += (diff / dist) * ((min_separation - dist) / min_separation)

            target_adjusted = target.copy() + (flock_vec + avoidance_gain * avoid_vec)
            cmd = controllers[drone_id].compute(state, target_adjusted, target_yaw=0.0, dt=dt)
            state_next = physics_step(state, cmd, params, dt, wind=wind, t=t, terrain=terrain)

            speed = np.linalg.norm(state_next.velocity)
            if speed > max_speed:
                state_next.velocity = (state_next.velocity / speed) * max_speed

            next_states[drone_id] = state_next

            dist = np.linalg.norm(state_next.position - target)
            if dist < waypoint_radius and speed < 1.2:
                hover_timer[drone_id] += dt
                if hover_timer[drone_id] >= hover_time:
                    wp_idx[drone_id] += 1
                    hover_timer[drone_id] = 0.0
                    controllers[drone_id].reset()
            else:
                hover_timer[drone_id] = 0.0

        states = next_states

        positions = np.array([states[drone_id].position.copy() for drone_id in drone_ids])
        velocities = np.array([states[drone_id].velocity.copy() for drone_id in drone_ids])
        records.append(SwarmRecord(t=t, positions=positions, velocities=velocities))

        if all_done:
            break

        t += dt

    return records
