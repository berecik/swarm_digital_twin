"""
Drone Physics Tests - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app
"""

import numpy as np
import pytest
from drone_physics import (
    DroneParams, DroneState, DroneCommand,
    PositionController, PIDController,
    physics_step, euler_to_rotation, rotation_to_euler,
    run_simulation, GRAVITY,
)


# ── Rotation helpers ─────────────────────────────────────────────────────────

class TestRotation:
    def test_identity(self):
        R = euler_to_rotation(0, 0, 0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_roundtrip(self):
        for _ in range(50):
            r, p, y = np.random.uniform(-0.5, 0.5, 3)
            R = euler_to_rotation(r, p, y)
            r2, p2, y2 = rotation_to_euler(R)
            R2 = euler_to_rotation(r2, p2, y2)
            np.testing.assert_allclose(R, R2, atol=1e-10)

    def test_orthogonality(self):
        R = euler_to_rotation(0.3, -0.2, 1.1)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


# ── Gravity free-fall ────────────────────────────────────────────────────────

class TestGravity:
    def test_freefall_no_thrust(self):
        """Drone with zero thrust should fall under gravity."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 100.0]))
        cmd = DroneCommand(thrust=0.0)

        for _ in range(200):  # 1 second at dt=0.005
            state = physics_step(state, cmd, params, dt=0.005)

        # After 1s of freefall: z ≈ 100 - 0.5*g*t² = 95.1 (with some drag)
        assert state.position[2] < 100.0
        assert state.velocity[2] < 0.0  # falling

    def test_freefall_analytical(self):
        """Without drag, freefall should match kinematics."""
        params = DroneParams(drag_coeff=0.0)
        state = DroneState(position=np.array([0.0, 0.0, 100.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.001
        t_total = 1.0

        for _ in range(int(t_total / dt)):
            state = physics_step(state, cmd, params, dt)

        expected_z = 100.0 - 0.5 * GRAVITY * t_total**2
        expected_vz = -GRAVITY * t_total

        np.testing.assert_allclose(state.position[2], expected_z, atol=0.05)
        np.testing.assert_allclose(state.velocity[2], expected_vz, atol=0.05)

    def test_ground_constraint(self):
        """Drone should not fall below z=0."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 1.0]))
        cmd = DroneCommand(thrust=0.0)

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005)

        assert state.position[2] >= 0.0
        assert state.velocity[2] >= 0.0


# ── Hover equilibrium ────────────────────────────────────────────────────────

class TestHover:
    def test_hover_thrust(self):
        """Thrust = mg should maintain hover (no drag at zero velocity)."""
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([0.0, 0.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):  # 10 seconds
            state = physics_step(state, cmd, params, dt=0.005)

        np.testing.assert_allclose(state.position[2], 10.0, atol=0.01)
        np.testing.assert_allclose(state.velocity[2], 0.0, atol=0.01)

    def test_hover_xy_stable(self):
        """Hover should not drift in XY."""
        params = DroneParams()
        hover_thrust = params.mass * GRAVITY
        state = DroneState(position=np.array([5.0, 3.0, 10.0]))
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005)

        np.testing.assert_allclose(state.position[0], 5.0, atol=0.001)
        np.testing.assert_allclose(state.position[1], 3.0, atol=0.001)


# ── Drag ─────────────────────────────────────────────────────────────────────

class TestDrag:
    def test_drag_slows_horizontal(self):
        """Horizontal velocity should decay due to drag during hover."""
        params = DroneParams(drag_coeff=0.5)
        hover_thrust = params.mass * GRAVITY
        state = DroneState(
            position=np.array([0.0, 0.0, 10.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
        )
        cmd = DroneCommand(thrust=hover_thrust)

        for _ in range(4000):  # 20 seconds
            state = physics_step(state, cmd, params, dt=0.005)

        assert abs(state.velocity[0]) < 0.1  # should have decayed

    def test_more_drag_slows_faster(self):
        """Higher drag coefficient should result in less displacement."""
        def fly(drag):
            params = DroneParams(drag_coeff=drag)
            hover_thrust = params.mass * GRAVITY
            state = DroneState(
                position=np.array([0.0, 0.0, 10.0]),
                velocity=np.array([10.0, 0.0, 0.0]),
            )
            cmd = DroneCommand(thrust=hover_thrust)
            for _ in range(1000):
                state = physics_step(state, cmd, params, dt=0.005)
            return state.position[0]

        x_low_drag = fly(0.05)
        x_high_drag = fly(1.0)
        assert x_high_drag < x_low_drag


# ── PID Controller ───────────────────────────────────────────────────────────

class TestPID:
    def test_converges_to_zero(self):
        pid = PIDController(kp=1.0, ki=0.2, kd=0.5)
        val = 10.0
        dt = 0.01
        for _ in range(5000):
            u = pid.update(val, dt=dt)
            val -= u * dt
        assert abs(val) < 0.1

    def test_limit(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, limit=5.0)
        out = pid.update(1000.0, dt=0.01)
        assert abs(out) <= 5.0


# ── Position Controller ─────────────────────────────────────────────────────

class TestPositionController:
    def test_reach_target(self):
        """Controller should fly drone to target position."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 0.0]))
        controller = PositionController(params)
        target = np.array([0.0, 0.0, 5.0])
        dt = 0.005

        for _ in range(4000):  # 20 seconds
            cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)
            state = physics_step(state, cmd, params, dt)

        np.testing.assert_allclose(state.position, target, atol=0.5)

    def test_horizontal_flight(self):
        """Controller should handle horizontal + vertical targets."""
        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 0.0]))
        controller = PositionController(params)
        target = np.array([10.0, 5.0, 8.0])
        dt = 0.005

        for _ in range(8000):  # 40 seconds
            cmd = controller.compute(state, target, target_yaw=0.0, dt=dt)
            state = physics_step(state, cmd, params, dt)

        dist = np.linalg.norm(state.position - target)
        assert dist < 1.0


# ── Full simulation run ─────────────────────────────────────────────────────

class TestSimulation:
    def test_simple_flight(self):
        """Run a two-waypoint mission and verify completion."""
        waypoints = [
            np.array([0.0, 0.0, 5.0]),
            np.array([5.0, 0.0, 5.0]),
        ]
        records = run_simulation(waypoints, dt=0.005, hover_time=1.0, max_time=60.0)

        assert len(records) > 0
        final = records[-1]
        dist = np.linalg.norm(final.position - waypoints[-1])
        assert dist < 1.0

    def test_records_have_increasing_time(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 3.0])],
            dt=0.01, hover_time=1.0, max_time=10.0,
        )
        times = [r.t for r in records]
        assert all(t2 > t1 for t1, t2 in zip(times, times[1:]))

    def test_thrust_always_positive(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 5.0])],
            dt=0.005, hover_time=1.0, max_time=20.0,
        )
        for r in records:
            assert r.thrust >= 0.0

    def test_altitude_stays_positive(self):
        records = run_simulation(
            [np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 0.5])],
            dt=0.005, hover_time=1.0, max_time=60.0,
        )
        for r in records:
            assert r.position[2] >= 0.0


# ── Energy ───────────────────────────────────────────────────────────────────

class TestEnergy:
    def test_freefall_energy_conservation(self):
        """Without drag, KE + PE should be approximately conserved."""
        params = DroneParams(drag_coeff=0.0, ang_drag_coeff=0.0)
        state = DroneState(position=np.array([0.0, 0.0, 50.0]))
        cmd = DroneCommand(thrust=0.0)
        dt = 0.001

        m = params.mass
        e0 = m * GRAVITY * state.position[2] + 0.5 * m * np.dot(state.velocity, state.velocity)

        for _ in range(500):  # 0.5 seconds
            state = physics_step(state, cmd, params, dt)

        e1 = m * GRAVITY * state.position[2] + 0.5 * m * np.dot(state.velocity, state.velocity)
        np.testing.assert_allclose(e0, e1, rtol=0.005)
