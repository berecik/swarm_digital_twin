"""
Test Safety

Auto-split from `test_drone_physics.py` into a focused per-domain test file.
Shared parametrize-time helpers live in `simulation/_test_common.py`.
"""

import numpy as np
import pytest
import warnings
from urllib.error import HTTPError
from pathlib import Path

from drone_physics import (
    DroneParams, DroneState, DroneCommand,
    PositionController, PIDController,
    physics_step, euler_to_rotation, rotation_to_euler,
    euler_rates_from_body_rates,
    run_simulation, run_trajectory_tracking, GRAVITY,
    AeroCoefficients, Atmosphere, _compute_quadratic_drag,
    _compute_lift, compute_aoa,
    FixedWingAero, QuadrotorAero, MotorModel, BatteryModel,
    make_generic_quad, make_holybro_x500, make_fixed_wing,
    make_valencia_fixed_wing, make_irs4_quadrotor,
    run_swarm_simulation, calculate_flocking_vector, FlockingParams,
)
from wind_model import WindField
from terrain import TerrainMap
from validation import (
    BENCHMARK_PROFILES,
    ValidationResult,
    ValidationEnvelope,
    assert_validation_pass,
    get_benchmark_profile,
    get_real_log_mission,
    assert_real_log_validation_pass,
    compute_rmse,
    compare_sim_real,
    auto_tune_wind_force_scale,
    ensure_real_log_logs,
)
from drone_scenario import run_benchmark, run_irs4_benchmark, replay_mission
from flight_log import FlightLog
from swarm_scenario import run_swarm_benchmark, SWARM_BENCHMARK_PROFILES, get_swarm_benchmark_profile
from sensor_models import GPSNoise, IMUNoise, BaroNoise

from _test_common import (
    SIM_DIR,
    PROJECT_ROOT,
    PARITY_MAX_DELTA_M,
    PARITY_RNG_SEED,
    PARITY_SAMPLE_COUNT,
    PROFILE_BASE_SPEED_MS,
    CRUISE_AGL_FLOOR_M,
    CRUISE_ALTITUDE_M,
    CLIMB_TIMEOUT_S,
    live_js_source,
    parity_entry_names,
    ramp_terrain,
    regression_mission,
    stress_mission,
    wind_profile_names_safe,
)

# Aliases preserve the underscore-prefixed names the existing test
# bodies still reference. New tests should use the public names from
# `_test_common` directly.
_LIVE_JS = SIM_DIR / "runtime_view" / "web" / "live.js"
_live_js_source = live_js_source
_parity_entry_names = parity_entry_names
_ramp_terrain = ramp_terrain
_regression_mission = regression_mission
_wind_profile_names = wind_profile_names_safe
_stress_mission = stress_mission
_PROFILE_BASE_SPEED_MS = PROFILE_BASE_SPEED_MS


class TestSafetyMonitor:
    """Tests for inter-drone and terrain collision detection (roadmap Phase 4)."""

    def test_no_events_when_well_separated(self):
        """No events when all drones are far apart."""
        from safety import SeparationMonitor
        mon = SeparationMonitor(min_separation=1.5, near_miss_threshold=3.0)
        positions = {
            0: np.array([0.0, 0.0, 5.0]),
            1: np.array([10.0, 0.0, 5.0]),
            2: np.array([0.0, 10.0, 5.0]),
        }
        mon.check(positions, t=1.0)
        assert mon.collision_count == 0
        assert mon.near_miss_count == 0
        assert mon.min_distance > 3.0

    def test_near_miss_detected(self):
        """Near miss when distance < threshold but > min_separation."""
        from safety import SeparationMonitor, NearMissEvent
        mon = SeparationMonitor(min_separation=1.5, near_miss_threshold=3.0)
        positions = {
            0: np.array([0.0, 0.0, 5.0]),
            1: np.array([2.0, 0.0, 5.0]),
        }
        mon.check(positions, t=1.0)
        assert mon.collision_count == 0
        assert mon.near_miss_count == 1
        assert isinstance(mon.events[0], NearMissEvent)
        assert mon.events[0].distance == pytest.approx(2.0, abs=0.01)

    def test_collision_detected(self):
        """Collision when distance < min_separation."""
        from safety import SeparationMonitor, CollisionEvent
        mon = SeparationMonitor(min_separation=1.5, near_miss_threshold=3.0)
        positions = {
            0: np.array([0.0, 0.0, 5.0]),
            1: np.array([1.0, 0.0, 5.0]),
        }
        mon.check(positions, t=2.5)
        assert mon.collision_count == 1
        assert mon.near_miss_count == 0
        assert isinstance(mon.events[0], CollisionEvent)
        assert mon.events[0].drone_a == 0
        assert mon.events[0].drone_b == 1

    def test_swarm_record_check(self):
        """check_swarm_record() works with SwarmRecord-like objects."""
        from safety import SeparationMonitor
        from drone_physics import SwarmRecord
        mon = SeparationMonitor(min_separation=1.5)
        rec = SwarmRecord(
            t=0.5,
            positions=np.array([[0, 0, 5], [1, 0, 5], [10, 10, 5]], dtype=float),
            velocities=np.zeros((3, 3)),
        )
        mon.check_swarm_record(rec)
        assert mon.collision_count == 1
        assert mon.min_distance == pytest.approx(1.0, abs=0.01)

    def test_terrain_collision_detected(self):
        """TerrainMonitor detects AGL < 0 as terrain collision."""
        from safety import TerrainMonitor, TerrainCollisionEvent
        terrain = TerrainMap.from_function(
            lambda x, y: np.full_like(x, 10.0),
            x_range=(-50, 50), y_range=(-50, 50), resolution=1.0,
        )
        mon = TerrainMonitor(terrain, min_agl=5.0)
        mon.check(drone_id=1, position=np.array([0.0, 0.0, 8.0]), t=1.0)
        assert mon.terrain_collision_count == 1
        assert isinstance(mon.events[0], TerrainCollisionEvent)
        assert mon.events[0].agl == pytest.approx(-2.0, abs=0.1)

    def test_clearance_violation_detected(self):
        """TerrainMonitor detects AGL < min_agl as clearance violation."""
        from safety import TerrainMonitor, ClearanceViolationEvent
        terrain = TerrainMap.from_function(
            lambda x, y: np.zeros_like(x),
            x_range=(-50, 50), y_range=(-50, 50), resolution=1.0,
        )
        mon = TerrainMonitor(terrain, min_agl=5.0)
        mon.check(drone_id=1, position=np.array([0.0, 0.0, 3.0]), t=1.0)
        assert mon.clearance_violation_count == 1
        assert mon.terrain_collision_count == 0
        assert isinstance(mon.events[0], ClearanceViolationEvent)

    def test_terrain_ok_when_above_min_agl(self):
        """No events when AGL > min_agl."""
        from safety import TerrainMonitor
        terrain = TerrainMap.from_function(
            lambda x, y: np.zeros_like(x),
            x_range=(-50, 50), y_range=(-50, 50), resolution=1.0,
        )
        mon = TerrainMonitor(terrain, min_agl=5.0)
        mon.check(drone_id=1, position=np.array([0.0, 0.0, 10.0]), t=1.0)
        assert len(mon.events) == 0
        assert mon.min_agl_observed == pytest.approx(10.0, abs=0.1)

    def test_safety_report_from_monitors(self):
        """SafetyReport aggregates KPIs from both monitors."""
        from safety import SeparationMonitor, TerrainMonitor, SafetyReport
        terrain = TerrainMap.from_function(
            lambda x, y: np.zeros_like(x),
            x_range=(-50, 50), y_range=(-50, 50), resolution=1.0,
        )
        sep = SeparationMonitor(min_separation=1.5, near_miss_threshold=3.0)
        ter = TerrainMonitor(terrain, min_agl=5.0)
        sep.check({0: np.array([0, 0, 5.0]), 1: np.array([2, 0, 5.0])}, t=1.0)
        ter.check(0, np.array([0.0, 0.0, 10.0]), t=1.0)
        report = SafetyReport.from_monitors(sep, ter)
        assert report.is_safe()
        assert report.near_miss_count == 1
        assert report.collision_count == 0
        assert report.min_separation == pytest.approx(2.0, abs=0.01)
        d = report.to_dict()
        assert d["verdict"] == "SAFE"

    def test_safety_report_unsafe_on_collision(self):
        """SafetyReport.is_safe() returns False when collisions exist."""
        from safety import SeparationMonitor, SafetyReport
        sep = SeparationMonitor(min_separation=1.5)
        sep.check({0: np.array([0, 0, 5.0]), 1: np.array([0.5, 0, 5.0])}, t=1.0)
        report = SafetyReport.from_monitors(sep)
        assert not report.is_safe()
        assert report.collision_count == 1
        assert "UNSAFE" in report.summary()

    def test_full_swarm_simulation_produces_report(self):
        """SeparationMonitor produces a valid SafetyReport from real swarm data.

        The boids swarm has legitimate close passes during formation
        transitions, so this test verifies the monitor *detects* events
        correctly rather than asserting zero collisions.
        """
        from safety import SeparationMonitor, SafetyReport
        from swarm_scenario import run_swarm_benchmark
        np.random.seed(42)
        run_swarm_benchmark("baseline")
        data = np.load(
            str(SIM_DIR / "swarm_data.npz"),
            allow_pickle=True,  # noqa: S301 — trusted local file
        )
        t = data["t"]
        positions = data["positions"]
        sep = SeparationMonitor(min_separation=1.0, near_miss_threshold=2.0)
        for i in range(len(t)):
            pos_dict = {j: positions[i][j] for j in range(positions.shape[1])}
            sep.check(pos_dict, float(t[i]))
        report = SafetyReport.from_monitors(sep)
        # Report must have valid structure
        assert report.total_events >= 0
        assert report.min_separation >= 0.0
        assert report.min_separation < 10.0  # sanity — drones are nearby
        d = report.to_dict()
        assert d["verdict"] in ("SAFE", "UNSAFE")
        assert isinstance(d["collision_count"], int)
        # The summary must be printable
        assert len(report.summary()) > 50


class TestSafetyResponseController:
    """Phase 4 close-out — Warning → HOVER → RTL → EMERGENCY state machine."""

    def test_no_events_stays_normal(self):
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        ctl.tick(t=0.0)
        assert ctl.state == SafetyMode.NORMAL
        assert ctl.incident_log == []

    def test_near_miss_transitions_normal_to_warning(self):
        from safety import NearMissEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        ctl.observe(NearMissEvent(t=1.0, drone_a=1, drone_b=2,
                                   distance=2.5), t=1.0)
        assert ctl.state == SafetyMode.WARNING
        assert ctl.incident_log[0].to_mode == SafetyMode.WARNING

    def test_collision_transitions_to_hover(self):
        from safety import CollisionEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        ctl.observe(CollisionEvent(t=2.0, drone_a=1, drone_b=2,
                                    distance=1.0), t=2.0)
        assert ctl.state == SafetyMode.HOVER

    def test_repeated_collisions_escalate_to_rtl(self):
        from safety import CollisionEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        for i in range(3):
            ctl.observe(CollisionEvent(t=float(i), drone_a=1, drone_b=2,
                                        distance=1.0), t=float(i))
        assert ctl.state == SafetyMode.RTL

    def test_terrain_collision_triggers_rtl(self):
        from safety import TerrainCollisionEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        ctl.observe(TerrainCollisionEvent(
            t=4.0, drone_id=1, position=np.zeros(3), agl=-0.5), t=4.0)
        assert ctl.state == SafetyMode.RTL

    def test_repeated_terrain_collisions_escalate_to_emergency(self):
        from safety import TerrainCollisionEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        for i in range(2):
            ctl.observe(TerrainCollisionEvent(
                t=float(i), drone_id=1,
                position=np.zeros(3), agl=-0.5), t=float(i))
        assert ctl.state == SafetyMode.EMERGENCY_STOP

    def test_warning_downgrades_to_normal_after_dwell(self):
        from safety import NearMissEvent
        from safety_response import (
            SafetyMode, SafetyResponseController, SafetyResponseThresholds,
        )
        ctl = SafetyResponseController(
            thresholds=SafetyResponseThresholds(warning_dwell_s=2.0))
        ctl.observe(NearMissEvent(t=1.0, drone_a=1, drone_b=2,
                                   distance=2.5), t=1.0)
        assert ctl.state == SafetyMode.WARNING
        ctl.tick(t=3.5)  # 2.5s after warning entered → past 2.0s dwell
        assert ctl.state == SafetyMode.NORMAL

    def test_rtl_does_not_auto_downgrade(self):
        """Once in RTL the controller never auto-clears (flight-safe default)."""
        from safety import CollisionEvent, NearMissEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        for i in range(3):
            ctl.observe(CollisionEvent(t=float(i), drone_a=1, drone_b=2,
                                        distance=1.0), t=float(i))
        assert ctl.state == SafetyMode.RTL
        # Even after 100s of "quiet" time, we don't downgrade out of RTL.
        ctl.tick(t=100.0)
        assert ctl.state == SafetyMode.RTL

    def test_on_transition_callback_fires(self):
        from safety import NearMissEvent
        from safety_response import SafetyMode, SafetyResponseController
        seen = []
        ctl = SafetyResponseController(
            on_transition=lambda old, new, reason: seen.append(
                (old, new, reason)))
        ctl.observe(NearMissEvent(t=1.0, drone_a=1, drone_b=2,
                                   distance=2.5), t=1.0)
        assert seen and seen[0][0] == SafetyMode.NORMAL
        assert seen[0][1] == SafetyMode.WARNING

    def test_replay_helper_consumes_event_list(self):
        from safety import CollisionEvent, NearMissEvent
        from safety_response import SafetyMode, SafetyResponseController
        ctl = SafetyResponseController()
        events = [
            NearMissEvent(t=0.5, drone_a=1, drone_b=2, distance=2.5),
            CollisionEvent(t=1.0, drone_a=1, drone_b=2, distance=1.0),
        ]
        ctl.replay(events)
        # NearMissEvent → WARNING, then CollisionEvent → HOVER.
        assert ctl.state == SafetyMode.HOVER
        # 2 transitions logged (NORMAL→WARNING, WARNING→HOVER).
        assert len(ctl.incident_log) == 2

    def test_to_dict_is_json_friendly(self):
        import json
        from safety import CollisionEvent
        from safety_response import SafetyResponseController
        ctl = SafetyResponseController()
        ctl.observe(CollisionEvent(t=1.0, drone_a=1, drone_b=2,
                                    distance=1.0), t=1.0)
        # Round-trips through json.dumps without raising.
        json.dumps(ctl.to_dict())


# ──────────────────────────────────────────────────────────────────────────────
# Phase 7 close-out — multi-user session isolation, launcher parity,
# replay-loop static smoke, mission-aware thumbnail generator.
# ──────────────────────────────────────────────────────────────────────────────
