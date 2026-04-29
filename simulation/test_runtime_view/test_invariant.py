"""
Test Invariant

Auto-split from `test_runtime_view.py` into a focused per-domain test file.
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


class TestDronesAlwaysVisibleInvariant:
    """Phase 7-0 — the drones-always-visible invariant in todo/live_view_backlog.md.

    Static checks against `live.js` so a regression that destroys the
    invariant fails CI before it ships. A future Playwright/headless-DOM
    smoke would be a stronger check; the static markers here are the
    minimum viable gate.
    """

    def test_placeholder_drone_created_before_websocket(self):
        """`createDroneMesh(1)` must run before `connectWS()`."""
        src = _live_js_source()
        idx_create = src.find("createDroneMesh(1)")
        idx_connect = src.find("connectWS()")
        assert idx_create != -1, "createDroneMesh(1) call missing from live.js"
        assert idx_connect != -1, "connectWS() call missing from live.js"
        assert idx_create < idx_connect, (
            "createDroneMesh(1) must run before connectWS() so the user "
            "sees a drone before the first WebSocket message"
        )

    def test_per_drone_meshes_pre_created_from_waypoints(self):
        """Loader for /api/waypoints must call getDrone(...) for each id."""
        src = _live_js_source()
        # The actual implementation iterates Object.keys of the dict and
        # calls getDrone(id) per entry. Both markers must be present.
        assert "fetch('/api/waypoints')" in src, \
            "live.js must fetch /api/waypoints on load"
        assert "getDrone(id)" in src or "getDrone(droneId" in src, (
            "live.js must pre-create per-drone meshes from /api/waypoints "
            "via getDrone() so swarm flights show all N drones at the origin "
            "during the SITL boot phase"
        )

    def test_waypoints_are_repolled_after_boot_window(self):
        """`_refreshWaypoints` must be retried once after initial page load."""
        src = _live_js_source()
        assert "_refreshWaypoints();" in src, (
            "live.js must run an initial /api/waypoints refresh on page load"
        )
        assert "setTimeout(_refreshWaypoints, 4000)" in src, (
            "live.js must repoll /api/waypoints after SITL boot so late "
            "waypoint publication still pre-creates all drone meshes"
        )

    def test_apply_sample_does_not_destroy_meshes(self):
        """`applySample` must use `getDrone()` (which reuses), not recreate."""
        src = _live_js_source()
        # Locate the applySample function body.
        start = src.find("function applySample")
        assert start != -1, "applySample function missing from live.js"
        end = src.find("\nfunction ", start + 1)
        end = end if end != -1 else len(src)
        body = src[start:end]
        assert "getDrone(droneId)" in body, (
            "applySample must reuse the existing mesh via getDrone(); a "
            "destroy/recreate path would break the always-visible invariant"
        )
        for forbidden in ("scene.remove(ds.group)", "drones.delete("):
            assert forbidden not in body, (
                f"applySample must not contain `{forbidden}` — that would "
                f"violate the always-visible invariant"
            )

    def test_camera_follow_preserved(self):
        """Camera-follow snap on first sample + lerp after must stay wired."""
        src = _live_js_source()
        for marker in ("_firstSampleReceived", "_cameraFollowActive",
                       "controls.target.copy", "controls.target.lerp"):
            assert marker in src, (
                f"live.js missing camera-follow marker '{marker}'; the "
                f"always-visible invariant requires the camera to keep at "
                f"least one drone in frame"
            )


class TestReplayLoopStaticSmoke:
    """`live.js` must survive replay-loop t-resets."""

    def test_websocket_handler_reuses_meshes_on_snapshot(self):
        """The snapshot branch (sent on every reconnect) must call
        applySample(), not re-init the scene."""
        src = _live_js_source()
        # Locate the websocket message handler.
        idx = src.find("ws.addEventListener('message'")
        assert idx != -1, "live.js missing ws message handler"
        snippet = src[idx:idx + 800]
        assert "msg.type === 'snapshot'" in snippet
        assert "applySample(s)" in snippet, (
            "snapshot path must replay each sample through applySample, "
            "which reuses meshes via getDrone(); a destroy/recreate path "
            "would break the always-visible invariant on replay-loop "
            "reconnects"
        )

    def test_no_destroy_recreate_in_message_path(self):
        src = _live_js_source()
        idx = src.find("ws.addEventListener('message'")
        snippet = src[idx:idx + 1200]
        for forbidden in ("scene.remove(ds.group)", "drones.delete(",
                           "drones.clear(", "scene.clear("):
            assert forbidden not in snippet, (
                f"`{forbidden}` in the WebSocket message path would break "
                f"the always-visible invariant during replay loops"
            )
