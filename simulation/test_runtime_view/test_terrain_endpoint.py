"""
Test Terrain Endpoint

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


class TestTerrainEndpoint:
    """Phase 7-1 — GET/POST /api/terrain serve the live viewer's mesh."""

    def _client(self):
        from fastapi.testclient import TestClient
        import runtime_view.server as srv
        srv.mission_terrain = None  # reset in-process state per test
        return TestClient(srv.app), srv

    def test_get_returns_204_when_no_terrain(self):
        client, _ = self._client()
        r = client.get("/api/terrain")
        assert r.status_code == 204

    def test_post_then_get_round_trip(self):
        client, _ = self._client()
        payload = {
            "vertices": [[0, 0, 0], [10, 0, 0], [0, 0, -10], [10, 0, -10]],
            "faces": [[0, 1, 3], [0, 3, 2]],
            "bounds": [0, -10, 10, 0],
        }
        r = client.post("/api/terrain", json=payload)
        assert r.status_code == 200, r.text
        body = r.json()
        assert body["vertex_count"] == 4 and body["face_count"] == 2

        r2 = client.get("/api/terrain")
        assert r2.status_code == 200
        out = r2.json()
        assert out["bounds"] == [0.0, -10.0, 10.0, 0.0]
        assert len(out["vertices"]) == 4 and len(out["faces"]) == 2

    def test_post_by_terrain_name_uses_manifest(self):
        client, _ = self._client()
        r = client.post("/api/terrain",
                        json={"terrain_name": "synthetic_rolling"})
        assert r.status_code == 200, r.text
        body = r.json()
        assert body["vertex_count"] > 0 and body["face_count"] > 0

    def test_post_unknown_terrain_name_returns_400(self):
        client, _ = self._client()
        r = client.post("/api/terrain",
                        json={"terrain_name": "does_not_exist"})
        assert r.status_code == 400
        assert "unknown terrain" in r.json()["detail"]

    def test_post_missing_vertices_returns_400(self):
        client, _ = self._client()
        r = client.post("/api/terrain", json={"faces": [[0, 1, 2]]})
        assert r.status_code == 400
        assert "vertices" in r.json()["detail"]

    def test_post_empty_payload_clears_terrain(self):
        client, _ = self._client()
        client.post("/api/terrain",
                    json={"terrain_name": "synthetic_rolling"})
        r = client.post("/api/terrain", json={})
        assert r.status_code == 200
        assert r.json()["status"] == "cleared"
        r2 = client.get("/api/terrain")
        assert r2.status_code == 204

    def test_live_js_loads_and_hides_grid_when_terrain_present(self):
        """live.js must hide grid+ground when terrain mesh loads."""
        src = _live_js_source()
        assert "fetch('/api/terrain')" in src, \
            "live.js must fetch /api/terrain on load"
        assert "grid.visible = false" in src, \
            "live.js must hide flat grid when terrain mesh is present"
        assert "ground.visible = false" in src, \
            "live.js must hide flat ground plane when terrain mesh is present"
