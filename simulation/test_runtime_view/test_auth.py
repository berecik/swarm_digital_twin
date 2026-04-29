"""
Test Auth

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


class TestAuthAndCSRF:
    """--auth flag turns on Bearer + CSRF for mutating endpoints."""

    @pytest.fixture(autouse=True)
    def _reset_auth(self):
        """Make sure no test in this class leaks the auth token to siblings."""
        import runtime_view.server as srv
        original = srv.auth_token()
        yield
        srv.set_auth_token(original)

    def _client(self, token=None):
        from fastapi.testclient import TestClient
        import runtime_view.server as srv
        srv.set_auth_token(token)
        srv.mission_terrain = None
        return TestClient(srv.app)

    def test_csrf_endpoint_reports_disabled(self):
        client = self._client(token=None)
        body = client.get("/api/csrf").json()
        assert body == {"token": None, "auth_required": False}

    def test_csrf_endpoint_reports_enabled(self):
        client = self._client(token="t0p_s3cr3t")
        body = client.get("/api/csrf").json()
        assert body["auth_required"] is True
        assert body["token"] == "t0p_s3cr3t"

    def test_get_endpoints_open_when_auth_disabled(self):
        client = self._client(token=None)
        for path in ("/api/status", "/api/snapshot", "/api/waypoints"):
            r = client.get(path)
            assert r.status_code == 200, (path, r.text)

    def test_get_endpoints_open_even_with_auth_enabled(self):
        client = self._client(token="abc")
        # Read-only paths must keep working so the static page loads
        # before the user has typed in the token.
        for path in ("/api/status", "/api/snapshot", "/api/waypoints",
                     "/api/csrf"):
            r = client.get(path)
            assert r.status_code == 200, (path, r.text)

    def test_post_blocked_without_authorization_header(self):
        client = self._client(token="abc")
        r = client.post("/api/waypoints", json={"1": [[0, 0, 5]]})
        assert r.status_code == 401
        assert "Bearer" in r.json()["detail"]

    def test_post_blocked_with_bad_token(self):
        client = self._client(token="abc")
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": "Bearer wrong",
                     "X-CSRF-Token": "wrong"},
        )
        assert r.status_code == 401

    def test_post_blocked_without_csrf_header(self):
        client = self._client(token="abc")
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": "Bearer abc"},
        )
        assert r.status_code == 403

    def test_post_passes_with_correct_token_and_csrf(self):
        client = self._client(token="abc")
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": "Bearer abc", "X-CSRF-Token": "abc"},
        )
        assert r.status_code == 200, r.text

    def test_terrain_post_also_gated(self):
        client = self._client(token="abc")
        r = client.post("/api/terrain", json={})
        assert r.status_code == 401
        r2 = client.post(
            "/api/terrain", json={"terrain_name": "synthetic_rolling"},
            headers={"Authorization": "Bearer abc", "X-CSRF-Token": "abc"},
        )
        assert r2.status_code == 200, r2.text


class TestMultiUserSessionIsolation:
    """POST /api/session mints per-browser tokens."""

    @pytest.fixture(autouse=True)
    def _reset(self):
        import runtime_view.server as srv
        original = srv.auth_token()
        srv.reset_sessions()
        yield
        srv.set_auth_token(original)
        srv.reset_sessions()

    def _client(self, token=None):
        from fastapi.testclient import TestClient
        import runtime_view.server as srv
        srv.set_auth_token(token)
        return TestClient(srv.app)

    def test_session_endpoint_404_when_auth_disabled(self):
        client = self._client(token=None)
        r = client.post("/api/session")
        assert r.status_code == 404

    def test_session_endpoint_requires_global_token(self):
        client = self._client(token="t0p")
        r = client.post("/api/session")
        assert r.status_code == 401, r.text

    def test_session_token_is_minted_with_global_auth(self):
        client = self._client(token="t0p")
        r = client.post(
            "/api/session",
            headers={"Authorization": "Bearer t0p", "X-CSRF-Token": "t0p"},
        )
        assert r.status_code == 200, r.text
        body = r.json()
        assert "token" in body and len(body["token"]) > 16
        assert body["ttl_s"] > 0

    def test_session_token_authorises_mutating_call(self):
        client = self._client(token="t0p")
        sess = client.post(
            "/api/session",
            headers={"Authorization": "Bearer t0p", "X-CSRF-Token": "t0p"},
        ).json()["token"]
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": f"Bearer {sess}",
                     "X-CSRF-Token": sess},
        )
        assert r.status_code == 200, r.text

    def test_two_sessions_are_independent(self):
        """Distinct sessions must mint distinct tokens."""
        client = self._client(token="t0p")
        h = {"Authorization": "Bearer t0p", "X-CSRF-Token": "t0p"}
        a = client.post("/api/session", headers=h).json()["token"]
        b = client.post("/api/session", headers=h).json()["token"]
        assert a != b
        # Both still usable.
        for tok in (a, b):
            r = client.post(
                "/api/waypoints",
                json={"1": [[0, 0, 5]]},
                headers={"Authorization": f"Bearer {tok}",
                         "X-CSRF-Token": tok},
            )
            assert r.status_code == 200, (tok, r.text)

    def test_revoked_session_is_rejected(self):
        client = self._client(token="t0p")
        h = {"Authorization": "Bearer t0p", "X-CSRF-Token": "t0p"}
        sess = client.post("/api/session", headers=h).json()["token"]
        r_revoke = client.request(
            "DELETE", "/api/session", json={"token": sess},
            headers={"Authorization": "Bearer t0p", "X-CSRF-Token": "t0p"},
        )
        assert r_revoke.status_code == 200
        assert r_revoke.json()["revoked"] is True
        # Now the session token alone should fail.
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": f"Bearer {sess}",
                     "X-CSRF-Token": sess},
        )
        assert r.status_code == 401

    def test_expired_session_is_rejected(self):
        import runtime_view.server as srv
        client = self._client(token="t0p")
        # Mint a session that expires immediately.
        sess = srv.create_session(ttl_s=0.0)
        # Force purge so the lookup fails on the next call.
        srv._purge_expired_sessions(now_s=1e12)
        r = client.post(
            "/api/waypoints",
            json={"1": [[0, 0, 5]]},
            headers={"Authorization": f"Bearer {sess}",
                     "X-CSRF-Token": sess},
        )
        assert r.status_code == 401
