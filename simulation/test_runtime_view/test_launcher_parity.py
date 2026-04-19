"""
Test Launcher Parity

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


class TestLauncherParity:
    """Phase 7 close-out — every mission card resolves to real assets."""

    def test_every_mission_thumbnail_exists(self):
        import json
        from pathlib import Path
        sim_dir = SIM_DIR
        catalogue = json.loads(
            (sim_dir / "runtime_view" / "missions.json")
            .read_text(encoding="utf-8"))
        web_root = sim_dir / "runtime_view" / "web"
        assert catalogue, "missions.json must list at least one mission"
        for entry in catalogue:
            thumb = entry.get("thumbnail", "")
            assert thumb.startswith("/web/img/"), entry
            path = web_root / thumb.lstrip("/").removeprefix("web/")
            assert path.is_file(), (
                f"mission {entry.get('id')!r} references missing thumbnail "
                f"{thumb} (expected at {path})"
            )

    def test_every_mission_start_command_resolves(self):
        """Every `start_command`'s first token must reference a real file."""
        import json
        from pathlib import Path
        sim_dir = SIM_DIR
        project_root = sim_dir.parent
        catalogue = json.loads(
            (sim_dir / "runtime_view" / "missions.json")
            .read_text(encoding="utf-8"))
        for entry in catalogue:
            cmd = (entry.get("start_command") or "").strip()
            if not cmd:
                continue
            first = cmd.split()[0]
            # `./run_scenario.sh ...` resolves to a real script
            if first.startswith("./") or first.startswith("/"):
                target = project_root / first.lstrip("./")
                assert target.is_file(), (
                    f"mission {entry.get('id')!r} command {cmd!r}: "
                    f"binary {first} not found at {target}"
                )


class TestThumbnailGenerator:
    """Phase 7 close-out — deterministic per-mission thumbnail PNGs."""

    def test_regenerate_writes_unique_files_per_mission(self, tmp_path):
        import json
        from runtime_view.scripts.generate_thumbnails import regenerate

        # Stage a missions.json + img dir under tmp so the test doesn't
        # mutate the in-tree thumbnails.
        m = tmp_path / "missions.json"
        catalogue = [
            {"id": "single", "thumbnail": "/web/img/single.png",
             "start_command": "./run_scenario.sh --single-live"},
            {"id": "swarm-3", "thumbnail": "/web/img/swarm3.png",
             "start_command": "./run_scenario.sh --swarm 3"},
        ]
        m.write_text(json.dumps(catalogue), encoding="utf-8")
        img_dir = tmp_path / "img"
        out = regenerate(missions_json=m, img_dir=img_dir)
        assert set(out) == {"single", "swarm-3"}
        single_bytes = (img_dir / "single.png").read_bytes()
        swarm_bytes = (img_dir / "swarm3.png").read_bytes()
        # Different missions must produce different images.
        assert single_bytes != swarm_bytes
        # Both must be non-trivially sized PNGs.
        assert len(single_bytes) > 200
        assert single_bytes.startswith(b"\x89PNG\r\n\x1a\n")
        assert swarm_bytes.startswith(b"\x89PNG\r\n\x1a\n")
