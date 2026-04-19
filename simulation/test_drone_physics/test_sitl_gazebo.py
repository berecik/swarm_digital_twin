"""
Test Sitl Gazebo

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


class TestSITLOrchestratorCli:
    """Smoke checks for the new --telemetry-forward CLI surface."""

    def test_orchestrator_help_lists_telemetry_forward(self):
        import subprocess
        import sys as _sys

        sim_dir = SIM_DIR
        for sub in ('single', 'swarm', 'swarm-formation'):
            result = subprocess.run(
                [_sys.executable, str(sim_dir / 'sitl_orchestrator.py'),
                 sub, '--help'],
                capture_output=True, text=True, timeout=15,
            )
            assert result.returncode == 0, (
                f'{sub} --help failed: {result.stderr}')
            assert '--telemetry-forward' in result.stdout, (
                f'{sub} --help did not advertise --telemetry-forward')


# ── Swarm-Ready Standalone Twin ──────────────────────────────────────────────


class TestGazeboModels:
    def test_sdf_model_exists(self):
        """Valencia fixed-wing SDF model file should exist."""
        import os
        sdf_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        assert os.path.exists(sdf_path), f"SDF not found: {sdf_path}"

    def test_sdf_model_config_exists(self):
        """Model config file should exist."""
        import os
        config_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "models", "valencia_fixed_wing", "model.config"
        )
        assert os.path.exists(config_path)

    def test_sdf_contains_liftdrag_plugin(self):
        """SDF should contain LiftDrag plugin with Table 3 coefficients."""
        import os
        sdf_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "LiftDragPlugin" in content
        assert "3.50141" in content  # C_La
        assert "0.63662" in content  # C_Da
        assert "-0.2040" in content  # C_Ma

    def test_sdf_mass_matches_paper(self):
        """SDF model mass should match paper Table 2 (2.5 kg)."""
        import os
        sdf_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "models", "valencia_fixed_wing", "model.sdf"
        )
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "<mass>2.5</mass>" in content

    def test_antisana_world_exists(self):
        """Antisana Gazebo world file should exist."""
        import os
        world_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "worlds", "antisana.world"
        )
        assert os.path.exists(world_path)

    def test_antisana_world_gps_origin(self):
        """Antisana world should have correct GPS coordinates."""
        import os
        world_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "worlds", "antisana.world"
        )
        with open(world_path, encoding="utf-8") as f:
            content = f.read()
        assert "-0.508333" in content  # Antisana latitude
        assert "-78.141667" in content  # Antisana longitude
        assert "4500" in content  # Elevation

    def test_parm_file_exists(self):
        """ArduPilot parameter file should exist."""
        import os
        parm_path = os.path.join(
            str(PROJECT_ROOT),
            "gazebo", "models", "valencia_fixed_wing", "valencia_fw.parm"
        )
        assert os.path.exists(parm_path)


# ── IRS-4 Gazebo Model & Docker SITL ─────────────────────────────────────


class TestIRS4GazeboModel:
    """Verify IRS-4 quadrotor Gazebo model (SDF, sensors, plugins)."""

    def _project_root(self):
        import os
        return str(PROJECT_ROOT)

    def test_irs4_sdf_exists(self):
        """IRS-4 quadrotor SDF model file should exist."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        assert os.path.exists(sdf_path), f"SDF not found: {sdf_path}"

    def test_irs4_model_config_exists(self):
        """IRS-4 model config file should exist."""
        import os
        config_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.config")
        assert os.path.exists(config_path)

    def test_irs4_parm_exists(self):
        """IRS-4 ArduPilot parameter file should exist."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        assert os.path.exists(parm_path)

    def test_irs4_sdf_mass_matches_preset(self):
        """SDF mass should match make_irs4_quadrotor() (1.8 kg)."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "<mass>1.8</mass>" in content

    def test_irs4_sdf_has_4_rotors(self):
        """SDF should have 4 rotor joints."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert content.count("rotor_") >= 8  # 4 links + 4 joints

    def test_irs4_sdf_has_ardupilot_plugin(self):
        """SDF should contain ArduPilot SITL plugin."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "ArduPilotPlugin" in content

    def test_irs4_sdf_has_liftdrag_plugin(self):
        """SDF should contain LiftDrag aero plugin with C_D=1.0."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "LiftDragPlugin" in content
        assert "<cda>1.0</cda>" in content

    def test_irs4_sdf_inertia_matches_preset(self):
        """SDF inertia should match make_irs4_quadrotor() values."""
        import os
        sdf_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "model.sdf")
        with open(sdf_path, encoding="utf-8") as f:
            content = f.read()
        assert "<ixx>0.025</ixx>" in content
        assert "<iyy>0.025</iyy>" in content
        assert "<izz>0.042</izz>" in content

    def test_irs4_parm_copter_frame(self):
        """Parameter file should configure copter frame."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        with open(parm_path, encoding="utf-8") as f:
            content = f.read()
        assert "FRAME_CLASS,1" in content

    def test_irs4_parm_carolina_origin(self):
        """Parameter file should have Carolina Park GPS origin."""
        import os
        parm_path = os.path.join(self._project_root(), "gazebo", "models", "irs4_quadrotor", "irs4_quad.parm")
        with open(parm_path, encoding="utf-8") as f:
            content = f.read()
        assert "SIM_OPOS_LAT,-0.189" in content
        assert "SIM_OPOS_ALT,2800" in content


class TestDockerSITL:
    """Verify Docker SITL configuration (Dockerfile, compose, entrypoint)."""

    def _project_root(self):
        import os
        return str(PROJECT_ROOT)

    def test_dockerfile_sitl_exists(self):
        """Dockerfile.sitl should exist."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        assert os.path.exists(path)

    def test_dockerfile_builds_copter_and_plane(self):
        """Dockerfile should build both arducopter and arduplane."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "arducopter" in content
        assert "arduplane" in content

    def test_dockerfile_exposes_ports(self):
        """Dockerfile should expose required UDP/TCP ports."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "9002" in content
        assert "9003" in content
        assert "14550" in content

    def test_dockerfile_has_healthcheck(self):
        """Dockerfile should have a MAVLink heartbeat health check."""
        import os
        path = os.path.join(self._project_root(), "Dockerfile.sitl")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "HEALTHCHECK" in content

    def test_compose_has_sitl_service(self):
        """docker-compose.yml should have ardupilot_sitl service."""
        import os
        path = os.path.join(self._project_root(), "docker-compose.yml")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "ardupilot_sitl" in content

    def test_compose_sitl_ports(self):
        """SITL compose service should expose correct ports."""
        import os
        path = os.path.join(self._project_root(), "docker-compose.yml")
        with open(path, encoding="utf-8") as f:
            content = f.read()
        assert "9002:9002" in content
        assert "14550:14550" in content

    def test_sitl_entrypoint_exists(self):
        """SITL entrypoint script should exist and be executable."""
        import os, stat
        path = os.path.join(self._project_root(), "scripts", "sitl_entrypoint.sh")
        assert os.path.exists(path)
        mode = os.stat(path).st_mode
        assert mode & stat.S_IXUSR


# ── Mission Waypoint Files ───────────────────────────────────────────────


class TestMissionFiles:
    """Verify paper mission waypoint files (.waypoints format)."""

    def _project_root(self):
        import os
        return str(PROJECT_ROOT)

    def test_fw_158_exists(self):
        """Fixed-wing mission 158 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_158.waypoints")
        assert os.path.exists(path)

    def test_fw_178_exists(self):
        """Fixed-wing mission 178 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_178.waypoints")
        assert os.path.exists(path)

    def test_fw_185_exists(self):
        """Fixed-wing mission 185 waypoint file should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "fw_185.waypoints")
        assert os.path.exists(path)

    def test_fw_missions_qgc_format(self):
        """All FW mission files should start with QGC WPL 110 header."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            path = os.path.join(missions_dir, name)
            with open(path, encoding="utf-8") as f:
                first_line = f.readline().strip()
            assert first_line == "QGC WPL 110", f"{name} missing QGC header"

    def test_fw_missions_have_antisana_origin(self):
        """FW missions should reference Antisana GPS coordinates."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            with open(os.path.join(missions_dir, name), encoding="utf-8") as f:
                content = f.read()
            assert "-0.508333" in content, f"{name} missing Antisana lat"
            assert "-78.141667" in content, f"{name} missing Antisana lon"

    def test_fw_missions_have_waypoints(self):
        """Each FW mission should have at least 5 waypoints."""
        import os
        missions_dir = os.path.join(self._project_root(), "missions")
        for name in ["fw_158.waypoints", "fw_178.waypoints", "fw_185.waypoints"]:
            with open(os.path.join(missions_dir, name), encoding="utf-8") as f:
                lines = f.readlines()
            # First line is header, rest are waypoints
            wp_count = len([l for l in lines[1:] if l.strip()])
            assert wp_count >= 5, f"{name} has only {wp_count} waypoints"

    def test_quad_missions_module_exists(self):
        """Quadrotor mission definitions module should exist."""
        import os
        path = os.path.join(self._project_root(), "missions", "quad_missions.py")
        assert os.path.exists(path)

    def test_quad_missions_importable(self):
        """Quadrotor missions should be importable and contain 4 missions."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import ALL_QUAD_MISSIONS, get_mission, mission_to_qgc_wpl
            assert len(ALL_QUAD_MISSIONS) == 4
            assert "carolina_40" in ALL_QUAD_MISSIONS
            assert "carolina_20" in ALL_QUAD_MISSIONS
            assert "epn_30" in ALL_QUAD_MISSIONS
            assert "epn_20" in ALL_QUAD_MISSIONS
        finally:
            sys.path.remove(missions_dir)

    def test_quad_mission_to_qgc_wpl(self):
        """mission_to_qgc_wpl should produce valid QGC format."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import get_mission, mission_to_qgc_wpl
            mission = get_mission("carolina_40")
            wpl = mission_to_qgc_wpl(mission)
            assert wpl.startswith("QGC WPL 110")
            lines = [l for l in wpl.strip().split("\n") if l.strip()]
            assert len(lines) >= 5
        finally:
            sys.path.remove(missions_dir)

    def test_quad_missions_have_correct_origins(self):
        """Carolina missions at -0.189, EPN missions at -0.210."""
        import sys, os
        missions_dir = os.path.join(self._project_root(), "missions")
        sys.path.insert(0, missions_dir)
        try:
            from quad_missions import get_mission
            carolina = get_mission("carolina_40")
            assert carolina["origin"]["lat"] == -0.189
            epn = get_mission("epn_30")
            assert epn["origin"]["lat"] == -0.210
        finally:
            sys.path.remove(missions_dir)


# ── SITL Lifecycle Script ─────────────────────────────────────────────────


class TestSITLLifecycle:
    def test_sitl_script_exists(self):
        """SITL mission lifecycle script should exist and be executable."""
        import os, stat
        script_path = os.path.join(
            str(PROJECT_ROOT),
            "scripts", "run_sitl_mission.sh"
        )
        assert os.path.exists(script_path)
        mode = os.stat(script_path).st_mode
        assert mode & stat.S_IXUSR  # executable

    def test_sitl_script_has_required_steps(self):
        """Script should contain all 6 lifecycle steps."""
        import os
        script_path = os.path.join(
            str(PROJECT_ROOT),
            "scripts", "run_sitl_mission.sh"
        )
        with open(script_path, encoding="utf-8") as f:
            content = f.read()
        assert "[1/6]" in content  # Start stack
        assert "[2/6]" in content  # Health check
        assert "[3/6]" in content  # Upload mission
        assert "[4/6]" in content  # Arm and fly
        assert "[5/6]" in content  # Capture logs
        assert "[6/6]" in content  # Validate


# ── 3D Wind Estimation ───────────────────────────────────────────────────


class TestCIPipeline:
    def test_ci_workflow_exists(self):
        """GitHub Actions CI workflow file should exist."""
        import os
        ci_path = os.path.join(
            str(PROJECT_ROOT),
            ".github", "workflows", "ci.yml"
        )
        assert os.path.exists(ci_path)

    def test_ci_workflow_has_required_jobs(self):
        """CI workflow should run tests and benchmarks."""
        import os
        ci_path = os.path.join(
            str(PROJECT_ROOT),
            ".github", "workflows", "ci.yml"
        )
        with open(ci_path, encoding="utf-8") as f:
            content = f.read()
        assert "pytest" in content
        assert "benchmark" in content.lower()
        assert "upload-artifact" in content

    def test_ci_workflow_triggers_on_push(self):
        """CI should trigger on push to master/main."""
        import os
        ci_path = os.path.join(
            str(PROJECT_ROOT),
            ".github", "workflows", "ci.yml"
        )
        with open(ci_path, encoding="utf-8") as f:
            content = f.read()
        assert "push" in content
        assert "master" in content or "main" in content


# ── IRS-4 Quadrotor Preset ───────────────────────────────────────────────


class TestGazeboSensorVisibility:
    """Checks that simulation model exposes visible sensors."""

    def test_x500_has_visible_sensors(self):
        import os
        import xml.etree.ElementTree as ET

        model_path = os.path.join(str(PROJECT_ROOT), 'gazebo',
                                  'models', 'x500', 'model.sdf')
        root = ET.parse(model_path).getroot()
        sensors = root.findall('.//model[@name="x500"]/link[@name="base_link"]/sensor')
        sensor_names = {s.get('name') for s in sensors}

        assert {'imu_sensor', 'gps_sensor', 'barometer_sensor', 'magnetometer_sensor', 'front_camera'} <= sensor_names
        for sensor in sensors:
            vis = sensor.find('visualize')
            assert vis is not None
            assert vis.text == 'true'


# ── Position-Aware Wind ──────────────────────────────────────────────────────
