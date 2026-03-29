"""Integration tests for Zenoh bridge containers.

Run with:
    pytest tests/test_integration_zenoh.py -v --timeout=120

Requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 zenoh_bridge_1
"""

import subprocess
import time

import pytest

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "zenoh_bridge_1"
SITL_CONTAINER = "sitl_drone_1"


def _container_state(name: str) -> dict:
    fmt = "{{.State.Running}}|{{.State.Health.Status}}"
    result = subprocess.run(
        ["docker", "inspect", "--format", fmt, name],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        return {"running": False, "health": "missing"}
    parts = result.stdout.strip().split("|")
    return {
        "running": parts[0] == "true",
        "health": parts[1] if len(parts) > 1 else "none",
    }


def _ensure_container(name: str, deps: list[str] | None = None, timeout: int = 30):
    targets = (deps or []) + [name]
    state = _container_state(name)
    started_here = False
    if not state["running"]:
        subprocess.run(
            COMPOSE_CMD + ["up", "-d"] + targets,
            check=True, capture_output=True,
        )
        started_here = True

    for _ in range(timeout):
        state = _container_state(name)
        if state["health"] == "healthy":
            break
        time.sleep(1)

    return state, started_here


@pytest.fixture(scope="module")
def zenoh_up():
    state, started = _ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=30,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


class TestZenohBridge:
    """Tests for zenoh_bridge_1 (Zenoh-ROS 2 DDS bridge)."""

    def test_container_running(self, zenoh_up):
        state = _container_state(CONTAINER)
        assert state["running"], "zenoh_bridge_1 is not running"

    def test_container_healthy(self, zenoh_up):
        state = _container_state(CONTAINER)
        assert state["health"] == "healthy", (
            f"zenoh_bridge_1 health: {state['health']} (expected healthy)"
        )

    def test_image_is_zenoh_bridge(self):
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", CONTAINER],
            capture_output=True, text=True,
        )
        assert "zenoh-bridge-ros2dds" in result.stdout, (
            f"Expected zenoh-bridge-ros2dds image, got: {result.stdout.strip()}"
        )

    def test_bridge_process_running(self, zenoh_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "pgrep", "-f", "zenoh-bridge-ros2dds"],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, "zenoh-bridge-ros2dds process not running"

    def test_ros_distro_is_humble(self, zenoh_up):
        """ROS_DISTRO env var is set to humble (prevents GID size mismatch)."""
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "sh", "-c", "echo $ROS_DISTRO"],
            capture_output=True, text=True,
        )
        assert result.stdout.strip() == "humble", (
            f"ROS_DISTRO={result.stdout.strip()!r}, expected 'humble'. "
            "Without this, the bridge defaults to 'iron' and misinterprets "
            "ROS 2 Humble discovery GIDs (24-byte vs 16-byte)."
        )

    def test_config_file_mounted(self, zenoh_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "test", "-f", "/etc/zenoh/bridge.json5"],
            capture_output=True,
        )
        assert result.returncode == 0, (
            "Bridge config not mounted at /etc/zenoh/bridge.json5"
        )

    def test_plugin_ros2dds_loaded(self, zenoh_up):
        """Plugin ros2dds started successfully."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "Successfully started plugin ros2dds" in combined, (
            "ros2dds plugin did not start"
        )

    def test_no_ros_distro_warning(self, zenoh_up):
        """No ROS_DISTRO fallback warning (bridge should use humble)."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "ROS_DISTRO environment variable is not set" not in combined, (
            "Bridge fell back to default ROS_DISTRO — env var not propagated"
        )

    def test_no_gid_mismatch_error(self, zenoh_up):
        """No GID byte-size mismatch errors from ros_discovery_info."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "using 24 bytes GIDs" not in combined, (
            "GID mismatch detected — ROS_DISTRO probably wrong"
        )

    def test_domain_id_matches(self, zenoh_up):
        """Bridge config uses ROS_DOMAIN_ID=1."""
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "sh", "-c", "echo $ROS_DOMAIN_ID"],
            capture_output=True, text=True,
        )
        assert result.stdout.strip() == "1", (
            f"ROS_DOMAIN_ID={result.stdout.strip()}, expected 1"
        )

    def test_namespace_in_config(self, zenoh_up):
        """Bridge config includes the drone_1 namespace."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "/swarm/drone_1" in combined, (
            "Namespace /swarm/drone_1 not found in bridge startup config"
        )
