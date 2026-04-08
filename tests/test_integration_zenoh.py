"""Integration tests for Zenoh bridge containers.

Run with:
    pytest tests/test_integration_zenoh.py -v --timeout=120

Supports both Docker and Kubernetes backends:
    pytest tests/test_integration_zenoh.py -v --backend=docker
    pytest tests/test_integration_zenoh.py -v --backend=k8s

Docker requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 zenoh_bridge_1
K8s requires: Helm release 'swarm' deployed in namespace 'swarm'.
"""

import subprocess
import time

import pytest

# ── Docker helpers ─────────────────────────────────────────────────────────

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "zenoh_bridge_1"
SITL_CONTAINER = "sitl_drone_1"


def _docker_container_state(name: str) -> dict:
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


def _docker_ensure_container(name: str, deps: list[str] | None = None, timeout: int = 30):
    targets = (deps or []) + [name]
    state = _docker_container_state(name)
    started_here = False
    if not state["running"]:
        subprocess.run(
            COMPOSE_CMD + ["up", "-d"] + targets,
            check=True, capture_output=True,
        )
        started_here = True

    for _ in range(timeout):
        state = _docker_container_state(name)
        if state["health"] == "healthy":
            break
        time.sleep(1)

    return state, started_here


def _docker_get_logs(name: str) -> str:
    result = subprocess.run(
        ["docker", "logs", name],
        capture_output=True, text=True,
    )
    return result.stdout + result.stderr


# ── K8s helpers ────────────────────────────────────────────────────────────

try:
    from k8s_helpers import (
        pod_name as k8s_pod_name,
        kubectl,
        container_logs as k8s_container_logs,
        wait_for_log as k8s_wait_for_log,
        exec_in_container as k8s_exec_in_container,
        is_container_running as k8s_is_container_running,
        wait_for_pods_ready,
        wait_for_pods_running,
        pod_status as k8s_pod_status,
    )
    K8S_AVAILABLE = True
except ImportError:
    K8S_AVAILABLE = False

K8S_ZENOH_CONTAINER = "zenoh-bridge"


# ── Backend-agnostic wrappers ─────────────────────────────────────────────


class BackendOps:
    """Dispatch test operations to Docker or K8s."""

    def __init__(self, backend: str):
        self.backend = backend

    # -- Container state ---------------------------------------------------

    def is_running(self) -> bool:
        if self.backend == "k8s":
            return k8s_is_container_running(0, K8S_ZENOH_CONTAINER)
        return _docker_container_state(CONTAINER)["running"]

    def is_healthy(self) -> bool:
        if self.backend == "k8s":
            status = k8s_pod_status(0)
            cs = status["containers"].get(K8S_ZENOH_CONTAINER, {})
            return cs.get("ready", False) and cs.get("state") == "running"
        return _docker_container_state(CONTAINER)["health"] == "healthy"

    # -- Logs --------------------------------------------------------------

    def get_logs(self) -> str:
        if self.backend == "k8s":
            return k8s_container_logs(0, K8S_ZENOH_CONTAINER)
        return _docker_get_logs(CONTAINER)

    def wait_for_log(self, needle: str, timeout: int = 60) -> str | None:
        if self.backend == "k8s":
            return k8s_wait_for_log(0, K8S_ZENOH_CONTAINER, needle, timeout)
        for _ in range(timeout):
            combined = _docker_get_logs(CONTAINER)
            if needle in combined:
                return combined
            time.sleep(1)
        return None

    # -- Exec / inspect ----------------------------------------------------

    def exec_cmd(self, cmd: list[str]) -> subprocess.CompletedProcess:
        if self.backend == "k8s":
            return k8s_exec_in_container(0, K8S_ZENOH_CONTAINER, cmd)
        return subprocess.run(
            ["docker", "exec", CONTAINER] + cmd,
            capture_output=True, text=True,
        )

    def get_env(self, var: str) -> str:
        result = self.exec_cmd(["sh", "-c", f"echo ${var}"])
        return result.stdout.strip() if result.returncode == 0 else ""

    def file_exists(self, path: str) -> bool:
        result = self.exec_cmd(["test", "-f", path])
        return result.returncode == 0

    def get_image(self) -> str:
        """Return the image name for the zenoh container."""
        if self.backend == "k8s":
            result = kubectl(
                "get", "pod", k8s_pod_name(0),
                "-o", f"jsonpath={{.spec.containers[?(@.name==\"{K8S_ZENOH_CONTAINER}\")].image}}",
            )
            return result.stdout.strip() if result.returncode == 0 else ""
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", CONTAINER],
            capture_output=True, text=True,
        )
        return result.stdout.strip() if result.returncode == 0 else ""

    def process_running(self, pattern: str) -> bool:
        result = self.exec_cmd(["pgrep", "-f", pattern])
        return result.returncode == 0


# ── Fixtures ──────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def ops(request):
    """Get backend operations wrapper."""
    backend_choice = request.config.getoption("--backend")
    return BackendOps(backend_choice)


@pytest.fixture(scope="module")
def zenoh_up(ops):
    """Ensure zenoh bridge is running."""
    if ops.backend == "k8s":
        if not K8S_AVAILABLE:
            pytest.skip("k8s_helpers not available")
        if not wait_for_pods_running(1, timeout=120):
            pytest.skip("Drone pods not running — is the Helm release deployed?")
        yield
        return

    # Docker backend
    state, started = _docker_ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=30,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


# ── Tests ─────────────────────────────────────────────────────────────────


class TestZenohBridge:
    """Tests for the Zenoh-ROS 2 DDS bridge."""

    def test_container_running(self, zenoh_up, ops):
        assert ops.is_running(), "zenoh bridge is not running"

    def test_container_healthy(self, zenoh_up, ops):
        assert ops.is_healthy(), "zenoh bridge is not healthy"

    def test_image_is_zenoh_bridge(self, zenoh_up, ops):
        image = ops.get_image()
        assert "zenoh-bridge-ros2dds" in image, (
            f"Expected zenoh-bridge-ros2dds image, got: {image}"
        )

    def test_bridge_process_running(self, zenoh_up, ops):
        assert ops.process_running("zenoh-bridge-ros2dds"), (
            "zenoh-bridge-ros2dds process not running"
        )

    def test_ros_distro_is_humble(self, zenoh_up, ops):
        """ROS_DISTRO env var is set to humble (prevents GID size mismatch)."""
        distro = ops.get_env("ROS_DISTRO")
        assert distro == "humble", (
            f"ROS_DISTRO={distro!r}, expected 'humble'. "
            "Without this, the bridge defaults to 'iron' and misinterprets "
            "ROS 2 Humble discovery GIDs (24-byte vs 16-byte)."
        )

    def test_config_file_mounted(self, zenoh_up, ops):
        # Docker mounts at /etc/zenoh/bridge.json5, K8s at /config/bridge.json5
        path = "/config/bridge.json5" if ops.backend == "k8s" else "/etc/zenoh/bridge.json5"
        assert ops.file_exists(path), (
            f"Bridge config not mounted at {path}"
        )

    def test_plugin_ros2dds_loaded(self, zenoh_up, ops):
        """Plugin ros2dds started successfully."""
        combined = ops.get_logs()
        assert "Successfully started plugin ros2dds" in combined, (
            "ros2dds plugin did not start"
        )

    def test_no_ros_distro_warning(self, zenoh_up, ops):
        """No ROS_DISTRO fallback warning (bridge should use humble)."""
        combined = ops.get_logs()
        assert "ROS_DISTRO environment variable is not set" not in combined, (
            "Bridge fell back to default ROS_DISTRO — env var not propagated"
        )

    def test_no_gid_mismatch_error(self, zenoh_up, ops):
        """No GID byte-size mismatch errors from ros_discovery_info."""
        combined = ops.get_logs()
        assert "using 24 bytes GIDs" not in combined, (
            "GID mismatch detected — ROS_DISTRO probably wrong"
        )

    def test_domain_id_matches(self, zenoh_up, ops):
        """Bridge config uses domain 1 for drone_1."""
        if ops.backend == "k8s":
            # In K8s the domain is configured in the bridge config file, not env var.
            # Read the config and check the domain field.
            config_path = "/config/bridge.json5"
            content = ops.exec_cmd(["cat", config_path]).stdout
            assert '"domain": 1' in content or '"domain":1' in content, (
                f"Bridge config does not contain domain 1"
            )
        else:
            domain_id = ops.get_env("ROS_DOMAIN_ID")
            assert domain_id == "1", (
                f"ROS_DOMAIN_ID={domain_id}, expected 1"
            )

    def test_namespace_in_config(self, zenoh_up, ops):
        """Bridge config includes the drone_1 namespace."""
        combined = ops.get_logs()
        assert "/swarm/drone_1" in combined, (
            "Namespace /swarm/drone_1 not found in bridge startup config"
        )
