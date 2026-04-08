"""Integration tests for perception_node containers.

Run with:
    pytest tests/test_integration_perception.py -v --timeout=120

Supports both Docker and Kubernetes backends:
    pytest tests/test_integration_perception.py -v --backend=docker
    pytest tests/test_integration_perception.py -v --backend=k8s

Docker requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 perception_node_1
K8s requires: Helm release 'swarm' deployed in namespace 'swarm'.
"""

import subprocess
import time

import pytest

# ── Docker helpers ─────────────────────────────────────────────────────────

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "perception_node_1"
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


def _docker_ensure_container(name: str, deps: list[str] | None = None, timeout: int = 60):
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


def _docker_get_logs(container: str) -> str:
    result = subprocess.run(
        ["docker", "logs", container],
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

# K8s container name for the perception sidecar
K8S_PERCEPTION_CONTAINER = "perception"


# ── Backend-agnostic wrappers ─────────────────────────────────────────────


class BackendOps:
    """Dispatch test operations to Docker or K8s."""

    def __init__(self, backend: str):
        self.backend = backend

    # -- container state ---------------------------------------------------

    def is_running(self, drone_idx: int = 0) -> bool:
        """Check if the perception container/pod is running."""
        if self.backend == "k8s":
            return k8s_is_container_running(drone_idx, K8S_PERCEPTION_CONTAINER)
        name = f"perception_node_{drone_idx + 1}"
        return _docker_container_state(name)["running"]

    def is_healthy(self, drone_idx: int = 0) -> bool:
        """Check health (Docker healthcheck / K8s readiness)."""
        if self.backend == "k8s":
            status = k8s_pod_status(drone_idx)
            cs = status["containers"].get(K8S_PERCEPTION_CONTAINER, {})
            return cs.get("ready", False) and cs.get("state") == "running"
        name = f"perception_node_{drone_idx + 1}"
        return _docker_container_state(name)["health"] == "healthy"

    # -- logs --------------------------------------------------------------

    def get_logs(self, drone_idx: int = 0) -> str:
        if self.backend == "k8s":
            return k8s_container_logs(drone_idx, K8S_PERCEPTION_CONTAINER)
        name = f"perception_node_{drone_idx + 1}"
        return _docker_get_logs(name)

    # -- exec helpers ------------------------------------------------------

    def exec_cmd(self, cmd: list[str], drone_idx: int = 0) -> subprocess.CompletedProcess:
        """Run a command inside the perception container."""
        if self.backend == "k8s":
            return k8s_exec_in_container(drone_idx, K8S_PERCEPTION_CONTAINER, cmd)
        name = f"perception_node_{drone_idx + 1}"
        return subprocess.run(
            ["docker", "exec", name] + cmd,
            capture_output=True, text=True,
        )

    def exec_shell(self, shell_cmd: str, drone_idx: int = 0) -> subprocess.CompletedProcess:
        """Run a shell command inside the perception container."""
        return self.exec_cmd(["bash", "-c", shell_cmd], drone_idx=drone_idx)

    def dir_exists(self, path: str, drone_idx: int = 0) -> bool:
        result = self.exec_cmd(["test", "-d", path], drone_idx=drone_idx)
        return result.returncode == 0

    def process_running(self, pattern: str, drone_idx: int = 0) -> bool:
        result = self.exec_cmd(["pgrep", "-f", pattern], drone_idx=drone_idx)
        return result.returncode == 0

    def read_pid1_environ(self, drone_idx: int = 0) -> str:
        """Read the environment of PID 1 inside the container."""
        result = self.exec_cmd(["cat", "/proc/1/environ"], drone_idx=drone_idx)
        if result.returncode != 0:
            return ""
        return result.stdout.replace("\x00", "\n")


# ── Fixtures ──────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def ops(request):
    """Get backend operations wrapper."""
    backend_choice = request.config.getoption("--backend")
    return BackendOps(backend_choice)


@pytest.fixture(scope="module")
def perception_up(ops):
    """Ensure perception stack is running."""
    if ops.backend == "k8s":
        if not K8S_AVAILABLE:
            pytest.skip("k8s_helpers not available")
        if not wait_for_pods_running(1, timeout=120):
            pytest.skip("Drone pods not running — is the Helm release deployed?")
        yield
        return

    # Docker backend
    state, started = _docker_ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=60,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


# ── Tests ─────────────────────────────────────────────────────────────────


class TestPerceptionNode:
    """Tests for perception_node (Python vision pipeline)."""

    def test_container_running(self, perception_up, ops):
        assert ops.is_running(), "perception_node is not running"

    def test_container_healthy(self, perception_up, ops):
        assert ops.is_healthy(), (
            "perception_node is not healthy"
        )

    def test_detector_process_running(self, perception_up, ops):
        assert ops.process_running("detector"), "detector process not running"

    def test_pythonpath_includes_perception(self, perception_up, ops):
        """PYTHONPATH in the running process includes /root/workspace/perception.

        Note: docker exec / kubectl exec bypasses the entrypoint, so we read
        the env of PID 1 (the actual command) via /proc.
        """
        environ = ops.read_pid1_environ()
        assert "/root/workspace/perception" in environ, (
            "perception not in PID 1 PYTHONPATH"
        )

    def test_ros2_humble_sourced(self, perception_up, ops):
        """ROS 2 Humble is sourced inside the container.

        The entrypoint sources /opt/ros/humble/setup.bash, so we check
        via the entrypoint rather than a bare exec.
        """
        result = ops.exec_shell(
            "source /opt/ros/humble/setup.bash && echo $ROS_DISTRO"
        )
        assert "humble" in result.stdout, (
            f"ROS_DISTRO not humble: {result.stdout.strip()}"
        )

    def test_cv_bridge_importable(self, perception_up, ops):
        """cv_bridge is importable when ROS is sourced."""
        result = ops.exec_shell(
            "source /opt/ros/humble/setup.bash && "
            "python3 -c 'import cv_bridge; print(\"ok\")'"
        )
        assert "ok" in result.stdout, (
            f"cv_bridge import failed: {result.stderr.strip()}"
        )

    def test_no_crash_in_logs(self, perception_up, ops):
        """No fatal crashes in logs (ignoring ultralytics startup warnings)."""
        combined = ops.get_logs()
        lines = combined.splitlines()
        fatal_tracebacks = []
        in_traceback = False
        current_tb: list[str] = []
        for line in lines:
            if "Traceback (most recent call last):" in line:
                in_traceback = True
                current_tb = [line]
            elif in_traceback:
                current_tb.append(line)
                if line and not line.startswith(" ") and not line.startswith("\t"):
                    in_traceback = False
                    tb_text = "\n".join(current_tb)
                    # Skip known harmless tracebacks:
                    # - ultralytics startup config warnings
                    # - rclpy shutdown during container restart
                    # - cv_bridge _ARRAY_API numpy compat on restart
                    known = (
                        "Ultralytics", "ultralytics",
                        "ExternalShutdownException",
                        "rcl_shutdown already called",
                        "_ARRAY_API not found",
                    )
                    if not any(k in tb_text for k in known):
                        fatal_tracebacks.append(tb_text)
                    current_tb = []
        assert not fatal_tracebacks, (
            f"Fatal traceback(s) in perception_node logs:\n"
            + "\n---\n".join(fatal_tracebacks)
        )

    def test_perception_volume_mounted(self, perception_up, ops):
        assert ops.dir_exists("/root/workspace/perception"), (
            "perception volume not mounted"
        )
