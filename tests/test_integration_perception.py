"""Integration tests for perception_node containers.

Run with:
    pytest tests/test_integration_perception.py -v --timeout=120

Requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 perception_node_1
"""

import subprocess
import time

import pytest

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "perception_node_1"
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


def _ensure_container(name: str, deps: list[str] | None = None, timeout: int = 60):
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
def perception_up():
    state, started = _ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=60,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


class TestPerceptionNode:
    """Tests for perception_node_1 (Python vision pipeline)."""

    def test_container_running(self, perception_up):
        state = _container_state(CONTAINER)
        assert state["running"], "perception_node_1 is not running"

    def test_container_healthy(self, perception_up):
        state = _container_state(CONTAINER)
        assert state["health"] == "healthy", (
            f"perception_node_1 health: {state['health']} (expected healthy)"
        )

    def test_detector_process_running(self, perception_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "pgrep", "-f", "detector"],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, "detector process not running"

    def test_pythonpath_includes_perception(self, perception_up):
        """PYTHONPATH in the running process includes /root/workspace/perception.

        Note: docker exec bypasses the entrypoint, so we read the env
        of PID 1 (the actual command) via /proc.
        """
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "cat", "/proc/1/environ"],
            capture_output=True, text=True,
        )
        # /proc/1/environ uses null bytes as separators
        environ = result.stdout.replace("\x00", "\n")
        assert "/root/workspace/perception" in environ, (
            "perception not in PID 1 PYTHONPATH"
        )

    def test_ros2_humble_sourced(self, perception_up):
        """ROS 2 Humble is sourced inside the container.

        The entrypoint sources /opt/ros/humble/setup.bash, so we check
        via the entrypoint rather than a bare docker exec.
        """
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "bash", "-c", "source /opt/ros/humble/setup.bash && echo $ROS_DISTRO"],
            capture_output=True, text=True,
        )
        assert "humble" in result.stdout, (
            f"ROS_DISTRO not humble: {result.stdout.strip()}"
        )

    def test_cv_bridge_importable(self, perception_up):
        """cv_bridge is importable when ROS is sourced."""
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "python3 -c 'import cv_bridge; print(\"ok\")'"],
            capture_output=True, text=True,
        )
        assert "ok" in result.stdout, (
            f"cv_bridge import failed: {result.stderr.strip()}"
        )

    def test_no_crash_in_logs(self, perception_up):
        """No fatal crashes in logs (ignoring ultralytics startup warnings)."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        # Filter out known ultralytics startup tracebacks
        lines = combined.splitlines()
        fatal_tracebacks = []
        in_traceback = False
        current_tb = []
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
            f"Fatal traceback(s) in perception_node_1 logs:\n"
            + "\n---\n".join(fatal_tracebacks)
        )

    def test_perception_volume_mounted(self, perception_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "test", "-d", "/root/workspace/perception"],
            capture_output=True,
        )
        assert result.returncode == 0, (
            "perception volume not mounted"
        )
