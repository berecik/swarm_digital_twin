"""Integration tests for swarm_node containers.

Run with:
    pytest tests/test_integration_swarm_node.py -v --timeout=300

Requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 swarm_node_1
"""

import subprocess
import time

import pytest

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "swarm_node_1"
SITL_CONTAINER = "sitl_drone_1"


def _wait_for_log(container: str, needle: str, timeout: int = 120) -> str | None:
    """Poll docker logs until `needle` appears or `timeout` seconds elapse.

    Returns the combined log output if found, or None on timeout.
    """
    for _ in range(timeout):
        result = subprocess.run(
            ["docker", "logs", container],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        if needle in combined:
            return combined
        time.sleep(1)
    return None


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


def _ensure_container(name: str, deps: list[str] | None = None, timeout: int = 300):
    """Start container and deps if not running; wait for healthy."""
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
def swarm_node_up():
    """Ensure swarm_node_1 (and its dep sitl_drone_1) are running."""
    state, started = _ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=300,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


class TestSwarmNode:
    """Tests for swarm_node_1 (Rust swarm control)."""

    def test_container_running(self, swarm_node_up):
        state = _container_state(CONTAINER)
        assert state["running"], "swarm_node_1 is not running"

    def test_container_healthy(self, swarm_node_up):
        state = _container_state(CONTAINER)
        assert state["health"] == "healthy", (
            f"swarm_node_1 health: {state['health']} (expected healthy)"
        )

    def test_image_is_swarm_companion(self):
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", CONTAINER],
            capture_output=True, text=True,
        )
        assert "swarm_companion" in result.stdout, (
            f"Expected swarm_companion image, got: {result.stdout.strip()}"
        )

    def test_rust_binary_compiles(self, swarm_node_up):
        """swarm_node binary compiled without errors."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "error[E" not in combined, (
            "Rust compilation errors found in swarm_node_1 logs"
        )

    def test_zenoh_connected(self, swarm_node_up):
        """Node successfully connected to Zenoh."""
        combined = _wait_for_log(CONTAINER, "Zenoh connected", timeout=180)
        assert combined is not None, (
            "swarm_node_1 did not report 'Zenoh connected' within timeout "
            "(binary may still be compiling)"
        )

    def test_control_loop_running(self, swarm_node_up):
        """Node entered main control loop."""
        combined = _wait_for_log(CONTAINER, "Control loop running", timeout=180)
        assert combined is not None, (
            "swarm_node_1 did not enter control loop within timeout "
            "(binary may still be compiling)"
        )

    def test_no_panic(self, swarm_node_up):
        """No panics in the log."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        combined = result.stdout + result.stderr
        assert "panicked at" not in combined, (
            f"Panic found in swarm_node_1 logs"
        )

    def test_swarm_control_volume_mounted(self, swarm_node_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER,
             "test", "-f", "/root/workspace/swarm_control/Cargo.toml"],
            capture_output=True,
        )
        assert result.returncode == 0, (
            "swarm_control volume not mounted at /root/workspace/swarm_control"
        )
