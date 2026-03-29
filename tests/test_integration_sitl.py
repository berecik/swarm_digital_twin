"""Integration tests for SITL drone containers.

Run with:
    pytest tests/test_integration_sitl.py -v --timeout=120

Requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1
"""

import socket
import subprocess
import time

import pytest

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "sitl_drone_1"
HOST_PORT = 5760


def _container_state(name: str) -> dict:
    """Return (running, health) for a container."""
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


@pytest.fixture(scope="module")
def sitl_up():
    """Ensure sitl_drone_1 is running; start it if not."""
    state = _container_state(CONTAINER)
    started_here = False
    if not state["running"]:
        subprocess.run(
            COMPOSE_CMD + ["up", "-d", CONTAINER],
            check=True, capture_output=True,
        )
        started_here = True

    # Wait for healthy (up to 30s)
    for _ in range(30):
        state = _container_state(CONTAINER)
        if state["health"] == "healthy":
            break
        time.sleep(1)

    yield state

    if started_here:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER],
            capture_output=True,
        )


class TestSitlDrone:
    """Tests for sitl_drone_1 (ArduPilot SITL)."""

    def test_container_running(self, sitl_up):
        assert sitl_up["running"] or _container_state(CONTAINER)["running"]

    def test_container_healthy(self, sitl_up):
        state = _container_state(CONTAINER)
        assert state["health"] == "healthy", (
            f"sitl_drone_1 health: {state['health']} (expected healthy)"
        )

    def test_image_is_ardupilot(self):
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", CONTAINER],
            capture_output=True, text=True,
        )
        assert "ardupilot-sitl" in result.stdout, (
            f"Expected ardupilot-sitl image, got: {result.stdout.strip()}"
        )

    def test_arducopter_process_running(self, sitl_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "pgrep", "-x", "arducopter"],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, "arducopter process not running"

    def test_mavlink_tcp_port_reachable(self, sitl_up):
        """Host can reach MAVLink TCP port 5760."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        try:
            sock.connect(("127.0.0.1", HOST_PORT))
        except (ConnectionRefusedError, OSError) as exc:
            pytest.fail(f"Cannot connect to 127.0.0.1:{HOST_PORT}: {exc}")
        finally:
            sock.close()

    def test_sitl_logs_volume_mounted(self, sitl_up):
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "ls", "/sitl/logs"],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, "/sitl/logs not accessible"

    def test_sitl_home_coordinates(self, sitl_up):
        """SITL started with correct Antisana volcano coordinates."""
        result = subprocess.run(
            ["docker", "logs", CONTAINER],
            capture_output=True, text=True,
        )
        assert "-0.508333" in result.stdout or "-0.508333" in result.stderr, (
            "Expected Antisana latitude in SITL output"
        )
