"""Integration tests for SITL drone containers.

Run with:
    pytest tests/test_integration_sitl.py -v --timeout=120

Supports both Docker and Kubernetes backends:
    pytest tests/test_integration_sitl.py -v --backend=docker
    pytest tests/test_integration_sitl.py -v --backend=k8s

Docker requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1
K8s requires: Helm release 'swarm' deployed in namespace 'swarm'.
"""

import socket
import subprocess
import time

import pytest

# ── Docker helpers ─────────────────────────────────────────────────────────

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "sitl_drone_1"
HOST_PORT = 5760


def _docker_container_state(name: str) -> dict:
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
        pod_status as k8s_pod_status,
    )
    K8S_AVAILABLE = True
except ImportError:
    K8S_AVAILABLE = False


# ── Backend-agnostic wrappers ─────────────────────────────────────────────


class BackendOps:
    """Dispatch SITL test operations to Docker or K8s."""

    def __init__(self, backend: str):
        self.backend = backend

    # ── container lifecycle ────────────────────────────────────────────

    def is_running(self, drone_idx: int = 0) -> bool:
        if self.backend == "k8s":
            return k8s_is_container_running(drone_idx, "sitl")
        name = f"sitl_drone_{drone_idx + 1}"
        return _docker_container_state(name)["running"]

    def is_healthy(self, drone_idx: int = 0) -> bool:
        if self.backend == "k8s":
            status = k8s_pod_status(drone_idx)
            cs = status["containers"].get("sitl", {})
            return cs.get("ready", False) and cs.get("state") == "running"
        name = f"sitl_drone_{drone_idx + 1}"
        return _docker_container_state(name)["health"] == "healthy"

    def health_status(self, drone_idx: int = 0) -> str:
        if self.backend == "k8s":
            status = k8s_pod_status(drone_idx)
            cs = status["containers"].get("sitl", {})
            if cs.get("ready", False) and cs.get("state") == "running":
                return "healthy"
            return cs.get("state", "unknown")
        name = f"sitl_drone_{drone_idx + 1}"
        return _docker_container_state(name)["health"]

    # ── logs ──────────────────────────────────────────────────────────

    def get_logs(self, drone_idx: int = 0) -> str:
        if self.backend == "k8s":
            return k8s_container_logs(drone_idx, "sitl")
        name = f"sitl_drone_{drone_idx + 1}"
        return _docker_get_logs(name)

    # ── exec ──────────────────────────────────────────────────────────

    def exec_cmd(self, cmd: list[str], drone_idx: int = 0) -> subprocess.CompletedProcess:
        if self.backend == "k8s":
            return k8s_exec_in_container(drone_idx, "sitl", cmd)
        name = f"sitl_drone_{drone_idx + 1}"
        return subprocess.run(
            ["docker", "exec", name] + cmd,
            capture_output=True, text=True,
        )

    # ── image inspection (Docker-only, skipped on K8s) ────────────────

    def inspect_image(self, drone_idx: int = 0) -> str | None:
        """Return the image name; None if not applicable (K8s)."""
        if self.backend == "k8s":
            return None
        name = f"sitl_drone_{drone_idx + 1}"
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", name],
            capture_output=True, text=True,
        )
        return result.stdout.strip() if result.returncode == 0 else ""


# ── Fixtures ──────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def ops(request):
    """Get backend operations wrapper."""
    backend_choice = request.config.getoption("--backend")
    return BackendOps(backend_choice)


@pytest.fixture(scope="module")
def sitl_up(ops):
    """Ensure SITL drone is running; start it if not."""
    if ops.backend == "k8s":
        if not K8S_AVAILABLE:
            pytest.skip("k8s_helpers not available")
        if not wait_for_pods_ready(1, timeout=120):
            pytest.fail("Drone pod did not become ready within timeout")
        yield
        return

    # Docker backend
    state = _docker_container_state(CONTAINER)
    started_here = False
    if not state["running"]:
        subprocess.run(
            COMPOSE_CMD + ["up", "-d", CONTAINER],
            check=True, capture_output=True,
        )
        started_here = True

    # Wait for healthy (up to 30s)
    for _ in range(30):
        state = _docker_container_state(CONTAINER)
        if state["health"] == "healthy":
            break
        time.sleep(1)

    yield

    if started_here:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER],
            capture_output=True,
        )


# ── Tests ──────────────────────────────────────────────────────────────────


class TestSitlDrone:
    """Tests for SITL drone (ArduPilot SITL)."""

    def test_container_running(self, sitl_up, ops):
        assert ops.is_running(0), "SITL container/pod is not running"

    def test_container_healthy(self, sitl_up, ops):
        status = ops.health_status(0)
        assert status == "healthy", (
            f"SITL health: {status} (expected healthy)"
        )

    def test_image_is_ardupilot(self, sitl_up, ops):
        image = ops.inspect_image(0)
        if image is None:
            pytest.skip("Image inspection not applicable on K8s backend")
        assert "ardupilot-sitl" in image, (
            f"Expected ardupilot-sitl image, got: {image}"
        )

    def test_arducopter_process_running(self, sitl_up, ops):
        result = ops.exec_cmd(["pgrep", "-x", "arducopter"], drone_idx=0)
        assert result.returncode == 0, "arducopter process not running"

    def test_mavlink_tcp_port_reachable(self, sitl_up, ops):
        """Host can reach MAVLink TCP port 5760."""
        if ops.backend == "k8s":
            # In K8s the port is not forwarded to localhost by default;
            # verify instead that arducopter is listening inside the pod.
            result = ops.exec_cmd(
                ["sh", "-c", "ss -tlnp | grep 5760 || netstat -tlnp 2>/dev/null | grep 5760"],
                drone_idx=0,
            )
            assert result.returncode == 0 or "5760" in (result.stdout + result.stderr), (
                "MAVLink TCP port 5760 not listening inside SITL pod"
            )
            return

        # Docker backend: connect from host
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        try:
            sock.connect(("127.0.0.1", HOST_PORT))
        except (ConnectionRefusedError, OSError) as exc:
            pytest.fail(f"Cannot connect to 127.0.0.1:{HOST_PORT}: {exc}")
        finally:
            sock.close()

    def test_sitl_logs_volume_mounted(self, sitl_up, ops):
        result = ops.exec_cmd(["ls", "/sitl/logs"], drone_idx=0)
        assert result.returncode == 0, "/sitl/logs not accessible"

    def test_sitl_home_coordinates(self, sitl_up, ops):
        """SITL started with correct Antisana volcano coordinates."""
        combined = ops.get_logs(0)
        assert "-0.508333" in combined, (
            "Expected Antisana latitude in SITL output"
        )
