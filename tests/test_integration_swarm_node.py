"""Integration tests for swarm_node containers.

Run with:
    pytest tests/test_integration_swarm_node.py -v --timeout=300

Supports both Docker and Kubernetes backends:
    pytest tests/test_integration_swarm_node.py -v --backend=docker
    pytest tests/test_integration_swarm_node.py -v --backend=k8s

Docker requires: docker compose stack with profile swarm_sitl running.
    docker compose --profile swarm_sitl up -d sitl_drone_1 swarm_node_1
K8s requires: Helm release 'swarm' deployed in namespace 'swarm'.
"""

import subprocess
import time

import pytest

# ── Docker helpers ─────────────────────────────────────────────────────────

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
CONTAINER = "swarm_node_1"
SITL_CONTAINER = "sitl_drone_1"


def _docker_wait_for_log(container: str, needle: str, timeout: int = 120) -> str | None:
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


def _docker_get_logs(container: str) -> str:
    result = subprocess.run(
        ["docker", "logs", container],
        capture_output=True, text=True,
    )
    return result.stdout + result.stderr


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


def _docker_ensure_container(name: str, deps: list[str] | None = None, timeout: int = 300):
    """Start container and deps if not running; wait for healthy."""
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


def _k8s_get_logs(ordinal: int) -> str:
    if not K8S_AVAILABLE:
        return ""
    return k8s_container_logs(ordinal, "swarm-node")


# ── Backend-agnostic wrappers ─────────────────────────────────────────────


class BackendOps:
    """Dispatch test operations to Docker or K8s."""

    def __init__(self, backend: str):
        self.backend = backend

    def wait_for_log(self, needle: str, timeout: int = 180) -> str | None:
        if self.backend == "k8s":
            return k8s_wait_for_log(0, "swarm-node", needle, timeout)
        return _docker_wait_for_log(CONTAINER, needle, timeout)

    def get_logs(self) -> str:
        if self.backend == "k8s":
            return _k8s_get_logs(0)
        return _docker_get_logs(CONTAINER)

    def is_running(self) -> bool:
        if self.backend == "k8s":
            return k8s_is_container_running(0, "swarm-node")
        return _docker_container_state(CONTAINER)["running"]

    def is_healthy(self) -> bool:
        if self.backend == "k8s":
            status = k8s_pod_status(0)
            return all(
                c.get("ready", False)
                for c in status["containers"].values()
            )
        return _docker_container_state(CONTAINER)["health"] == "healthy"

    def file_exists(self, path: str) -> bool:
        if self.backend == "k8s":
            result = k8s_exec_in_container(
                0, "swarm-node", ["test", "-f", path]
            )
            return result.returncode == 0
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "test", "-f", path],
            capture_output=True,
        )
        return result.returncode == 0

    def inspect_image(self) -> str:
        """Return the container image name (Docker only)."""
        if self.backend == "k8s":
            # In K8s the image is set by the Helm chart; return the container
            # image from the pod spec.
            import json as _json
            result = kubectl(
                "get", "pod", k8s_pod_name(0),
                "-o", "jsonpath={.spec.containers[?(@.name=='swarm-node')].image}",
            )
            return result.stdout.strip() if result.returncode == 0 else ""
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Config.Image}}", CONTAINER],
            capture_output=True, text=True,
        )
        return result.stdout.strip()


# ── Fixtures ──────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def ops(request):
    """Get backend operations wrapper."""
    backend_choice = request.config.getoption("--backend")
    return BackendOps(backend_choice)


@pytest.fixture(scope="module")
def swarm_node_up(ops):
    """Ensure swarm_node_1 (and its dep sitl_drone_1) are running."""
    if ops.backend == "k8s":
        if not K8S_AVAILABLE:
            pytest.skip("k8s_helpers not available")
        if not wait_for_pods_ready(1, timeout=300):
            pytest.fail("Drone pod did not become ready within timeout")
        yield
        return

    # Docker backend
    state, started = _docker_ensure_container(
        CONTAINER, deps=[SITL_CONTAINER], timeout=300,
    )
    yield state
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop", CONTAINER, SITL_CONTAINER],
            capture_output=True,
        )


# ── Tests ─────────────────────────────────────────────────────────────────


class TestSwarmNode:
    """Tests for swarm_node_1 (Rust swarm control)."""

    def test_container_running(self, swarm_node_up, ops):
        assert ops.is_running(), "swarm_node is not running"

    def test_container_healthy(self, swarm_node_up, ops):
        assert ops.is_healthy(), "swarm_node is not healthy"

    def test_image_is_swarm_companion(self, swarm_node_up, ops):
        image = ops.inspect_image()
        assert "swarm_companion" in image or "swarm-companion" in image, (
            f"Expected swarm_companion image, got: {image}"
        )

    def test_rust_binary_compiles(self, swarm_node_up, ops):
        """swarm_node binary compiled without errors."""
        combined = ops.get_logs()
        assert "error[E" not in combined, (
            "Rust compilation errors found in swarm_node logs"
        )

    def test_zenoh_connected(self, swarm_node_up, ops):
        """Node successfully connected to Zenoh."""
        combined = ops.wait_for_log("Zenoh connected", timeout=180)
        assert combined is not None, (
            "swarm_node did not report 'Zenoh connected' within timeout "
            "(binary may still be compiling)"
        )

    def test_control_loop_running(self, swarm_node_up, ops):
        """Node entered main control loop."""
        combined = ops.wait_for_log("Control loop running", timeout=180)
        assert combined is not None, (
            "swarm_node did not enter control loop within timeout "
            "(binary may still be compiling)"
        )

    def test_no_panic(self, swarm_node_up, ops):
        """No panics in the log."""
        combined = ops.get_logs()
        assert "panicked at" not in combined, (
            "Panic found in swarm_node logs"
        )

    def test_swarm_control_volume_mounted(self, swarm_node_up, ops):
        assert ops.file_exists("/root/workspace/swarm_control/Cargo.toml"), (
            "swarm_control volume not mounted at /root/workspace/swarm_control"
        )
