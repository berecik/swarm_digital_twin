"""Integration tests for swarm formation flight.

Run with:
    pytest tests/test_integration_swarm_formation.py -v

Supports both Docker and Kubernetes backends:
    pytest tests/test_integration_swarm_formation.py -v --backend=docker
    pytest tests/test_integration_swarm_formation.py -v --backend=k8s

Docker requires: docker compose stack with profile swarm_sitl running.
K8s requires: Helm release 'swarm' deployed in namespace 'swarm'.
"""

import json
import subprocess
import time

import pytest

# ── Docker helpers ─────────────────────────────────────────────────────────

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
SWARM_NODES = ["swarm_node_1", "swarm_node_2"]
SITL_DRONES = ["sitl_drone_1", "sitl_drone_2"]
ALL_CONTAINERS = SWARM_NODES + SITL_DRONES


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


def _docker_wait_for_log(container: str, needle: str, timeout: int = 180) -> str | None:
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


def _docker_ensure_containers(names: list[str], timeout: int = 300):
    need_start = any(
        not _docker_container_state(n)["running"] for n in names
    )
    started = False
    if need_start:
        subprocess.run(
            COMPOSE_CMD + ["up", "-d"] + names,
            check=True, capture_output=True,
        )
        started = True

    for name in names:
        for _ in range(timeout):
            state = _docker_container_state(name)
            if state["health"] == "healthy":
                break
            time.sleep(1)

    return started


def _docker_deploy_formation_config(containers: list[str], n_drones: int = 2):
    config = {
        "pattern": {"type": "Ring", "radius": 8.0},
        "altitude": 20.0,
        "waypoints": [
            [0.0, 0.0, 20.0],
            [20.0, 0.0, 20.0],
            [20.0, 20.0, 20.0],
            [0.0, 0.0, 20.0],
        ],
        "waypoint_accept_radius": 3.0,
        "cruise_speed": 4.0,
        "n_drones": n_drones,
    }
    config_json = json.dumps(config)

    for container in containers:
        subprocess.run(
            ["docker", "exec", container, "sh", "-c",
             f"echo '{config_json}' > /root/workspace/swarm_mission.json"],
            capture_output=True,
        )

    return config


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

    def __init__(self, backend: str, n_drones: int = 2):
        self.backend = backend
        self.n_drones = n_drones

    def wait_for_log(self, drone_idx: int, needle: str, timeout: int = 180) -> str | None:
        if self.backend == "k8s":
            return k8s_wait_for_log(drone_idx, "swarm-node", needle, timeout)
        name = f"swarm_node_{drone_idx + 1}"
        return _docker_wait_for_log(name, needle, timeout)

    def get_logs(self, drone_idx: int) -> str:
        if self.backend == "k8s":
            return _k8s_get_logs(drone_idx)
        name = f"swarm_node_{drone_idx + 1}"
        return _docker_get_logs(name)

    def is_running(self, drone_idx: int, service: str = "swarm_node") -> bool:
        if self.backend == "k8s":
            container = {
                "swarm_node": "swarm-node",
                "sitl_drone": "sitl",
            }.get(service, service)
            return k8s_is_container_running(drone_idx, container)
        name = f"{service}_{drone_idx + 1}"
        return _docker_container_state(name)["running"]

    def is_healthy(self, drone_idx: int) -> bool:
        if self.backend == "k8s":
            status = k8s_pod_status(drone_idx)
            return all(
                c.get("ready", False)
                for c in status["containers"].values()
            )
        name = f"swarm_node_{drone_idx + 1}"
        return _docker_container_state(name)["health"] == "healthy"

    def file_exists(self, drone_idx: int, path: str) -> bool:
        if self.backend == "k8s":
            result = k8s_exec_in_container(
                drone_idx, "swarm-node", ["test", "-f", path]
            )
            return result.returncode == 0
        name = f"swarm_node_{drone_idx + 1}"
        result = subprocess.run(
            ["docker", "exec", name, "test", "-f", path],
            capture_output=True,
        )
        return result.returncode == 0

    def dir_exists(self, drone_idx: int, path: str) -> bool:
        if self.backend == "k8s":
            result = k8s_exec_in_container(
                drone_idx, "swarm-node", ["test", "-d", path]
            )
            return result.returncode == 0
        name = f"swarm_node_{drone_idx + 1}"
        result = subprocess.run(
            ["docker", "exec", name, "test", "-d", path],
            capture_output=True,
        )
        return result.returncode == 0

    def read_file(self, drone_idx: int, path: str) -> str:
        if self.backend == "k8s":
            result = k8s_exec_in_container(
                drone_idx, "swarm-node", ["cat", path]
            )
            return result.stdout if result.returncode == 0 else ""
        name = f"swarm_node_{drone_idx + 1}"
        result = subprocess.run(
            ["docker", "exec", name, "cat", path],
            capture_output=True, text=True,
        )
        return result.stdout if result.returncode == 0 else ""


# ── Fixtures ──────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def ops(request):
    """Get backend operations wrapper."""
    backend_choice = request.config.getoption("--backend")
    return BackendOps(backend_choice)


@pytest.fixture(scope="module")
def swarm_formation_up(ops):
    """Ensure swarm formation stack is running."""
    if ops.backend == "k8s":
        if not K8S_AVAILABLE:
            pytest.skip("k8s_helpers not available")
        if not wait_for_pods_ready(ops.n_drones, timeout=300):
            pytest.fail("Drone pods did not become ready within timeout")
        yield
        return

    # Docker backend
    started = _docker_ensure_containers(ALL_CONTAINERS, timeout=300)
    _docker_deploy_formation_config(SWARM_NODES, n_drones=2)
    yield
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop"] + ALL_CONTAINERS,
            capture_output=True,
        )


# ── Container health ────────────────────────────────────────────────────────


class TestFormationContainerHealth:
    """Basic health checks for swarm formation containers."""

    def test_swarm_nodes_running(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            assert ops.is_running(i, "swarm_node"), (
                f"swarm_node drone_idx={i} is not running"
            )

    def test_swarm_nodes_healthy(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            assert ops.is_healthy(i), (
                f"swarm_node drone_idx={i} is not healthy"
            )

    def test_sitl_drones_running(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            assert ops.is_running(i, "sitl_drone"), (
                f"sitl_drone drone_idx={i} is not running"
            )


# ── Formation config loading ────────────────────────────────────────────────


class TestFormationConfigLoad:
    """Verify swarm_nodes load the formation config."""

    def test_formation_config_deployed(self, swarm_formation_up, ops):
        """swarm_mission.json exists in each container."""
        for i in range(ops.n_drones):
            assert ops.file_exists(i, "/root/workspace/swarm_mission.json"), (
                f"drone_idx={i}: swarm_mission.json not found"
            )

    def test_formation_config_valid_json(self, swarm_formation_up, ops):
        """swarm_mission.json is valid JSON with expected fields."""
        for i in range(ops.n_drones):
            content = ops.read_file(i, "/root/workspace/swarm_mission.json")
            assert content, f"drone_idx={i}: swarm_mission.json is empty"
            config = json.loads(content)
            assert "pattern" in config
            assert "waypoints" in config
            assert "n_drones" in config
            assert config["n_drones"] >= 2
            assert len(config["waypoints"]) >= 2


# ── Zenoh + control loop ────────────────────────────────────────────────────


class TestFormationStartup:
    """Verify swarm_nodes connect to Zenoh and enter control loop."""

    def test_zenoh_connected(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            combined = ops.wait_for_log(i, "Zenoh connected", timeout=180)
            assert combined is not None, (
                f"drone_idx={i} did not report 'Zenoh connected' within timeout"
            )

    def test_control_loop_running(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            combined = ops.wait_for_log(i, "Control loop running", timeout=180)
            assert combined is not None, (
                f"drone_idx={i} did not enter control loop within timeout"
            )

    def test_formation_loaded(self, swarm_formation_up, ops):
        """Nodes log that they loaded the formation config."""
        for i in range(ops.n_drones):
            combined = ops.wait_for_log(i, "Formation loaded", timeout=180)
            assert combined is not None, (
                f"drone_idx={i} did not report 'Formation loaded' within timeout "
                "(binary may still be compiling)"
            )

    def test_no_panic(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            combined = ops.get_logs(i)
            assert "panicked at" not in combined, (
                f"Panic found in drone_idx={i} logs"
            )

    def test_no_compilation_errors(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            combined = ops.get_logs(i)
            assert "error[E" not in combined, (
                f"Rust compilation errors found in drone_idx={i} logs"
            )


# ── Formation flight state machine ──────────────────────────────────────────


class TestFormationFlight:
    """Verify drones progress through formation flight states."""

    def test_enters_formation_state(self, swarm_formation_up, ops):
        """At least one node should reach FormUp or Formation state."""
        for i in range(ops.n_drones):
            found = ops.wait_for_log(i, "state=FormUp", timeout=240)
            if found is None:
                found = ops.wait_for_log(i, "state=Formation", timeout=60)
            assert found is not None, (
                f"drone_idx={i} did not reach FormUp or Formation state within timeout"
            )

    def test_reports_neighbors(self, swarm_formation_up, ops):
        """Nodes should see at least 1 neighbor via Zenoh boid broadcast."""
        found = False
        for i in range(ops.n_drones):
            combined = ops.wait_for_log(i, "neighbors=", timeout=240)
            if combined is None:
                continue
            for line in combined.splitlines():
                if "neighbors=" in line:
                    for part in line.split():
                        if part.startswith("neighbors="):
                            count_str = part.split("=")[1]
                            count = int(count_str.split()[0])
                            if count > 0:
                                found = True
                                break
                if found:
                    break
            if found:
                break

        assert found, (
            "No swarm_node reported seeing any neighbors — "
            "Zenoh boid broadcast may not be working"
        )

    def test_waypoint_progress_reported(self, swarm_formation_up, ops):
        """Nodes should log waypoint progress (wp=N/M)."""
        for i in range(ops.n_drones):
            combined = ops.wait_for_log(i, "wp=", timeout=240)
            assert combined is not None, (
                f"drone_idx={i} did not report waypoint progress within timeout"
            )


# ── Volume mounts ────────────────────────────────────────────────────────────


class TestFormationVolumes:
    """Verify required volume mounts for formation flight."""

    def test_swarm_control_volume(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            assert ops.file_exists(i, "/root/workspace/swarm_control/Cargo.toml"), (
                f"drone_idx={i}: swarm_control volume not mounted"
            )

    def test_simulation_volume(self, swarm_formation_up, ops):
        for i in range(ops.n_drones):
            assert ops.dir_exists(i, "/root/workspace/simulation"), (
                f"drone_idx={i}: simulation volume not mounted"
            )
