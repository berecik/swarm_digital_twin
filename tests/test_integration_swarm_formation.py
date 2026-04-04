"""Integration tests for swarm formation flight.

Run with:
    pytest tests/test_integration_swarm_formation.py -v

Requires: docker compose stack with profile swarm_sitl running
(at least 2 swarm_node containers with swarm_mission.json deployed).

    docker compose --profile swarm_sitl up -d \
        sitl_drone_1 swarm_node_1 sitl_drone_2 swarm_node_2
"""

import json
import subprocess
import time

import pytest

COMPOSE_CMD = ["docker", "compose", "--profile", "swarm_sitl"]
SWARM_NODES = ["swarm_node_1", "swarm_node_2"]
SITL_DRONES = ["sitl_drone_1", "sitl_drone_2"]
ALL_CONTAINERS = SWARM_NODES + SITL_DRONES


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


def _wait_for_log(container: str, needle: str, timeout: int = 180) -> str | None:
    """Poll docker logs until `needle` appears or `timeout` seconds elapse."""
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


def _get_logs(container: str) -> str:
    result = subprocess.run(
        ["docker", "logs", container],
        capture_output=True, text=True,
    )
    return result.stdout + result.stderr


def _ensure_containers(names: list[str], timeout: int = 300):
    """Start containers if not running; wait for healthy."""
    need_start = any(
        not _container_state(n)["running"] for n in names
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
            state = _container_state(name)
            if state["health"] == "healthy":
                break
            time.sleep(1)

    return started


def _deploy_formation_config(containers: list[str], n_drones: int = 2):
    """Generate and deploy a test formation config to swarm_node containers."""
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


@pytest.fixture(scope="module")
def swarm_formation_up():
    """Ensure swarm_node_1, swarm_node_2, and their SITL deps are running."""
    started = _ensure_containers(ALL_CONTAINERS, timeout=300)
    _deploy_formation_config(SWARM_NODES, n_drones=2)
    yield
    if started:
        subprocess.run(
            COMPOSE_CMD + ["stop"] + ALL_CONTAINERS,
            capture_output=True,
        )


# ── Container health ────────────────────────────────────────────────────────


class TestFormationContainerHealth:
    """Basic health checks for swarm formation containers."""

    def test_swarm_nodes_running(self, swarm_formation_up):
        for name in SWARM_NODES:
            state = _container_state(name)
            assert state["running"], f"{name} is not running"

    def test_swarm_nodes_healthy(self, swarm_formation_up):
        for name in SWARM_NODES:
            state = _container_state(name)
            assert state["health"] == "healthy", (
                f"{name} health: {state['health']} (expected healthy)"
            )

    def test_sitl_drones_running(self, swarm_formation_up):
        for name in SITL_DRONES:
            state = _container_state(name)
            assert state["running"], f"{name} is not running"


# ── Formation config loading ────────────────────────────────────────────────


class TestFormationConfigLoad:
    """Verify swarm_nodes load the formation config."""

    def test_formation_config_deployed(self, swarm_formation_up):
        """swarm_mission.json exists in each container."""
        for name in SWARM_NODES:
            result = subprocess.run(
                ["docker", "exec", name,
                 "test", "-f", "/root/workspace/swarm_mission.json"],
                capture_output=True,
            )
            assert result.returncode == 0, (
                f"{name}: swarm_mission.json not found"
            )

    def test_formation_config_valid_json(self, swarm_formation_up):
        """swarm_mission.json is valid JSON with expected fields."""
        for name in SWARM_NODES:
            result = subprocess.run(
                ["docker", "exec", name,
                 "cat", "/root/workspace/swarm_mission.json"],
                capture_output=True, text=True,
            )
            assert result.returncode == 0
            config = json.loads(result.stdout)
            assert "pattern" in config
            assert "waypoints" in config
            assert "n_drones" in config
            assert config["n_drones"] >= 2
            assert len(config["waypoints"]) >= 2


# ── Zenoh + control loop ────────────────────────────────────────────────────


class TestFormationStartup:
    """Verify swarm_nodes connect to Zenoh and enter control loop."""

    def test_zenoh_connected(self, swarm_formation_up):
        for name in SWARM_NODES:
            combined = _wait_for_log(name, "Zenoh connected", timeout=180)
            assert combined is not None, (
                f"{name} did not report 'Zenoh connected' within timeout"
            )

    def test_control_loop_running(self, swarm_formation_up):
        for name in SWARM_NODES:
            combined = _wait_for_log(name, "Control loop running", timeout=180)
            assert combined is not None, (
                f"{name} did not enter control loop within timeout"
            )

    def test_formation_loaded(self, swarm_formation_up):
        """Nodes log that they loaded the formation config."""
        for name in SWARM_NODES:
            combined = _wait_for_log(name, "Formation loaded", timeout=180)
            assert combined is not None, (
                f"{name} did not report 'Formation loaded' within timeout "
                "(binary may still be compiling)"
            )

    def test_no_panic(self, swarm_formation_up):
        for name in SWARM_NODES:
            combined = _get_logs(name)
            assert "panicked at" not in combined, (
                f"Panic found in {name} logs"
            )

    def test_no_compilation_errors(self, swarm_formation_up):
        for name in SWARM_NODES:
            combined = _get_logs(name)
            assert "error[E" not in combined, (
                f"Rust compilation errors found in {name} logs"
            )


# ── Formation flight state machine ──────────────────────────────────────────


class TestFormationFlight:
    """Verify drones progress through formation flight states."""

    def test_enters_formation_state(self, swarm_formation_up):
        """At least one node should reach FormUp or Formation state."""
        for name in SWARM_NODES:
            # Look for periodic state log showing formation progress
            found = _wait_for_log(name, "state=FormUp", timeout=240)
            if found is None:
                found = _wait_for_log(name, "state=Formation", timeout=60)
            assert found is not None, (
                f"{name} did not reach FormUp or Formation state within timeout"
            )

    def test_reports_neighbors(self, swarm_formation_up):
        """Nodes should see at least 1 neighbor via Zenoh boid broadcast."""
        found = False
        for name in SWARM_NODES:
            combined = _wait_for_log(name, "neighbors=", timeout=240)
            if combined is None:
                continue
            # Look for neighbors=N where N > 0
            for line in combined.splitlines():
                if "neighbors=" in line:
                    # Extract neighbor count: "neighbors=1"
                    for part in line.split():
                        if part.startswith("neighbors="):
                            count_str = part.split("=")[1]
                            # May have trailing text like "wp=0/4"
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

    def test_waypoint_progress_reported(self, swarm_formation_up):
        """Nodes should log waypoint progress (wp=N/M)."""
        for name in SWARM_NODES:
            combined = _wait_for_log(name, "wp=", timeout=240)
            assert combined is not None, (
                f"{name} did not report waypoint progress within timeout"
            )


# ── Volume mounts ────────────────────────────────────────────────────────────


class TestFormationVolumes:
    """Verify required volume mounts for formation flight."""

    def test_swarm_control_volume(self, swarm_formation_up):
        for name in SWARM_NODES:
            result = subprocess.run(
                ["docker", "exec", name,
                 "test", "-f", "/root/workspace/swarm_control/Cargo.toml"],
                capture_output=True,
            )
            assert result.returncode == 0, (
                f"{name}: swarm_control volume not mounted"
            )

    def test_simulation_volume(self, swarm_formation_up):
        for name in SWARM_NODES:
            result = subprocess.run(
                ["docker", "exec", name,
                 "test", "-d", "/root/workspace/simulation"],
                capture_output=True,
            )
            assert result.returncode == 0, (
                f"{name}: simulation volume not mounted"
            )
