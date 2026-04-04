"""Kubernetes helper utilities for integration tests.

Provides kubectl wrappers that mirror the Docker helpers used in
test_integration_*.py files, enabling the same test logic to run
against a Kubernetes cluster instead of docker-compose.

Expects:
    - kubectl configured and authenticated
    - Helm release deployed in the 'swarm' namespace
"""

import json
import subprocess
import time
from typing import Optional

NAMESPACE = "swarm"
RELEASE_NAME = "swarm"
STATEFULSET_NAME = "swarm-swarm-digital-twin"

# Container names inside each drone pod (must match statefulset.yaml)
CONTAINERS = ("sitl", "swarm-node", "perception", "zenoh-bridge")


def kubectl(*args: str, capture: bool = True) -> subprocess.CompletedProcess:
    """Run a kubectl command in the swarm namespace."""
    cmd = ["kubectl", "-n", NAMESPACE] + list(args)
    return subprocess.run(cmd, capture_output=capture, text=True)


def pod_name(ordinal: int) -> str:
    """Return the pod name for a given ordinal (0-based)."""
    return f"{STATEFULSET_NAME}-{ordinal}"


def drone_id(ordinal: int) -> int:
    """Return the drone ID (1-based) for a given ordinal."""
    return ordinal + 1


def pod_status(ordinal: int) -> dict:
    """Get pod phase and per-container readiness.

    Returns:
        {
            "phase": "Running",
            "containers": {
                "sitl": {"ready": True, "state": "running"},
                "swarm-node": {"ready": False, "state": "waiting"},
                ...
            }
        }
    """
    name = pod_name(ordinal)
    result = kubectl("get", "pod", name, "-o", "json")
    if result.returncode != 0:
        return {"phase": "NotFound", "containers": {}}

    pod = json.loads(result.stdout)
    phase = pod.get("status", {}).get("phase", "Unknown")

    containers = {}
    for cs in pod.get("status", {}).get("containerStatuses", []):
        state_dict = cs.get("state", {})
        state = list(state_dict.keys())[0] if state_dict else "unknown"
        containers[cs["name"]] = {
            "ready": cs.get("ready", False),
            "state": state,
        }

    return {"phase": phase, "containers": containers}


def container_logs(ordinal: int, container: str, tail: int = 0) -> str:
    """Get logs from a specific container in a drone pod."""
    name = pod_name(ordinal)
    cmd = ["logs", name, "-c", container]
    if tail > 0:
        cmd += [f"--tail={tail}"]
    result = kubectl(*cmd)
    return result.stdout + result.stderr if result.returncode == 0 else ""


def exec_in_container(
    ordinal: int, container: str, command: list[str]
) -> subprocess.CompletedProcess:
    """Execute a command in a specific container."""
    name = pod_name(ordinal)
    return kubectl("exec", name, "-c", container, "--", *command)


def wait_for_log(
    ordinal: int,
    container: str,
    needle: str,
    timeout: int = 180,
) -> Optional[str]:
    """Poll container logs until `needle` appears or timeout."""
    for _ in range(timeout):
        logs = container_logs(ordinal, container)
        if needle in logs:
            return logs
        time.sleep(1)
    return None


def wait_for_pods_ready(count: int, timeout: int = 300) -> bool:
    """Wait until all drone pods are Ready."""
    for _ in range(timeout):
        result = kubectl(
            "get", "statefulset", STATEFULSET_NAME,
            "-o", "jsonpath={.status.readyReplicas}",
        )
        try:
            ready = int(result.stdout.strip() or "0")
        except ValueError:
            ready = 0
        if ready >= count:
            return True
        time.sleep(1)
    return False


def pod_exists(ordinal: int) -> bool:
    """Check if a pod exists."""
    result = kubectl("get", "pod", pod_name(ordinal), "--no-headers")
    return result.returncode == 0


def is_container_running(ordinal: int, container: str) -> bool:
    """Check if a specific container is running and ready."""
    status = pod_status(ordinal)
    cs = status["containers"].get(container, {})
    return cs.get("ready", False) and cs.get("state") == "running"


def helm_installed() -> bool:
    """Check if the Helm release is installed."""
    result = subprocess.run(
        ["helm", "list", "-n", NAMESPACE, "-q"],
        capture_output=True, text=True,
    )
    return RELEASE_NAME in (result.stdout or "")
