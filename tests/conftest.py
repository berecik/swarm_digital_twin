"""Shared fixtures and configuration for integration tests."""

import subprocess

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--keep-containers", action="store_true", default=False,
        help="Don't stop containers after tests",
    )
    parser.addoption(
        "--backend", default="k8s",
        choices=["docker", "k8s"],
        help="Orchestration backend (default: k8s)",
    )


def _get_backend(request) -> str:
    """Resolve the backend from the --backend option."""
    return request.config.getoption("--backend")


@pytest.fixture(scope="session")
def backend(request):
    """Return the orchestration backend: 'docker' or 'k8s'."""
    return _get_backend(request)


@pytest.fixture(scope="session", autouse=True)
def check_backend_prerequisites(request):
    """Verify prerequisites for the selected backend."""
    be = _get_backend(request)

    if be == "k8s":
        result = subprocess.run(
            ["kubectl", "cluster-info"],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            pytest.fail(
                "kubectl not configured or cluster not reachable"
            )
    else:
        result = subprocess.run(
            ["docker", "info"], capture_output=True, text=True,
        )
        if result.returncode != 0:
            pytest.fail("Docker not available")

        result = subprocess.run(
            ["docker", "compose", "config", "--quiet"],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            pytest.fail(
                f"docker-compose.yml is invalid: {result.stderr}"
            )
