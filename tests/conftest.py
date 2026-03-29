"""Shared fixtures and configuration for integration tests."""

import subprocess

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--keep-containers", action="store_true", default=False,
        help="Don't stop containers after tests",
    )


@pytest.fixture(scope="session", autouse=True)
def check_docker():
    """Verify Docker is available before running any tests."""
    result = subprocess.run(
        ["docker", "info"], capture_output=True, text=True,
    )
    if result.returncode != 0:
        pytest.skip("Docker not available")


@pytest.fixture(scope="session", autouse=True)
def check_compose_config():
    """Verify docker-compose.yml is valid."""
    result = subprocess.run(
        ["docker", "compose", "config", "--quiet"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        pytest.fail(
            f"docker-compose.yml is invalid: {result.stderr}"
        )
