"""Pytest configuration: skip hardware tests unless explicitly requested.

Tests marked ``@pytest.mark.hardware`` need the physical robot and are skipped by
default (including in CI). Run them on hardware with ``pytest --run-hardware``.
"""

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--run-hardware",
        action="store_true",
        default=False,
        help="run @pytest.mark.hardware tests (requires the physical robot)",
    )


def pytest_collection_modifyitems(config, items):
    if config.getoption("--run-hardware"):
        return
    skip_hardware = pytest.mark.skip(reason="hardware test; pass --run-hardware to run")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip_hardware)
