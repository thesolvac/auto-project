"""Tests for the component factory (sim wiring; real selection)."""

import pytest

from autoproject.comms.interfaces import IRobotComms
from autoproject.comms.sim_comms import SimRobotComms
from autoproject.factory import (
    RuntimeMode,
    build_camera,
    build_detector,
    build_robot_comms,
    load_runtime,
    resolve_mode,
)
from autoproject.simulation.geometry import Pose
from autoproject.simulation.world import World
from autoproject.vision.interfaces import IAprilTagDetector, ICamera


def _world() -> World:
    return World(
        width_m=4.0,
        height_m=3.0,
        obstacles=[],
        tags={},
        robot_pose=Pose(2.0, 1.5, 0.0),
        wheelbase_m=0.15,
    )


def test_load_runtime_defaults_to_sim():
    runtime = load_runtime()
    assert runtime["mode"] == "sim"


def test_resolve_mode_global_and_override():
    assert resolve_mode({"mode": "sim", "components": {}}, "comms") is RuntimeMode.SIM
    assert (
        resolve_mode({"mode": "sim", "components": {"comms": "real"}}, "comms") is RuntimeMode.REAL
    )
    assert resolve_mode({"mode": "real", "components": None}, "camera") is RuntimeMode.REAL


def test_build_sim_components():
    runtime = {"mode": "sim", "components": {}}
    world = _world()
    comms = build_robot_comms(runtime, world=world)
    camera = build_camera(runtime, world=world)
    detector = build_detector(runtime, world=world)
    assert isinstance(comms, SimRobotComms)
    assert isinstance(comms, IRobotComms)
    assert isinstance(camera, ICamera)
    assert isinstance(detector, IAprilTagDetector)


def test_sim_mode_requires_world():
    with pytest.raises(ValueError, match="requires a World"):
        build_robot_comms({"mode": "sim", "components": {}})


def test_real_components_import_cleanly():
    # Validates the Real* modules import without the hardware attached. Skipped
    # locally if the hardware libs aren't installed; runs in CI (they are).
    pytest.importorskip("serial")
    pytest.importorskip("cv2")
    pytest.importorskip("pupil_apriltags")
    runtime = {"mode": "real", "components": {}}
    assert isinstance(build_robot_comms(runtime), IRobotComms)
    assert isinstance(build_camera(runtime), ICamera)
    assert isinstance(build_detector(runtime), IAprilTagDetector)
