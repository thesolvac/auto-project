"""Tests for SimCamera and SimAprilTagDetector (ground-truth-backed vision)."""

import math

import numpy as np
import pytest

from autoproject.simulation.geometry import Pose
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.vision.sim_camera import SimCamera
from autoproject.vision.sim_detector import SimAprilTagDetector


def _world_with_tag_ahead(**kwargs) -> World:
    defaults = dict(
        width_m=4.0,
        height_m=3.0,
        obstacles=[],
        tags={5: Pose(3.0, 1.5, math.pi)},  # 1 m straight ahead of the robot
        robot_pose=Pose(2.0, 1.5, 0.0),
        wheelbase_m=0.15,
    )
    defaults.update(kwargs)
    return World(**defaults)


def test_detector_matches_ground_truth():
    detector = SimAprilTagDetector(_world_with_tag_ahead(), max_range_m=5.0)
    detections = detector.detect()
    assert len(detections) == 1
    det = detections[0]
    assert det.tag_id == 5
    assert det.range_m == pytest.approx(1.0, abs=1e-6)
    assert det.bearing_rad == pytest.approx(0.0, abs=1e-6)


def test_detector_empty_when_facing_away():
    world = _world_with_tag_ahead(robot_pose=Pose(2.0, 1.5, math.pi))  # tag behind
    detector = SimAprilTagDetector(world, fov_rad=math.radians(60), max_range_m=5.0)
    assert detector.detect() == []


def test_detector_noise_is_seeded_and_bounded():
    noise_world = _world_with_tag_ahead(noise=NoiseConfig(seed=7))
    detector = SimAprilTagDetector(noise_world, range_sigma_m=0.05, bearing_sigma_rad=0.02)
    d1 = detector.detect()[0]
    # Re-seeded detector on an identical world reproduces the same noisy reading.
    detector2 = SimAprilTagDetector(
        _world_with_tag_ahead(noise=NoiseConfig(seed=7)),
        range_sigma_m=0.05,
        bearing_sigma_rad=0.02,
    )
    d2 = detector2.detect()[0]
    assert d1.range_m == pytest.approx(d2.range_m)
    assert d1.range_m == pytest.approx(1.0, abs=0.3)  # within a few sigma of truth


def test_camera_frame_shape_and_dtype():
    camera = SimCamera(_world_with_tag_ahead(), width=320, height=240)
    frame = camera.get_frame()
    assert frame.shape == (240, 320, 3)
    assert frame.dtype == np.uint8


def test_camera_draws_visible_tag():
    camera = SimCamera(_world_with_tag_ahead(), width=320, height=240, fov_rad=math.radians(90))
    frame = camera.get_frame()
    assert frame.max() == 255  # the tag square is rendered white

    blank_world = _world_with_tag_ahead(robot_pose=Pose(2.0, 1.5, math.pi))  # tag behind
    assert SimCamera(blank_world, width=320, height=240).get_frame().max() < 255
