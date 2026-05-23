"""Tests for autoproject.simulation.kinematics (differential-drive integration)."""

import math

import pytest

from autoproject.simulation.geometry import Pose
from autoproject.simulation.kinematics import diff_drive_step


def test_straight_line_motion():
    # Equal wheel speeds -> pure translation along the heading.
    pose = diff_drive_step(Pose(0.0, 0.0, 0.0), 1.0, 1.0, wheelbase=0.15, dt=1.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_rotate_in_place():
    # Opposite, equal wheel speeds -> spin with no translation.
    pose = diff_drive_step(Pose(0.0, 0.0, 0.0), -1.0, 1.0, wheelbase=1.0, dt=0.5)
    assert pose.x == pytest.approx(0.0, abs=1e-12)
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    assert pose.theta == pytest.approx(1.0)  # omega = 2 rad/s * 0.5 s


def test_quarter_circle_arc():
    # v = 1.0, omega = 1.0 over dt = pi/2 -> exact quarter turn of unit radius.
    pose = diff_drive_step(Pose(0.0, 0.0, 0.0), 0.5, 1.5, wheelbase=1.0, dt=math.pi / 2)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(1.0)
    assert pose.theta == pytest.approx(math.pi / 2)


def test_heading_is_wrapped():
    # Large rotation must come back wrapped into (-pi, pi].
    pose = diff_drive_step(Pose(0.0, 0.0, 3.0), -1.0, 1.0, wheelbase=1.0, dt=1.0)
    assert -math.pi < pose.theta <= math.pi
