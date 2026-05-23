"""Tests for wheel odometry (arc model) including the 5 m straight-run accuracy."""

import math

import pytest

from autoproject.comms.sim_comms import SimRobotComms
from autoproject.localization.wheel_odometry import WheelOdometry, integrate_arc
from autoproject.simulation.geometry import Pose
from autoproject.simulation.world import World


def test_integrate_arc_straight():
    assert integrate_arc(0.0, 0.0, 0.0, 1.0, 1.0, 0.15) == pytest.approx((1.0, 0.0, 0.0))


def test_integrate_arc_turns_left():
    x, y, theta = integrate_arc(0.0, 0.0, 0.0, -0.1, 0.1, 0.2)
    assert theta == pytest.approx(1.0)  # d_theta = (0.1 - -0.1)/0.2
    assert math.hypot(x, y) < 0.11  # near-spin: little translation


def test_first_update_sets_baseline():
    odom = WheelOdometry(wheelbase_m=0.15)
    assert odom.update_counts(1000, 1000) == (0.0, 0.0, 0.0)  # no motion on the first sample


def test_five_metre_straight_run_under_5pct_error():
    world = World(
        width_m=6.5,
        height_m=2.0,
        obstacles=[],
        tags={},
        robot_pose=Pose(0.3, 1.0, 0.0),
        wheelbase_m=0.15,
        dt_s=0.1,
    )
    comms = SimRobotComms(world)  # no noise, no slip
    odom = WheelOdometry(wheelbase_m=0.15, initial_pose=(0.3, 1.0, 0.0))
    comms.move(0.2, 0.2)
    for _ in range(260):  # 26 s * 0.2 m/s ~= 5.2 m
        tel = comms.step()
        odom.update_from_telemetry(tel)

    travelled = world.pose.x - 0.3
    assert travelled > 5.0
    error = abs(odom.pose[0] - world.pose.x)
    assert error / travelled < 0.05  # < 5% positional error vs ground truth
