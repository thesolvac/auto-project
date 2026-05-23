"""Tests for SimRobotComms (World-backed IRobotComms)."""

import math

import pytest

from autoproject.comms.sim_comms import SimRobotComms
from autoproject.simulation.geometry import Pose, Rectangle
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World

_CIRCUMFERENCE = 2.0 * math.pi * 0.040  # default wheel radius


def _world(**kwargs) -> World:
    defaults = dict(
        width_m=4.0,
        height_m=3.0,
        obstacles=[],
        tags={},
        robot_pose=Pose(2.0, 1.5, 0.0),
        wheelbase_m=0.15,
        dt_s=0.1,
    )
    defaults.update(kwargs)
    return World(**defaults)


def test_telemetry_none_before_first_step():
    comms = SimRobotComms(_world())
    assert comms.get_telemetry() is None


def test_straight_drive_integrates_encoders():
    comms = SimRobotComms(_world(dt_s=0.1))
    comms.move(0.2, 0.2)  # 0.2 m/s each wheel
    for _ in range(10):  # 1.0 s -> 0.2 m travelled
        comms.step()
    tel = comms.get_telemetry()
    expected = (0.2 * 1.0) / _CIRCUMFERENCE * 4096  # counts
    assert tel.enc_left_counts == tel.enc_right_counts
    assert tel.enc_left_counts == pytest.approx(expected, rel=0.02)


def test_ultrasonic_matches_world_raycast():
    comms = SimRobotComms(_world())  # robot at x=2 facing +x, wall at x=4
    tel = comms.step()
    assert tel.dist_front_m == pytest.approx(2.0, abs=1e-6)
    assert tel.dist_rear_m == pytest.approx(2.0, abs=1e-6)


def test_obstacle_callback_fires_near_wall():
    world = _world(robot_pose=Pose(3.85, 1.5, 0.0))  # 0.15 m from the right wall
    comms = SimRobotComms(world)
    events: list[tuple[str, float]] = []
    comms.on_obstacle(lambda sensor, dist: events.append((sensor, dist)))
    comms.step()
    assert any(sensor == "front" for sensor, _ in events)


def test_slip_callback_fires_under_slip():
    noise = NoiseConfig(seed=3, wheel_slip_prob=1.0, wheel_slip_factor=0.0)
    comms = SimRobotComms(_world(noise=noise))
    fired = []
    comms.on_slip(lambda: fired.append(True))
    comms.move(0.2, 0.2)
    comms.step()
    assert fired  # every tick slips with prob 1.0


def test_stop_zeroes_commands():
    comms = SimRobotComms(_world())
    comms.move(0.3, 0.3)
    comms.stop()
    start_x = comms.world.pose.x
    comms.step()
    assert comms.world.pose.x == pytest.approx(start_x)  # no motion after stop


def test_obstacle_inside_room():
    obstacle = Rectangle(3.0, 1.0, 3.5, 2.0)
    world = _world(obstacles=[obstacle])
    comms = SimRobotComms(world)
    tel = comms.step()
    assert tel.dist_front_m == pytest.approx(1.0, abs=1e-6)  # obstacle face at x=3
