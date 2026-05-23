"""Tests for autoproject.simulation.world (the ground-truth simulator)."""

import math

import pytest

from autoproject.simulation.geometry import Pose, Rectangle
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.utils.config import CONFIG_DIR


def _empty_world(**kwargs) -> World:
    """A 4x3 m empty room with the robot at the centre facing +x."""
    defaults = dict(
        width_m=4.0,
        height_m=3.0,
        obstacles=[],
        tags={},
        robot_pose=Pose(2.0, 1.5, 0.0),
        wheelbase_m=0.15,
    )
    defaults.update(kwargs)
    return World(**defaults)


def test_step_advances_pose_and_time():
    world = _empty_world(dt_s=0.1)
    world.step(0.5, 0.5)  # straight, 0.5 m/s for 0.1 s -> +0.05 m in x
    assert world.pose.x == pytest.approx(2.05)
    assert world.time_s == pytest.approx(0.1)
    assert world.step_count == 1


def test_reset_restores_initial_pose():
    world = _empty_world()
    world.step(1.0, 1.0)
    world.reset()
    assert world.pose.as_tuple() == (2.0, 1.5, 0.0)
    assert world.time_s == 0.0
    assert world.step_count == 0


def test_raycast_hits_far_wall():
    world = _empty_world()  # robot at x=2 facing +x, right wall at x=4
    assert world.raycast(2.0, 1.5, 0.0) == pytest.approx(2.0)


def test_raycast_clamped_to_max_range():
    world = _empty_world()
    assert world.raycast(2.0, 1.5, 0.0, max_range=0.5) == pytest.approx(0.5)


def test_raycast_hits_obstacle_before_wall():
    obstacle = Rectangle(3.0, 1.0, 3.5, 2.0)
    world = _empty_world(obstacles=[obstacle])
    # Facing +x from x=2: obstacle face at x=3 is nearer than the wall at x=4.
    assert world.raycast(2.0, 1.5, 0.0) == pytest.approx(1.0)


def test_in_collision_bounds_and_obstacles():
    obstacle = Rectangle(1.0, 1.0, 2.0, 2.0)
    world = _empty_world(obstacles=[obstacle], collision_radius_m=0.1)
    assert world.in_collision(1.5, 1.5)  # inside the obstacle
    assert world.in_collision(0.05, 1.5)  # circle pokes through the left wall
    assert world.is_free(3.0, 0.5)  # open space


def test_step_latches_collision_when_driving_into_wall():
    world = _empty_world(
        robot_pose=Pose(3.9, 1.5, 0.0), collision_radius_m=0.05, dt_s=1.0
    )
    world.step(0.5, 0.5)  # drives past the right wall
    assert world.collided


def test_visible_tags_in_front_only():
    tags = {7: Pose(3.0, 1.5, math.pi)}  # 1 m straight ahead
    world = _empty_world(tags=tags)
    sightings = world.visible_tags(
        Pose(2.0, 1.5, 0.0), fov_rad=math.radians(60), max_range=5.0
    )
    assert len(sightings) == 1
    assert sightings[0].tag_id == 7
    assert sightings[0].range_m == pytest.approx(1.0)
    assert sightings[0].bearing_rad == pytest.approx(0.0)


def test_tag_outside_fov_not_seen():
    tags = {1: Pose(2.0, 2.5, 0.0)}  # directly to the left (90 deg bearing)
    world = _empty_world(tags=tags)
    sightings = world.visible_tags(
        Pose(2.0, 1.5, 0.0), fov_rad=math.radians(60), max_range=5.0
    )
    assert sightings == []


def test_tag_beyond_range_not_seen():
    tags = {1: Pose(3.0, 1.5, 0.0)}
    world = _empty_world(tags=tags)
    sightings = world.visible_tags(
        Pose(2.0, 1.5, 0.0), fov_rad=math.radians(90), max_range=0.5
    )
    assert sightings == []


def test_tag_occluded_by_obstacle_not_seen():
    tags = {1: Pose(3.5, 1.5, math.pi)}
    obstacle = Rectangle(2.8, 1.0, 3.0, 2.0)  # wall between camera and tag
    world = _empty_world(tags=tags, obstacles=[obstacle])
    sightings = world.visible_tags(
        Pose(2.0, 1.5, 0.0), fov_rad=math.radians(60), max_range=5.0
    )
    assert sightings == []


def test_determinism_with_same_seed():
    noise = NoiseConfig(seed=42, wheel_slip_prob=0.5, wheel_slip_factor=0.0)
    poses_a = []
    poses_b = []
    for sink, _ in ((poses_a, None), (poses_b, None)):
        world = _empty_world(noise=NoiseConfig(**noise.model_dump()), dt_s=0.1)
        for _ in range(20):
            world.step(0.5, 0.5)
            sink.append(world.pose.as_tuple())
    assert poses_a == poses_b


def test_full_slip_freezes_the_robot():
    noise = NoiseConfig(seed=1, wheel_slip_prob=1.0, wheel_slip_factor=0.0)
    world = _empty_world(noise=noise, dt_s=0.1)
    for _ in range(10):
        world.step(1.0, 1.0)
    assert world.pose.as_tuple() == (2.0, 1.5, 0.0)  # every tick fully slipped


def test_from_scenario_loads_demo_room():
    world = World.from_scenario(CONFIG_DIR / "sim_scenarios" / "demo_room.yaml")
    assert world.width_m == pytest.approx(4.0)
    assert world.height_m == pytest.approx(3.0)
    assert len(world.obstacles) == 3
    assert len(world.tags) == 4
    assert world.wheelbase_m == pytest.approx(0.15)  # from robot_params.yaml
    assert world.collision_radius_m == pytest.approx(0.125)
    assert world.pose.as_tuple() == (0.3, 0.3, 0.0)
    assert world.goal is not None
    assert not world.collided
