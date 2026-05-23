"""Integration tests for the navigation stack — 100% in simulation."""

import math

import pytest

from autoproject.navigation.navigator import NavState
from autoproject.navigation.sim_setup import (
    build_sim_navigation,
    build_sim_navigation_from_world,
)
from autoproject.simulation.geometry import Pose, Rectangle
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.utils.config import CONFIG_DIR

_DEMO = CONFIG_DIR / "sim_scenarios" / "demo_room.yaml"


def _goal_error(nav, world) -> float:
    return math.hypot(world.pose.x - nav.goal[0], world.pose.y - nav.goal[1])


def _low_noise(seed: int) -> NoiseConfig:
    return NoiseConfig(
        seed=seed, encoder_sigma_rad=0.01, wheel_slip_prob=0.01, wheel_slip_factor=0.5
    )


def test_reaches_goal_on_demo_room():
    nav, world = build_sim_navigation(_DEMO)
    assert nav.run(max_ticks=6000) == NavState.REACHED
    assert _goal_error(nav, world) < 0.10
    assert not world.collided


def test_ten_runs_at_least_nine_arrivals_no_collisions():
    arrivals = 0
    for seed in range(10):
        nav, world = build_sim_navigation(_DEMO, noise=_low_noise(seed))
        final = nav.run(max_ticks=8000)
        if final == NavState.REACHED and _goal_error(nav, world) < 0.10:
            arrivals += 1
        assert not world.collided  # no collisions in ANY run
    assert arrivals >= 9


def _open_world(seed: int) -> World:
    return World(
        width_m=5.0,
        height_m=2.0,
        obstacles=[],
        tags={0: Pose(4.9, 1.0, math.pi), 1: Pose(2.5, 1.95, math.pi)},
        robot_pose=Pose(0.3, 1.0, 0.0),
        wheelbase_m=0.15,
        goal=Pose(4.7, 1.0, 0.0),
        noise=_low_noise(seed),
    )


def test_dynamic_obstacle_triggers_replan_and_completion():
    arrivals = 0
    for seed in range(10):
        world = _open_world(seed)
        nav, _ = build_sim_navigation_from_world(world, 0.05)
        nav.state = NavState.PLANNING
        spawned = False
        for _ in range(8000):
            nav.tick()
            if not spawned and world.pose.x > 1.5:
                world.obstacles.append(
                    Rectangle(2.4, 0.6, 2.6, 1.4)
                )  # block the path mid-run
                spawned = True
            if nav.state in (NavState.REACHED, NavState.EMERGENCY):
                break
        if nav.state == NavState.REACHED and _goal_error(nav, world) < 0.10:
            arrivals += 1
        assert not world.collided
    assert arrivals >= 7


def test_no_path_reaches_emergency():
    # Goal walled off behind a full-height barrier.
    world = World(
        width_m=4.0,
        height_m=3.0,
        obstacles=[Rectangle(2.0, 0.0, 2.2, 3.0)],
        tags={},
        robot_pose=Pose(0.5, 1.5, 0.0),
        wheelbase_m=0.15,
        goal=Pose(3.5, 1.5, 0.0),
    )
    nav, _ = build_sim_navigation_from_world(world, 0.05)
    assert nav.run(max_ticks=2000) == NavState.EMERGENCY


def test_slip_recovery_does_not_deadlock():
    # Heavy slip forces RELOCALIZING; the timeout must keep it from deadlocking,
    # and it should still reach the goal.
    world = World(
        width_m=5.0,
        height_m=2.0,
        obstacles=[],
        tags={0: Pose(4.9, 1.0, math.pi)},
        robot_pose=Pose(0.3, 1.0, 0.0),
        wheelbase_m=0.15,
        goal=Pose(4.5, 1.0, 0.0),
        noise=NoiseConfig(seed=1, wheel_slip_prob=0.08, wheel_slip_factor=0.3),
    )
    nav, _ = build_sim_navigation_from_world(world, 0.05)
    nav.state = NavState.PLANNING
    saw_relocalizing = False
    for _ in range(10000):
        if nav.tick() == NavState.RELOCALIZING:
            saw_relocalizing = True
        if nav.state in (NavState.REACHED, NavState.EMERGENCY):
            break
    assert saw_relocalizing  # slip was detected and handled
    assert nav.state == NavState.REACHED
    assert not world.collided


@pytest.mark.parametrize(
    ("start", "goal", "obstacles"),
    [
        ((0.3, 0.3, 0.0), (3.6, 0.3, 0.0), []),  # straight across an empty room
        ((0.3, 0.3, 0.0), (0.3, 2.6, 0.0), [(0.0, 1.0, 2.0, 1.2)]),  # around a ledge
        (
            (3.6, 0.3, 0.0),
            (0.3, 2.6, 0.0),
            [(1.5, 0.0, 1.7, 2.0)],
        ),  # diagonal past a wall
    ],
)
def test_various_start_goal_maps(start, goal, obstacles):
    world = World(
        width_m=4.0,
        height_m=3.0,
        obstacles=[Rectangle(*o) for o in obstacles],
        tags={0: Pose(2.0, 2.95, math.pi)},
        robot_pose=Pose(*start),
        wheelbase_m=0.15,
        goal=Pose(*goal),
    )
    nav, _ = build_sim_navigation_from_world(world, 0.05)
    assert nav.run(max_ticks=8000) == NavState.REACHED
    assert _goal_error(nav, world) < 0.10
    assert not world.collided
