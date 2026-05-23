"""Wire up a full simulated navigation stack from a scenario file.

Builds the World, sim I/O (comms + detector), planning grid, pose filter, pure
pursuit, and the Navigator. Used by ``tools/run_simulation.py`` and the
integration tests so the wiring lives in one place.
"""

from __future__ import annotations

import math
from pathlib import Path

from autoproject.algorithms.occupancy_grid import OccupancyGrid
from autoproject.algorithms.pure_pursuit import PurePursuit
from autoproject.comms.sim_comms import SimRobotComms
from autoproject.localization.pose_fusion import make_pose_filter
from autoproject.navigation.navigator import Navigator
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.utils.config import CONFIG_DIR, load_config
from autoproject.vision.sim_detector import SimAprilTagDetector

_DETECTOR_FOV_RAD = math.radians(90.0)
_DETECTOR_MAX_RANGE_M = 10.0


def build_sim_navigation(
    scenario_path: str | Path,
    *,
    filter_kind: str = "ekf",
    noise: NoiseConfig | None = None,
    lookahead_m: float = 0.25,
) -> tuple[Navigator, World]:
    """Construct a (Navigator, World) pair for a sim scenario.

    The World is returned alongside so callers can read ground truth (for
    collision checks and error metrics in tests/tools).
    """
    scenario = load_config(scenario_path)
    world = World.from_scenario(scenario_path, noise=noise)
    resolution_m = scenario["map"]["resolution_m"]
    return build_sim_navigation_from_world(
        world, resolution_m, filter_kind=filter_kind, lookahead_m=lookahead_m
    )


def build_sim_navigation_from_world(
    world: World,
    resolution_m: float,
    *,
    filter_kind: str = "ekf",
    lookahead_m: float = 0.25,
) -> tuple[Navigator, World]:
    """Wire a Navigator around an already-constructed World (used by tests)."""
    obstacles = [(r.x_min, r.y_min, r.x_max, r.y_max) for r in world.obstacles]
    base_grid = OccupancyGrid.from_obstacles(
        obstacles, world.width_m, world.height_m, resolution_m
    )

    start = world.pose.as_tuple()
    goal = world.goal.as_tuple() if world.goal is not None else start

    comms = SimRobotComms(world)
    detector = SimAprilTagDetector(
        world, fov_rad=_DETECTOR_FOV_RAD, max_range_m=_DETECTOR_MAX_RANGE_M
    )
    tag_map = {tag_id: (p.x, p.y) for tag_id, p in world.tags.items()}
    pose_filter = make_pose_filter(filter_kind, world.wheelbase_m, initial_pose=start)

    params = load_config(CONFIG_DIR / "robot_params.yaml")
    pursuit = PurePursuit(
        lookahead_m=lookahead_m,
        wheelbase_m=world.wheelbase_m,
        cruise_speed_mps=params["stepper"]["max_speed_mps"],
        goal_tolerance_m=params["goal_tolerance"]["position_m"],
    )

    # Inflate by the collision radius plus a margin so pure-pursuit corner-cutting
    # (it tracks a look-ahead point, not the path exactly) cannot graze obstacles.
    inflate_radius_m = world.collision_radius_m + 0.10
    navigator = Navigator(
        comms,
        detector,
        base_grid,
        pose_filter,
        pursuit,
        goal=goal,
        tag_map=tag_map,
        inflate_radius_m=inflate_radius_m,
    )
    return navigator, world
