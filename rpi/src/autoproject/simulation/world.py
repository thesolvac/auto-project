"""Layer 0 — the simulation core.

:class:`World` is the honest oracle of the system: it holds ground truth (true
robot pose, obstacle map, AprilTag placements, simulated time) and advances the
robot by differential-drive physics. ``Sim*`` components (Phase 2) query it to
synthesize what real hardware would report; tests assert that the system's
*estimated* state converges to the World's *ground truth* under modeled noise.

The World never imports hardware libraries and uses only the standard library
plus a seeded :class:`random.Random` for reproducibility.
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass
from pathlib import Path

from autoproject.simulation.geometry import (
    Pose,
    Rectangle,
    normalize_angle,
    ray_box_intersection,
)
from autoproject.simulation.kinematics import diff_drive_step
from autoproject.simulation.noise import NoiseConfig
from autoproject.utils.config import CONFIG_DIR, load_config

logger = logging.getLogger(__name__)

DEFAULT_DT_S = 0.02  # 50 Hz physics tick (CLAUDE.md simulation spec)
DEFAULT_ULTRASONIC_MAX_M = 4.0  # HC-SR04 practical max range [m]
_HIT_EPS = 1e-9  # ignore intersections at/behind the ray origin


@dataclass(frozen=True)
class TagSighting:
    """A geometric AprilTag observation from the camera's viewpoint."""

    tag_id: int
    range_m: float
    bearing_rad: float  # angle to the tag relative to camera heading (+left)


class World:
    """Ground-truth physics simulator for the robot and its environment."""

    def __init__(
        self,
        *,
        width_m: float,
        height_m: float,
        obstacles: list[Rectangle],
        tags: dict[int, Pose],
        robot_pose: Pose,
        wheelbase_m: float,
        collision_radius_m: float = 0.0,
        dt_s: float = DEFAULT_DT_S,
        noise: NoiseConfig | None = None,
        goal: Pose | None = None,
    ) -> None:
        self.width_m = width_m
        self.height_m = height_m
        self.obstacles = list(obstacles)
        self.tags = dict(tags)
        self.wheelbase_m = wheelbase_m
        self.collision_radius_m = collision_radius_m
        self.dt_s = dt_s
        self.noise = noise or NoiseConfig()
        self.goal = goal

        self._initial_pose = robot_pose
        self._pose = robot_pose
        self._time_s = 0.0
        self._step_count = 0
        self.collided = False
        self.last_step_slipped = False
        self._rng = random.Random(self.noise.seed)

    # ------------------------------------------------------------------ #
    # Ground-truth state (read-only views)
    # ------------------------------------------------------------------ #
    @property
    def pose(self) -> Pose:
        """Current ground-truth robot pose."""
        return self._pose

    @property
    def time_s(self) -> float:
        """Elapsed simulated time [s]."""
        return self._time_s

    @property
    def step_count(self) -> int:
        """Number of physics ticks advanced since construction/reset."""
        return self._step_count

    def reset(self, robot_pose: Pose | None = None) -> None:
        """Reset pose, time, and collision flag (keeps map, tags, and RNG state)."""
        self._pose = robot_pose if robot_pose is not None else self._initial_pose
        self._time_s = 0.0
        self._step_count = 0
        self.collided = False
        self.last_step_slipped = False

    # ------------------------------------------------------------------ #
    # Physics
    # ------------------------------------------------------------------ #
    def _apply_slip(self, wheel_velocity: float) -> tuple[float, bool]:
        """Randomly reduce a wheel's realized velocity to model slip.

        Returns the (possibly reduced) velocity and whether a slip occurred.
        """
        if self.noise.wheel_slip_prob > 0.0 and self._rng.random() < self.noise.wheel_slip_prob:
            return wheel_velocity * self.noise.wheel_slip_factor, True
        return wheel_velocity, False

    def step(self, v_left: float, v_right: float) -> Pose:
        """Advance ground truth by one tick given commanded wheel velocities [m/s].

        Wheel slip (if configured) is applied to the realized velocities, so the
        true pose diverges from what perfect odometry would predict. Latches
        :attr:`collided` if the robot's footprint leaves bounds or hits an
        obstacle. Returns the new pose.
        """
        realized_left, slipped_left = self._apply_slip(v_left)
        realized_right, slipped_right = self._apply_slip(v_right)
        self.last_step_slipped = slipped_left or slipped_right
        self._pose = diff_drive_step(
            self._pose, realized_left, realized_right, self.wheelbase_m, self.dt_s
        )
        self._time_s += self.dt_s
        self._step_count += 1
        if self.in_collision(self._pose.x, self._pose.y):
            self.collided = True
        return self._pose

    # ------------------------------------------------------------------ #
    # Spatial queries
    # ------------------------------------------------------------------ #
    def in_collision(self, x: float, y: float) -> bool:
        """True if the robot's collision circle leaves bounds or hits an obstacle."""
        r = self.collision_radius_m
        if x - r < 0.0 or x + r > self.width_m or y - r < 0.0 or y + r > self.height_m:
            return True
        return any(rect.distance_to_point(x, y) < r for rect in self.obstacles)

    def is_free(self, x: float, y: float) -> bool:
        """Negation of :meth:`in_collision`."""
        return not self.in_collision(x, y)

    def raycast(
        self, x: float, y: float, angle: float, max_range: float = DEFAULT_ULTRASONIC_MAX_M
    ) -> float:
        """Distance from ``(x, y)`` along ``angle`` to the nearest obstacle or wall.

        Clamped to ``max_range``. Used to synthesize ultrasonic returns and for
        line-of-sight checks. No sensor noise is applied here.
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        nearest = max_range

        # Distance to the surrounding wall: exit point of the world box (origin inside).
        bounds = ray_box_intersection(
            x, y, dx, dy, Rectangle(0.0, 0.0, self.width_m, self.height_m)
        )
        if bounds is not None and bounds[1] > _HIT_EPS:
            nearest = min(nearest, bounds[1])

        # Nearest obstacle entry in front of the origin.
        for rect in self.obstacles:
            hit = ray_box_intersection(x, y, dx, dy, rect)
            if hit is not None and hit[0] > _HIT_EPS:
                nearest = min(nearest, hit[0])

        return nearest

    def ultrasonic_reading(
        self, sensor_pose: Pose, max_range: float = DEFAULT_ULTRASONIC_MAX_M
    ) -> float:
        """Noise-free ultrasonic distance for a sensor at ``sensor_pose``."""
        return self.raycast(sensor_pose.x, sensor_pose.y, sensor_pose.theta, max_range)

    def visible_tags(
        self, camera_pose: Pose, fov_rad: float, max_range: float
    ) -> list[TagSighting]:
        """AprilTags visible from ``camera_pose``: within range, FOV, and line of sight.

        Returns sightings sorted by tag id. A tag is occluded if an obstacle lies
        between the camera and the tag (checked via :meth:`raycast`).
        """
        sightings: list[TagSighting] = []
        half_fov = 0.5 * fov_rad
        for tag_id, tag_pose in sorted(self.tags.items()):
            dx = tag_pose.x - camera_pose.x
            dy = tag_pose.y - camera_pose.y
            range_m = math.hypot(dx, dy)
            if range_m < _HIT_EPS or range_m > max_range:
                continue
            world_bearing = math.atan2(dy, dx)
            bearing = normalize_angle(world_bearing - camera_pose.theta)
            if abs(bearing) > half_fov:
                continue
            # Occlusion: nearest surface along the ray must be at least as far as the tag.
            if (
                self.raycast(camera_pose.x, camera_pose.y, world_bearing, max_range)
                < range_m - 1e-3
            ):
                continue
            sightings.append(TagSighting(tag_id=tag_id, range_m=range_m, bearing_rad=bearing))
        return sightings

    # ------------------------------------------------------------------ #
    # Construction from config
    # ------------------------------------------------------------------ #
    @classmethod
    def from_scenario(
        cls,
        scenario_path: str | Path,
        *,
        robot_params_path: str | Path | None = None,
        noise: NoiseConfig | None = None,
    ) -> World:
        """Build a World from a ``config/sim_scenarios/*.yaml`` scenario file.

        Pulls the map, obstacles, start/goal from the scenario; the AprilTag
        placements from the scenario's ``tags_file`` (default
        ``config/world_tags.yaml``); and the wheelbase and collision radius from
        ``config/robot_params.yaml``.
        """
        scenario_path = Path(scenario_path)
        scenario = load_config(scenario_path)

        grid = scenario["map"]
        obstacles = [Rectangle(*rect) for rect in grid.get("obstacles", [])]

        tags_ref = scenario.get("tags_file")
        tags_path = (
            (scenario_path.parent / tags_ref).resolve()
            if tags_ref
            else CONFIG_DIR / "world_tags.yaml"
        )
        tag_cfg = load_config(tags_path)
        tags = {
            int(tag_id): Pose(t["x"], t["y"], t["theta"])
            for tag_id, t in tag_cfg.get("tags", {}).items()
        }

        params = load_config(robot_params_path or CONFIG_DIR / "robot_params.yaml")
        wheelbase_m = params["drive"]["wheelbase_m"]
        collision_radius_m = params.get("footprint", {}).get("collision_radius_m", 0.0)

        start = scenario["robot"]["start"]
        robot_pose = Pose(start["x"], start["y"], start["theta"])
        goal_cfg = scenario["robot"].get("goal")
        goal = Pose(goal_cfg["x"], goal_cfg["y"], goal_cfg["theta"]) if goal_cfg else None

        logger.info(
            "Loaded scenario '%s': %.1fx%.1f m, %d obstacles, %d tags",
            scenario.get("name", scenario_path.stem),
            grid["width_m"],
            grid["height_m"],
            len(obstacles),
            len(tags),
        )
        return cls(
            width_m=grid["width_m"],
            height_m=grid["height_m"],
            obstacles=obstacles,
            tags=tags,
            robot_pose=robot_pose,
            wheelbase_m=wheelbase_m,
            collision_radius_m=collision_radius_m,
            noise=noise,
            goal=goal,
        )
