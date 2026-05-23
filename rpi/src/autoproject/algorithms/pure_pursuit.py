"""Pure-pursuit path follower.

Given the current pose and a path, picks a look-ahead point and computes the body
twist that steers toward it, then converts that twist to differential-drive wheel
velocities. Pure Python, no I/O. Pose is passed as ``(x, y, theta)`` so this layer
stays decoupled from the simulation/localization pose types.
"""

from __future__ import annotations

import math

from autoproject.utils.config import CONFIG_DIR, load_config

PoseTuple = tuple[float, float, float]


class PurePursuit:
    """Differential-drive pure-pursuit controller."""

    def __init__(
        self,
        lookahead_m: float,
        wheelbase_m: float,
        cruise_speed_mps: float,
        goal_tolerance_m: float = 0.05,
    ) -> None:
        self.lookahead_m = lookahead_m
        self.wheelbase_m = wheelbase_m
        self.cruise_speed_mps = cruise_speed_mps
        self.goal_tolerance_m = goal_tolerance_m

    @classmethod
    def from_config(cls, lookahead_m: float = 0.3, config_path: str | None = None) -> PurePursuit:
        """Build using wheelbase / cruise speed / goal tolerance from robot_params.yaml."""
        params = load_config(config_path or CONFIG_DIR / "robot_params.yaml")
        return cls(
            lookahead_m=lookahead_m,
            wheelbase_m=params["drive"]["wheelbase_m"],
            cruise_speed_mps=params["stepper"]["max_speed_mps"],
            goal_tolerance_m=params["goal_tolerance"]["position_m"],
        )

    def compute(self, pose: PoseTuple, path: list[tuple[float, float]]) -> tuple[float, float]:
        """Return ``(v_left, v_right)`` [m/s] to follow ``path`` from ``pose``.

        Returns ``(0, 0)`` when the path is empty or the goal is within tolerance.
        """
        if not path:
            return 0.0, 0.0
        x, y, theta = pose
        goal = path[-1]
        if math.hypot(goal[0] - x, goal[1] - y) < self.goal_tolerance_m:
            return 0.0, 0.0

        tx, ty = self._lookahead_point(x, y, path)
        # Target in the robot frame (x forward, y left).
        dx = tx - x
        dy = ty - y
        local_y = -math.sin(theta) * dx + math.cos(theta) * dy
        ld_sq = dx * dx + dy * dy
        curvature = (2.0 * local_y / ld_sq) if ld_sq > 1e-9 else 0.0

        v = self.cruise_speed_mps
        omega = v * curvature
        v_left = v - omega * self.wheelbase_m / 2.0
        v_right = v + omega * self.wheelbase_m / 2.0
        return v_left, v_right

    def _lookahead_point(
        self, x: float, y: float, path: list[tuple[float, float]]
    ) -> tuple[float, float]:
        # Start from the nearest waypoint, then take the first one at least a
        # look-ahead distance away; fall back to the goal.
        nearest_i = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
        for i in range(nearest_i, len(path)):
            if math.hypot(path[i][0] - x, path[i][1] - y) >= self.lookahead_m:
                return path[i]
        return path[-1]
