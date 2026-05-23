"""Pure-pursuit path follower.

Given the current pose and a path, picks a look-ahead point ahead on the path and
steers toward it, converting the result to differential-drive wheel velocities.
Pure Python, no I/O. Pose is passed as ``(x, y, theta)`` so this layer stays
decoupled from the simulation/localization pose types.

Steering law (a damped pure-pursuit variant chosen for stability in tight maps):
the angular velocity is proportional to the bearing error to the look-ahead point
(clamped); the forward velocity is gated by ``cos(bearing_error)`` so the robot
slows — and ultimately pivots in place — when the target is far off its heading,
and eases off near the goal. A monotonic progress index keeps the look-ahead
point ahead of the robot so it never targets a point it has already passed.
"""

from __future__ import annotations

import math

from autoproject.simulation.geometry import normalize_angle
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
        min_speed_frac: float = 0.25,
        heading_gain: float = 2.5,
        max_omega_radps: float = 3.0,
    ) -> None:
        self.lookahead_m = lookahead_m
        self.wheelbase_m = wheelbase_m
        self.cruise_speed_mps = cruise_speed_mps
        self.goal_tolerance_m = goal_tolerance_m
        self.min_speed_frac = min_speed_frac
        self.heading_gain = heading_gain
        self.max_omega_radps = max_omega_radps
        self._progress_idx = 0  # monotonic index of the last waypoint reached

    def reset(self) -> None:
        """Reset path progress; call when a new path is assigned."""
        self._progress_idx = 0

    @classmethod
    def from_config(
        cls, lookahead_m: float = 0.3, config_path: str | None = None
    ) -> PurePursuit:
        """Build using wheelbase / cruise speed / goal tolerance from robot_params.yaml."""
        params = load_config(config_path or CONFIG_DIR / "robot_params.yaml")
        return cls(
            lookahead_m=lookahead_m,
            wheelbase_m=params["drive"]["wheelbase_m"],
            cruise_speed_mps=params["stepper"]["max_speed_mps"],
            goal_tolerance_m=params["goal_tolerance"]["position_m"],
        )

    def compute(
        self, pose: PoseTuple, path: list[tuple[float, float]]
    ) -> tuple[float, float]:
        """Return ``(v_left, v_right)`` [m/s] to follow ``path`` from ``pose``.

        Returns ``(0, 0)`` when the path is empty or the goal is within tolerance.
        """
        if not path:
            return 0.0, 0.0
        x, y, theta = pose
        gx, gy = path[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)
        if dist_to_goal < self.goal_tolerance_m:
            return 0.0, 0.0

        tx, ty = self._lookahead_point(x, y, path)
        alpha = normalize_angle(math.atan2(ty - y, tx - x) - theta)

        # Angular velocity: proportional to the bearing error, clamped.
        omega = max(
            -self.max_omega_radps, min(self.max_omega_radps, self.heading_gain * alpha)
        )

        # Forward velocity: full when aligned, zero (pure pivot) when >= 90 deg off,
        # and eased down within a look-ahead of the goal.
        align = max(0.0, math.cos(alpha))
        goal_scale = min(1.0, max(self.min_speed_frac, dist_to_goal / self.lookahead_m))
        v = self.cruise_speed_mps * align * goal_scale

        v_left = v - omega * self.wheelbase_m / 2.0
        v_right = v + omega * self.wheelbase_m / 2.0
        return v_left, v_right

    def _lookahead_point(
        self, x: float, y: float, path: list[tuple[float, float]]
    ) -> tuple[float, float]:
        # Advance the monotonic progress index past every waypoint already within a
        # look-ahead, so the target is always the next waypoint ahead (never one
        # the robot has passed). Falls back to the goal at the end of the path.
        while self._progress_idx < len(path) - 1 and (
            math.hypot(path[self._progress_idx][0] - x, path[self._progress_idx][1] - y)
            < self.lookahead_m
        ):
            self._progress_idx += 1
        return path[self._progress_idx]
