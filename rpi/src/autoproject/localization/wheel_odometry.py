"""Wheel odometry via the differential-drive arc (midpoint) model.

Derivation: over a short interval each wheel travels d_left / d_right metres
(from encoder counts). The body advances by the mean and turns by the difference
over the wheelbase::

    d_center = (d_left + d_right) / 2
    d_theta  = (d_right - d_left) / wheelbase

Integrating exactly along the resulting circular arc, the position update equals
a straight step of length ``d_center`` taken at the *midpoint* heading
``theta + d_theta/2`` (this is the second-order-accurate midpoint rule, exact for
the arc to first order in d_theta and far better than a naive Euler step at the
start heading)::

    x += d_center * cos(theta + d_theta/2)
    y += d_center * sin(theta + d_theta/2)
    theta += d_theta

Encoder counts convert to metres via the wheel circumference and counts/rev.
"""

from __future__ import annotations

import math

from autoproject.comms.interfaces import Telemetry
from autoproject.simulation.geometry import normalize_angle

DEFAULT_WHEEL_RADIUS_M = 0.040
DEFAULT_COUNTS_PER_REV = 4096


def integrate_arc(
    x: float, y: float, theta: float, d_left: float, d_right: float, wheelbase_m: float
) -> tuple[float, float, float]:
    """Advance a pose by wheel travels ``d_left``/``d_right`` (midpoint arc model)."""
    d_center = 0.5 * (d_left + d_right)
    d_theta = (d_right - d_left) / wheelbase_m
    mid_heading = theta + 0.5 * d_theta
    x += d_center * math.cos(mid_heading)
    y += d_center * math.sin(mid_heading)
    return x, y, normalize_angle(theta + d_theta)


class WheelOdometry:
    """Dead-reckoned pose from encoder telemetry (no absolute correction)."""

    def __init__(
        self,
        wheelbase_m: float,
        wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        counts_per_rev: int = DEFAULT_COUNTS_PER_REV,
        initial_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.wheelbase_m = wheelbase_m
        self.metres_per_count = (2.0 * math.pi * wheel_radius_m) / counts_per_rev
        self._x, self._y, self._theta = initial_pose
        self._prev_left: int | None = None
        self._prev_right: int | None = None

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._theta)

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self._x, self._y, self._theta = pose
        self._prev_left = None
        self._prev_right = None

    def update_counts(
        self, left_counts: int, right_counts: int
    ) -> tuple[float, float, float]:
        """Integrate the pose from absolute encoder counts (first call sets the baseline)."""
        if self._prev_left is None or self._prev_right is None:
            self._prev_left, self._prev_right = left_counts, right_counts
            return self.pose
        d_left = (left_counts - self._prev_left) * self.metres_per_count
        d_right = (right_counts - self._prev_right) * self.metres_per_count
        self._prev_left, self._prev_right = left_counts, right_counts
        self._x, self._y, self._theta = integrate_arc(
            self._x, self._y, self._theta, d_left, d_right, self.wheelbase_m
        )
        return self.pose

    def update_from_telemetry(self, telemetry: Telemetry) -> tuple[float, float, float]:
        """Convenience wrapper consuming a :class:`Telemetry` sample."""
        return self.update_counts(telemetry.enc_left_counts, telemetry.enc_right_counts)
