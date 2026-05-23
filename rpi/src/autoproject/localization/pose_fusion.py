"""Pose fusion: combine wheel odometry with AprilTag landmark observations.

Two interchangeable filters (select via :func:`make_pose_filter`):

- :class:`ComplementaryFilter` — integrates odometry, then nudges the position
  estimate radially so its distance to the tag matches the measured range:
  ``pos = alpha * pos_odom + (1 - alpha) * pos_range``. Using range only (not
  bearing) keeps the correction independent of heading error; two tags seen from
  different directions triangulate the position over successive updates.
- :class:`EKFFusion` — an Extended Kalman Filter with state ``[x, y, theta]``,
  a nonlinear odometry prediction, and a nonlinear range/bearing landmark update
  (the textbook known-landmark measurement model), built on
  ``filterpy.kalman.ExtendedKalmanFilter``.

Both consume odometry increments (per-wheel metres) via :meth:`predict` and tag
sightings (known tag world position + measured range/bearing) via :meth:`update`.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

from autoproject.localization.wheel_odometry import integrate_arc
from autoproject.simulation.geometry import normalize_angle

Pose = tuple[float, float, float]


class PoseFilter(ABC):
    """Common interface for odometry+landmark pose estimators."""

    @abstractmethod
    def predict(self, d_left_m: float, d_right_m: float) -> None:
        """Advance the estimate by an odometry increment (per-wheel metres)."""

    @abstractmethod
    def update(self, tag_world_xy: tuple[float, float], range_m: float, bearing_rad: float) -> None:
        """Correct the estimate with a sighting of a tag at a known world position."""

    @property
    @abstractmethod
    def pose(self) -> Pose:
        """Current best pose estimate ``(x, y, theta)``."""


class ComplementaryFilter(PoseFilter):
    """Odometry integration with a complementary position correction from tags."""

    def __init__(
        self, wheelbase_m: float, *, alpha: float = 0.85, initial_pose: Pose = (0.0, 0.0, 0.0)
    ) -> None:
        self.wheelbase_m = wheelbase_m
        self.alpha = alpha
        self._x, self._y, self._theta = initial_pose

    def predict(self, d_left_m: float, d_right_m: float) -> None:
        self._x, self._y, self._theta = integrate_arc(
            self._x, self._y, self._theta, d_left_m, d_right_m, self.wheelbase_m
        )

    def update(self, tag_world_xy: tuple[float, float], range_m: float, bearing_rad: float) -> None:
        # Range-only radial correction (bearing is intentionally unused: it would
        # couple the position fix to the heading error). Move the estimate along
        # the tag->estimate ray so its distance to the tag equals the measurement.
        del bearing_rad
        tx, ty = tag_world_xy
        dx, dy = self._x - tx, self._y - ty
        d_est = math.hypot(dx, dy)
        if d_est < 1e-6:
            return
        target_x = tx + (dx / d_est) * range_m
        target_y = ty + (dy / d_est) * range_m
        self._x = self.alpha * self._x + (1.0 - self.alpha) * target_x
        self._y = self.alpha * self._y + (1.0 - self.alpha) * target_y

    @property
    def pose(self) -> Pose:
        return (self._x, self._y, self._theta)


class EKFFusion(PoseFilter):
    """EKF over ``[x, y, theta]`` with range/bearing landmark updates."""

    def __init__(
        self,
        wheelbase_m: float,
        *,
        process_var: float = 1e-4,
        range_var: float = 0.04,
        bearing_var: float = 0.01,
        initial_pose: Pose = (0.0, 0.0, 0.0),
        initial_cov: float = 0.1,
    ) -> None:
        self.wheelbase_m = wheelbase_m
        self._ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)
        self._ekf.x = np.array(initial_pose, dtype=float)
        self._ekf.P = np.eye(3) * initial_cov
        self._q = np.eye(3) * process_var
        self._r = np.diag([range_var, bearing_var])

    def predict(self, d_left_m: float, d_right_m: float) -> None:
        x, y, theta = self._ekf.x
        d_center = 0.5 * (d_left_m + d_right_m)
        d_theta = (d_right_m - d_left_m) / self.wheelbase_m
        mid = theta + 0.5 * d_theta
        # Jacobian of the motion model w.r.t. the state.
        f_jac = np.array(
            [
                [1.0, 0.0, -d_center * math.sin(mid)],
                [0.0, 1.0, d_center * math.cos(mid)],
                [0.0, 0.0, 1.0],
            ]
        )
        nx, ny, ntheta = integrate_arc(x, y, theta, d_left_m, d_right_m, self.wheelbase_m)
        self._ekf.x = np.array([nx, ny, ntheta])
        self._ekf.P = f_jac @ self._ekf.P @ f_jac.T + self._q

    def update(self, tag_world_xy: tuple[float, float], range_m: float, bearing_rad: float) -> None:
        tx, ty = tag_world_xy

        def hx(state: np.ndarray) -> np.ndarray:
            dx, dy = tx - state[0], ty - state[1]
            return np.array([math.hypot(dx, dy), normalize_angle(math.atan2(dy, dx) - state[2])])

        def h_jacobian(state: np.ndarray) -> np.ndarray:
            dx, dy = tx - state[0], ty - state[1]
            q = max(dx * dx + dy * dy, 1e-9)
            r = math.sqrt(q)
            return np.array([[-dx / r, -dy / r, 0.0], [dy / q, -dx / q, -1.0]])

        def residual(a: np.ndarray, b: np.ndarray) -> np.ndarray:
            diff = a - b
            diff[1] = normalize_angle(diff[1])
            return diff

        z = np.array([range_m, bearing_rad])
        self._ekf.update(z, HJacobian=h_jacobian, Hx=hx, R=self._r, residual=residual)
        self._ekf.x[2] = normalize_angle(self._ekf.x[2])

    @property
    def pose(self) -> Pose:
        return (float(self._ekf.x[0]), float(self._ekf.x[1]), float(self._ekf.x[2]))

    @property
    def covariance_trace(self) -> float:
        """Trace of the state covariance (a scalar uncertainty measure)."""
        return float(np.trace(self._ekf.P))


def make_pose_filter(
    kind: str, wheelbase_m: float, *, initial_pose: Pose = (0.0, 0.0, 0.0)
) -> PoseFilter:
    """Construct a pose filter by name: ``"ekf"`` or ``"complementary"``."""
    key = kind.lower()
    if key == "ekf":
        return EKFFusion(wheelbase_m, initial_pose=initial_pose)
    if key == "complementary":
        return ComplementaryFilter(wheelbase_m, initial_pose=initial_pose)
    raise ValueError(f"unknown pose filter kind: {kind!r} (expected 'ekf' or 'complementary')")
