"""Differential-drive kinematics: exact arc integration over a fixed timestep.

Given the two wheel linear velocities, the body twist is::

    v     = (v_right + v_left) / 2          # linear speed [m/s]
    omega = (v_right - v_left) / wheelbase   # yaw rate    [rad/s]

For constant ``(v, omega)`` over a step ``dt`` the exact closed-form integral of
the unicycle model is the arc (constant-curvature) update::

    theta' = theta + omega * dt
    x'     = x + (v / omega) * (sin theta' - sin theta)
    y'     = y - (v / omega) * (cos theta' - cos theta)

As ``omega -> 0`` this is singular, but the limit is straight-line motion, which
we handle separately. Using the exact arc (rather than an Euler step) keeps the
ground-truth pose accurate even at large per-step rotations.
"""

from __future__ import annotations

import math

from autoproject.simulation.geometry import Pose, normalize_angle

# Below this |omega| the arc update is replaced by straight-line motion to avoid
# dividing by ~0. At 1e-9 rad/s the rotational drift over a tick is negligible.
_STRAIGHT_OMEGA_EPS = 1e-9


def diff_drive_step(
    pose: Pose, v_left: float, v_right: float, wheelbase: float, dt: float
) -> Pose:
    """Advance a differential-drive ``pose`` by one timestep ``dt``.

    ``v_left`` / ``v_right`` are wheel linear velocities [m/s]; ``wheelbase`` is
    the track width [m]. Returns the new :class:`Pose` with heading wrapped to
    ``(-pi, pi]``.
    """
    v = 0.5 * (v_right + v_left)
    omega = (v_right - v_left) / wheelbase
    x, y, theta = pose.x, pose.y, pose.theta

    if abs(omega) < _STRAIGHT_OMEGA_EPS:
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        new_theta = theta
    else:
        new_theta = theta + omega * dt
        radius = v / omega
        x += radius * (math.sin(new_theta) - math.sin(theta))
        y -= radius * (math.cos(new_theta) - math.cos(theta))

    return Pose(x, y, normalize_angle(new_theta))
