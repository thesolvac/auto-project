"""Geometry primitives for the simulation core.

Pure functions and immutable value types: 2D poses, axis-aligned rectangles,
angle wrapping, and ray/rectangle intersection (the building block for the
ultrasonic ray-caster and line-of-sight checks in ``world.py``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi

# Below this, a ray-direction component is treated as exactly axis-parallel
# (avoids division by zero in the slab intersection test).
_RAY_DIR_EPS = 1e-12


def normalize_angle(theta: float) -> float:
    """Wrap an angle to the half-open interval ``(-pi, pi]``."""
    wrapped = math.fmod(theta, TWO_PI)
    if wrapped <= -math.pi:
        wrapped += TWO_PI
    elif wrapped > math.pi:
        wrapped -= TWO_PI
    return wrapped


@dataclass(frozen=True)
class Pose:
    """A 2D pose: position ``(x, y)`` in metres and heading ``theta`` in radians."""

    x: float
    y: float
    theta: float

    def as_tuple(self) -> tuple[float, float, float]:
        """Return ``(x, y, theta)``."""
        return (self.x, self.y, self.theta)


@dataclass(frozen=True)
class Rectangle:
    """An axis-aligned rectangle, used for obstacles and the world boundary."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if ``(x, y)`` lies inside the rectangle, optionally grown by ``margin``."""
        return (
            self.x_min - margin <= x <= self.x_max + margin
            and self.y_min - margin <= y <= self.y_max + margin
        )

    def distance_to_point(self, x: float, y: float) -> float:
        """Euclidean distance from ``(x, y)`` to the rectangle (0 if inside)."""
        dx = max(self.x_min - x, 0.0, x - self.x_max)
        dy = max(self.y_min - y, 0.0, y - self.y_max)
        return math.hypot(dx, dy)


def ray_box_intersection(
    ox: float, oy: float, dx: float, dy: float, rect: Rectangle
) -> tuple[float, float] | None:
    """Intersect a ray with an axis-aligned rectangle (slab method).

    The ray starts at ``(ox, oy)`` with direction ``(dx, dy)`` (assumed unit
    length). Returns the parametric enter/exit distances ``(t_enter, t_exit)``
    along the ray, or ``None`` if the ray never crosses the rectangle. Values may
    be negative, meaning the crossing lies behind the origin — callers decide
    which sign is meaningful (e.g. an obstacle in front uses ``t_enter > 0``; the
    world boundary seen from inside uses ``t_exit > 0``).
    """
    t_enter = -math.inf
    t_exit = math.inf
    for origin, direction, lo, hi in (
        (ox, dx, rect.x_min, rect.x_max),
        (oy, dy, rect.y_min, rect.y_max),
    ):
        if abs(direction) < _RAY_DIR_EPS:
            # Ray is parallel to this slab: only crosses if the origin is within it.
            if origin < lo or origin > hi:
                return None
        else:
            t1 = (lo - origin) / direction
            t2 = (hi - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            t_enter = max(t_enter, t1)
            t_exit = min(t_exit, t2)
            if t_enter > t_exit:
                return None
    return (t_enter, t_exit)
