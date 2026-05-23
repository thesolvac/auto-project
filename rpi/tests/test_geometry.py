"""Tests for autoproject.simulation.geometry."""

import math

import pytest

from autoproject.simulation.geometry import (
    Rectangle,
    normalize_angle,
    ray_box_intersection,
)


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),  # -pi wraps to +pi (interval is half-open)
        (3 * math.pi, math.pi),
        (1.5 * math.pi, -0.5 * math.pi),
        (-1.5 * math.pi, 0.5 * math.pi),
    ],
)
def test_normalize_angle(raw, expected):
    assert normalize_angle(raw) == pytest.approx(expected)


def test_rectangle_contains_and_margin():
    rect = Rectangle(1.0, 1.0, 2.0, 2.0)
    assert rect.contains(1.5, 1.5)
    assert not rect.contains(2.5, 1.5)
    assert rect.contains(2.4, 1.5, margin=0.5)


def test_rectangle_distance_to_point():
    rect = Rectangle(1.0, 1.0, 2.0, 2.0)
    assert rect.distance_to_point(1.5, 1.5) == 0.0  # inside
    assert rect.distance_to_point(3.0, 1.5) == pytest.approx(1.0)  # right of edge
    assert rect.distance_to_point(0.0, 0.0) == pytest.approx(
        math.hypot(1.0, 1.0)
    )  # corner


def test_ray_box_hit_in_front():
    rect = Rectangle(2.0, -1.0, 3.0, 1.0)
    result = ray_box_intersection(0.0, 0.0, 1.0, 0.0, rect)
    assert result is not None
    t_enter, t_exit = result
    assert t_enter == pytest.approx(2.0)
    assert t_exit == pytest.approx(3.0)


def test_ray_box_miss():
    rect = Rectangle(2.0, 2.0, 3.0, 3.0)
    # Ray along +x from origin never reaches a box sitting above it.
    assert ray_box_intersection(0.0, 0.0, 1.0, 0.0, rect) is None


def test_ray_box_axis_parallel_inside_slab():
    rect = Rectangle(-1.0, 2.0, 1.0, 3.0)
    # Vertical ray at x=0 (within the box's x-slab) should hit.
    result = ray_box_intersection(0.0, 0.0, 0.0, 1.0, rect)
    assert result is not None
    assert result[0] == pytest.approx(2.0)


def test_ray_box_from_inside_gives_negative_enter():
    rect = Rectangle(-1.0, -1.0, 1.0, 1.0)
    result = ray_box_intersection(0.0, 0.0, 1.0, 0.0, rect)
    assert result is not None
    t_enter, t_exit = result
    assert t_enter < 0.0 < t_exit
    assert t_exit == pytest.approx(1.0)
