"""Tests for line-of-sight smoothing."""

from autoproject.algorithms.astar import astar
from autoproject.algorithms.occupancy_grid import OccupancyGrid
from autoproject.algorithms.path_smoother import line_of_sight, smooth_path


def test_line_of_sight_clear_and_blocked():
    grid = OccupancyGrid.from_obstacles([(1.5, 0.0, 2.0, 3.0)], 4.0, 3.0, 0.25)
    assert line_of_sight(grid, (0.5, 0.5), (1.0, 0.5))  # clear
    assert not line_of_sight(grid, (0.5, 1.5), (3.5, 1.5))  # crosses the wall


def test_smoothing_reduces_waypoints_on_open_map():
    grid = OccupancyGrid.from_obstacles([], 4.0, 4.0, 0.1)
    path = astar(grid, (0.05, 0.05), (3.95, 3.95))
    assert path is not None
    smoothed = smooth_path(grid, path)
    assert len(smoothed) <= 0.6 * len(path)  # >= 40% reduction
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]


def test_short_paths_unchanged():
    grid = OccupancyGrid.from_obstacles([], 2.0, 2.0, 0.5)
    assert smooth_path(grid, [(0.0, 0.0)]) == [(0.0, 0.0)]
    two = [(0.0, 0.0), (1.0, 1.0)]
    assert smooth_path(grid, two) == two


def test_smoothed_path_stays_collision_free():
    grid = OccupancyGrid.from_obstacles([(1.5, 0.0, 2.0, 2.0)], 4.0, 3.0, 0.1)
    path = astar(grid, (0.5, 0.5), (3.5, 0.5))
    assert path is not None
    smoothed = smooth_path(grid, path)
    for i in range(len(smoothed) - 1):
        assert line_of_sight(grid, smoothed[i], smoothed[i + 1])
