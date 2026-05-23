"""A* scenarios: empty, detour, no-path, narrow corridor, U-shape, edge cases, perf."""

import time

import numpy as np
import pytest

from autoproject.algorithms.astar import astar
from autoproject.algorithms.occupancy_grid import OccupancyGrid


def _empty(width=4.0, height=3.0, res=0.25) -> OccupancyGrid:
    return OccupancyGrid.from_obstacles([], width, height, res)


def test_empty_map_has_direct_path():
    grid = _empty()
    path = astar(grid, (0.25, 0.25), (3.75, 2.75))
    assert path is not None
    assert path[0] == pytest.approx(grid.cell_to_world(*grid.world_to_cell(0.25, 0.25)))
    assert path[-1] == pytest.approx(grid.cell_to_world(*grid.world_to_cell(3.75, 2.75)))


def test_path_routes_around_obstacle():
    grid = OccupancyGrid.from_obstacles([(1.5, 0.0, 2.0, 2.0)], 4.0, 3.0, 0.25)
    path = astar(grid, (0.5, 1.0), (3.5, 1.0))
    assert path is not None
    # No waypoint should fall inside the obstacle.
    assert all(grid.is_free(x, y) for x, y in path)


def test_no_path_when_walled_off():
    # Full-height wall across the map splits start from goal.
    grid = OccupancyGrid.from_obstacles([(2.0, 0.0, 2.25, 3.0)], 4.0, 3.0, 0.25)
    assert astar(grid, (0.5, 1.5), (3.5, 1.5)) is None


def test_start_equals_goal():
    grid = _empty()
    path = astar(grid, (1.0, 1.0), (1.0, 1.0))
    assert path is not None
    assert len(path) == 1


def test_blocked_start_or_goal_returns_none():
    grid = OccupancyGrid.from_obstacles([(0.0, 0.0, 0.5, 0.5)], 4.0, 3.0, 0.25)
    assert astar(grid, (0.1, 0.1), (3.0, 2.0)) is None  # start blocked
    assert astar(grid, (3.0, 2.0), (0.1, 0.1)) is None  # goal blocked


def test_narrow_corridor_closes_after_inflation():
    # A one-cell gap in a wall: passable raw, impassable once inflated by a cell.
    occupied = np.zeros((12, 12), dtype=bool)
    occupied[:, 5] = True  # vertical wall at column 5
    occupied[6, 5] = False  # single-cell doorway
    grid = OccupancyGrid(occupied, 0.25)
    start = grid.cell_to_world(2, 6)
    goal = grid.cell_to_world(9, 6)
    assert astar(grid, start, goal) is not None  # fits through the gap
    assert astar(grid.inflate(0.25), start, goal) is None  # inflation seals it


def test_u_shape_requires_going_around():
    # U opening upward; goal inside the pocket, start outside.
    occupied = np.zeros((20, 20), dtype=bool)
    occupied[5:15, 6] = True  # left wall
    occupied[5:15, 13] = True  # right wall
    occupied[5, 6:14] = True  # bottom wall
    grid = OccupancyGrid(occupied, 0.2)
    path = astar(grid, grid.cell_to_world(10, 17), grid.cell_to_world(10, 10))
    assert path is not None
    assert all(grid.is_free(x, y) for x, y in path)


def test_diagonal_shorter_than_manhattan():
    grid = _empty(2.0, 2.0, 0.25)
    path = astar(grid, (0.125, 0.125), (1.875, 1.875))
    assert path is not None
    # An 8-connected diagonal run uses far fewer waypoints than a Manhattan route.
    assert len(path) < 12


def test_performance_40x40_under_100ms():
    rng = np.random.default_rng(0)
    occupied = rng.random((40, 40)) < 0.10  # ~10% obstacles
    occupied[0, 0] = occupied[39, 39] = False  # keep endpoints free
    grid = OccupancyGrid(occupied, 0.1)
    start = grid.cell_to_world(0, 0)
    goal = grid.cell_to_world(39, 39)
    t0 = time.perf_counter()
    astar(grid, start, goal)
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.1
