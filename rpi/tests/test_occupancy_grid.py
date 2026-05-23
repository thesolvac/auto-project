"""Tests for the occupancy grid."""

import numpy as np
import pytest

from autoproject.algorithms.occupancy_grid import OccupancyGrid


def test_from_obstacles_marks_cells():
    grid = OccupancyGrid.from_obstacles([(1.0, 1.0, 2.0, 2.0)], 4.0, 3.0, 0.5)
    assert grid.nx == 8
    assert grid.ny == 6
    assert grid.is_free(0.25, 0.25)
    assert not grid.is_free(1.5, 1.5)


def test_world_cell_roundtrip():
    grid = OccupancyGrid.from_obstacles([], 4.0, 3.0, 0.5)
    assert grid.world_to_cell(0.6, 0.6) == (1, 1)
    assert grid.cell_to_world(1, 1) == pytest.approx((0.75, 0.75))


def test_out_of_bounds_counts_as_occupied():
    grid = OccupancyGrid.from_obstacles([], 4.0, 3.0, 0.5)
    assert grid.cell_occupied(-1, 0)
    assert grid.cell_occupied(0, 100)
    assert not grid.is_free(-0.5, 0.5)  # outside the world


def test_inflate_grows_obstacles():
    occupied = np.zeros((5, 5), dtype=bool)
    occupied[2, 2] = True
    grid = OccupancyGrid(occupied, 1.0)
    inflated = grid.inflate(1.0)
    assert inflated.occupied[2, 3]  # orthogonal neighbour now occupied
    assert inflated.occupied[3, 2]
    assert not inflated.occupied[1, 1]  # diagonal beyond radius stays free


def test_to_image():
    # One occupied cell out of four -> image has both 0 and 255.
    grid = OccupancyGrid.from_obstacles([(0.0, 0.0, 0.5, 0.5)], 2.0, 2.0, 1.0)
    img = grid.to_image()
    assert img.dtype == np.uint8
    assert img.max() == 255
    assert img.min() == 0
