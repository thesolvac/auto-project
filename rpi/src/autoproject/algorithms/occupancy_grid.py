"""Occupancy grid: a discretized free/occupied map for planning.

Pure Python + numpy, no I/O. Obstacles are axis-aligned rectangles in world
metres; the grid stores a boolean ``occupied`` array indexed ``[iy, ix]`` (row =
y, col = x), with the origin at world ``(0, 0)`` and uniform cell ``resolution``.
"""

from __future__ import annotations

import math

import numpy as np

# (x_min, y_min, x_max, y_max) in world metres.
Rect = tuple[float, float, float, float]


class OccupancyGrid:
    """Boolean occupancy grid with world<->cell conversions and obstacle inflation."""

    def __init__(self, occupied: np.ndarray, resolution_m: float) -> None:
        if occupied.ndim != 2:
            raise ValueError("occupied must be a 2D array")
        self.occupied = occupied.astype(bool)
        self.resolution_m = resolution_m
        self.ny, self.nx = self.occupied.shape

    # --- construction ---
    @classmethod
    def from_obstacles(
        cls, obstacles: list[Rect], width_m: float, height_m: float, resolution_m: float
    ) -> OccupancyGrid:
        """Rasterize rectangular ``obstacles`` into a grid of the given extent."""
        nx = max(1, int(math.ceil(width_m / resolution_m)))
        ny = max(1, int(math.ceil(height_m / resolution_m)))
        occupied = np.zeros((ny, nx), dtype=bool)
        for x_min, y_min, x_max, y_max in obstacles:
            ix0 = max(0, int(math.floor(x_min / resolution_m)))
            ix1 = min(nx - 1, int(math.floor(x_max / resolution_m)))
            iy0 = max(0, int(math.floor(y_min / resolution_m)))
            iy1 = min(ny - 1, int(math.floor(y_max / resolution_m)))
            if ix0 <= ix1 and iy0 <= iy1:
                occupied[iy0 : iy1 + 1, ix0 : ix1 + 1] = True
        return cls(occupied, resolution_m)

    # --- world <-> cell ---
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """World metres -> ``(ix, iy)`` cell indices (no bounds clamping)."""
        return int(math.floor(x / self.resolution_m)), int(
            math.floor(y / self.resolution_m)
        )

    def cell_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        """``(ix, iy)`` cell indices -> world metres at the cell centre."""
        return (ix + 0.5) * self.resolution_m, (iy + 0.5) * self.resolution_m

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def cell_occupied(self, ix: int, iy: int) -> bool:
        """True if the cell is out of bounds or occupied."""
        if not self.in_bounds(ix, iy):
            return True
        return bool(self.occupied[iy, ix])

    def is_free(self, x: float, y: float) -> bool:
        """True if the world point lies in a free, in-bounds cell."""
        ix, iy = self.world_to_cell(x, y)
        return not self.cell_occupied(ix, iy)

    # --- operations ---
    def inflate(self, radius_m: float) -> OccupancyGrid:
        """Return a copy with obstacles dilated by ``radius_m`` (robot half-width).

        Planning on the inflated grid lets the robot be treated as a point while
        still keeping clear of obstacles by its collision radius.
        """
        r = int(math.ceil(radius_m / self.resolution_m))
        if r <= 0:
            return OccupancyGrid(self.occupied.copy(), self.resolution_m)
        offsets = [
            (dx, dy)
            for dx in range(-r, r + 1)
            for dy in range(-r, r + 1)
            if dx * dx + dy * dy <= r * r
        ]
        inflated = self.occupied.copy()
        for iy, ix in np.argwhere(self.occupied):
            for dx, dy in offsets:
                nx_, ny_ = ix + dx, iy + dy
                if 0 <= nx_ < self.nx and 0 <= ny_ < self.ny:
                    inflated[ny_, nx_] = True
        return OccupancyGrid(inflated, self.resolution_m)

    def mark_rect(self, x_min: float, y_min: float, x_max: float, y_max: float) -> None:
        """Mark the cells overlapping a world rectangle as occupied (in place).

        Used to add dynamically discovered obstacles to the planning map.
        """
        ix0 = max(0, int(math.floor(x_min / self.resolution_m)))
        ix1 = min(self.nx - 1, int(math.floor(x_max / self.resolution_m)))
        iy0 = max(0, int(math.floor(y_min / self.resolution_m)))
        iy1 = min(self.ny - 1, int(math.floor(y_max / self.resolution_m)))
        if ix0 <= ix1 and iy0 <= iy1:
            self.occupied[iy0 : iy1 + 1, ix0 : ix1 + 1] = True

    def copy(self) -> OccupancyGrid:
        """Return a deep copy (independent occupancy array)."""
        return OccupancyGrid(self.occupied.copy(), self.resolution_m)

    def to_image(self) -> np.ndarray:
        """Return a uint8 image (occupied = 255, free = 0) for visualization."""
        return (self.occupied.astype(np.uint8)) * 255
