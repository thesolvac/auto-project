"""Line-of-sight path smoothing.

A* on an 8-connected grid produces staircase paths with many waypoints. This
greedy smoother keeps a waypoint only when the straight line to the next kept
waypoint would cross an obstacle, using a Bresenham line walk on the (inflated)
grid for the visibility test. Pure Python, no I/O.
"""

from __future__ import annotations

from autoproject.algorithms.occupancy_grid import OccupancyGrid


def line_of_sight(
    grid: OccupancyGrid, a_world: tuple[float, float], b_world: tuple[float, float]
) -> bool:
    """True if the straight segment a->b crosses only free cells (Bresenham)."""
    x0, y0 = grid.world_to_cell(*a_world)
    x1, y1 = grid.world_to_cell(*b_world)
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    err = dx - dy
    while True:
        if grid.cell_occupied(x0, y0):
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def smooth_path(grid: OccupancyGrid, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Greedily drop intermediate waypoints that are not needed for clearance.

    Keeps the first point, then repeatedly advances to the farthest subsequent
    waypoint still in line of sight, anchoring there. Preserves start and goal.
    """
    if len(path) <= 2:
        return list(path)

    smoothed = [path[0]]
    anchor = 0
    for i in range(2, len(path)):
        if not line_of_sight(grid, path[anchor], path[i]):
            smoothed.append(path[i - 1])  # last visible point becomes the new anchor
            anchor = i - 1
    smoothed.append(path[-1])
    return smoothed
