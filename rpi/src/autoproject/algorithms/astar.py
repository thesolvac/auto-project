"""A* grid search with 8-connectivity.

Pure Python (heapq open set), no I/O. Returns a path in world coordinates (cell
centres) or ``None`` if the goal is unreachable.

Admissibility: step costs are the true Euclidean cell-to-cell distances
(``resolution`` orthogonally, ``sqrt(2)*resolution`` diagonally). The heuristic is
the straight-line Euclidean distance to the goal, which can never exceed the cost
of any actual path (the shortest connection between two points is the straight
line). An admissible (and here consistent) heuristic makes A* optimal.
"""

from __future__ import annotations

import heapq
import math

from autoproject.algorithms.occupancy_grid import OccupancyGrid

# (dx, dy, step_cost_factor) for the 8 neighbours.
_NEIGHBOURS = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


def _heuristic(ix: int, iy: int, gx: int, gy: int) -> float:
    return math.hypot(gx - ix, gy - iy)


def _nearest_free_cell(
    grid: OccupancyGrid, ix: int, iy: int, max_radius: int = 12
) -> tuple[int, int] | None:
    """Nearest free, in-bounds cell to ``(ix, iy)`` by expanding-ring search.

    Lets planning recover when the robot sits in the inflation margin (close to,
    but not colliding with, an obstacle): it is treated as starting from the
    nearest truly-free cell.
    """
    if not grid.cell_occupied(ix, iy):
        return ix, iy
    for radius in range(1, max_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue  # only the ring at this radius
                if not grid.cell_occupied(ix + dx, iy + dy):
                    return ix + dx, iy + dy
    return None


def astar(
    grid: OccupancyGrid,
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """Plan a path from ``start_world`` to ``goal_world`` (both in metres).

    Returns the list of world-coordinate waypoints (cell centres) including start
    and goal cells, or ``None`` if either endpoint is blocked or no path exists.
    """
    goal = grid.world_to_cell(*goal_world)
    if grid.cell_occupied(*goal):
        return None
    # Snap a blocked start to the nearest free cell (robot in the inflation margin).
    snapped_start = _nearest_free_cell(grid, *grid.world_to_cell(*start_world))
    if snapped_start is None:
        return None
    start = snapped_start

    gx, gy = goal
    open_heap: list[tuple[float, int, int]] = [(0.0, start[0], start[1])]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _, ix, iy = heapq.heappop(open_heap)
        current = (ix, iy)
        if current == goal:
            return _reconstruct(grid, came_from, current)
        if current in closed:
            continue
        closed.add(current)

        for dx, dy, cost in _NEIGHBOURS:
            nx_, ny_ = ix + dx, iy + dy
            neighbour = (nx_, ny_)
            if grid.cell_occupied(nx_, ny_) or neighbour in closed:
                continue
            tentative = g_score[current] + cost * grid.resolution_m
            if tentative < g_score.get(neighbour, math.inf):
                came_from[neighbour] = current
                g_score[neighbour] = tentative
                f = tentative + _heuristic(nx_, ny_, gx, gy) * grid.resolution_m
                heapq.heappush(open_heap, (f, nx_, ny_))

    return None


def _reconstruct(
    grid: OccupancyGrid,
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[float, float]]:
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return [grid.cell_to_world(ix, iy) for ix, iy in cells]
