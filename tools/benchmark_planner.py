"""A* planner benchmark across a battery of grid sizes and obstacle densities.

Measures solve time vs grid size for a few densities and writes a matplotlib
plot. Pure offline analysis; not run in CI.

Run with:  python tools/benchmark_planner.py [--out runs/planner_benchmark.png]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Allow running directly from the repo without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rpi" / "src"))

from autoproject.algorithms.astar import astar  # noqa: E402
from autoproject.algorithms.occupancy_grid import OccupancyGrid  # noqa: E402

GRID_SIZES = [5, 10, 20, 40, 80]
DENSITIES = [0.0, 0.10, 0.20]
TRIALS = 5


def _time_solve(size: int, density: float, seed: int) -> float:
    rng = np.random.default_rng(seed)
    occupied = rng.random((size, size)) < density
    occupied[0, 0] = occupied[size - 1, size - 1] = False
    grid = OccupancyGrid(occupied, 0.1)
    start = grid.cell_to_world(0, 0)
    goal = grid.cell_to_world(size - 1, size - 1)
    t0 = time.perf_counter()
    astar(grid, start, goal)
    return time.perf_counter() - t0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("runs/planner_benchmark.png"))
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    for density in DENSITIES:
        means = [
            float(np.mean([_time_solve(size, density, seed) for seed in range(TRIALS)])) * 1e3
            for size in GRID_SIZES
        ]
        ax.plot(GRID_SIZES, means, marker="o", label=f"{int(density * 100)}% obstacles")

    ax.set_xlabel("grid size (cells per side)")
    ax.set_ylabel("A* solve time [ms]")
    ax.set_title("A* planner benchmark")
    ax.legend()
    ax.grid(True)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
