"""End-to-end simulation runner.

Runs the full navigation stack on a named scenario, logs each tick to JSONL, and
saves a trajectory plot (obstacles, planned path, true vs estimated trajectory).

Run with:  python tools/run_simulation.py demo_room [--filter ekf] [--seed 0]
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "rpi" / "src"))

from autoproject.navigation.navigator import NavState  # noqa: E402
from autoproject.navigation.sim_setup import build_sim_navigation  # noqa: E402
from autoproject.simulation.noise import NoiseConfig  # noqa: E402
from autoproject.utils.config import CONFIG_DIR  # noqa: E402

_TERMINAL = {NavState.REACHED, NavState.EMERGENCY}


def _resolve_scenario(name: str) -> Path:
    candidate = Path(name)
    if candidate.exists():
        return candidate
    return CONFIG_DIR / "sim_scenarios" / f"{name}.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario", help="scenario name (under config/sim_scenarios) or a path"
    )
    parser.add_argument("--filter", default="ekf", choices=["ekf", "complementary"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-ticks", type=int, default=8000)
    parser.add_argument("--out-dir", type=Path, default=_REPO / "runs")
    args = parser.parse_args()

    noise = NoiseConfig(
        seed=args.seed,
        encoder_sigma_rad=0.01,
        wheel_slip_prob=0.01,
        wheel_slip_factor=0.5,
    )
    nav, world = build_sim_navigation(
        _resolve_scenario(args.scenario), filter_kind=args.filter, noise=noise
    )
    nav.state = NavState.PLANNING

    run_dir = args.out_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    true_xy: list[tuple[float, float]] = []
    est_xy: list[tuple[float, float]] = []

    with (run_dir / "run.jsonl").open("w", encoding="utf-8") as log:
        for _ in range(args.max_ticks):
            state = nav.tick()
            est = nav.pose_filter.pose
            true_xy.append((world.pose.x, world.pose.y))
            est_xy.append((est[0], est[1]))
            log.write(
                json.dumps(
                    {
                        "t": round(world.time_s, 3),
                        "state": state.value,
                        "true": [
                            round(world.pose.x, 4),
                            round(world.pose.y, 4),
                            round(world.pose.theta, 4),
                        ],
                        "est": [round(est[0], 4), round(est[1], 4), round(est[2], 4)],
                    }
                )
                + "\n"
            )
            if state in _TERMINAL:
                break

    _plot(run_dir, nav, world, true_xy, est_xy)
    err = ((world.pose.x - nav.goal[0]) ** 2 + (world.pose.y - nav.goal[1]) ** 2) ** 0.5
    print(
        f"{run_dir}: state={nav.state.value} ticks={world.step_count} goal_error={err:.3f} m"
    )


def _plot(run_dir: Path, nav, world, true_xy, est_xy) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle as PltRect

    fig, ax = plt.subplots()
    for r in world.obstacles:
        ax.add_patch(
            PltRect(
                (r.x_min, r.y_min), r.x_max - r.x_min, r.y_max - r.y_min, color="0.4"
            )
        )
    if nav.path:
        px, py = zip(*nav.path, strict=True)
        ax.plot(px, py, "g--o", markersize=3, label="planned path")
    if true_xy:
        tx, ty = zip(*true_xy, strict=True)
        ax.plot(tx, ty, "b-", label="true")
    if est_xy:
        ex, ey = zip(*est_xy, strict=True)
        ax.plot(ex, ey, "r:", label="estimate")
    ax.plot(nav.goal[0], nav.goal[1], "k*", markersize=14, label="goal")
    ax.set_xlim(0, world.width_m)
    ax.set_ylim(0, world.height_m)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(run_dir / "trajectory.png", dpi=120)


if __name__ == "__main__":
    main()
