"""Post-hoc plotting of logged runs.

Reads a ``run.jsonl`` and produces the figures the project book needs: the pose
trace (true vs estimated), position error over time, and ultrasonic readings over
time. Deterministic given a log, so repeated runs of the same scenario yield
reproducible plots. matplotlib is imported lazily (Agg backend) so importing this
module is cheap.
"""

from __future__ import annotations

import math
from pathlib import Path

from autoproject.utils.logger import read_run


def plot_run(run_dir: str | Path, out_dir: str | Path | None = None) -> list[Path]:
    """Generate summary figures for a run; returns the written file paths."""
    run_dir = Path(run_dir)
    records = read_run(run_dir)
    out = Path(out_dir) if out_dir is not None else run_dir
    out.mkdir(parents=True, exist_ok=True)
    if not records:
        return []

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in records]
    true = [r["true"] for r in records]
    est = [r["est"] for r in records]
    paths: list[Path] = []

    # 1) Pose trace: true vs estimated path through the world.
    fig, ax = plt.subplots()
    ax.plot([p[0] for p in true], [p[1] for p in true], "b-", label="true")
    ax.plot([p[0] for p in est], [p[1] for p in est], "r:", label="estimate")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Pose trace")
    ax.legend()
    p1 = out / "pose_trace.png"
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    paths.append(p1)

    # 2) Position error (true vs estimate) over time.
    err = [
        math.hypot(tr[0] - es[0], tr[1] - es[1])
        for tr, es in zip(true, est, strict=True)
    ]
    fig, ax = plt.subplots()
    ax.plot(t, err, "m-")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|true - estimate| [m]")
    ax.set_title("Localization error over time")
    p2 = out / "error_over_time.png"
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    paths.append(p2)

    # 3) Ultrasonic readings over time (if present in the log).
    if any(r.get("dist_front") is not None for r in records):
        fig, ax = plt.subplots()
        ax.plot(t, [r.get("dist_front") for r in records], label="front")
        ax.plot(t, [r.get("dist_rear") for r in records], label="rear")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("distance [m]")
        ax.set_title("Ultrasonic readings over time")
        ax.legend()
        p3 = out / "sensors.png"
        fig.savefig(p3, dpi=120)
        plt.close(fig)
        paths.append(p3)

    return paths
