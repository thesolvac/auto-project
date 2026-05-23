"""Post-hoc analysis: produce summary figures from a logged run.

Run with:  python tools/plot_logs.py [RUN_DIR | latest]

With no argument (or ``latest``) it plots the most recent run under ``runs/``.
Figures (pose trace, error over time, sensor readings) are written into the run
directory. Implementation lives in autoproject.utils.plots so it is unit-tested.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "rpi" / "src"))

from autoproject.utils.plots import plot_run  # noqa: E402

RUNS_DIR = _REPO / "runs"


def _resolve(arg: str | None) -> Path:
    if arg and arg != "latest":
        return Path(arg)
    runs = sorted(p for p in RUNS_DIR.iterdir() if (p / "run.jsonl").exists())
    if not runs:
        raise SystemExit("no runs found under runs/")
    return runs[-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", default="latest", help="run dir or 'latest'")
    args = parser.parse_args()
    paths = plot_run(_resolve(args.run))
    for p in paths:
        print(f"wrote {p}")


if __name__ == "__main__":
    main()
