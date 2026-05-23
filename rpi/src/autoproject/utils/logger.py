"""Structured JSONL run logging.

Each simulated run gets its own directory ``runs/YYYYMMDD_HHMMSS/`` containing a
``run.jsonl`` (one JSON record per tick). Records are flushed immediately so a run
can be replayed or inspected even if interrupted. ``read_run`` loads a log back
for replay (Phase 6 UI) and post-hoc plotting (tools/plot_logs.py).
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

RUN_LOG_NAME = "run.jsonl"


def new_run_dir(base_dir: str | Path, run_id: str | None = None) -> Path:
    """Create and return ``base_dir/<run_id or timestamp>``."""
    run_dir = Path(base_dir) / (run_id or datetime.now().strftime("%Y%m%d_%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


class RunLogger:
    """Append-only JSONL logger for one run (use as a context manager)."""

    def __init__(self, base_dir: str | Path, run_id: str | None = None) -> None:
        self.run_dir = new_run_dir(base_dir, run_id)
        self._fh = (self.run_dir / RUN_LOG_NAME).open("w", encoding="utf-8")

    def log(self, record: dict[str, Any]) -> None:
        """Write one record as a JSON line and flush."""
        self._fh.write(json.dumps(record) + "\n")
        self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()

    def __enter__(self) -> RunLogger:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_run(path: str | Path) -> list[dict[str, Any]]:
    """Read a run log back into a list of records.

    Accepts either the run directory or the ``run.jsonl`` file directly.
    """
    path = Path(path)
    log_file = path / RUN_LOG_NAME if path.is_dir() else path
    records: list[dict[str, Any]] = []
    for line in log_file.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            records.append(json.loads(line))
    return records
