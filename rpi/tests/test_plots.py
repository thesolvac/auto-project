"""Test that summary plots are produced from a logged run."""

from autoproject.ui.session import SimSession
from autoproject.utils.config import CONFIG_DIR
from autoproject.utils.logger import RunLogger
from autoproject.utils.plots import plot_run

_DEMO = CONFIG_DIR / "sim_scenarios" / "demo_room.yaml"


def test_plot_run_produces_figures(tmp_path):
    with RunLogger(tmp_path, run_id="run") as logger:
        session = SimSession(_DEMO, logger=logger)
        for _ in range(60):
            session.step()
    paths = plot_run(tmp_path / "run")
    assert len(paths) >= 2  # pose trace + error + (sensors)
    for p in paths:
        assert p.exists()
        assert p.suffix == ".png"


def test_plot_run_empty_log_returns_nothing(tmp_path):
    (tmp_path / "empty").mkdir()
    (tmp_path / "empty" / "run.jsonl").write_text("", encoding="utf-8")
    assert plot_run(tmp_path / "empty") == []
