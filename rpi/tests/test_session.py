"""Tests for the UI SimSession (auto / manual stepping, snapshots)."""

from autoproject.navigation.navigator import NavState
from autoproject.ui.session import SimSession
from autoproject.utils.config import CONFIG_DIR
from autoproject.utils.logger import RunLogger

_DEMO = CONFIG_DIR / "sim_scenarios" / "demo_room.yaml"


def test_auto_session_reaches_goal():
    session = SimSession(_DEMO)
    for _ in range(6000):
        session.step()
        if session.done:
            break
    assert session.navigator.state == NavState.REACHED
    assert not session.world.collided


def test_manual_mode_drives_robot():
    session = SimSession(_DEMO)
    start_x = session.world.pose.x
    session.set_manual(0.2, 0.2)  # forward
    for _ in range(50):
        session.step()
    assert session.mode == "manual"
    assert session.world.pose.x > start_x + 0.05


def test_emergency_stop_halts():
    session = SimSession(_DEMO)
    session.set_manual(0.3, 0.3)
    for _ in range(20):
        session.step()
    session.emergency_stop()
    x_after_estop = session.world.pose.x
    for _ in range(20):
        session.step()
    assert abs(session.world.pose.x - x_after_estop) < 1e-6  # no further motion


def test_snapshot_and_scene_shape():
    session = SimSession(_DEMO)
    snap = session.step()
    assert set(snap) >= {
        "t",
        "mode",
        "state",
        "true",
        "est",
        "goal",
        "path",
        "collided",
    }
    assert len(snap["true"]) == 3
    scene = session.static_scene()
    assert scene["width"] > 0 and scene["height"] > 0
    assert len(scene["obstacles"]) == 3  # demo_room


def test_session_logs_each_step(tmp_path):
    from autoproject.utils.logger import read_run

    with RunLogger(tmp_path, run_id="s") as logger:
        session = SimSession(_DEMO, logger=logger)
        for _ in range(10):
            session.step()
    assert len(read_run(tmp_path / "s")) == 10
