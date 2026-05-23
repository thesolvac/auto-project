"""Flask + SocketIO web UI for the simulated robot.

Routes:
    /            scenario selection + mode (auto / manual)
    /live        live SVG visualization of a running simulation
    /logs        list and replay past runs
    /api/...     JSON endpoints (scenarios, run list, run replay)

WebSocket events drive a single active :class:`SimSession`: ``start`` launches a
background stepping loop that emits ``state`` snapshots (<500 ms latency on
localhost); ``manual`` / ``estop`` / ``auto`` control the robot; replay reads logs.

Run with:  python -m autoproject.ui.app   (then open http://localhost:5000)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO

from autoproject.simulation.noise import NoiseConfig
from autoproject.ui.session import SimSession
from autoproject.utils.config import CONFIG_DIR
from autoproject.utils.logger import RunLogger, read_run

SCENARIO_DIR = CONFIG_DIR / "sim_scenarios"
RUNS_DIR = CONFIG_DIR.parent / "runs"
_DEFAULT_NOISE = NoiseConfig(
    seed=0, encoder_sigma_rad=0.01, wheel_slip_prob=0.01, wheel_slip_factor=0.5
)


@dataclass
class _AppState:
    session: SimSession | None = None
    logger: RunLogger | None = None
    running: bool = False
    lock: Any = field(default=None)


def list_scenarios() -> list[str]:
    return sorted(p.stem for p in SCENARIO_DIR.glob("*.yaml"))


def list_runs() -> list[str]:
    if not RUNS_DIR.exists():
        return []
    return sorted(
        (p.name for p in RUNS_DIR.iterdir() if (p / "run.jsonl").exists()), reverse=True
    )


def create_app() -> tuple[Flask, SocketIO]:
    """Application factory: returns the Flask app and its SocketIO server."""
    app = Flask(__name__)
    app.config["SECRET_KEY"] = "autoproject-sim"  # noqa: S105  (local dev UI only)
    socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")
    state = _AppState()

    # --- HTTP routes ---
    @app.route("/")
    def index() -> str:
        return render_template("index.html", scenarios=list_scenarios())

    @app.route("/live")
    def live() -> str:
        return render_template("live.html")

    @app.route("/logs")
    def logs() -> str:
        return render_template("logs.html", runs=list_runs())

    @app.route("/api/scenarios")
    def api_scenarios() -> Any:
        return jsonify(list_scenarios())

    @app.route("/api/runs")
    def api_runs() -> Any:
        return jsonify(list_runs())

    @app.route("/api/runs/<run_id>")
    def api_run(run_id: str) -> Any:
        run_dir = RUNS_DIR / Path(run_id).name  # guard against path traversal
        if not (run_dir / "run.jsonl").exists():
            return jsonify({"error": "not found"}), 404
        return jsonify(read_run(run_dir))

    # --- WebSocket events ---
    def _run_loop() -> None:
        assert state.session is not None
        # Emit roughly every 5 physics ticks (~10 Hz at 50 Hz sim) to bound traffic.
        tick = 0
        while state.running and not state.session.done:
            snapshot = state.session.step()
            if tick % 5 == 0 or state.session.done:
                socketio.emit("state", snapshot)
            tick += 1
            socketio.sleep(state.session.world.dt_s)
        socketio.emit("state", state.session.snapshot())
        socketio.emit("finished", {"state": state.session.navigator.state.value})
        if state.logger is not None:
            state.logger.close()
            state.logger = None
        state.running = False

    @socketio.on("start")
    def on_start(data: dict[str, Any]) -> None:
        scenario = Path(data.get("scenario", "demo_room")).name
        state.running = False  # stop any prior loop
        logger = RunLogger(RUNS_DIR)
        session = SimSession(
            SCENARIO_DIR / f"{scenario}.yaml", noise=_DEFAULT_NOISE, logger=logger
        )
        if data.get("mode") == "manual":
            session.set_manual(0.0, 0.0)
        state.session = session
        state.logger = logger
        state.running = True
        socketio.emit("scene", session.static_scene())
        socketio.start_background_task(_run_loop)

    @socketio.on("manual")
    def on_manual(data: dict[str, Any]) -> None:
        if state.session is not None:
            state.session.set_manual(
                float(data.get("left", 0.0)), float(data.get("right", 0.0))
            )

    @socketio.on("estop")
    def on_estop() -> None:
        if state.session is not None:
            state.session.emergency_stop()

    @socketio.on("auto")
    def on_auto() -> None:
        if state.session is not None:
            state.session.resume_auto()

    @socketio.on("stop")
    def on_stop() -> None:
        state.running = False

    app.extensions["sim_state"] = state  # expose for tests
    return app, socketio


def main() -> None:
    app, socketio = create_app()
    socketio.run(app, host="0.0.0.0", port=5000)  # noqa: S104  (local dev)


if __name__ == "__main__":
    main()
