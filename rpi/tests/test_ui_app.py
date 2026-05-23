"""Tests for the Flask + SocketIO UI: HTTP routes and core WebSocket events."""

import pytest

from autoproject.ui.app import create_app


@pytest.fixture
def app_and_socketio():
    return create_app()


def test_http_routes_render(app_and_socketio):
    app, _ = app_and_socketio
    client = app.test_client()
    for route in ("/", "/live", "/logs"):
        assert client.get(route).status_code == 200


def test_api_scenarios_lists_demo_room(app_and_socketio):
    app, _ = app_and_socketio
    client = app.test_client()
    resp = client.get("/api/scenarios")
    assert resp.status_code == 200
    assert "demo_room" in resp.get_json()


def test_api_runs_and_missing_run(app_and_socketio):
    app, _ = app_and_socketio
    client = app.test_client()
    assert client.get("/api/runs").status_code == 200
    assert client.get("/api/runs/does_not_exist").status_code == 404


def test_socketio_connects_and_manual_is_safe_without_session(app_and_socketio):
    app, socketio = app_and_socketio
    client = socketio.test_client(app)
    assert client.is_connected()
    # Manual before any run must not raise (no active session).
    client.emit("manual", {"left": 0.1, "right": 0.1})
    client.emit("estop")
    client.disconnect()


def test_socketio_start_emits_scene(app_and_socketio):
    app, socketio = app_and_socketio
    client = socketio.test_client(app)
    client.emit("start", {"scenario": "demo_room", "mode": "manual"})
    client.emit("stop")  # halt the background loop promptly
    events = {msg["name"] for msg in client.get_received()}
    assert "scene" in events
    client.disconnect()
