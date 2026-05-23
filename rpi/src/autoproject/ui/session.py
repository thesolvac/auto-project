"""Simulation session: a UI-facing wrapper around the navigation stack.

Drives the simulated robot one tick at a time in either ``auto`` mode (the
Navigator runs the state machine) or ``manual`` mode (wheel velocities come from
the operator), produces JSON-serializable snapshots for the web UI, and logs each
tick. Pure logic — no Flask — so it is unit-testable on its own.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from autoproject.navigation.navigator import NavState
from autoproject.navigation.sim_setup import build_sim_navigation
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.utils.logger import RunLogger

_TERMINAL = {NavState.REACHED, NavState.EMERGENCY}


class SimSession:
    """Steppable simulation session backing the live UI and manual control."""

    def __init__(
        self,
        scenario_path: str | Path,
        *,
        filter_kind: str = "ekf",
        noise: NoiseConfig | None = None,
        logger: RunLogger | None = None,
    ) -> None:
        self.navigator, self.world = build_sim_navigation(
            scenario_path, filter_kind=filter_kind, noise=noise
        )
        self.navigator.state = NavState.PLANNING
        self.mode = "auto"
        self.logger = logger
        self._manual_left = 0.0
        self._manual_right = 0.0

    # --- control ---
    def set_manual(self, left_mps: float, right_mps: float) -> None:
        """Switch to manual mode and command wheel velocities."""
        self.mode = "manual"
        self._manual_left = left_mps
        self._manual_right = right_mps

    def emergency_stop(self) -> None:
        """Halt immediately and stay stopped (manual mode, zero velocity)."""
        self.mode = "manual"
        self._manual_left = 0.0
        self._manual_right = 0.0
        self.navigator.comms.stop()

    def resume_auto(self) -> None:
        """Hand control back to the navigator (replans from the current pose)."""
        self.mode = "auto"
        if self.navigator.state not in _TERMINAL:
            self.navigator.state = NavState.PLANNING

    # --- stepping ---
    def step(self) -> dict[str, Any]:
        """Advance one tick and return (and log) a snapshot."""
        if self.mode == "manual":
            self.navigator.comms.move(self._manual_left, self._manual_right)
            self.navigator.comms.step()
        else:
            self.navigator.tick()
        snapshot = self.snapshot()
        if self.logger is not None:
            self.logger.log(snapshot)
        return snapshot

    @property
    def done(self) -> bool:
        return self.mode == "auto" and self.navigator.state in _TERMINAL

    def snapshot(self) -> dict[str, Any]:
        """JSON-serializable view of the current state for the UI / log."""
        true_pose = self.world.pose
        est = self.navigator.pose_filter.pose
        tel = self.navigator.comms.get_telemetry()
        return {
            "t": round(self.world.time_s, 3),
            "mode": self.mode,
            "state": self.navigator.state.value,
            "true": [
                round(true_pose.x, 4),
                round(true_pose.y, 4),
                round(true_pose.theta, 4),
            ],
            "est": [round(est[0], 4), round(est[1], 4), round(est[2], 4)],
            "goal": [round(c, 4) for c in self.navigator.goal],
            "path": [[round(x, 3), round(y, 3)] for x, y in self.navigator.path],
            "dist_front": round(tel.dist_front_m, 3) if tel else None,
            "dist_rear": round(tel.dist_rear_m, 3) if tel else None,
            "collided": self.world.collided,
        }

    def static_scene(self) -> dict[str, Any]:
        """World extent, obstacles, and tags — sent once for the UI backdrop."""
        return {
            "width": self.world.width_m,
            "height": self.world.height_m,
            "obstacles": [
                [r.x_min, r.y_min, r.x_max, r.y_max] for r in self.world.obstacles
            ],
            "tags": {str(tid): [p.x, p.y] for tid, p in self.world.tags.items()},
            "goal": [round(c, 4) for c in self.navigator.goal],
        }


def world_extent(world: World) -> tuple[float, float]:
    """Convenience accessor for a world's (width, height) in metres."""
    return world.width_m, world.height_m
