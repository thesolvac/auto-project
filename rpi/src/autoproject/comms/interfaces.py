"""Abstract interface for robot communications (Layer 2).

Production code depends only on :class:`IRobotComms`; the factory injects either
``SimRobotComms`` (queries the simulation :class:`~autoproject.simulation.world.World`)
or ``RealRobotComms`` (talks to the ESP32 over UART). The base class owns the
event-callback plumbing so both implementations share it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

# Callback signatures.
ObstacleCallback = Callable[[str, float], None]  # (sensor name, distance_m)
SlipCallback = Callable[[], None]
DoneCallback = Callable[[], None]


@dataclass(frozen=True)
class Telemetry:
    """One telemetry sample from the robot (mirrors the firmware ``TEL`` line)."""

    enc_left_counts: int
    enc_right_counts: int
    dist_front_m: float
    dist_rear_m: float
    timestamp_s: float


class IRobotComms(ABC):
    """Drive the robot and receive telemetry/events, regardless of sim vs real."""

    def __init__(self) -> None:
        self._on_obstacle: ObstacleCallback | None = None
        self._on_slip: SlipCallback | None = None
        self._on_done: DoneCallback | None = None

    # --- commands ---
    @abstractmethod
    def move(self, left_mps: float, right_mps: float) -> None:
        """Command left/right wheel linear velocities [m/s]."""

    @abstractmethod
    def stop(self) -> None:
        """Halt both wheels."""

    @abstractmethod
    def get_telemetry(self) -> Telemetry | None:
        """Return the most recent telemetry sample, or ``None`` if none yet."""

    # --- event callbacks ---
    def on_obstacle(self, callback: ObstacleCallback | None) -> None:
        """Register a callback fired when an ultrasonic reading trips the threshold."""
        self._on_obstacle = callback

    def on_slip(self, callback: SlipCallback | None) -> None:
        """Register a callback fired when wheel slip is detected."""
        self._on_slip = callback

    def on_done(self, callback: DoneCallback | None) -> None:
        """Register a callback fired when a commanded motion completes."""
        self._on_done = callback

    # --- protected emit helpers for subclasses ---
    def _emit_obstacle(self, sensor: str, distance_m: float) -> None:
        if self._on_obstacle is not None:
            self._on_obstacle(sensor, distance_m)

    def _emit_slip(self) -> None:
        if self._on_slip is not None:
            self._on_slip()

    def _emit_done(self) -> None:
        if self._on_done is not None:
            self._on_done()
