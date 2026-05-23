"""Simulated robot communications: the World-backed ``IRobotComms``.

Reads commanded wheel velocities, advances the ground-truth pose via the world's
physics tick, and synthesizes the telemetry real hardware would report:

- **encoder counts** integrated from the *commanded* wheel rotation (so under
  wheel slip the encoders over-report relative to the true pose — the realistic
  source of odometry drift), with optional Gaussian read noise;
- **ultrasonic distances** ray-cast from the sensor poses against the world, with
  optional noise and dropout.

Obstacle, slip, and done events are emitted through the base-class callbacks.
"""

from __future__ import annotations

import math
import random

from autoproject.comms.interfaces import IRobotComms, Telemetry
from autoproject.simulation.geometry import Pose
from autoproject.simulation.world import DEFAULT_ULTRASONIC_MAX_M, World

# Matches the firmware obstacle threshold (config / docs/uart_protocol.md).
DEFAULT_OBSTACLE_THRESHOLD_M = 0.30
DEFAULT_WHEEL_RADIUS_M = 0.040
DEFAULT_ENCODER_COUNTS_PER_REV = 4096

_TWO_PI = 2.0 * math.pi


class SimRobotComms(IRobotComms):
    """``IRobotComms`` backed by the simulation :class:`World`."""

    def __init__(
        self,
        world: World,
        *,
        wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        encoder_counts_per_rev: int = DEFAULT_ENCODER_COUNTS_PER_REV,
        obstacle_threshold_m: float = DEFAULT_OBSTACLE_THRESHOLD_M,
        ultrasonic_max_m: float = DEFAULT_ULTRASONIC_MAX_M,
    ) -> None:
        super().__init__()
        self.world = world
        self.circumference_m = _TWO_PI * wheel_radius_m
        self.counts_per_rev = encoder_counts_per_rev
        self.obstacle_threshold_m = obstacle_threshold_m
        self.ultrasonic_max_m = ultrasonic_max_m

        self._cmd_left = 0.0
        self._cmd_right = 0.0
        self._enc_left = 0.0
        self._enc_right = 0.0
        self._telemetry: Telemetry | None = None
        # Independent RNG (offset from the world seed) for sensor read noise.
        self._rng = random.Random(world.noise.seed + 1)

    # --- IRobotComms commands ---
    def move(self, left_mps: float, right_mps: float) -> None:
        self._cmd_left = left_mps
        self._cmd_right = right_mps

    def stop(self) -> None:
        self._cmd_left = 0.0
        self._cmd_right = 0.0

    def get_telemetry(self) -> Telemetry | None:
        return self._telemetry

    # --- simulation driver ---
    def step(self, dt: float | None = None) -> Telemetry:
        """Advance the world one tick at the commanded velocities and update telemetry."""
        tick = self.world.dt_s if dt is None else dt
        self.world.step(self._cmd_left, self._cmd_right)

        self._enc_left += self._counts_delta(self._cmd_left, tick)
        self._enc_right += self._counts_delta(self._cmd_right, tick)

        front = self._read_sonar(0.0)
        rear = self._read_sonar(math.pi)
        self._telemetry = Telemetry(
            enc_left_counts=int(round(self._enc_left)),
            enc_right_counts=int(round(self._enc_right)),
            dist_front_m=front,
            dist_rear_m=rear,
            timestamp_s=self.world.time_s,
        )

        if front < self.obstacle_threshold_m:
            self._emit_obstacle("front", front)
        if rear < self.obstacle_threshold_m:
            self._emit_obstacle("rear", rear)
        if self.world.last_step_slipped:
            self._emit_slip()

        return self._telemetry

    # --- helpers ---
    def _counts_delta(self, wheel_mps: float, dt: float) -> float:
        revs = (wheel_mps * dt) / self.circumference_m
        counts = revs * self.counts_per_rev
        sigma_rad = self.world.noise.encoder_sigma_rad
        if sigma_rad > 0.0:
            counts += self._rng.gauss(0.0, sigma_rad) * self.counts_per_rev / _TWO_PI
        return counts

    def _read_sonar(self, heading_offset: float) -> float:
        noise = self.world.noise
        if (
            noise.ultrasonic_dropout_prob > 0.0
            and self._rng.random() < noise.ultrasonic_dropout_prob
        ):
            return self.ultrasonic_max_m
        pose = self.world.pose
        sensor = Pose(pose.x, pose.y, pose.theta + heading_offset)
        distance = self.world.ultrasonic_reading(sensor, self.ultrasonic_max_m)
        if noise.ultrasonic_sigma_m > 0.0:
            distance += self._rng.gauss(0.0, noise.ultrasonic_sigma_m)
        return max(0.0, min(distance, self.ultrasonic_max_m))
