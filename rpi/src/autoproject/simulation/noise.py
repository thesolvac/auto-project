"""Configurable noise-model parameters for the simulation core.

The :class:`World` holds one :class:`NoiseConfig`. Wheel-slip parameters affect
ground truth directly (a slipping wheel moves the robot less than commanded) and
are applied in :meth:`World.step`. The sensor-noise parameters are stored here
and consumed by the ``Sim*`` components built in Phase 2 (encoder, ultrasonic,
and camera read-out noise). All defaults are zero, so an unconfigured world is
fully deterministic.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class NoiseConfig(BaseModel):
    """Noise/slip parameters. Zero everywhere == ideal, deterministic world."""

    seed: int = 0  # seeds the world RNG for reproducible noise/slip

    # --- Wheel slip (ground truth, applied in World.step) ---
    wheel_slip_prob: float = Field(0.0, ge=0.0, le=1.0)  # per wheel, per tick
    wheel_slip_factor: float = Field(0.0, ge=0.0, le=1.0)  # motion retained on slip

    # --- Sensor read-out noise (consumed by Sim* components, Phase 2) ---
    encoder_sigma_rad: float = Field(0.0, ge=0.0)
    ultrasonic_sigma_m: float = Field(0.0, ge=0.0)
    ultrasonic_dropout_prob: float = Field(0.0, ge=0.0, le=1.0)
    camera_dropout_prob: float = Field(0.0, ge=0.0, le=1.0)
