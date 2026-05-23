"""Layer 0 — simulation core: World model, physics, sensor noise, ground truth."""

from autoproject.simulation.geometry import (
    Pose,
    Rectangle,
    normalize_angle,
    ray_box_intersection,
)
from autoproject.simulation.kinematics import diff_drive_step
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import (
    DEFAULT_DT_S,
    DEFAULT_ULTRASONIC_MAX_M,
    TagSighting,
    World,
)

__all__ = [
    "DEFAULT_DT_S",
    "DEFAULT_ULTRASONIC_MAX_M",
    "NoiseConfig",
    "Pose",
    "Rectangle",
    "TagSighting",
    "World",
    "diff_drive_step",
    "normalize_angle",
    "ray_box_intersection",
]
