"""Component factory: construct Real* or Sim* implementations from runtime config.

Dependency-injection seam for the whole system. Production code accepts the
abstract interfaces (``IRobotComms``, ``ICamera``, ``IAprilTagDetector``) and
never instantiates a concrete class directly. This factory reads
``config/runtime.yaml`` (global ``mode`` plus optional per-component overrides)
and builds the matching implementations, so swapping simulation for hardware is a
config change.

Real* implementations are imported lazily, inside the ``real`` branch, so a
simulation-only environment never needs pyserial/opencv/pupil-apriltags installed.
"""

from __future__ import annotations

import math
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING, Any

from autoproject.utils.config import CONFIG_DIR, load_config

if TYPE_CHECKING:
    from autoproject.comms.interfaces import IRobotComms
    from autoproject.simulation.world import World
    from autoproject.vision.interfaces import IAprilTagDetector, ICamera


class RuntimeMode(StrEnum):
    """Selects which family of component implementations the factory builds."""

    SIM = "sim"
    REAL = "real"


def load_runtime(path: str | Path | None = None) -> dict[str, Any]:
    """Load ``config/runtime.yaml`` (or an explicit path)."""
    return load_config(path or CONFIG_DIR / "runtime.yaml")


def resolve_mode(runtime: dict[str, Any], component: str) -> RuntimeMode:
    """Resolve the mode for a component: its override if set, else the global mode."""
    override = (runtime.get("components") or {}).get(component)
    return RuntimeMode(override if override is not None else runtime["mode"])


def _require_world(world: World | None, what: str) -> World:
    if world is None:
        raise ValueError(f"sim mode requires a World to build {what}")
    return world


def build_robot_comms(runtime: dict[str, Any], *, world: World | None = None) -> IRobotComms:
    """Construct the robot-comms implementation selected by ``runtime``."""
    if resolve_mode(runtime, "comms") is RuntimeMode.SIM:
        from autoproject.comms.sim_comms import SimRobotComms

        return SimRobotComms(_require_world(world, "SimRobotComms"))

    from autoproject.comms.real_comms import RealRobotComms

    return RealRobotComms()


def build_camera(runtime: dict[str, Any], *, world: World | None = None) -> ICamera:
    """Construct the camera implementation selected by ``runtime``."""
    if resolve_mode(runtime, "camera") is RuntimeMode.SIM:
        from autoproject.vision.sim_camera import SimCamera

        cam = load_config(CONFIG_DIR / "camera_calibration.yaml")
        intr = cam["intrinsics"]
        width = cam["resolution"]["width"]
        fov = 2.0 * math.atan((width / 2.0) / intr["fx"])
        return SimCamera(
            _require_world(world, "SimCamera"),
            width=width,
            height=cam["resolution"]["height"],
            fx=intr["fx"],
            cx=intr["cx"],
            cy=intr["cy"],
            fov_rad=fov,
            tag_size_m=cam["apriltag"]["tag_size_m"],
        )

    from autoproject.vision.real_camera import RealCamera

    return RealCamera()


def build_detector(runtime: dict[str, Any], *, world: World | None = None) -> IAprilTagDetector:
    """Construct the AprilTag-detector implementation selected by ``runtime``."""
    if resolve_mode(runtime, "apriltag") is RuntimeMode.SIM:
        from autoproject.vision.sim_detector import SimAprilTagDetector

        cam = load_config(CONFIG_DIR / "camera_calibration.yaml")
        intr = cam["intrinsics"]
        fov = 2.0 * math.atan((cam["resolution"]["width"] / 2.0) / intr["fx"])
        return SimAprilTagDetector(
            _require_world(world, "SimAprilTagDetector"),
            fov_rad=fov,
            fx=intr["fx"],
            cx=intr["cx"],
        )

    from autoproject.vision.real_detector import RealAprilTagDetector

    return RealAprilTagDetector()
