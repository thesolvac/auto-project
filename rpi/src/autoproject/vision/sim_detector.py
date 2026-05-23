"""Simulated AprilTag detector.

Default (fast) path: bypass image rendering/decoding and emit detections straight
from the world's ground-truth tag visibility, with configurable Gaussian noise on
range and bearing. This matches the project spec ("default: bypass for speed")
and lets localization tests assert convergence of the *estimate* toward *truth*.

The ``detect(frame)`` signature is honored for interface compatibility; the frame
argument is ignored in bypass mode (the same camera->detector pipeline still runs
end-to-end for UI and timing).
"""

from __future__ import annotations

import math
import random

import numpy as np

from autoproject.simulation.geometry import Pose
from autoproject.simulation.world import World
from autoproject.vision.interfaces import Detection, IAprilTagDetector

_TWO_PI = 2.0 * math.pi


class SimAprilTagDetector(IAprilTagDetector):
    """Ground-truth-backed AprilTag detector with optional measurement noise."""

    def __init__(
        self,
        world: World,
        *,
        fov_rad: float = math.radians(60.0),
        max_range_m: float = 5.0,
        yaw_offset_rad: float = 0.0,
        fx: float = 554.0,
        cx: float = 320.0,
        range_sigma_m: float = 0.0,
        bearing_sigma_rad: float = 0.0,
    ) -> None:
        self.world = world
        self.fov_rad = fov_rad
        self.max_range_m = max_range_m
        self.yaw_offset_rad = yaw_offset_rad
        self.fx = fx
        self.cx = cx
        self.range_sigma_m = range_sigma_m
        self.bearing_sigma_rad = bearing_sigma_rad
        self._rng = random.Random(world.noise.seed + 2)

    def detect(
        self, frame: np.ndarray | None = None
    ) -> list[Detection]:  # noqa: ARG002
        p = self.world.pose
        camera_pose = Pose(p.x, p.y, p.theta + self.yaw_offset_rad)
        detections: list[Detection] = []
        for sighting in self.world.visible_tags(
            camera_pose, self.fov_rad, self.max_range_m
        ):
            range_m = sighting.range_m
            bearing = sighting.bearing_rad
            if self.range_sigma_m > 0.0:
                range_m = max(0.0, range_m + self._rng.gauss(0.0, self.range_sigma_m))
            if self.bearing_sigma_rad > 0.0:
                bearing += self._rng.gauss(0.0, self.bearing_sigma_rad)
            x_px = self.cx - self.fx * math.tan(bearing)
            detections.append(
                Detection(
                    tag_id=sighting.tag_id,
                    center_px=(x_px, 0.0),
                    range_m=range_m,
                    bearing_rad=bearing,
                )
            )
        return detections
