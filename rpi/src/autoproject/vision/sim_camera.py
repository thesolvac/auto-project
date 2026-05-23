"""Simulated camera: renders a simple synthetic frame of the visible AprilTags.

Numpy-only rendering (no OpenCV needed): a mid-gray background with a white
square drawn at each visible tag's projected image position, sized by range. Used
for the UI/visualization path and as an input to ``SimAprilTagDetector`` when it
is configured to consume frames. The detector's default (fast) path bypasses
rendering and reads ground truth directly.
"""

from __future__ import annotations

import math

import numpy as np

from autoproject.simulation.geometry import Pose
from autoproject.simulation.world import World
from autoproject.vision.interfaces import ICamera

_BACKGROUND_GRAY = 60
_TAG_WHITE = 255


class SimCamera(ICamera):
    """Renders a top-down-projected synthetic frame from the world's tags."""

    def __init__(
        self,
        world: World,
        *,
        width: int = 640,
        height: int = 480,
        fx: float = 554.0,
        cx: float = 320.0,
        cy: float = 240.0,
        fov_rad: float = math.radians(60.0),
        max_range_m: float = 5.0,
        yaw_offset_rad: float = 0.0,
        tag_size_m: float = 0.20,
    ) -> None:
        self.world = world
        self.width = width
        self.height = height
        self.fx = fx
        self.cx = cx
        self.cy = cy
        self.fov_rad = fov_rad
        self.max_range_m = max_range_m
        self.yaw_offset_rad = yaw_offset_rad
        self.tag_size_m = tag_size_m

    def camera_pose(self) -> Pose:
        """Camera pose in the world frame (robot pose + yaw offset)."""
        p = self.world.pose
        return Pose(p.x, p.y, p.theta + self.yaw_offset_rad)

    def get_frame(self) -> np.ndarray:
        frame = np.full((self.height, self.width, 3), _BACKGROUND_GRAY, dtype=np.uint8)
        for sighting in self.world.visible_tags(self.camera_pose(), self.fov_rad, self.max_range_m):
            x_px = int(round(self.cx - self.fx * math.tan(sighting.bearing_rad)))
            half = max(2, int(self.fx * self.tag_size_m / (2.0 * sighting.range_m)))
            x0 = max(0, x_px - half)
            x1 = min(self.width, x_px + half)
            y0 = max(0, int(self.cy) - half)
            y1 = min(self.height, int(self.cy) + half)
            if x0 < x1 and y0 < y1:
                frame[y0:y1, x0:x1, :] = _TAG_WHITE
        return frame
