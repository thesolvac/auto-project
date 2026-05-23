"""Real AprilTag detector via pupil-apriltags (Phase 7 hardware).

Written and type-checked now; executed only on the physical robot. Loads camera
intrinsics + tag size from ``config/camera_calibration.yaml`` and recovers each
tag's pose, exposing range and bearing in the camera frame.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path

import numpy as np
from pupil_apriltags import Detector  # imported only when used (real mode)

from autoproject.utils.config import CONFIG_DIR, load_config
from autoproject.vision.interfaces import Detection, IAprilTagDetector

logger = logging.getLogger(__name__)


class RealAprilTagDetector(IAprilTagDetector):
    """``IAprilTagDetector`` backed by pupil-apriltags with YAML calibration."""

    def __init__(self, calibration_path: str | Path | None = None) -> None:
        cfg = load_config(calibration_path or CONFIG_DIR / "camera_calibration.yaml")
        intr = cfg["intrinsics"]
        self._fx = float(intr["fx"])
        self._fy = float(intr["fy"])
        self._cx = float(intr["cx"])
        self._cy = float(intr["cy"])
        self._tag_size_m = float(cfg["apriltag"]["tag_size_m"])
        self._family = str(cfg["apriltag"]["family"])
        self._detector = Detector(families=self._family)

    def detect(self, frame: np.ndarray) -> list[Detection]:
        gray = frame if frame.ndim == 2 else frame[:, :, 0]
        results = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self._fx, self._fy, self._cx, self._cy),
            tag_size=self._tag_size_m,
        )
        detections: list[Detection] = []
        for r in results:
            # Pose translation is in the camera frame: x right, y down, z forward.
            tx, _, tz = (float(v) for v in r.pose_t.flatten())
            range_m = math.hypot(tx, tz)
            bearing_rad = math.atan2(-tx, tz)  # +left
            center = (float(r.center[0]), float(r.center[1]))
            detections.append(
                Detection(
                    tag_id=int(r.tag_id), center_px=center, range_m=range_m, bearing_rad=bearing_rad
                )
            )
        return detections
