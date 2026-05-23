"""Abstract interfaces for the vision pipeline (Layer 2): camera + tag detector.

Production code depends on :class:`ICamera` and :class:`IAprilTagDetector`; the
factory injects the simulated or real implementations.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A single AprilTag detection.

    ``range_m`` / ``bearing_rad`` are the tag's position relative to the camera
    (bearing positive to the left). They may be ``None`` if a real detector could
    not recover pose (e.g. missing calibration), but the simulated detector always
    fills them from ground truth + noise.
    """

    tag_id: int
    center_px: tuple[float, float]
    range_m: float | None = None
    bearing_rad: float | None = None


class ICamera(ABC):
    """Source of camera frames."""

    @abstractmethod
    def get_frame(self) -> np.ndarray:
        """Return the latest frame as an ``(H, W, 3)`` uint8 BGR array."""


class IAprilTagDetector(ABC):
    """Detects AprilTags in a frame."""

    @abstractmethod
    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Return all tags detected in ``frame`` (may be empty)."""
