"""Real camera via OpenCV VideoCapture (Phase 7 hardware).

Written and type-checked now; executed only on the physical robot. Uses the V4L2
backend for the USB webcam on the Raspberry Pi.
"""

from __future__ import annotations

import logging

import cv2  # opencv-python; imported only when this module is used (real mode)
import numpy as np

from autoproject.vision.interfaces import ICamera

logger = logging.getLogger(__name__)

DEFAULT_DEVICE_INDEX = 0


class RealCamera(ICamera):
    """``ICamera`` backed by an OpenCV ``VideoCapture`` (V4L2)."""

    def __init__(
        self, device_index: int = DEFAULT_DEVICE_INDEX, width: int = 640, height: int = 480
    ) -> None:
        self._device_index = device_index
        self._width = width
        self._height = height
        self._capture: cv2.VideoCapture | None = None

    def open(self) -> None:
        """Open the capture device and apply the requested resolution."""
        self._capture = cv2.VideoCapture(self._device_index, cv2.CAP_V4L2)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if not self._capture.isOpened():
            raise RuntimeError(f"Could not open camera device {self._device_index}")

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def get_frame(self) -> np.ndarray:
        if self._capture is None:
            raise RuntimeError("Camera not opened; call open() first")
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Failed to read frame from camera")
        return frame
