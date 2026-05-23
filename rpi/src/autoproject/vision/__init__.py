"""Layer 2 — vision: ICamera and IAprilTagDetector interfaces (Real + Sim).

Only the interfaces and the Sim implementations are re-exported here; the Real
implementations are imported explicitly from their modules (real mode) so a
simulation-only environment never needs opencv/pupil-apriltags installed.
"""

from autoproject.vision.interfaces import Detection, IAprilTagDetector, ICamera
from autoproject.vision.sim_camera import SimCamera
from autoproject.vision.sim_detector import SimAprilTagDetector

__all__ = [
    "Detection",
    "IAprilTagDetector",
    "ICamera",
    "SimAprilTagDetector",
    "SimCamera",
]
