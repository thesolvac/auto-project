"""Layer 2 — robot comms: IRobotComms interface with Real and Sim implementations.

Only the interface and the Sim implementation are re-exported here; RealRobotComms
is imported explicitly from autoproject.comms.real_comms (real mode) so a
simulation-only environment never needs pyserial installed.
"""

from autoproject.comms.interfaces import IRobotComms, Telemetry
from autoproject.comms.sim_comms import SimRobotComms

__all__ = ["IRobotComms", "SimRobotComms", "Telemetry"]
