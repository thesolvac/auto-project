"""Component factory: construct Real* or Sim* implementations from runtime config.

Dependency-injection seam for the whole system. Production code accepts the
abstract interfaces (``IRobotComms``, ``ICamera``, ``IAprilTagDetector``) and
never instantiates a concrete class directly. This factory reads
``config/runtime.yaml`` (``mode: sim`` | ``mode: real``) and builds the matching
implementations, so swapping simulation for hardware is a config change.

Phase 0: documented placeholder. Concrete wiring is added in Phase 2 once the
interfaces and Sim*/Real* implementations exist.
"""

from __future__ import annotations

from enum import StrEnum


class RuntimeMode(StrEnum):
    """Selects which family of component implementations the factory builds."""

    SIM = "sim"
    REAL = "real"
