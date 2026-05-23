# Architecture

Layered design, built bottom-up. Lower layers expose stable APIs; higher layers
consume them. Each layer is fully tested before the next begins.

```
Layer 6  UI & Logging          Flask + WebSocket + matplotlib
Layer 5  Navigation            state machine: plan / execute / recover
Layer 4  Localization          wheel odometry + AprilTag -> EKF
Layer 3  Algorithms            occupancy grid, A*, smoothing, pure pursuit
Layer 2  I/O Abstraction       RobotComms, Camera, AprilTagDetector (+Real/Sim)
Layer 1  ESP32 Firmware        C++, compiled & host-tested (flashed in Phase 7)
Layer 0  Simulation Core       World model, physics, sensor noise, ground truth
```

## Dependency injection seam

Production code depends only on abstract interfaces. `autoproject.factory` reads
`config/runtime.yaml` (`mode: sim | real`) and constructs the matching
implementations. Swapping simulation for hardware is a config change.

## Interface contracts (Layer 2 — defined in Phase 2)

> Stubs documented here now; signatures are finalized when Phase 2 lands.

- **`IRobotComms`** — `move(left, right)`, `stop()`, `get_telemetry()`;
  callbacks `on_obstacle`, `on_slip`, `on_done`.
- **`ICamera`** — `get_frame() -> np.ndarray`.
- **`IAprilTagDetector`** — `detect(frame) -> list[Detection]`.

Each has a `Real*` implementation (hardware) and a `Sim*` implementation (queries
the Layer 0 `World`).

## Navigation state machine (Layer 5 — defined in Phase 5)

States: `IDLE / PLANNING / EXECUTING / AVOIDING / RELOCALIZING / EMERGENCY`.
The committed Mermaid state diagram (with transition triggers) is added in
Phase 5 under `docs/figures/`.

## UART link (Layer 1 ↔ Layer 2)

The ESP32 ↔ RPi text protocol is specified in [`uart_protocol.md`](uart_protocol.md).
