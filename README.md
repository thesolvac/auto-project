# Autonomous Indoor Navigation Robot

A differential-drive indoor robot that plans and follows a path to a goal,
avoids obstacles, and localizes itself by fusing wheel odometry with AprilTag
landmarks. Compute is split between a **Raspberry Pi 5** (planning, vision,
localization, UI) and an **ESP32** (real-time motor control and sensors).

Grade-14 final-year engineering project — Makif He' Darka, Ashkelon
(institution code 644450). Student: Adar Azulai.

## Simulation-First Methodology

The entire system runs end-to-end **without any hardware attached**, validated
through high-fidelity simulation (Software-in-the-Loop). This is standard
practice for autonomous systems: validate in software before committing to
hardware.

- Every hardware-dependent component has an **abstract interface** with two
  implementations: `Real<Component>` (talks to physical hardware) and
  `Sim<Component>` (deterministic synthetic behavior).
- A central physics simulator (`autoproject.simulation.world.World`) holds
  ground truth — true robot pose, obstacle map, AprilTag positions — and the
  `Sim*` components query it.
- **Dependency injection**: production code depends on interfaces only. A
  factory reads `config/runtime.yaml` and builds `Sim*` or `Real*`. Switching to
  hardware is a config change, not a refactor.

## Architecture

```
Layer 6  UI & Logging          Flask + WebSocket + matplotlib
Layer 5  Navigation            state machine: plan / execute / recover
Layer 4  Localization          wheel odometry + AprilTag -> EKF
Layer 3  Algorithms            occupancy grid, A*, smoothing, pure pursuit
Layer 2  I/O Abstraction       RobotComms, Camera, AprilTagDetector (+Real/Sim)
Layer 1  ESP32 Firmware        C++, compiled & host-tested (flashed in Phase 7)
Layer 0  Simulation Core       World model, physics, sensor noise, ground truth
```

See [`docs/architecture.md`](docs/architecture.md) for interface contracts and
[`docs/simulation.md`](docs/simulation.md) for the simulator design.

## Repository Layout

```
firmware/   ESP32 PlatformIO project (C++; Unity host tests)
rpi/        Python codebase (src/autoproject + tests)
config/     YAML config: pins, robot params, calibration, scenarios, runtime mode
tools/      simulation runner, benchmarks, log plotting
docs/       setup, protocol, architecture, simulation, hardware integration
```

## Quick Start (Raspberry Pi 5 / Linux)

Full bring-up — including system packages — is in [`docs/setup.md`](docs/setup.md).

```bash
git clone https://github.com/thesolvac/auto-project.git
cd auto-project

# Python side
python3 -m venv rpi/.venv
source rpi/.venv/bin/activate
pip install -r rpi/requirements.txt
cd rpi && pytest -q && cd ..

# Firmware side (build only; flashing is Phase 7)
pip install platformio
cd firmware && pio run -e esp32dev && pio test -e native && cd ..
```

## Current Status

**Phase 0 — Bootstrap.** Project skeleton, dependencies, config stubs, and CI in
place. Firmware compiles an empty sketch; the Python test suite runs clean.

Phases build bottom-up; each is fully tested before the next begins:

| Phase | Scope | State |
|------:|-------|-------|
| 0 | Bootstrap (structure, deps, CI) | in progress |
| 1 | ESP32 firmware | pending |
| 2 | I/O abstraction (interfaces + Real + Sim) | pending |
| 3 | Algorithms (grid, A*, smoothing, pursuit) | pending |
| 4 | Localization & sensor fusion | pending |
| 5 | Navigation stack | pending |
| 6 | UI, logging, manual override | pending |
| 7 | Hardware integration | deferred |

## License

MIT.
