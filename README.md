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

## Running the simulation

```bash
# End-to-end run on a scenario -> logs + trajectory plot under runs/
python tools/run_simulation.py demo_room

# Post-hoc figures from the latest run
python tools/plot_logs.py latest

# Live web UI (auto + manual control); open http://localhost:5000
python -m autoproject.ui.app
```

## Current Status

**Simulation complete (`v1.0-sim-complete`).** The full stack runs end-to-end in
simulation: firmware compiles + host-tests pass, the navigation stack drives the
simulated robot to its goal with obstacle avoidance and slip recovery, and a web
UI visualizes live runs and replays logs. Hardware integration (Phase 7) is the
remaining, deferred step.

| Phase | Scope | State |
|------:|-------|-------|
| 0 | Bootstrap (structure, deps, CI) | ✅ done |
| 1 | ESP32 firmware | ✅ done |
| 2 | I/O abstraction (interfaces + Real + Sim) | ✅ done |
| 3 | Algorithms (grid, A*, smoothing, pursuit) | ✅ done |
| 4 | Localization & sensor fusion | ✅ done |
| 5 | Navigation stack | ✅ done |
| 6 | UI, logging, manual override | ✅ done |
| 7 | Hardware integration | deferred |

## License

MIT.
