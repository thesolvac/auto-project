# CLAUDE.md — Autonomous Navigation Robot Project

## Mission

You are building a complete autonomous indoor navigation robot system, end-to-end, layer by layer, **using a simulation-first methodology**. This is an Israeli grade-14 final-year engineering project (Makif He' Darka Ashkelon, institution code 644450, student Adar Azulai).

Your job in this phase: write **all production code, firmware, tests, and supporting infrastructure**. The project book will be written in a later phase based on the completed codebase. Do **not** write book chapters now unless explicitly asked.

The system must be **fully runnable end-to-end without any hardware attached**, validated through high-fidelity simulation. Hardware integration on the physical robot is documented as a deferred deployment step (Phase 7).

## Deployment Target & Development Workflow

- **Deployment target:** Raspberry Pi 5 (4GB), Debian-based Linux. All code is written for this target. Paths, commands, and tool conventions in code and docs follow Linux/RPi norms.
- **Editor at deployment:** VS Code on the RPi (directly or via Remote-SSH).
- **Current development host:** the user works from Windows for writing/committing, then `git pull`s on the RPi to run. This is incidental — the code itself is Linux-targeted and platform-agnostic where it matters (pure Python + simulation).
- If you (Claude Code) happen to be operating on Windows during development, adapt local commands silently (e.g., `.venv\Scripts\activate` vs `source .venv/bin/activate`) — but never commit Windows-specific paths or shims into the project. The repository must be clean Linux/RPi style.
- `requirements.txt` is the source of truth for Python dependencies. `.venv/` is in `.gitignore` and recreated fresh on each machine.

## Development Approach: Simulation-First

Standard industry practice for autonomous systems is Software-in-the-Loop (SIL) validation before hardware. This project follows that approach:

1. Every hardware-dependent component has an **abstract interface** with two implementations:
   - `Real<Component>` — interfaces with physical hardware (pyserial, OpenCV camera, etc.). Written, type-checked, but not executed during this phase.
   - `Sim<Component>` — produces realistic deterministic synthetic behavior. Used by all tests, demos, and benchmarks in this phase.
2. A central **physics simulator** (`autoproject.simulation.world.World`) holds ground truth (true robot pose, obstacle map, AprilTag positions) and is queried by `Sim*` components.
3. **Dependency injection** is the rule: production code accepts the interface, never instantiates a concrete class. A factory reads `config/runtime.yaml` (`mode: sim` or `mode: real`) and constructs the appropriate implementations.
4. ESP32 firmware is written in full C++ and compiled with PlatformIO. Host-side unit tests run via Unity. Flashing to the physical ESP32 is deferred to Phase 7.

When hardware is connected later, switching from `Sim*` → `Real*` becomes a config change, not a refactor.

## Operating Environment

- **OS:** Debian/Raspberry Pi OS on the deployment target. Development happens wherever convenient; CI runs on `ubuntu-latest`.
- **Python:** 3.11+
- **Virtual environment:** `./rpi/.venv`, created fresh on each machine. Activation: `source rpi/.venv/bin/activate` (on RPi/Linux).
- **ESP32 toolchain:** PlatformIO Core CLI (`pio`), Arduino framework. Install via `pip install platformio`.
- **System packages assumed present on RPi:** `git`, `python3-venv`, `python3-dev`, `build-essential`, `libatlas-base-dev`, `libopenblas-dev`, `v4l-utils`, `i2c-tools`. If missing, install with `sudo apt install -y <pkg>` and document in `docs/setup.md`.
- **Serial port (Phase 7):** ESP32 USB connection appears as `/dev/ttyUSB0` or `/dev/ttyACM0`. User must be in the `dialout` group.

## Target Hardware (assembled, awaiting Phase 7 integration)

Documented for code completeness and future deployment. Not required to be present during current development.

- **Compute:** RPi 5 (4GB) + ESP32 DEVKIT V1 (USB-connected to RPi)
- **Motion:** 2× NEMA 17 stepper motors driven by 2× TMC2209 in differential drive layout + 1 caster wheel
- **Feedback:** 2× AS5600 magnetic encoders (fixed I2C addr 0x36) behind a TCA9548A I2C multiplexer (addr 0x70) on ESP32's I2C bus
- **Sensors:** 2× HC-SR04 ultrasonic (front + rear, ESP32 GPIO with voltage divider on Echo), 1× USB webcam
- **Power:** 12V battery pack → TMC2209 VM; 5V via LM2596 → RPi; 3.3V from ESP32 → TMC VIO and AS5600 VDD
- **Manual override:** Bluetooth gamepad paired to RPi
- **Pin assignments:** documented in `config/hardware_pins.yaml` (single source of truth)

## Architecture

```
Layer 7: (Future) Hardware Integration                         ← Phase 7, deferred
─────────────────────────────────────────────────────────────
Layer 6: UI & Logging (Flask + WebSocket + matplotlib)
Layer 5: Navigation State Machine (planner ↔ executor ↔ recovery)
Layer 4: Localization & Sensor Fusion (wheel odometry + AprilTag → EKF)
Layer 3: Algorithms (Occupancy Grid, A*, Path Smoothing, Pure Pursuit)
Layer 2: I/O Abstraction (RobotComms, Camera, AprilTagDetector — interfaces + Real + Sim)
Layer 1: ESP32 Firmware (C++, compiled, host-tested, not flashed yet)
Layer 0: Simulation Core (World model, physics, sensor noise, ground truth)
```

Layer 0 is the foundation — every `Sim*` implementation queries it. Build it early.

Each layer is built and fully tested before the next layer starts. Lower layers expose stable APIs; higher layers consume them. No skipping ahead.

## Simulation Core (Layer 0) — Specification

`autoproject.simulation.world.World` holds:
- **Map:** 2D occupancy grid with rectangular obstacles
- **Robot ground truth:** `pose = (x, y, θ)` updated by differential-drive kinematics from commanded wheel velocities
- **AprilTag world map:** `tag_id → (x, y, θ)` placements
- **Time:** simulated, advanced in fixed steps (50Hz physics tick)
- **Noise models (configurable):** Gaussian noise on encoder reads, multi-path noise on ultrasonics, motion blur and dropout on camera frames, slip probability on wheels

`Sim*` components query the world for their inputs:
- `SimRobotComms` reads commanded velocities, advances ground-truth pose via the world's physics tick, returns simulated encoder ticks (with slip), emits simulated ultrasonic readings based on robot pose vs obstacles.
- `SimCamera` renders a synthetic frame containing visible AprilTags (or returns pre-rendered fixtures).
- `SimAprilTagDetector` can either consume `SimCamera` frames or emit synthetic detections directly with configurable noise.

The simulator is the honest oracle: it knows ground truth and produces what hardware would. Tests assert that the system's *estimated* state converges toward the simulator's *ground truth* under the modeled noise.

## Repository Layout

```
auto-project/
├── CLAUDE.md                    # this file
├── README.md                    # human-facing project overview
├── .gitignore
├── docs/
│   ├── setup.md                 # RPi development environment bring-up
│   ├── uart_protocol.md         # ESP32 ↔ RPi protocol spec
│   ├── architecture.md          # layered architecture + interface contracts
│   ├── simulation.md            # simulator design and noise models
│   ├── hardware_integration.md  # Phase 7 plan (filled in later)
│   └── figures/                 # Mermaid sources committed
├── firmware/                    # ESP32 PlatformIO project
│   ├── platformio.ini
│   ├── src/
│   ├── lib/
│   └── test/                    # host-side unit tests via Unity
├── rpi/                         # Python codebase
│   ├── requirements.txt
│   ├── pyproject.toml
│   ├── src/autoproject/
│   │   ├── simulation/          # Layer 0 — world model, physics
│   │   ├── comms/               # interface + Real + Sim
│   │   ├── vision/              # interface + Real + Sim
│   │   ├── algorithms/          # pure Python, no I/O
│   │   ├── localization/
│   │   ├── navigation/
│   │   ├── ui/
│   │   ├── utils/
│   │   └── factory.py           # constructs Real* or Sim* per config
│   └── tests/                   # pytest, all hardware-free
├── config/
│   ├── hardware_pins.yaml
│   ├── robot_params.yaml        # wheelbase, wheel radius, gear ratio, etc.
│   ├── camera_calibration.yaml  # synthetic intrinsics for sim; real ones added in Phase 7
│   ├── world_tags.yaml          # AprilTag world coordinates
│   ├── sim_scenarios/           # named test maps + start/goal pairs
│   └── runtime.yaml             # selects Real vs Sim implementations
├── tools/
│   ├── run_simulation.py        # end-to-end demo runner
│   ├── benchmark_planner.py
│   ├── benchmark_fusion.py
│   └── plot_logs.py
├── .github/
│   └── workflows/
│       └── ci.yml               # lint + test on push (Ubuntu runner)
└── PROJECT_BOOK/                # empty for now — filled in a later phase
    └── .gitkeep
```

## Coding Standards

### Python
- **Style:** Black-formatted, 100-char lines, type hints on every public function
- **Linter:** `ruff` with strict config; CI must pass before push
- **Testing:** `pytest` + `pytest-cov`; minimum 70% coverage on `algorithms/` and `simulation/`, 50% elsewhere
- **Structure:** Each module exposes a clear class or function set; `__init__.py` re-exports the public API
- **Logging:** `logging` module, structured; never `print` in production code
- **Configuration:** YAML files in `config/`, loaded once at startup, validated with `pydantic`
- **No magic numbers:** Every constant has a name and a comment explaining its source (datasheet page, measurement, etc.)
- **Interfaces:** `abc.ABC` for hardware-component interfaces. `Real*` and `Sim*` accept the same abstract config object where possible.
- **Paths:** Always `pathlib.Path`, never raw strings. Never hardcode absolute paths.

### C++ (ESP32 firmware)
- **Style:** Google C++ Style (4-space indent), `clang-format` config in repo
- **Headers:** include guards `#pragma once`; one class per `.h`/`.cpp` pair
- **No `delay()`:** Use non-blocking timing via `millis()` or hardware timers
- **No dynamic allocation** in hot paths (motor ISR, encoder reading)
- **Host-side testability:** Pure logic (UART parsing, math) lives in classes compilable on the host (PlatformIO `native` env), separate from hardware interaction
- **Comment intent, not mechanics**

### Docstrings & Comments
- Public APIs: English docstrings with example usage
- Inline reasoning comments: English preferred; Hebrew acceptable for tricky design rationale (the book will eventually reference these)
- Every non-obvious algorithmic choice gets a one-line rationale comment

## Git Workflow (CRITICAL — follow strictly)

You are responsible for git hygiene. The user expects a clean, well-organized history.

### Branch strategy
- `main` is always working: `pytest` passes and `pio run` succeeds
- For each phase, create a branch: `phase-N-short-description` (e.g., `phase-1-esp32-firmware`)
- Sub-tasks within a phase get individual commits, not branches
- Merge to `main` via fast-forward or `--no-ff` merge with summary when phase is complete and acceptance criteria pass

### Commit rules — Conventional Commits
- `feat(scope): subject` — new feature
- `fix(scope): subject` — bug fix
- `refactor(scope): subject`
- `test(scope): subject`
- `docs(scope): subject`
- `chore(scope): subject`

Subject: imperative mood, ≤72 chars, no period. Body: explain *why*, not what.

Scopes: `firmware`, `sim`, `comms`, `vision`, `algorithms`, `localization`, `navigation`, `ui`, `docs`, `tools`, `ci`, `config`

### Push policy
**Push after every commit that compiles and passes the relevant tests.**
1. Sub-task completed with acceptance criteria met → commit + push
2. Bug fix confirmed by a test → commit + push
3. Documentation section completed → commit + push
4. End of working session → all WIP either committed and pushed, or stashed with descriptive name
5. **Never push broken code to `main`.** Mid-task commits go to the phase branch with `wip:` subject prefix

### Tags
- Tag each completed phase: `v0.0-bootstrap`, `v0.1-phase1`, ..., `v0.6-phase6`
- Tag codebase-complete milestone: `v1.0-sim-complete`
- Annotated tags: `git tag -a v0.1-phase1 -m "ESP32 firmware: compiles, host tests pass"`

### Before every push
```bash
cd rpi
source .venv/bin/activate    # adapt locally if on Windows; never commit Windows paths
ruff check src/ tests/
pytest tests/ -q
cd ..
cd firmware && pio run && cd ..
```
If anything fails, fix it first. If genuinely blocked, push to a `wip:` branch and ask the user.

## Phase Plan

Each phase has: **goal**, **tasks**, **acceptance criteria**. All acceptance criteria are simulation/test-based — no hardware required.

### Phase 0 — Project Bootstrap
**Goal:** Working skeleton with CI, dependencies, and an initial `README.md`.

Tasks:
1. Create directory structure as specified
2. Write `rpi/requirements.txt` (numpy, scipy, opencv-python, pupil-apriltags, pyserial, pydantic, pyyaml, flask, flask-socketio, pytest, pytest-cov, pytest-mock, ruff, black, filterpy, matplotlib)
3. Write `rpi/pyproject.toml` with project metadata and tool configs (ruff, black, pytest)
4. Create `firmware/platformio.ini` for `esp32dev` board with Arduino framework, plus a `native` env for host-side unit tests
5. Write `.gitignore` (Python, PlatformIO, VS Code, OS junk, log files, `.venv/`, build artifacts)
6. Set up GitHub Actions CI: lint + test on push (`.github/workflows/ci.yml`). Runs on `ubuntu-latest`: setup Python + PlatformIO, run `ruff check`, `pytest`, `pio run`, `pio test -e native`.
7. Stub `config/*.yaml` files with sensible defaults and inline comments
8. Write initial `README.md` (project description, simulation-first methodology, quick-start for RPi, current status)
9. Write `docs/setup.md` with full RPi bring-up steps (apt packages, Python venv, repo clone, dependency install)

Acceptance:
- `pio run` (in firmware/) compiles an empty `app_main` successfully
- `pytest` (in rpi/) passes with zero tests collected
- `ruff check` passes
- CI green on push

Tag: `v0.0-bootstrap`

---

### Phase 1 — ESP32 Firmware
**Goal:** Complete ESP32 firmware that controls motors, reads encoders + ultrasonics, communicates over UART **when connected to hardware**. Compiles cleanly and passes host-side unit tests. Not flashed in this phase.

Tasks (in order):
1. `UartProtocol`: line-based text protocol (spec in `docs/uart_protocol.md`). Command parsing and telemetry emission. Fully host-testable — Unity tests under `firmware/test/test_native/`.
2. `MotorDriver` using ESP32 RMT peripheral for STEP pulse generation. Support: setSpeed, moveSteps, trapezoidal acceleration ramp, shared EN pin, MS1/MS2 microstepping configuration. Hardware-touching code wrapped behind a thin abstraction so ramp/velocity logic is unit-testable.
3. `TCA9548A` I2C multiplexer driver: `selectChannel(uint8_t ch)`.
4. `AS5600Encoder`: reads RAW_ANGLE register (0x0C-0x0D) via multiplexer, handles wrap-around correctly across the 4095↔0 boundary, computes angular velocity with low-pass filter (α=0.2 EMA). Wrap-around logic and EMA filter are host-testable.
5. `UltrasonicSensor`: interrupt-driven on Echo, 5-sample moving average, 30ms timeout fallback to MAX_DISTANCE.
6. `ClosedLoopController`: PID at 200Hz, primary role is **slip detection** (large persistent step-vs-encoder error → emit `ERR SLIP`). Document this design choice in code header.
7. `main.cpp`: cooperative scheduler — UART parsing 1kHz, telemetry 20Hz, control loop 200Hz, sensor read 50Hz. No `delay()` anywhere.
8. Unity tests: UART parsing round-trip, encoder wrap-around math, PID step response on synthetic input, trapezoidal ramp profile

Acceptance:
- `pio run` compiles for `esp32dev` with no warnings
- `pio test -e native` passes all host-side unit tests
- `clang-tidy` (if configured) shows no warnings

Tag: `v0.1-phase1`

---

### Phase 2 — I/O Abstraction (Interfaces + Real + Sim)
**Goal:** Abstract interfaces for hardware-dependent components. Working `Sim*` implementations and compiling but unexecuted `Real*` implementations.

Tasks:
1. Abstract interfaces in `autoproject.comms.interfaces`, `autoproject.vision.interfaces`:
   - `IRobotComms`: `move(L, R)`, `stop()`, `get_telemetry()`, callbacks `on_obstacle`, `on_slip`, `on_done`
   - `ICamera`: `get_frame()` → `np.ndarray`
   - `IAprilTagDetector`: `detect(frame)` → `list[Detection]`
2. **Real implementations** (write, do not execute):
   - `RealRobotComms`: pyserial, background reader thread, auto-reconnect, configurable port (`/dev/ttyUSB0` default)
   - `RealCamera`: OpenCV `VideoCapture` with v4l2 backend
   - `RealAprilTagDetector`: pupil-apriltags with calibration from YAML
   - Tests for these marked `@pytest.mark.hardware` and skipped in CI
3. **Sim implementations** (the working ones for now):
   - `SimRobotComms`: queries `World`, advances robot pose given commanded wheel velocities, returns simulated encoder telemetry with configurable noise/slip, emits simulated ultrasonic distances via ray-casting against obstacles
   - `SimCamera`: renders simple top-down or oblique view containing visible AprilTags, or returns recorded fixture frames
   - `SimAprilTagDetector`: configurable to consume `SimCamera` frames or bypass rendering and emit synthetic detections with noise (default: bypass for speed)
4. `factory.py`: reads `config/runtime.yaml` (`mode: sim` or `mode: real`), constructs implementations
5. Comprehensive tests on `Sim*`: command sequences, telemetry shapes, noise within tolerances

Acceptance:
- `python -m autoproject.examples.sim_drive` runs and prints simulated telemetry as commands are issued
- `SimAprilTagDetector` produces consistent detections matching simulator ground truth (within configured noise)
- All tests pass; no test requires hardware
- `Real*` modules import cleanly, pass `mypy`/type checks, but are not executed

Tag: `v0.2-phase2`

---

### Phase 3 — Algorithms (Pure Python)
**Goal:** All planning algorithms implemented and unit-tested. No I/O dependency.

Tasks:
1. `algorithms/occupancy_grid.py`: `OccupancyGrid` with `from_obstacles(rects)`, `inflate(radius)`, `is_free(x, y)`, `to_image()`, `cell_to_world(i, j)`, `world_to_cell(x, y)`
2. `algorithms/astar.py`: A* with 8-connectivity, Euclidean heuristic, `heapq` open set. Admissibility derivation in module docstring. Returns world coordinates. Handles "no path" gracefully.
3. `algorithms/path_smoother.py`: line-of-sight smoother using Bresenham on the inflated grid
4. `algorithms/pure_pursuit.py`: PurePursuit controller. Given a path and current pose, returns `(v_left, v_right)`. Uses robot params from YAML for conversion.
5. `tools/benchmark_planner.py`: A* on a battery of test maps (5×5, 10×10, 20×20, 40×40, 80×80, varying obstacle density), records timing, produces matplotlib plot
6. Tests: 8+ scenarios — empty map, dense obstacles, no-path, narrow corridor exactly robot-width wide (verify inflation), U-shape requiring backtracking, large map performance smoke test

Acceptance:
- A* on 40×40 grid with 10% obstacles solves in <100ms
- Smoothing reduces waypoint count by ≥40% on random maps
- All test scenarios produce expected results
- Coverage on `algorithms/` ≥ 80%

Tag: `v0.3-phase3`

---

### Phase 4 — Localization & Sensor Fusion
**Goal:** Robust pose tracking combining wheel odometry and AprilTag observations, validated against simulator ground truth.

Tasks:
1. `localization/wheel_odometry.py`: `WheelOdometry` using the arc model (differential drive midpoint integration). Subscribes to `IRobotComms` telemetry. Derivation in docstring.
2. `localization/pose_fusion.py`: implement both:
   - `ComplementaryFilter`: `pose = α * (pose + Δodom) + (1-α) * pose_apriltag` when tag visible
   - `EKFFusion`: state `[x, y, θ]`, prediction from odometry, measurement update from AprilTag, using `filterpy.kalman.ExtendedKalmanFilter`
   Select via config.
3. `tools/benchmark_fusion.py`: drives simulator on identical scenarios with each filter, computes RMSE vs ground truth, produces comparison plots. Scenarios: continuous AprilTag visibility, intermittent (50% blackout), total blackout for 5 seconds, high wheel slip.
4. Tests with simulator-generated telemetry: convergence within tolerances, bounded drift during blackouts, recovery on tag re-acquisition

Acceptance:
- Wheel odometry over a simulated 5m straight run shows <5% positional error vs ground truth (with zero slip)
- EKF reduces RMSE vs odometry-only by ≥40% in scenarios with intermittent AprilTag visibility
- Filter state remains bounded when AprilTags are absent for 10 seconds

Tag: `v0.4-phase4`

---

### Phase 5 — Navigation Stack
**Goal:** Full closed-loop navigation in simulation. Robot receives goal, plans, executes, recovers from obstacles, reaches goal — all in `SimWorld`.

Tasks:
1. `navigation/navigator.py`: state machine `IDLE / PLANNING / EXECUTING / AVOIDING / RELOCALIZING / EMERGENCY`. Transitions documented in `docs/architecture.md` with a Mermaid state diagram committed to the repo.
2. Reactive obstacle layer: simulated ultrasonic < 30cm triggers `AVOIDING`, halt motion, replan from fused pose
3. Slip recovery: on `ERR SLIP` from `SimRobotComms` (simulator injects slip events), transition to `RELOCALIZING`, wait for next AprilTag fix, then replan
4. Goal tolerance: declare goal reached when `||pose - goal|| < 5cm` and `|θ - θ_goal| < 5°`
5. End-to-end integration tests, 100% in simulation: 10 different start/goal/map combinations, varying obstacle density, varying tag visibility
6. `tools/run_simulation.py`: command-line runner that takes a scenario name, runs the full stack, dumps logs and plots

Acceptance:
- Robot navigates simulated 5m point-to-point on a known map with 3 obstacles, 10 runs, ≥9/10 successful arrivals with <10cm error vs ground truth
- Dynamic obstacle (simulator spawns mid-run) triggers replan and completion in ≥7/10 runs
- No simulated collisions in any run

Tag: `v0.5-phase5`

---

### Phase 6 — UI, Logging, and Manual Override
**Goal:** Visualization and observability. Replay capability for analysis.

Tasks:
1. `ui/app.py`: Flask + Flask-SocketIO. Routes: `/` (scenario selection + map input form), `/live` (live visualization of simulated run), `/logs` (past runs replay)
2. SVG-based live map renderer (browser JS) subscribing via WebSocket to pose updates
3. `utils/logger.py`: structured JSONL logger. Every simulated run gets `runs/YYYYMMDD_HHMMSS/run.jsonl` + auto-generated summary plots at run end
4. Manual mode: keyboard arrow keys in the web UI map to wheel velocity commands. Drives the simulator in sim mode; will drive the robot in real mode. Emergency stop button always halts.
5. `tools/plot_logs.py`: post-hoc analysis producing all figures the project book will eventually need (pose trace over map, error over time, sensor readings over time, A* search visualization)

Acceptance:
- UI shows live simulated robot position with <500ms latency
- Past run can be replayed step-by-step from log file
- Manual control mode works in simulation
- 10 logged runs produce reproducible plots via `tools/plot_logs.py`

Tag: `v1.0-sim-complete`

---

### Phase 7 — Hardware Integration (DEFERRED — do not start without explicit user instruction)

This phase is intentionally deferred. When the user explicitly says "begin Phase 7":
1. Flash ESP32 firmware (compiled in Phase 1) and verify telemetry stream over USB UART
2. Run camera calibration on the physical camera in its mounted position
3. Map AprilTag world positions (measure and record in `world_tags.yaml`)
4. Switch `runtime.yaml` to `mode: real`
5. Field-test each phase's acceptance criteria on hardware, documenting actual vs simulated behavior
6. Update `docs/hardware_integration.md` with discovered deltas

**Do not start this phase without explicit user instruction. Do not create branches or scaffolding for it pre-emptively.**

---

## Handling Hardware-Required Tests (during current phases)

If you find yourself wanting to write a test that needs hardware:
1. Stop. The simulation-first principle says hardware tests should not be required in any current phase.
2. Re-evaluate: is there a `Sim*` equivalent? If yes, use it.
3. If the test truly cannot be run in simulation, mark `@pytest.mark.hardware`, ensure it does NOT run in CI, document for Phase 7.
4. Continue with the simulation-based equivalent for current acceptance criteria.

## When You Are Stuck

In order of preference:
1. **Read the datasheet/docs.** Hallucinations about pin behavior, register layouts, or library APIs are the #1 source of bugs. Cite the page/section in a code comment when you do.
2. **Write a minimal reproduction** and run it
3. **Ask the user.** Be specific: state what you tried, what you expected, what happened. Do not silently pick an arbitrary direction.
4. **Never fake test results.** If something can't be verified yet, say so explicitly and mark the test appropriately.

## Communication Style With the User

The user is a strong student but a student — explain non-obvious choices briefly. Reply in Hebrew when the user writes in Hebrew. When showing diffs or output, English is fine.

## First Action

When you receive your first message in this project:
1. Verify you are in the repo root (the directory containing this file)
2. Run `git status` and `git log --oneline -5` to understand current state
3. Check that Python (3.11+) and PlatformIO are available
4. Report what you found, then propose: "Starting Phase 0 (bootstrap)" or "Resuming Phase N based on the repo state"
5. Wait for user confirmation before making changes or pushing commits

Begin.<img width="686" height="210" alt="image" src="https://github.com/user-attachments/assets/48c12581-bd5b-412f-afd5-e91a685fead6" />
