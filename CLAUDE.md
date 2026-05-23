# CLAUDE.md — Autonomous Navigation Robot Project

## Mission

You are building a complete autonomous indoor navigation robot system, end-to-end, layer by layer. The hardware is already assembled and wired. Your job is to write **all the production code, firmware, tests, documentation, and the project book** for an Israeli grade-14 final-year engineering project (Makif He' Darka Ashkelon, institution code 644450, student Adar Azulai).

This project will be graded. Code quality, engineering rigor, honest documentation of failures, and benchmarking are evaluated as heavily as functional correctness.

## Operating Environment

- **Development host:** Raspberry Pi 5 (4GB), accessed via VS Code Remote-SSH
- **Working directory:** project root (the directory containing this file)
- **Repository:** https://github.com/thesolvac/auto-project.git (already initialized; you have push access via configured credentials)
- **Python:** 3.11+, use a venv at `./rpi/.venv`
- **ESP32 toolchain:** PlatformIO Core CLI (`pio`), Arduino framework
- **OS packages assumed present:** `git`, `python3-venv`, `build-essential`, `libatlas-base-dev` (for numpy on ARM), `libopenblas-dev`, `v4l-utils`
- If a tool is missing, install it with `sudo apt install -y <pkg>` and document it in `docs/setup.md`

## Hardware (already wired per schematic)

- **Compute:** RPi 5 (Linux, this host) + ESP32 DEVKIT V1 (USB-connected to RPi at `/dev/ttyUSB0` or `/dev/ttyACM0`)
- **Motion:** 2× NEMA 17 stepper motors driven by 2× TMC2209 in differential drive layout + 1 caster wheel
- **Feedback:** 2× AS5600 magnetic encoders (fixed I2C addr 0x36) behind a TCA9548A I2C multiplexer (addr 0x70) on ESP32's I2C bus
- **Sensors:** 2× HC-SR04 ultrasonic (front + rear, ESP32 GPIO with voltage divider on Echo), 1× USB webcam (RPi USB)
- **Power:** 12V battery pack → TMC2209 VM; 5V via LM2596 → RPi; 3.3V from ESP32 → TMC VIO and AS5600 VDD
- **Manual override:** Bluetooth gamepad paired to RPi
- **GPIO/pin assignments:** documented in `config/hardware_pins.yaml` (you will create/maintain this file as the single source of truth)

You must **never assume hardware behavior without testing**. If you cannot test (e.g., user is not physically present to observe the robot), write the test, mark it as "requires hardware verification" in code, and ask the user to run it.

## Architecture (six layers, build in order)

```
Layer 6: UI & Logging (Flask + WebSocket + matplotlib)
Layer 5: Navigation State Machine (planner ↔ executor ↔ recovery)
Layer 4: Localization & Sensor Fusion (wheel odometry + AprilTag → EKF)
Layer 3: Algorithms (Occupancy Grid, A*, Path Smoothing, Pure Pursuit)
Layer 2: RPi I/O (UART comms to ESP32, Camera, AprilTag detection)
Layer 1: ESP32 Firmware (MotorDriver, EncoderReader, Ultrasonic, PID, UART protocol)
```

Each layer is built and **fully tested** before the next layer starts. Lower layers expose stable APIs; higher layers consume them. No skipping ahead.

## Repository Layout (create on first run)

```
auto-project/
├── CLAUDE.md                    # this file
├── README.md                    # human-facing project overview
├── .gitignore
├── docs/
│   ├── setup.md                 # how to bring up dev env
│   ├── uart_protocol.md         # ESP32 ↔ RPi protocol spec
│   ├── architecture.md          # diagrams + design decisions
│   └── figures/                 # diagrams, plots
├── firmware/                    # ESP32 PlatformIO project
│   ├── platformio.ini
│   ├── src/
│   ├── lib/
│   └── test/
├── rpi/                         # Python codebase
│   ├── pyproject.toml
│   ├── requirements.txt
│   ├── src/autoproject/
│   │   ├── comms/
│   │   ├── vision/
│   │   ├── algorithms/
│   │   ├── localization/
│   │   ├── navigation/
│   │   ├── ui/
│   │   └── utils/
│   └── tests/
├── config/
│   ├── hardware_pins.yaml
│   ├── robot_params.yaml        # wheelbase, wheel radius, gear ratio, etc.
│   ├── camera_calibration.yaml
│   └── world_tags.yaml          # AprilTag world coordinates
├── tools/
│   ├── calibrate_camera.py
│   ├── plot_logs.py
│   └── simulate_run.py
└── PROJECT_BOOK/                # Hebrew project book, Markdown source
    ├── 00_abstract.md
    ├── 01_introduction.md
    ├── 02_problem.md
    ├── 03_background.md
    ├── 04_system_design.md
    ├── 05_architecture.md
    ├── 06_algorithms.md
    ├── 07_experiments.md
    ├── 08_discussion.md
    ├── 09_future_work.md
    ├── 10_conclusion.md
    ├── 11_references.md
    └── appendix/
```

## Coding Standards

### Python (RPi side)
- **Style:** Black-formatted, 100-char lines, type hints on every public function
- **Linter:** `ruff` with strict config; CI must pass before push
- **Testing:** `pytest` + `pytest-cov`; minimum 70% coverage on algorithms module, 50% elsewhere
- **Structure:** Each module exposes a clear class or function set; `__init__.py` re-exports the public API
- **Logging:** `logging` module, structured; never `print` in production code
- **Configuration:** YAML files in `config/`, loaded once at startup, validated with `pydantic`
- **No magic numbers:** Every constant has a name and a comment explaining its source (datasheet page, measurement, etc.)

### C++ (ESP32 firmware)
- **Style:** Google C++ Style (4-space indent), `clang-format` config in repo
- **Headers:** include guards `#pragma once`; one class per `.h`/`.cpp` pair
- **No `delay()`:** Use non-blocking timing via `millis()` or hardware timers
- **No dynamic allocation** in hot paths (motor ISR, encoder reading)
- **Comment intent, not mechanics:** "// Read AS5600 raw angle register" not "// read i2c"

### Docstrings & Comments
- Public APIs: docstrings in English, with example usage
- Inline reasoning comments: bilingual OK (Hebrew comments are welcome when explaining tricky decisions, since the project book is Hebrew)
- Every non-obvious algorithmic choice gets a comment with a one-line rationale

## Git Workflow (CRITICAL — follow strictly)

You are responsible for git hygiene. The user expects to see a clean, well-organized commit history that tells the story of the project's development.

### Branch strategy
- `main` is always working and tested
- For each phase, create a branch: `phase-N-short-description` (e.g., `phase-1-esp32-firmware`)
- Sub-tasks within a phase get individual commits, not branches
- Merge to `main` via fast-forward or `--no-ff` merge with summary message when phase is complete and acceptance criteria pass

### Commit rules
- **Conventional Commits format:**
  - `feat(scope): subject` — new feature
  - `fix(scope): subject` — bug fix
  - `refactor(scope): subject`
  - `test(scope): subject`
  - `docs(scope): subject`
  - `chore(scope): subject`
- Subject: imperative mood, ≤72 chars, no period
- Body: explain *why*, not what; reference issues if any
- Scopes: `firmware`, `comms`, `vision`, `algorithms`, `localization`, `navigation`, `ui`, `docs`, `book`, `tools`, `ci`, `config`

### Push policy
**Push after every commit that compiles and passes the relevant tests.** Specifically:
1. After completing any sub-task with acceptance criteria met → commit + push
2. After fixing a bug confirmed by a test → commit + push
3. After writing/updating documentation that completes a section → commit + push
4. At the end of each working session → ensure all WIP is either committed and pushed, or stashed with a descriptive name
5. **Never push broken code to `main`.** If you must commit broken code (e.g., to hand off mid-task), push to the phase branch with `wip:` prefix in the subject

### Tags
- Tag each completed phase: `v0.1-phase1`, `v0.2-phase2`, etc.
- Tag the demo-ready version: `v1.0-submission`
- Use annotated tags with summary: `git tag -a v0.1-phase1 -m "ESP32 firmware complete: motors + encoders + ultrasonics + UART"`

### Before every push
Run this sequence:
```bash
# Python side
cd rpi && source .venv/bin/activate
ruff check src/ tests/
pytest tests/ -q
cd ..

# Firmware side (compile only; flashing requires user)
cd firmware && pio run
cd ..
```
If anything fails, fix it first. If genuinely blocked, push to a `wip:` branch and ask the user.

## Phase Plan

Each phase has: **goal**, **tasks**, **acceptance criteria**, **project book update**.

### Phase 0 — Project Bootstrap
**Goal:** Working skeleton with CI, dependencies, and an initial `README.md`.

Tasks:
1. Create directory structure as specified above
2. Initialize `rpi/.venv`, write `requirements.txt` (numpy, scipy, opencv-python, pupil-apriltags, pyserial, pydantic, pyyaml, flask, flask-socketio, pytest, ruff, black, filterpy, matplotlib)
3. Create `firmware/platformio.ini` for `esp32dev` board with Arduino framework
4. Write `.gitignore` (Python, PlatformIO, VS Code, OS junk, log files, `.venv/`, build artifacts)
5. Set up GitHub Actions CI: lint + test on push (`.github/workflows/ci.yml`)
6. Stub `config/*.yaml` files with sensible defaults and inline comments
7. Write initial `README.md` with project description, quick-start, and current status
8. Initialize `PROJECT_BOOK/` with chapter shells (heading + 1-line outline per chapter)

Acceptance:
- `pio run` (in firmware/) compiles an empty `app_main` successfully
- `pytest` (in rpi/) passes with zero tests collected
- `ruff check` passes
- CI green on push

Book update: write `00_abstract.md` (≤1 page) and `01_introduction.md` outline.

Tag: `v0.0-bootstrap`

---

### Phase 1 — ESP32 Firmware
**Goal:** Standalone ESP32 firmware controlling motors, reading encoders, reading ultrasonics, communicating over UART. RPi not involved yet.

Tasks (in order):
1. `UartProtocol` class: line-based text protocol (spec in `docs/uart_protocol.md`). Implement command parsing and telemetry emission scaffolding.
2. `MotorDriver` class using ESP32 RMT peripheral for STEP pulse generation (not `delayMicroseconds`, not AccelStepper). Support: setSpeed, moveSteps, trapezoidal acceleration ramp, EN pin shared between drivers, MS1/MS2 microstepping configuration.
3. `TCA9548A` I2C multiplexer driver: `selectChannel(uint8_t ch)`.
4. `AS5600Encoder` class: reads RAW_ANGLE register (0x0C-0x0D) via multiplexer, handles wrap-around correctly across the 4095↔0 boundary, computes angular velocity with low-pass filter (α=0.2 EMA).
5. `UltrasonicSensor` class: interrupt-driven on Echo pin, 5-sample moving average, timeout fallback to MAX_DISTANCE on no echo within 30ms.
6. `ClosedLoopController` class: PID at 200Hz, computes correction. Critical interpretation: with steppers, the PID's primary role is **slip detection** (large persistent error → emit `ERR SLIP` telemetry). Document this design choice in code and book.
7. `main.cpp`: cooperative scheduler — UART parsing at 1kHz, telemetry emission at 20Hz, control loop at 200Hz, sensor read at 50Hz. No `delay()` anywhere.
8. Unit tests under `firmware/test/` using PlatformIO's Unity framework where feasible (host-side tests for protocol parsing, etc.)

Acceptance:
- All classes compile, link, and flash
- `screen /dev/ttyUSB0 115200` shows telemetry stream
- Sending `CMD MOVE 1600 1600 800\n` rotates both motors one full revolution (verify visually, then by encoder feedback)
- Encoder reports ~4096 ticks per motor revolution (within ±5%)
- Pulling cable on a motor while moving produces `ERR SLIP` within 1 second

Book update: `03_background.md` sections on stepper motors, TMC2209, AS5600, PID + slip detection rationale. `04_system_design.md` component selection justifications.

Tag: `v0.1-phase1`

---

### Phase 2 — RPi I/O Foundation
**Goal:** RPi can talk to ESP32 over UART, capture camera frames, detect AprilTags, and produce calibrated pose estimates.

Tasks:
1. `comms/robot_comms.py`: `RobotComms` class with background reader thread, thread-safe state, sync API (`move`, `stop`, `get_telemetry`), and async callbacks (`on_obstacle`, `on_slip`, `on_done`). Auto-reconnect on UART drop.
2. `vision/camera_capture.py`: `Camera` class wrapping `cv2.VideoCapture`. Disables auto-exposure, locks resolution to 640×480, exposes `get_frame()`.
3. `tools/calibrate_camera.py`: ChArUco-based calibration script. Captures 20 frames, computes intrinsics, writes `config/camera_calibration.yaml`.
4. `vision/apriltag_detector.py`: `AprilTagDetector` class using `pupil-apriltags`, family `tag36h11`. Loads calibration. Returns list of detections with global pose computed from `config/world_tags.yaml`. Handles multi-tag frames with distance-weighted pose averaging.
5. Comprehensive tests with mocked serial (`pytest` + `pytest-mock`) and pre-recorded video frames stored in `rpi/tests/fixtures/`

Acceptance:
- `python -m autoproject.comms.robot_comms` opens connection and prints live telemetry
- Camera captures and shows a frame (saved to `/tmp/capture.jpg`)
- `python -m autoproject.vision.apriltag_detector` shows detected tags with pose, visualized with `cv2.imshow` (or saved to disk if headless)
- Pose accuracy: place tag at 1m, 0° → measured distance within ±2cm, angle within ±2°

Book update: `03_background.md` sections on perspective-n-point (PnP) and camera calibration. `06_algorithms.md` AprilTag pose computation derivation.

Tag: `v0.2-phase2`

---

### Phase 3 — Algorithms (Offline)
**Goal:** All planning algorithms work in pure simulation, no hardware needed.

Tasks:
1. `algorithms/occupancy_grid.py`: `OccupancyGrid` class. Methods: `from_obstacles(rects)`, `inflate(radius)`, `is_free(x, y)`, `to_image()` for visualization.
2. `algorithms/astar.py`: A* with 8-connectivity, Euclidean heuristic, `heapq`-based open set. Prove admissibility in docstring with derivation comment. Returns list of world coordinates.
3. `algorithms/path_smoother.py`: line-of-sight smoother using Bresenham on the inflated grid.
4. `algorithms/pure_pursuit.py`: PurePursuit controller. Given a path and current pose, returns `(v_left, v_right)` in steps/sec.
5. `tools/simulate_run.py`: pure-Python simulator that takes a map, start, goal, runs planner, simulates differential drive forward (no hardware), produces a matplotlib plot of planned vs. simulated path.
6. Tests: 5+ test maps including adversarial cases (no path, narrow corridor, U-shape requiring inflation correctness)

Acceptance:
- A* on 40×40 grid with 10 obstacles solves in <100ms
- Smoothing reduces waypoint count by ≥40% on random maps
- Simulated robot reaches goal within `cell_size` tolerance in 95% of test scenarios

Book update: `06_algorithms.md` A* full treatment including admissibility proof, complexity analysis, smoothing rationale with before/after waypoint count benchmark.

Tag: `v0.3-phase3`

---

### Phase 4 — Localization & Sensor Fusion
**Goal:** Robust pose tracking combining wheel odometry and AprilTag observations.

Tasks:
1. `localization/wheel_odometry.py`: `WheelOdometry` class using the arc model (differential drive midpoint integration). Subscribes to RobotComms telemetry. Document derivation in docstring + book.
2. `localization/pose_fusion.py`: **Implement both** complementary filter AND EKF (using `filterpy.kalman.ExtendedKalmanFilter`). State: `[x, y, θ]`. Prediction step from odometry, measurement update from AprilTag. Configurable via YAML.
3. Benchmarking script `tools/benchmark_fusion.py`: runs both methods on identical telemetry traces (record actual traces during Phase 2 and save as test fixtures), produces comparison plots.
4. Tests with synthetic odometry + tag observations to verify filter convergence and AprilTag-loss recovery.

Acceptance:
- Wheel odometry over a 5m straight run shows <5% positional error in pure simulation
- EKF reduces RMSE vs odometry-only by ≥40% in scenarios with intermittent AprilTag visibility
- Filter state remains bounded when AprilTags are absent for 10 seconds

Book update: `03_background.md` EKF mathematical foundations. `06_algorithms.md` fusion implementation. `07_experiments.md` benchmark plot of complementary vs EKF (this is one of the headline experiments).

Tag: `v0.4-phase4`

---

### Phase 5 — Navigation Stack
**Goal:** Full closed-loop navigation. Robot receives goal, plans, executes, recovers from obstacles, reaches goal.

Tasks:
1. `navigation/navigator.py`: state machine with states `IDLE / PLANNING / EXECUTING / AVOIDING / RELOCALIZING / EMERGENCY`. Transitions documented in `docs/architecture.md` with a state diagram (PlantUML or Mermaid source committed).
2. Reactive obstacle layer: ultrasonic readings < 30cm trigger `AVOIDING` state, halt motion, replan from current fused pose.
3. Slip recovery: on `ERR SLIP` from ESP32, transition to `RELOCALIZING`, wait for next AprilTag fix, then replan.
4. Goal tolerance: declare goal reached when `||pose - goal|| < 5cm` and `|θ - θ_goal| < 5°`.
5. End-to-end integration tests that exercise the full stack with the ESP32 firmware via real UART (mark as `@pytest.mark.hardware`).

Acceptance:
- Robot navigates 5m point-to-point on a known map with 3 obstacles, 10 runs, ≥9/10 successful arrivals with <10cm error
- Dynamic obstacle (hand placed in path mid-run) triggers replan and completion in ≥7/10 runs
- No collisions in any run

Book update: `05_architecture.md` state machine. `07_experiments.md` end-to-end navigation results.

Tag: `v0.5-phase5`

---

### Phase 6 — UI, Logging, and Manual Override
**Goal:** Polish, observability, and demo readiness.

Tasks:
1. `ui/app.py`: Flask + Flask-SocketIO. Routes: `/` (map input form), `/live` (live visualization), `/logs` (past runs).
2. SVG-based live map renderer (browser-side, JS) that subscribes via WebSocket to pose updates.
3. `utils/logger.py`: structured JSONL logger. Every run gets `runs/YYYYMMDD_HHMMSS/run.jsonl` + auto-generated summary plots after run end.
4. Bluetooth gamepad integration via `python-evdev` or `pygame`: hold a button to enter manual mode, sticks map to wheel velocities. Manual mode bypasses planner but still respects ultrasonic emergency stop.
5. `tools/plot_logs.py`: post-hoc analysis script that produces all the figures needed for the book's experiments chapter.

Acceptance:
- UI shows live robot position with <500ms latency
- Past run can be replayed from log file
- Gamepad override works and emergency stop is still active
- 10 logged runs produce reproducible plots

Book update: `07_experiments.md` final figures. `08_discussion.md` honest limitations section.

Tag: `v1.0-submission`

---

## How to Handle Hardware-Required Tests

When a task requires running on the physical robot:
1. Write the test/script fully
2. Mark it with `@pytest.mark.hardware` (Python) or note in firmware test
3. Add to `docs/hardware_tests.md` a numbered list with exact steps the user needs to run
4. Tell the user: "Phase X step Y is ready. Please run `<exact command>` and paste the output so I can verify."
5. Wait for the result before declaring the task done

## When You Are Stuck

In order of preference:
1. **Read the datasheet/docs.** Hallucinations about pin behavior, register layouts, or library APIs are the #1 source of bugs. Cite the page/section in a code comment when you do.
2. **Write a minimal reproduction** and run it
3. **Ask the user.** Be specific: state what you tried, what you expected, what happened. Do not silently pick an arbitrary direction.
4. **Never fake test results.** If something can't be verified yet (e.g., requires hardware you can't access), say so explicitly.

## Project Book Writing Rules

The book will be evaluated against last year's APME book (which got criticized for being "too clean — no engineering scars"). Avoid that failure mode:

- **Document failures.** When a sub-task surfaces a bug, real measurement that contradicts expectation, or design pivot, write a short paragraph in the relevant chapter under a heading "Debugging note" or "Design pivot."
- **Show your math.** Derive odometry, prove A* admissibility, write out the EKF equations. Don't paraphrase Wikipedia.
- **Quote real numbers.** Every claim about behavior comes with a measurement. "The robot navigates accurately" → "On 10 runs of a 5m straight path, mean final position error was 4.2cm (std 1.1cm)."
- **State limitations explicitly.** Chapter 8 must be honest.
- **Write in Hebrew.** Code comments and identifiers in English, but the book itself is in Hebrew. Mathematical formulas are universal.
- **One commit per book section.** Use `docs(book): section X.Y title`.

## Communication Style With the User

The user is a strong student but a student — explain non-obvious choices briefly. Reply in Hebrew when the user writes in Hebrew. When showing diffs or output, English is fine. When asking for hardware tests, give exact commands the user can copy-paste.

## First Action

When you receive your first message in this project, do all of the following before asking anything:
1. Verify you are in the repo root (the directory containing this file)
2. Run `git status` and `git log --oneline -5` to understand current state
3. Check Python and PlatformIO are available (`python3 --version`, `pio --version`)
4. Report what you found, then propose: "Starting Phase 0 (bootstrap)" or "Resuming Phase N based on the repo state"
5. Wait for user confirmation before pushing any commits

Begin.
