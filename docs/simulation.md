# Simulation Core (Layer 0)

The simulator is the honest oracle: it holds ground truth and produces what
hardware would. Tests assert that the system's *estimated* state converges toward
the simulator's *ground truth* under modeled noise.

> Skeleton — fleshed out as Layer 0 is implemented (early, before Phase 2 needs
> it).

## `autoproject.simulation.world.World`

Holds:

- **Map** — 2D occupancy grid with rectangular obstacles.
- **Robot ground truth** — `pose = (x, y, θ)`, advanced by differential-drive
  kinematics from commanded wheel velocities.
- **AprilTag world map** — `tag_id → (x, y, θ)` placements.
- **Time** — simulated, advanced in fixed steps (50 Hz physics tick).
- **Noise models (configurable)** — Gaussian noise on encoder reads, multi-path
  noise on ultrasonics, motion blur / dropout on camera frames, wheel slip
  probability.

## `Sim*` components query the world

- **`SimRobotComms`** — reads commanded velocities, advances ground-truth pose,
  returns simulated encoder ticks (with slip), emits ultrasonic readings from
  robot pose vs obstacles (ray-casting).
- **`SimCamera`** — renders a synthetic frame containing visible AprilTags, or
  returns pre-rendered fixtures.
- **`SimAprilTagDetector`** — consumes `SimCamera` frames, or (default, for
  speed) emits synthetic detections directly with configurable noise.

## Noise model parameters

Configured per scenario; defaults documented here once Layer 0 lands.
