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

## Implemented API (Layer 0)

The core is pure standard-library Python (no numpy/hardware), deterministic given
a seed. Modules under `autoproject.simulation`:

- `geometry` — `Pose(x, y, theta)`, `Rectangle`, `normalize_angle`, and
  `ray_box_intersection` (slab method) — the basis of the ray-caster.
- `kinematics` — `diff_drive_step(pose, v_left, v_right, wheelbase, dt)`: exact
  arc integration of the unicycle model (straight-line limit handled separately).
- `noise` — `NoiseConfig` (pydantic): slip + sensor-noise parameters.
- `world` — `World`, the ground-truth simulator.

Key `World` methods:

| Method | Purpose |
|---|---|
| `step(v_left, v_right)` | advance ground truth one tick; applies slip; latches `collided` |
| `raycast(x, y, angle, max_range)` | nearest obstacle/wall distance along a ray |
| `ultrasonic_reading(sensor_pose, max_range)` | noise-free ultrasonic return |
| `visible_tags(camera_pose, fov_rad, max_range)` | tags in range + FOV + line of sight |
| `in_collision(x, y)` / `is_free(x, y)` | footprint-circle collision test |
| `from_scenario(path)` | build from `config/sim_scenarios/*.yaml` + robot_params + world_tags |
| `pose`, `time_s`, `step_count` | read-only ground-truth views |

## Noise model parameters

`NoiseConfig` — all default to zero, so an unconfigured world is fully
deterministic. The RNG is seeded by `seed`.

| Field | Applied by | Effect |
|---|---|---|
| `wheel_slip_prob`, `wheel_slip_factor` | `World.step` (ground truth) | per wheel/tick, realized velocity is scaled by the factor with the given probability |
| `encoder_sigma_rad` | `SimRobotComms` (Phase 2) | Gaussian noise on encoder angle |
| `ultrasonic_sigma_m`, `ultrasonic_dropout_prob` | `SimRobotComms` (Phase 2) | multi-path noise / dropout on range |
| `camera_dropout_prob` | `SimCamera` (Phase 2) | dropped frames |

Wheel slip affects ground truth directly (the robot moves less than commanded),
so the system's odometry-based estimate diverges from truth — exactly the
condition the EKF (Phase 4) and slip recovery (Phase 5) must handle.
