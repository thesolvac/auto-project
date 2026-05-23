"""Sensor-fusion benchmark: odometry-only vs complementary filter vs EKF.

Drives the simulator on identical scenarios with each estimator, computes the
position RMSE vs ground truth, and writes a grouped bar chart. Offline analysis;
not run in CI.

Run with:  python tools/benchmark_fusion.py [--out runs/fusion_benchmark.png]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rpi" / "src"))

from autoproject.localization.pose_fusion import (  # noqa: E402
    ComplementaryFilter,
    EKFFusion,
    PoseFilter,
)
from autoproject.comms.sim_comms import SimRobotComms  # noqa: E402
from autoproject.simulation.geometry import Pose  # noqa: E402
from autoproject.simulation.noise import NoiseConfig  # noqa: E402
from autoproject.simulation.world import World  # noqa: E402
from autoproject.vision.sim_detector import SimAprilTagDetector  # noqa: E402

_METRES_PER_COUNT = (2.0 * math.pi * 0.040) / 4096
_START = (0.3, 1.0, 0.0)
_STEPS = 200

SCENARIOS = ["continuous", "intermittent", "blackout_5s", "high_slip"]
# Partial slip (factor 0.5) keeps the heading drift bounded so the robot stays
# pointed down the corridor and the tags remain within the camera FOV; the slip
# still injects the odometry error the fusion is meant to correct.
NOISES = {
    "continuous": NoiseConfig(seed=1, wheel_slip_prob=0.15, wheel_slip_factor=0.5),
    "intermittent": NoiseConfig(seed=2, wheel_slip_prob=0.2, wheel_slip_factor=0.5),
    "blackout_5s": NoiseConfig(seed=3, wheel_slip_prob=0.2, wheel_slip_factor=0.5),
    "high_slip": NoiseConfig(seed=4, wheel_slip_prob=0.4, wheel_slip_factor=0.5),
}


def _world(noise: NoiseConfig) -> World:
    return World(
        width_m=6.5,
        height_m=2.0,
        obstacles=[],
        # Tags inside the bounds (far wall ahead + top wall) give range + bearing
        # for both x and y with clear line of sight throughout the drive.
        tags={0: Pose(6.3, 1.0, math.pi), 1: Pose(3.0, 1.9, math.pi)},
        robot_pose=Pose(*_START),
        wheelbase_m=0.15,
        dt_s=0.1,
        noise=noise,
    )


def _tags_visible(scenario: str, k: int) -> bool:
    if scenario == "intermittent":
        return k % 2 == 0
    if scenario == "blackout_5s":
        return not (100 <= k < 150)  # 5 s blackout (50 ticks @ dt 0.1 s) mid-run
    if scenario == "high_slip":
        return k % 5 == 0
    return True  # continuous


def _rmse(
    filt: PoseFilter, scenario: str, noise: NoiseConfig, *, use_updates: bool
) -> float:
    world = _world(noise)
    comms = SimRobotComms(world)
    detector = SimAprilTagDetector(world, fov_rad=math.radians(170), max_range_m=30.0)
    tag_xy = {tid: (p.x, p.y) for tid, p in world.tags.items()}
    prev_l = prev_r = None
    sq = 0.0
    comms.move(0.2, 0.2)
    for k in range(_STEPS):
        tel = comms.step()
        if prev_l is None:
            prev_l, prev_r = tel.enc_left_counts, tel.enc_right_counts
        d_left = (tel.enc_left_counts - prev_l) * _METRES_PER_COUNT
        d_right = (tel.enc_right_counts - prev_r) * _METRES_PER_COUNT
        prev_l, prev_r = tel.enc_left_counts, tel.enc_right_counts

        filt.predict(d_left, d_right)
        if use_updates and _tags_visible(scenario, k):
            for det in detector.detect():
                filt.update(tag_xy[det.tag_id], det.range_m, det.bearing_rad)

        est = filt.pose
        truth = world.pose
        sq += (est[0] - truth.x) ** 2 + (est[1] - truth.y) ** 2
    return math.sqrt(sq / _STEPS)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("runs/fusion_benchmark.png"))
    args = parser.parse_args()

    results: dict[str, list[float]] = {"odometry": [], "complementary": [], "ekf": []}
    for scenario in SCENARIOS:
        noise = NOISES[scenario]
        results["odometry"].append(
            _rmse(
                EKFFusion(0.15, initial_pose=_START), scenario, noise, use_updates=False
            )
        )
        results["complementary"].append(
            _rmse(
                ComplementaryFilter(0.15, initial_pose=_START),
                scenario,
                noise,
                use_updates=True,
            )
        )
        results["ekf"].append(
            _rmse(
                EKFFusion(0.15, initial_pose=_START), scenario, noise, use_updates=True
            )
        )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    x = np.arange(len(SCENARIOS))
    width = 0.25
    fig, ax = plt.subplots()
    for i, (name, vals) in enumerate(results.items()):
        ax.bar(x + (i - 1) * width, vals, width, label=name)
    ax.set_xticks(x)
    ax.set_xticklabels(SCENARIOS, rotation=15)
    ax.set_ylabel("position RMSE [m]")
    ax.set_title("Pose-fusion RMSE by scenario")
    ax.legend()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")
    for scenario in SCENARIOS:
        i = SCENARIOS.index(scenario)
        print(
            f"{scenario:14s} odom={results['odometry'][i]:.3f} "
            f"comp={results['complementary'][i]:.3f} ekf={results['ekf'][i]:.3f}"
        )


if __name__ == "__main__":
    main()
