"""Tests for pose fusion: EKF vs odometry RMSE, blackout bounding, recovery."""

import math

import pytest

from autoproject.comms.sim_comms import SimRobotComms
from autoproject.localization.pose_fusion import (
    ComplementaryFilter,
    EKFFusion,
    PoseFilter,
    make_pose_filter,
)
from autoproject.simulation.geometry import Pose
from autoproject.simulation.noise import NoiseConfig
from autoproject.simulation.world import World
from autoproject.vision.sim_detector import SimAprilTagDetector

_METRES_PER_COUNT = (2.0 * math.pi * 0.040) / 4096
_START = (0.3, 1.0, 0.0)


def _world(noise: NoiseConfig | None = None) -> World:
    # Corridor with tags inside the bounds: one on the far wall ahead and one on
    # the top wall, both within a wide FOV with clear line of sight, so the filter
    # gets range/bearing fixes (for x and y) as the robot drives +x.
    return World(
        width_m=6.5,
        height_m=2.0,
        obstacles=[],
        tags={0: Pose(6.3, 1.0, math.pi), 1: Pose(3.0, 1.9, math.pi)},
        robot_pose=Pose(*_START),
        wheelbase_m=0.15,
        dt_s=0.1,
        noise=noise,
    )


def _run(filt: PoseFilter, *, world: World, steps: int, update_every: int) -> tuple[float, float]:
    """Drive straight, feed odometry to the filter, fuse tag sightings.

    Returns ``(position_rmse, final_position_error)``. ``update_every <= 0``
    disables tag updates (pure odometry baseline).
    """
    comms = SimRobotComms(world)
    detector = SimAprilTagDetector(world, fov_rad=math.radians(170), max_range_m=30.0)
    tag_xy = {tid: (p.x, p.y) for tid, p in world.tags.items()}
    prev_l = prev_r = None
    sq_sum = 0.0
    final_error = 0.0
    comms.move(0.2, 0.2)
    for k in range(steps):
        tel = comms.step()
        if prev_l is None:
            prev_l, prev_r = tel.enc_left_counts, tel.enc_right_counts
        d_left = (tel.enc_left_counts - prev_l) * _METRES_PER_COUNT
        d_right = (tel.enc_right_counts - prev_r) * _METRES_PER_COUNT
        prev_l, prev_r = tel.enc_left_counts, tel.enc_right_counts

        filt.predict(d_left, d_right)
        if update_every > 0 and k % update_every == 0:
            for det in detector.detect():
                filt.update(tag_xy[det.tag_id], det.range_m, det.bearing_rad)

        truth = world.pose
        est = filt.pose
        final_error = math.hypot(est[0] - truth.x, est[1] - truth.y)
        sq_sum += final_error**2
    return math.sqrt(sq_sum / steps), final_error


def test_predict_only_matches_odometry():
    ekf = EKFFusion(0.15, initial_pose=_START)
    ekf.predict(0.1, 0.1)
    assert ekf.pose[0] == pytest.approx(0.4)
    assert ekf.pose[1] == pytest.approx(1.0)


def test_ekf_update_pulls_estimate_toward_truth():
    # Estimate starts 0.3 m short in x; one fix to a tag dead ahead should help.
    ekf = EKFFusion(0.15, initial_pose=(0.0, 1.0, 0.0))
    before = ekf.pose[0]
    for _ in range(5):
        ekf.update((20.0, 1.0), range_m=20.0 - 0.3, bearing_rad=0.0)  # truth x = 0.3
    assert ekf.pose[0] > before
    assert ekf.pose[0] == pytest.approx(0.3, abs=0.1)


def test_ekf_beats_odometry_under_slip_intermittent_tags():
    slip = NoiseConfig(seed=5, wheel_slip_prob=0.3, wheel_slip_factor=0.0)
    odom_rmse, _ = _run(
        EKFFusion(0.15, initial_pose=_START), world=_world(slip), steps=250, update_every=0
    )
    ekf_rmse, _ = _run(
        EKFFusion(0.15, initial_pose=_START), world=_world(slip), steps=250, update_every=5
    )
    assert ekf_rmse < 0.6 * odom_rmse  # >= 40% RMSE reduction


def test_ekf_state_bounded_during_long_blackout():
    ekf = EKFFusion(0.15, initial_pose=_START)
    rmse, _ = _run(
        ekf, world=_world(NoiseConfig(seed=1, wheel_slip_prob=0.2)), steps=100, update_every=0
    )
    x, y, theta = ekf.pose
    assert math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)
    assert math.isfinite(ekf.covariance_trace)
    assert abs(x) < 50.0 and abs(y) < 50.0  # no divergence to infinity
    assert math.isfinite(rmse)


def test_recovery_after_blackout():
    # Deterministic recovery: odometry over-reports during a blackout (estimate
    # ends 0.5 m ahead of the true x=1.0), then re-acquiring a known tag pulls the
    # estimate back. Avoids the EKF divergence that huge differential-slip drift
    # against a distant landmark would (legitimately) cause.
    ekf = EKFFusion(0.15, initial_pose=(0.0, 0.0, 0.0))
    ekf.predict(1.5, 1.5)  # thinks it moved 1.5 m; truth is x = 1.0
    error_before = abs(ekf.pose[0] - 1.0)
    for _ in range(10):
        ekf.update((10.0, 0.0), range_m=9.0, bearing_rad=0.0)  # tag ahead, true range 9.0
    error_after = abs(ekf.pose[0] - 1.0)
    assert error_before == pytest.approx(0.5, abs=1e-6)
    assert error_after < 0.1  # converged back near truth


def test_complementary_range_correction_reduces_error():
    # Estimate drifts 0.5 m past the truth (x=1.0); range fixes to a tag ahead
    # should pull it back via the heading-independent radial correction.
    cf = ComplementaryFilter(0.15, alpha=0.8, initial_pose=(0.0, 0.0, 0.0))
    cf.predict(1.5, 1.5)  # thinks x = 1.5
    error_before = abs(cf.pose[0] - 1.0)
    for _ in range(20):
        cf.update((10.0, 0.0), range_m=9.0, bearing_rad=0.0)  # implies x = 1.0
    error_after = abs(cf.pose[0] - 1.0)
    assert error_after < error_before
    assert error_after < 0.1


def test_make_pose_filter_selects_type():
    assert isinstance(make_pose_filter("ekf", 0.15), EKFFusion)
    assert isinstance(make_pose_filter("complementary", 0.15), ComplementaryFilter)
    with pytest.raises(ValueError, match="unknown pose filter"):
        make_pose_filter("particle", 0.15)
