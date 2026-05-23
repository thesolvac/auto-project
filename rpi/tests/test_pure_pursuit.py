"""Tests for the pure-pursuit follower."""

import math

import pytest

from autoproject.algorithms.pure_pursuit import PurePursuit


def test_straight_path_drives_forward_evenly():
    pp = PurePursuit(lookahead_m=0.5, wheelbase_m=0.15, cruise_speed_mps=0.3)
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    v_left, v_right = pp.compute((0.0, 0.0, 0.0), path)
    assert v_left == pytest.approx(v_right, abs=1e-6)  # no steering on a straight line
    assert v_left > 0.0


def test_goal_reached_stops():
    pp = PurePursuit(0.5, 0.15, 0.3, goal_tolerance_m=0.05)
    path = [(0.0, 0.0), (1.0, 0.0)]
    assert pp.compute((1.0, 0.0, 0.0), path) == (0.0, 0.0)


def test_left_turn_spins_right_wheel_faster():
    pp = PurePursuit(lookahead_m=0.5, wheelbase_m=0.15, cruise_speed_mps=0.3)
    # Target up and to the left of a robot facing +x -> turn left -> right wheel faster.
    path = [(0.0, 0.0), (0.3, 0.4)]
    v_left, v_right = pp.compute((0.0, 0.0, 0.0), path)
    assert v_right > v_left


def test_empty_path_stops():
    pp = PurePursuit(0.5, 0.15, 0.3)
    assert pp.compute((0.0, 0.0, 0.0), []) == (0.0, 0.0)


def test_from_config_uses_robot_params():
    pp = PurePursuit.from_config(lookahead_m=0.4)
    assert pp.wheelbase_m == pytest.approx(0.15)  # config/robot_params.yaml
    assert pp.cruise_speed_mps == pytest.approx(0.30)
    assert pp.lookahead_m == pytest.approx(0.4)


def test_omega_sign_matches_geometry():
    # Robot facing +y, target to its left (-x world) -> should steer left.
    pp = PurePursuit(0.5, 0.15, 0.3)
    path = [(0.0, 0.0), (-0.3, 0.4)]
    v_left, v_right = pp.compute((0.0, 0.0, math.pi / 2), path)
    assert v_right > v_left
