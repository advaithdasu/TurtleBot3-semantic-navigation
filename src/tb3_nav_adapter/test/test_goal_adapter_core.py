#!/usr/bin/env python3
"""Unit tests for goal_adapter_core (pure math, no rclpy needed)."""

import math

import pytest

from tb3_nav_adapter.goal_adapter_core import compute_approach_pose, yaw_to_quaternion


def test_standoff_along_robot_target_ray():
    # Robot at (1, 1), target at (4, 1): goal should back off along +X.
    result = compute_approach_pose(4.0, 1.0, 1.0, 1.0,
                                   approach_distance=0.9, min_standoff=0.6)
    assert result is not None
    gx, gy, yaw = result
    assert gx == pytest.approx(3.1)
    assert gy == pytest.approx(1.0)
    assert yaw == pytest.approx(0.0)


def test_diagonal_approach_faces_target():
    rx, ry = 2.0, -1.0
    tx, ty = 5.0, 2.0
    result = compute_approach_pose(tx, ty, rx, ry,
                                   approach_distance=0.9, min_standoff=0.6)
    assert result is not None
    gx, gy, yaw = result
    expected_dir = math.atan2(ty - ry, tx - rx)
    assert yaw == pytest.approx(expected_dir)
    # Goal sits approach_distance back from the target along the ray.
    assert math.hypot(tx - gx, ty - gy) == pytest.approx(0.9)
    # Goal lies on the robot→target segment.
    assert math.atan2(ty - gy, tx - gx) == pytest.approx(expected_dir)


def test_target_near_map_origin_not_rejected():
    # Regression: min_standoff must be measured from the ROBOT, not the
    # map origin.  Target at map origin, robot 3 m away → valid goal.
    result = compute_approach_pose(0.0, 0.0, 3.0, 0.0,
                                   approach_distance=0.9, min_standoff=0.6)
    assert result is not None
    gx, gy, yaw = result
    assert gx == pytest.approx(0.9)
    assert gy == pytest.approx(0.0)
    assert yaw == pytest.approx(math.pi)


def test_target_too_close_to_robot_rejected():
    result = compute_approach_pose(1.2, 1.0, 1.0, 1.0,
                                   approach_distance=0.9, min_standoff=0.6)
    assert result is None


def test_close_target_clamps_offset_to_min_standoff():
    # Robot 1 m from target with approach 0.9: offset clamps to
    # dist - min_standoff = 0.4, goal ends up at min_standoff from target.
    result = compute_approach_pose(1.0, 0.0, 0.0, 0.0,
                                   approach_distance=0.9, min_standoff=0.6)
    assert result is not None
    gx, gy, _ = result
    assert math.hypot(1.0 - gx, 0.0 - gy) == pytest.approx(0.4)


def test_robot_at_origin_matches_body_frame_case():
    # With the robot at the origin the new signature reproduces the old
    # body-frame behaviour.
    result = compute_approach_pose(2.0, 0.0, 0.0, 0.0,
                                   approach_distance=0.5, min_standoff=0.3)
    assert result is not None
    gx, gy, yaw = result
    assert gx == pytest.approx(1.5)
    assert gy == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_yaw_to_quaternion_identity_and_halfpi():
    assert yaw_to_quaternion(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2.0)
    assert (qx, qy) == (0.0, 0.0)
    assert qz == pytest.approx(math.sin(math.pi / 4.0))
    assert qw == pytest.approx(math.cos(math.pi / 4.0))
