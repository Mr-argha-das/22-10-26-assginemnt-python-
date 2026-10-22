#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Unit tests for Pure Pursuit geometry, speed regulation, and fail-safes."""

import math
from pathlib import Path
import sys

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[2]
for p in (PKG_ROOT, REPO_ROOT / 'tools'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    ensure_ros2_environment,
)

ensure_ros2_environment(str(REPO_ROOT))

from nav_msgs.msg import Odometry, Path as PathMsg  # noqa: E402
from pure_pursuit.controller_core import (  # noqa: E402
    ControllerParams,
    compute_adaptive_lookahead,
    compute_pure_pursuit_step,
    euler_from_quaternion,
)
from pure_pursuit.pure_pursuit_node import PurePursuit  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402


def _make_straight_path(length_m: float = 20.0, v: float = 4.0):
    return [(0.2 * i, 0.0, v) for i in range(int(length_m / 0.2) + 1)]


def _make_circle_path(
    radius_m: float = 5.0, n_pts: int = 180, v: float = 3.0
):
    return [
        (
            radius_m * math.cos(2.0 * math.pi * k / n_pts),
            radius_m * math.sin(2.0 * math.pi * k / n_pts),
            v,
        )
        for k in range(n_pts)
    ]


def test_exact_ackermann_pure_pursuit_circle_and_symmetry():
    params = ControllerParams(
        wheelbase=0.3302,
        steering_gain=1.0,
        kp=0.0,
        kd=0.0,
        steering_alpha=1.0,
        max_steering_rate=100.0,
        closed_loop=True,
    )
    radius = 5.0
    circle = _make_circle_path(radius_m=radius, n_pts=360, v=2.5)
    # Vehicle tangent at (R, 0) pointing +Y (yaw = +pi/2) -> positive CCW steer
    out, _, _ = compute_pure_pursuit_step(
        path=circle,
        x=radius,
        y=0.0,
        yaw=math.pi / 2.0,
        current_speed=2.5,
        dt=0.01,
        params=params,
    )
    expected_delta = math.atan(params.wheelbase / radius)
    assert abs(out.steering_angle - expected_delta) < 2e-3
    assert not out.is_safe_stop

    # Straight path lateral offset symmetry: y = -0.3 -> +steer, +0.3 -> -steer
    straight = _make_straight_path()
    params_open = ControllerParams(
        steering_gain=1.0, kd=0.0, steering_alpha=1.0, closed_loop=False
    )
    left_corr, _, _ = compute_pure_pursuit_step(
        straight, 1.0, -0.30, 0.0, 2.5, 0.01, params_open
    )
    right_corr, _, _ = compute_pure_pursuit_step(
        straight, 1.0, 0.30, 0.0, 2.5, 0.01, params_open
    )
    assert left_corr.steering_angle > 0.0
    assert right_corr.steering_angle < 0.0
    assert abs(left_corr.steering_angle + right_corr.steering_angle) < 1e-6


def test_lookahead_scaling_and_curvature_speed_reduction():
    l_low = compute_adaptive_lookahead(0.5, 0.75, 2.0, 1.0, 5.0, 0.28)
    l_mid = compute_adaptive_lookahead(3.0, 0.75, 2.0, 1.0, 5.0, 0.28)
    l_high = compute_adaptive_lookahead(9.0, 0.75, 2.0, 1.0, 5.0, 0.28)
    assert abs(l_low - 0.89) < 1e-6
    assert l_low < l_mid < l_high
    assert abs(l_high - 2.0) < 1e-6

    params = ControllerParams(
        a_lat_max=3.0, max_velocity=6.0, closed_loop=True
    )
    tight_circle = _make_circle_path(radius_m=1.5, n_pts=180, v=6.0)
    out_tight, _, _ = compute_pure_pursuit_step(
        tight_circle, 1.5, 0.0, math.pi / 2.0, 4.0, 0.05, params
    )
    # Must regulate below 3.0 m/s since sqrt(a_lat_max * R) = sqrt(4.5) ~= 2.12
    assert out_tight.speed < 3.0


def test_steering_saturation_rate_limit_and_safety_stops():
    params = ControllerParams(
        max_steering_angle=0.4189,
        max_steering_rate=3.2,
        steering_alpha=1.0,
        closed_loop=False,
    )
    straight = _make_straight_path()
    # Large step limited by max_steering_rate * dt = 3.2 * 0.01 = 0.032 rad
    step1, _, _ = compute_pure_pursuit_step(
        straight, 0.5, -1.5, 0.0, 2.0, 0.01, params, prev_steer=0.0
    )
    assert abs(step1.steering_angle) <= 0.032 + 1e-6

    # Paused / empty path / NaN state / open-loop goal reached -> zero speed
    paused_out, _, _ = compute_pure_pursuit_step(
        straight, 1.0, 0.0, 0.0, 3.0, 0.01, params, paused=True
    )
    assert paused_out.speed == 0.0 and paused_out.is_safe_stop

    empty_out, _, _ = compute_pure_pursuit_step(
        [], 1.0, 0.0, 0.0, 3.0, 0.01, params
    )
    assert empty_out.speed == 0.0 and empty_out.is_safe_stop

    nan_out, _, _ = compute_pure_pursuit_step(
        straight, float('nan'), 0.0, 0.0, 3.0, 0.01, params
    )
    assert nan_out.speed == 0.0 and nan_out.is_safe_stop

    goal_out, _, _ = compute_pure_pursuit_step(
        straight, 19.9, 0.0, 0.0, 2.0, 0.01, params
    )
    assert goal_out.goal_reached and goal_out.speed == 0.0

    # Quaternion domain safety clamp
    r, p, y = euler_from_quaternion(
        [0.0, 0.0, math.sin(0.4), math.cos(0.4)]
    )
    assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y - 0.8) < 1e-6


def test_pure_pursuit_ros_node_stale_odometry_and_pause():
    from ros2_sim_harness.ros2_compat import GLOBAL_BUS

    GLOBAL_BUS.reset()
    node = PurePursuit()
    # Publish empty path over real ROS 2 topic subscription -> safe stop
    GLOBAL_BUS.publish('/pp_path', PathMsg())
    GLOBAL_BUS.step_by_ns(20_000_000)
    assert GLOBAL_BUS.last_msg['/drive'].drive.speed == 0.0

    # Publish valid odom at t = 0.02s, then advance > odom_timeout (1.0 s)
    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.twist.twist.linear.x = 2.0
    GLOBAL_BUS.publish('/ego_racecar/odom', odom)
    GLOBAL_BUS.step_by_ns(1_000_000_000)
    assert GLOBAL_BUS.last_msg['/drive'].drive.speed == 0.0

    p_msg = Bool()
    p_msg.data = True
    GLOBAL_BUS.publish('/pause', p_msg)
    assert node.stop is True
    node.destroy_node()
