#!/usr/bin/env python3
"""
Core Pure Pursuit geometric tracking, curvature-preview velocity regulation,
actuator saturation, and safety validation logic for F1TENTH racing.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class ControllerParams:
    """Parameters for Pure Pursuit lateral and longitudinal control."""

    wheelbase: float = 0.3302
    min_lookahead_distance: float = 0.75
    max_lookahead_distance: float = 1.95
    lookahead_gain: float = 0.28
    use_linear_lad_interp: bool = True
    min_velocity: float = 1.6
    max_velocity: float = 5.0
    vel_division_factor: float = 1.0
    a_lat_max: float = 4.6
    curvature_preview_m: float = 2.6
    k_sigmoid: float = 8.0
    skidding_velocity_thresh: float = 0.35
    max_decel_step: float = 0.80
    steering_gain: float = 1.08
    kp: float = 0.0
    kd: float = 0.02
    steering_alpha: float = 0.72
    max_steering_angle: float = 0.4189
    max_steering_rate: float = 3.2
    closed_loop: bool = True
    goal_tolerance_m: float = 0.35
    reacquire_dist_m: float = 2.5


@dataclass
class ControlOutput:
    """Output of a single Pure Pursuit control step."""

    steering_angle: float
    speed: float
    lookahead_distance: float
    gamma: float
    cross_track_error_m: float
    heading_error_rad: float
    lookahead_point: Optional[Tuple[float, float, float]]
    closest_idx: int
    goal_reached: bool
    is_safe_stop: bool
    reason: str = 'ok'


def euler_from_quaternion(
    quaternion: Sequence[float],
) -> Tuple[float, float, float]:
    """
    Convert quaternion `[x, y, z, w]` to `(roll, pitch, yaw)` in radians
    with numerical clamping to avoid domain errors in `arcsin`.
    """
    x, y, z, w = (
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError(f'Invalid or zero-norm quaternion: {quaternion}')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle in radians into `[-pi, +pi)`."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def transform_to_vehicle_frame(
    point: Sequence[float], x: float, y: float, yaw: float
) -> Tuple[float, float]:
    """Transform global `(px, py, ...)` coordinates into vehicle body frame."""
    dx = float(point[0]) - float(x)
    dy = float(point[1]) - float(y)
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    lx = cos_y * dx - sin_y * dy
    ly = sin_y * dx + cos_y * dy
    return lx, ly


def compute_adaptive_lookahead(
    speed: float,
    min_lad: float,
    max_lad: float,
    min_velocity: float,
    max_velocity: float,
    lookahead_gain: Optional[float] = None,
) -> float:
    """Compute adaptive lookahead distance clamped to `[min_lad, max_lad]`."""
    lo = min(min_lad, max_lad)
    hi = max(min_lad, max_lad)
    v = max(0.0, float(speed))
    if lookahead_gain is not None and lookahead_gain > 0.0:
        lad = lo + lookahead_gain * v
    elif abs(max_velocity - min_velocity) > 1e-6:
        slope = (hi - lo) / (max_velocity - min_velocity)
        lad = lo + slope * (v - min_velocity)
    else:
        lad = lo
    return max(lo, min(hi, lad))


def _circle_segment_intersection(
    p0: Tuple[float, float, float],
    p1: Tuple[float, float, float],
    cx: float,
    cy: float,
    radius: float,
) -> Optional[Tuple[float, float, float]]:
    """Return farthest intersection on segment `[p0, p1]` at `radius`."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    fx = p0[0] - cx
    fy = p0[1] - cy

    a = dx * dx + dy * dy
    if a < 1e-12:
        return None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None

    sqrt_disc = math.sqrt(disc)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    for t in (t2, t1):
        if 0.0 <= t <= 1.0:
            ix = p0[0] + t * dx
            iy = p0[1] + t * dy
            iv = p0[2] + t * (p1[2] - p0[2])
            return (ix, iy, iv)
    return None


def select_lookahead_point(
    path: Sequence[Tuple[float, float, float]],
    x: float,
    y: float,
    lookahead_distance: float,
    last_closest_idx: Optional[int] = None,
    closed_loop: bool = True,
    search_window: int = 65,
    reacquire_dist_m: float = 2.5,
) -> Tuple[
    Optional[Tuple[float, float, float]],
    Optional[Tuple[float, float, float]],
    Optional[int],
]:
    """
    Select continuous lookahead point along path starting from closest point.

    Uses a bounded forward/backward index window when tracking continuously so
    the controller cannot jump across narrow interior walls onto an adjacent
    lane of the circuit.
    """
    n = len(path)
    if n == 0:
        return None, None, None
    if n == 1:
        return path[0], path[0], 0

    candidates: List[int]
    if last_closest_idx is not None and 0 <= last_closest_idx < n:
        back = min(12, n)
        fwd = min(search_window, n)
        if closed_loop:
            candidates = [
                (last_closest_idx - back + k) % n for k in range(back + fwd)
            ]
        else:
            lo = max(0, last_closest_idx - back)
            hi = min(n, last_closest_idx + fwd)
            candidates = list(range(lo, hi))
    else:
        candidates = list(range(n))

    closest_idx = candidates[0]
    min_dist = float('inf')
    for idx in candidates:
        dist = math.hypot(path[idx][0] - x, path[idx][1] - y)
        if dist < min_dist:
            min_dist = dist
            closest_idx = idx

    # Global fallback if vehicle was reset via /initialpose far from last index
    if min_dist > reacquire_dist_m and len(candidates) < n:
        for idx in range(n):
            dist = math.hypot(path[idx][0] - x, path[idx][1] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx

    closest_pt = path[closest_idx]
    num_search = n if closed_loop else max(0, n - 1 - closest_idx)

    for step in range(num_search):
        i0 = (closest_idx + step) % n
        i1 = (i0 + 1) % n
        interp = _circle_segment_intersection(
            path[i0], path[i1], x, y, lookahead_distance
        )
        if interp is not None:
            return interp, closest_pt, i1
        if math.hypot(path[i1][0] - x, path[i1][1] - y) >= lookahead_distance:
            return path[i1], closest_pt, i1

    if not closed_loop:
        return path[-1], closest_pt, n - 1

    return None, closest_pt, closest_idx


def estimate_preview_curvature(
    path: Sequence[Tuple[float, float, float]],
    start_idx: int,
    preview_dist_m: float,
    closed_loop: bool = True,
) -> float:
    """Compute peak discrete path curvature `[1/m]` over preview horizon."""
    n = len(path)
    if n < 3 or preview_dist_m <= 0.0:
        return 0.0

    travel = 0.0
    max_kappa = 0.0
    idx = start_idx % n
    steps = 0
    while travel < preview_dist_m and steps < n - 2:
        if not closed_loop and idx + 2 >= n:
            break
        p0 = path[idx % n]
        p1 = path[(idx + 1) % n]
        p2 = path[(idx + 2) % n]

        v1x, v1y = p1[0] - p0[0], p1[1] - p0[1]
        v2x, v2y = p2[0] - p1[0], p2[1] - p1[1]
        v3x, v3y = p2[0] - p0[0], p2[1] - p0[1]
        a = math.hypot(v1x, v1y)
        b = math.hypot(v2x, v2y)
        c = math.hypot(v3x, v3y)
        denom = a * b * c
        if denom > 1e-9:
            cross = abs(v1x * v2y - v1y * v2x)
            max_kappa = max(max_kappa, 2.0 * cross / denom)
        travel += max(a, 1e-4)
        idx += 1
        steps += 1
    return max_kappa


def compute_pure_pursuit_step(
    path: Sequence[Tuple[float, float, float]],
    x: float,
    y: float,
    yaw: float,
    current_speed: float,
    dt: float,
    params: ControllerParams,
    prev_gamma: float = 0.0,
    prev_steer: float = 0.0,
    last_closest_idx: Optional[int] = None,
    autonomous_enabled: bool = True,
    paused: bool = False,
) -> Tuple[ControlOutput, float, int]:
    """
    Execute one validated Pure Pursuit control iteration.

    Returns `(ControlOutput, new_prev_gamma, new_closest_idx)`.
    """
    if not all(math.isfinite(val) for val in (x, y, yaw, current_speed, dt)):
        return (
            ControlOutput(
                steering_angle=0.0,
                speed=0.0,
                lookahead_distance=0.0,
                gamma=0.0,
                cross_track_error_m=0.0,
                heading_error_rad=0.0,
                lookahead_point=None,
                closest_idx=0,
                goal_reached=False,
                is_safe_stop=True,
                reason='invalid_vehicle_state',
            ),
            0.0,
            last_closest_idx or 0,
        )

    min_len = 3 if params.closed_loop else 2
    if path is None or len(path) < min_len:
        return (
            ControlOutput(
                steering_angle=0.0,
                speed=0.0,
                lookahead_distance=0.0,
                gamma=0.0,
                cross_track_error_m=0.0,
                heading_error_rad=0.0,
                lookahead_point=None,
                closest_idx=0,
                goal_reached=False,
                is_safe_stop=True,
                reason='invalid_or_empty_path',
            ),
            0.0,
            0,
        )

    lad = compute_adaptive_lookahead(
        speed=current_speed,
        min_lad=params.min_lookahead_distance,
        max_lad=params.max_lookahead_distance,
        min_velocity=params.min_velocity,
        max_velocity=params.max_velocity,
        lookahead_gain=(
            None if params.use_linear_lad_interp else params.lookahead_gain
        ),
    )

    if not params.closed_loop:
        goal_dist = math.hypot(path[-1][0] - x, path[-1][1] - y)
        if goal_dist <= params.goal_tolerance_m:
            return (
                ControlOutput(
                    steering_angle=0.0,
                    speed=0.0,
                    lookahead_distance=lad,
                    gamma=0.0,
                    cross_track_error_m=0.0,
                    heading_error_rad=0.0,
                    lookahead_point=path[-1],
                    closest_idx=len(path) - 1,
                    goal_reached=True,
                    is_safe_stop=True,
                    reason='goal_reached',
                ),
                0.0,
                len(path) - 1,
            )

    target_pt, closest_pt, target_idx = select_lookahead_point(
        path=path,
        x=x,
        y=y,
        lookahead_distance=lad,
        last_closest_idx=last_closest_idx,
        closed_loop=params.closed_loop,
        reacquire_dist_m=params.reacquire_dist_m,
    )
    if target_pt is None or closest_pt is None or target_idx is None:
        return (
            ControlOutput(
                steering_angle=0.0,
                speed=0.0,
                lookahead_distance=lad,
                gamma=0.0,
                cross_track_error_m=0.0,
                heading_error_rad=0.0,
                lookahead_point=None,
                closest_idx=last_closest_idx or 0,
                goal_reached=False,
                is_safe_stop=True,
                reason='no_lookahead_point',
            ),
            0.0,
            last_closest_idx or 0,
        )

    n = len(path)
    new_closest_idx = path.index(closest_pt) if closest_pt in path else 0
    next_idx = (
        (new_closest_idx + 1) % n
        if params.closed_loop
        else min(new_closest_idx + 1, n - 1)
    )
    path_yaw = math.atan2(
        path[next_idx][1] - closest_pt[1], path[next_idx][0] - closest_pt[0]
    )
    heading_err = wrap_angle(yaw - path_yaw)
    _, signed_cte = transform_to_vehicle_frame(
        (x, y, 0.0), closest_pt[0], closest_pt[1], path_yaw
    )

    if paused or not autonomous_enabled:
        return (
            ControlOutput(
                steering_angle=0.0,
                speed=0.0,
                lookahead_distance=lad,
                gamma=0.0,
                cross_track_error_m=signed_cte,
                heading_error_rad=heading_err,
                lookahead_point=target_pt,
                closest_idx=new_closest_idx,
                goal_reached=False,
                is_safe_stop=True,
                reason='paused' if paused else 'autonomous_disabled',
            ),
            0.0,
            new_closest_idx,
        )

    lx, ly = transform_to_vehicle_frame(target_pt, x, y, yaw)
    chord_sq = max(lx * lx + ly * ly, 0.0625)
    gamma = 2.0 * ly / chord_sq

    # Exact kinematic Pure Pursuit steering angle + optional PD damping
    geom_steer = math.atan(params.wheelbase * gamma)
    dt_safe = max(dt, 1e-3)
    d_gamma = (gamma - prev_gamma) / dt_safe
    raw_steer = (
        params.steering_gain * geom_steer
        + params.kp * gamma
        + params.kd * max(-8.0, min(8.0, d_gamma))
    )

    # First-order low-pass filter + servo rate saturation + hard angle clamp
    alpha = max(0.05, min(1.0, params.steering_alpha))
    filtered_steer = prev_steer + alpha * (raw_steer - prev_steer)
    max_delta_step = params.max_steering_rate * dt_safe
    rate_limited = max(
        prev_steer - max_delta_step,
        min(prev_steer + max_delta_step, filtered_steer),
    )
    steer_cmd = max(
        -params.max_steering_angle,
        min(params.max_steering_angle, rate_limited),
    )

    # Curvature-aware speed regulation across forward preview horizon
    preview_kappa = estimate_preview_curvature(
        path, new_closest_idx, params.curvature_preview_m, params.closed_loop
    )
    eff_kappa = max(abs(gamma), preview_kappa)
    v_curve = math.sqrt(max(params.a_lat_max, 0.1) / max(eff_kappa, 1e-4))

    ref_v = (
        closest_pt[2] / max(params.vel_division_factor, 1e-3)
        if closest_pt[2] > 0.0
        else v_curve
    )
    desired_v = min(ref_v, v_curve)
    desired_v = max(params.min_velocity, min(params.max_velocity, desired_v))

    # Reduce speed gently when cross-track error is large
    if abs(signed_cte) > 0.55:
        desired_v = max(params.min_velocity * 0.75, desired_v * 0.7)

    # Smooth longitudinal acceleration ramp to prevent wheel spin / instability
    if desired_v - current_speed > params.skidding_velocity_thresh:
        cmd_v = max(0.0, current_speed) + params.skidding_velocity_thresh
    else:
        cmd_v = desired_v
    cmd_v = max(0.0, min(params.max_velocity, cmd_v))

    return (
        ControlOutput(
            steering_angle=steer_cmd,
            speed=cmd_v,
            lookahead_distance=lad,
            gamma=gamma,
            cross_track_error_m=signed_cte,
            heading_error_rad=heading_err,
            lookahead_point=target_pt,
            closest_idx=new_closest_idx,
            goal_reached=False,
            is_safe_stop=False,
            reason='ok',
        ),
        gamma,
        new_closest_idx,
    )


def sigmoid_velocity_profile(
    gamma: float, v_min: float, v_max: float, k_sigmoid: float = 8.0
) -> float:
    """Upstream-compatible sigmoid speed curve for curvature `gamma`."""
    if v_max <= v_min:
        return max(0.0, v_min)
    c = -(1.0 / max(k_sigmoid, 1e-3)) * float(
        np.log((v_max - v_max * 0.999) / max(v_max * 0.999 - v_min, 1e-6))
    )
    vel = v_min + (v_max - v_min) / (
        1.0 + float(np.exp(min(40.0, k_sigmoid * (abs(gamma) - c))))
    )
    return max(v_min, min(v_max, vel))
