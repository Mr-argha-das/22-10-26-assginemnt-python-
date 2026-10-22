#!/usr/bin/env python3
"""
Occupancy-grid path planner and minimum-curvature speed-profile generator for
the GIU Berlin F1TENTH circuit.

Two complementary planning stages operate directly on the ROS OccupancyGridMap:
1. Harmonic streamfunction + Euclidean distance ridge centerline extraction:
   Solves Laplace's equation nabla^2(phi) = 0 on free space with phi = 0 on the
   inner island boundary and phi = 1 on the outer boundary, extracts the unique
   closed contour phi = 0.5, and centers points along grad(EDT) within the
   drivable corridor.
2. Corridor-constrained minimum-curvature smoothing + forward-backward friction
   speed profiling:
   Iteratively reduces discrete curvature while strictly clamping every point
   to satisfy clearance(x, y) >= 0.5 * vehicle_width + safety_margin, then
   computes a dynamically feasible velocity profile v(s).
"""

import csv
import math
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import sparse  # noqa: E402
from scipy.sparse.linalg import spsolve  # noqa: E402

from .map_utils import OccupancyGridMap  # noqa: E402
from .path_validator import compute_signed_curvature  # noqa: E402


def resample_closed_polyline(xy: np.ndarray, spacing_m: float) -> np.ndarray:
    """Resample a closed 2D loop to exact uniform arc-length spacing."""
    pts = np.asarray(xy, dtype=np.float64)
    closed = np.vstack([pts, pts[0]])
    seg_lens = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    keep = np.concatenate([[True], seg_lens > 1e-7])
    closed = closed[keep]
    seg_lens = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = float(cum_s[-1])
    num_pts = max(12, int(round(total_len / spacing_m)))
    target_s = np.linspace(0.0, total_len, num_pts, endpoint=False)
    rx = np.interp(target_s, cum_s, closed[:, 0])
    ry = np.interp(target_s, cum_s, closed[:, 1])
    return np.column_stack([rx, ry])


def align_loop_to_start_pose(
    xy: np.ndarray, start_x: float, start_y: float, start_yaw: float
) -> np.ndarray:
    """
    Orient a closed 2D polyline to match `start_yaw` and circularly shift index
    0 to the waypoint nearest `(start_x, start_y)`.
    """
    pts = np.asarray(xy, dtype=np.float64)
    dists = np.hypot(pts[:, 0] - start_x, pts[:, 1] - start_y)
    idx0 = int(np.argmin(dists))
    pts = np.roll(pts, -idx0, axis=0)

    fwd = pts[1] - pts[0]
    desired = np.array([math.cos(start_yaw), math.sin(start_yaw)])
    if float(np.dot(fwd, desired)) < 0.0:
        pts = np.vstack([pts[:1], pts[1:][::-1]])
    return pts


def extract_harmonic_centerline_xy(
    grid_map: OccupancyGridMap,
    spacing_m: float = 0.20,
    start_pose: Tuple[float, float, float] = (2.2933, -17.2435, -1.4965),
    ridge_steps: int = 35,
    ridge_step_size_m: float = 0.03,
) -> np.ndarray:
    """
    Extract a smooth, single-loop medial centerline directly from a closed
    annular occupancy grid map via harmonic potential + EDT ridge ascent.
    """
    free = grid_map.free_mask
    occ_labels = grid_map.occ_labels
    outer_label = int(occ_labels[0, 0])
    outer_mask = (~free) & (occ_labels == outer_label)
    inner_mask = (~free) & (occ_labels != outer_label)
    if not np.any(inner_mask):
        raise ValueError(
            'Map does not contain an enclosed inner boundary island.'
        )

    height, width = free.shape
    idx_map = -np.ones((height, width), dtype=np.int32)
    free_rc = np.argwhere(free)
    n_free = len(free_rc)
    idx_map[free] = np.arange(n_free)

    rows = list(range(n_free))
    cols = list(range(n_free))
    vals = [4.0] * n_free
    rhs = np.zeros(n_free, dtype=np.float64)

    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr = free_rc[:, 0] + dr
        nc = free_rc[:, 1] + dc
        nb_idx = idx_map[nr, nc]
        free_nb = nb_idx >= 0
        rows.extend(np.nonzero(free_nb)[0].tolist())
        cols.extend(nb_idx[free_nb].tolist())
        vals.extend([-1.0] * int(np.sum(free_nb)))
        rhs[outer_mask[nr, nc]] += 1.0

    laplacian = sparse.csr_matrix(
        (vals, (rows, cols)), shape=(n_free, n_free)
    )
    phi_free = spsolve(laplacian, rhs)
    phi = np.where(outer_mask, 1.0, 0.0)
    phi[free] = phi_free

    fig, ax = plt.subplots()
    contour_set = ax.contour(np.arange(width), np.arange(height), phi, [0.5])
    segs = contour_set.allsegs[0]
    plt.close(fig)
    if not segs:
        raise RuntimeError(
            'Failed to extract harmonic level contour phi=0.5.'
        )

    raw_cr = max(segs, key=len)
    wx, wy = grid_map.grid_to_world(raw_cr[:, 1], raw_cr[:, 0])
    pts = resample_closed_polyline(np.column_stack([wx, wy]), spacing_m)

    # Ascend the Euclidean distance field normal to the curve for max margin
    for _ in range(ridge_steps):
        gx, gy = grid_map.clearance_gradient_at(pts[:, 0], pts[:, 1])
        grad = np.column_stack([gx, gy])
        tangents = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
        t_norm = np.maximum(
            np.linalg.norm(tangents, axis=1, keepdims=True), 1e-8
        )
        t_hat = tangents / t_norm
        normal_grad = (
            grad - np.sum(grad * t_hat, axis=1, keepdims=True) * t_hat
        )
        pts = pts + ridge_step_size_m * normal_grad
        pts = 0.85 * pts + 0.075 * (
            np.roll(pts, -1, axis=0) + np.roll(pts, 1, axis=0)
        )

    pts = resample_closed_polyline(pts, spacing_m)
    pts = align_loop_to_start_pose(
        pts, start_pose[0], start_pose[1], start_pose[2]
    )
    return pts


def optimize_minimum_curvature_xy(
    initial_xy: np.ndarray,
    grid_map: OccupancyGridMap,
    vehicle_width_m: float = 0.31,
    safety_margin_m: float = 0.65,
    iterations: int = 260,
    smooth_weight: float = 0.35,
    spacing_m: float = 0.20,
    start_pose: Optional[Tuple[float, float, float]] = (
        2.2933,
        -17.2435,
        -1.4965,
    ),
) -> np.ndarray:
    """
    Corridor-constrained iterative minimum-curvature smoothing.

    Reduces discrete bending energy while strictly enforcing that every
    waypoint remains at least `0.5 * vehicle_width_m + safety_margin_m` away
    from both inner and outer walls.
    """
    min_clearance = 0.5 * vehicle_width_m + safety_margin_m
    pts = np.asarray(initial_xy, dtype=np.float64).copy()

    for it in range(iterations):
        # Biharmonic / second-difference curvature reduction step
        p_m2 = np.roll(pts, 2, axis=0)
        p_m1 = np.roll(pts, 1, axis=0)
        p_p1 = np.roll(pts, -1, axis=0)
        p_p2 = np.roll(pts, -2, axis=0)
        lap2 = p_m2 - 4.0 * p_m1 + 6.0 * pts - 4.0 * p_p1 + p_p2
        candidate = pts - (smooth_weight * 0.12) * lap2

        # Push any violating points strictly back inside safety corridor
        clr = grid_map.clearance_at(candidate[:, 0], candidate[:, 1])
        viol = clr < min_clearance
        if np.any(viol):
            gx, gy = grid_map.clearance_gradient_at(
                candidate[viol, 0], candidate[viol, 1]
            )
            gvec = np.column_stack([gx, gy])
            gnorm = np.maximum(
                np.linalg.norm(gvec, axis=1, keepdims=True), 1e-6
            )
            deficit = (min_clearance - clr[viol] + 0.015)[:, None]
            repaired = candidate[viol] + (gvec / gnorm) * deficit
            rep_clr = grid_map.clearance_at(repaired[:, 0], repaired[:, 1])
            still_bad = rep_clr < min_clearance
            repaired[still_bad] = pts[viol][still_bad]
            candidate[viol] = repaired

        pts = candidate
        if (it + 1) % 40 == 0:
            pts = resample_closed_polyline(pts, spacing_m)

    pts = resample_closed_polyline(pts, spacing_m)
    # Final hard safety projection check after final resampling
    for _ in range(15):
        clr = grid_map.clearance_at(pts[:, 0], pts[:, 1])
        viol = clr < min_clearance
        if not np.any(viol):
            break
        gx, gy = grid_map.clearance_gradient_at(pts[viol, 0], pts[viol, 1])
        gvec = np.column_stack([gx, gy])
        gnorm = np.maximum(np.linalg.norm(gvec, axis=1, keepdims=True), 1e-6)
        pts[viol] += (gvec / gnorm) * (min_clearance - clr[viol] + 0.02)[
            :, None
        ]

    if start_pose is not None:
        pts = align_loop_to_start_pose(
            pts, start_pose[0], start_pose[1], start_pose[2]
        )
    return pts


def compute_speed_profile(
    xy: np.ndarray,
    v_min: float = 1.5,
    v_max: float = 4.5,
    a_lat_max: float = 4.2,
    a_accel_max: float = 3.2,
    a_brake_max: float = 4.0,
    closed_loop: bool = True,
) -> np.ndarray:
    """
    Compute dynamically feasible speed profile v(s) using lateral grip limit
    v <= sqrt(a_lat_max / |kappa|) plus forward-backward longitudinal passes.
    """
    pts = np.asarray(xy, dtype=np.float64)
    n = len(pts)
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    if n < 3:
        return np.full(n, v_min, dtype=np.float64)

    kappa = np.abs(compute_signed_curvature(pts, closed_loop=closed_loop))
    # Smooth curvature slightly across neighbors to avoid discretization noise
    if closed_loop:
        kappa_eff = np.maximum(
            kappa,
            0.5 * (np.roll(kappa, 1) + np.roll(kappa, -1)),
        )
    else:
        kappa_eff = kappa

    v_lat = np.sqrt(a_lat_max / np.maximum(kappa_eff, 1e-5))
    v = np.clip(v_lat, v_min, v_max)

    if closed_loop:
        ds = np.linalg.norm(np.diff(pts, axis=0, append=pts[:1]), axis=1)
        # Two periodic sweeps ensure exact boundary continuity across index 0
        for _ in range(2):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                v[i] = min(
                    v[i], math.sqrt(v[j] * v[j] + 2.0 * a_brake_max * ds[i])
                )
        for _ in range(2):
            for i in range(n):
                j = (i + 1) % n
                v[j] = min(
                    v[j], math.sqrt(v[i] * v[i] + 2.0 * a_accel_max * ds[i])
                )
    else:
        ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        v[-1] = v_min
        for i in range(n - 2, -1, -1):
            v[i] = min(
                v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_brake_max * ds[i])
            )
        for i in range(n - 1):
            v[i + 1] = min(
                v[i + 1], math.sqrt(v[i] ** 2 + 2.0 * a_accel_max * ds[i])
            )

    return np.clip(v, v_min, v_max)


def plan_berlin_path(
    yaml_path: str,
    mode: str = 'min_curvature',
    spacing_m: float = 0.20,
    vehicle_width_m: float = 0.31,
    safety_margin_m: float = 0.65,
    v_min: float = 1.6,
    v_max: float = 4.2,
    a_lat_max: float = 4.0,
    a_accel_max: float = 3.0,
    a_brake_max: float = 3.8,
    start_pose: Tuple[float, float, float] = (2.2933, -17.2435, -1.4965),
) -> Tuple[List[Tuple[float, float, float]], OccupancyGridMap]:
    """Generate a validated `(x, y, v)` reference path directly from map."""
    grid_map = OccupancyGridMap(yaml_path)
    center_xy = extract_harmonic_centerline_xy(
        grid_map=grid_map,
        spacing_m=spacing_m,
        start_pose=start_pose,
    )
    if mode == 'harmonic_centerline':
        xy = center_xy
    elif mode == 'min_curvature':
        xy = optimize_minimum_curvature_xy(
            initial_xy=center_xy,
            grid_map=grid_map,
            vehicle_width_m=vehicle_width_m,
            safety_margin_m=safety_margin_m,
            spacing_m=spacing_m,
            start_pose=start_pose,
        )
    else:
        raise ValueError(
            f"Unsupported planner mode '{mode}'. "
            "Expected 'harmonic_centerline' or 'min_curvature'."
        )

    speeds = compute_speed_profile(
        xy,
        v_min=v_min,
        v_max=v_max,
        a_lat_max=a_lat_max,
        a_accel_max=a_accel_max,
        a_brake_max=a_brake_max,
        closed_loop=True,
    )
    waypoints = [
        (float(xy[i, 0]), float(xy[i, 1]), float(speeds[i]))
        for i in range(len(xy))
    ]
    return waypoints, grid_map


def save_waypoints_csv(
    csv_path: str,
    waypoints: Sequence[Tuple[float, float, float]],
    header_comment: str = 'x_m, y_m, v_m/s',
) -> None:
    """Write `(x, y, v)` waypoints in standard AROLA CSV format."""
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        f.write(f'# {header_comment}\n')
        writer = csv.writer(f)
        for x, y, v in waypoints:
            writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{v:.6f}'])
