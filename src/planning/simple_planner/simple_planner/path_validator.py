#!/usr/bin/env python3
"""
Path continuity, spacing, curvature, and map-clearance validation utilities.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .map_utils import OccupancyGridMap


@dataclass
class PathValidationReport:
    """Structured result of path continuity and safety-margin validation."""

    is_valid: bool
    num_waypoints: int
    total_length_m: float
    min_spacing_m: float
    max_spacing_m: float
    mean_spacing_m: float
    loop_closure_gap_m: float
    max_heading_change_rad: float
    max_curvature_inv_m: float
    min_speed_mps: float
    max_speed_mps: float
    min_wall_clearance_m: float
    median_wall_clearance_m: float
    required_clearance_m: float
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def deduplicate_waypoints(
    waypoints: Sequence[Tuple[float, float, float]],
    min_dist_m: float = 1e-3,
    closed_loop: bool = True,
) -> List[Tuple[float, float, float]]:
    """Remove duplicate consecutive points and duplicate loop-closing point."""
    cleaned: List[Tuple[float, float, float]] = []
    for wp in waypoints:
        x, y, v = float(wp[0]), float(wp[1]), float(wp[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(v)):
            raise ValueError(f'Non-finite waypoint encountered: {wp}')
        if cleaned:
            dx = x - cleaned[-1][0]
            dy = y - cleaned[-1][1]
            if math.hypot(dx, dy) < min_dist_m:
                continue
        cleaned.append((x, y, v))

    if closed_loop and len(cleaned) >= 2:
        gap = math.hypot(
            cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]
        )
        if gap < min_dist_m:
            cleaned.pop()

    return cleaned


def compute_signed_curvature(
    xy: np.ndarray, closed_loop: bool = True
) -> np.ndarray:
    """Compute discrete signed curvature kappa(s) [1/m] along 2D polyline."""
    pts = np.asarray(xy, dtype=np.float64)
    n = len(pts)
    if n < 3:
        return np.zeros(n, dtype=np.float64)

    if closed_loop:
        prev_pts = np.roll(pts, 1, axis=0)
        next_pts = np.roll(pts, -1, axis=0)
    else:
        prev_pts = np.vstack([pts[0], pts[:-1]])
        next_pts = np.vstack([pts[1:], pts[-1]])

    v1 = pts - prev_pts
    v2 = next_pts - pts
    v3 = next_pts - prev_pts

    a = np.linalg.norm(v1, axis=1)
    b = np.linalg.norm(v2, axis=1)
    c = np.linalg.norm(v3, axis=1)
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    denom = np.maximum(a * b * c, 1e-9)
    kappa = 2.0 * cross / denom
    if not closed_loop:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def validate_path(
    waypoints: Sequence[Tuple[float, float, float]],
    grid_map: Optional[OccupancyGridMap] = None,
    closed_loop: bool = True,
    vehicle_width_m: float = 0.31,
    safety_margin_m: float = 0.25,
    min_spacing_m: float = 0.02,
    max_spacing_m: float = 1.50,
    max_heading_jump_rad: float = 0.85,
) -> PathValidationReport:
    """
    Validate waypoint finiteness, ordering, spacing, heading continuity,
    loop closure, speed bounds, and dense sub-segment wall clearance.
    """
    errors: List[str] = []
    warnings: List[str] = []
    req_clearance = 0.5 * vehicle_width_m + safety_margin_m

    min_pts = 3 if closed_loop else 2
    if waypoints is None or len(waypoints) < min_pts:
        return PathValidationReport(
            is_valid=False,
            num_waypoints=0 if waypoints is None else len(waypoints),
            total_length_m=0.0,
            min_spacing_m=0.0,
            max_spacing_m=0.0,
            mean_spacing_m=0.0,
            loop_closure_gap_m=0.0,
            max_heading_change_rad=0.0,
            max_curvature_inv_m=0.0,
            min_speed_mps=0.0,
            max_speed_mps=0.0,
            min_wall_clearance_m=0.0,
            median_wall_clearance_m=0.0,
            required_clearance_m=req_clearance,
            errors=[f'Path requires at least {min_pts} waypoints.'],
        )

    arr = np.asarray(waypoints, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        return PathValidationReport(
            is_valid=False,
            num_waypoints=len(waypoints),
            total_length_m=0.0,
            min_spacing_m=0.0,
            max_spacing_m=0.0,
            mean_spacing_m=0.0,
            loop_closure_gap_m=0.0,
            max_heading_change_rad=0.0,
            max_curvature_inv_m=0.0,
            min_speed_mps=0.0,
            max_speed_mps=0.0,
            min_wall_clearance_m=0.0,
            median_wall_clearance_m=0.0,
            required_clearance_m=req_clearance,
            errors=['Each waypoint must have at least 3 columns (x, y, v).'],
        )

    if not np.all(np.isfinite(arr[:, :3])):
        return PathValidationReport(
            is_valid=False,
            num_waypoints=len(arr),
            total_length_m=0.0,
            min_spacing_m=0.0,
            max_spacing_m=0.0,
            mean_spacing_m=0.0,
            loop_closure_gap_m=0.0,
            max_heading_change_rad=0.0,
            max_curvature_inv_m=0.0,
            min_speed_mps=0.0,
            max_speed_mps=0.0,
            min_wall_clearance_m=0.0,
            median_wall_clearance_m=0.0,
            required_clearance_m=req_clearance,
            errors=['Waypoints contain NaN or Inf values.'],
        )

    xy = arr[:, :2]
    speeds = arr[:, 2]
    if np.any(speeds <= 0.0):
        errors.append(
            'Waypoints contain non-positive target speeds (v <= 0).'
        )

    if closed_loop:
        seg_vecs = np.diff(xy, axis=0, append=xy[:1])
    else:
        seg_vecs = np.diff(xy, axis=0)

    dists = np.linalg.norm(seg_vecs, axis=1)
    loop_gap = float(dists[-1]) if closed_loop else 0.0
    min_sp = float(np.min(dists))
    max_sp = float(np.max(dists))
    mean_sp = float(np.mean(dists))
    total_len = float(np.sum(dists))

    if min_sp < min_spacing_m:
        errors.append(
            f'Minimum waypoint spacing {min_sp:.4f}m < {min_spacing_m:.4f}m '
            f'(duplicate or near-duplicate points).'
        )
    if max_sp > max_spacing_m:
        errors.append(
            f'Maximum segment gap {max_sp:.4f}m > {max_spacing_m:.4f}m '
            f'(path discontinuity).'
        )

    headings = np.arctan2(seg_vecs[:, 1], seg_vecs[:, 0])
    if closed_loop:
        dpsi = np.diff(headings, append=headings[0])
    else:
        dpsi = np.diff(headings)
    dpsi = (dpsi + np.pi) % (2.0 * np.pi) - np.pi
    max_dpsi = float(np.max(np.abs(dpsi))) if len(dpsi) > 0 else 0.0
    if max_dpsi > max_heading_jump_rad:
        errors.append(
            f'Sharp heading discontinuity {max_dpsi:.3f} rad > '
            f'{max_heading_jump_rad:.3f} rad.'
        )

    kappa = compute_signed_curvature(xy, closed_loop=closed_loop)
    max_kappa = float(np.max(np.abs(kappa)))

    min_clr = float('nan')
    med_clr = float('nan')
    if grid_map is not None:
        dense_pts = []
        num_segs = len(xy) if closed_loop else len(xy) - 1
        for i in range(num_segs):
            p0 = xy[i]
            p1 = xy[(i + 1) % len(xy)]
            seg_len = float(np.linalg.norm(p1 - p0))
            steps = max(1, int(math.ceil(seg_len / 0.025)))
            ts = np.linspace(0.0, 1.0, steps, endpoint=False)
            dense_pts.append(p0[None, :] + ts[:, None] * (p1 - p0)[None, :])
        if not closed_loop:
            dense_pts.append(xy[-1:])
        sample_xy = np.vstack(dense_pts)

        if not np.all(grid_map.in_bounds(sample_xy[:, 0], sample_xy[:, 1])):
            errors.append('Path leaves occupancy grid map bounds.')

        clearances = grid_map.clearance_at(sample_xy[:, 0], sample_xy[:, 1])
        min_clr = float(np.min(clearances))
        med_clr = float(np.median(clearances))
        if min_clr <= 0.0:
            errors.append(
                'Path intersects occupied wall or non-drivable cell '
                f'(min clearance = {min_clr:.4f}m).'
            )
        elif min_clr < req_clearance:
            errors.append(
                f'Minimum wall clearance {min_clr:.4f}m violates vehicle '
                f'half-width + safety margin ({req_clearance:.4f}m).'
            )

    return PathValidationReport(
        is_valid=len(errors) == 0,
        num_waypoints=len(arr),
        total_length_m=total_len,
        min_spacing_m=min_sp,
        max_spacing_m=max_sp,
        mean_spacing_m=mean_sp,
        loop_closure_gap_m=loop_gap,
        max_heading_change_rad=max_dpsi,
        max_curvature_inv_m=max_kappa,
        min_speed_mps=float(np.min(speeds)),
        max_speed_mps=float(np.max(speeds)),
        min_wall_clearance_m=min_clr,
        median_wall_clearance_m=med_clr,
        required_clearance_m=req_clearance,
        errors=errors,
        warnings=warnings,
    )
