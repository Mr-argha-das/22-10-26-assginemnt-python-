#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Unit tests for GIU Berlin map loading, transforms, and path planning."""

from pathlib import Path
import sys

import numpy as np
from PIL import Image

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[2]
for p in (PKG_ROOT, REPO_ROOT / 'tools'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    ensure_ros2_environment,
)

ensure_ros2_environment(str(REPO_ROOT))

from simple_planner.map_utils import OccupancyGridMap  # noqa: E402
from simple_planner.path_planner import plan_berlin_path  # noqa: E402
from simple_planner.path_validator import (  # noqa: E402
    deduplicate_waypoints,
    validate_path,
)
from simple_planner.simple_path_node import (  # noqa: E402
    SimplePathNode,
    parse_waypoints_csv_file,
)


def test_berlin_map_ground_truth_and_8bit_parity():
    yaml_path = REPO_ROOT / 'f1tenth_gym_ros' / 'maps' / 'berlin.yaml'
    png_path = REPO_ROOT / 'f1tenth_gym_ros' / 'maps' / 'berlin.png'
    pgm_path = REPO_ROOT / 'f1tenth_gym_ros' / 'maps' / 'berlin.pgm'

    png_img = Image.open(png_path)
    pgm_img = Image.open(pgm_path)
    assert png_img.mode == 'L'
    assert pgm_img.mode == 'L'
    assert np.array_equal(np.asarray(png_img), np.asarray(pgm_img))

    grid = OccupancyGridMap(str(yaml_path))
    assert grid.meta.width == 600
    assert grid.meta.height == 600
    assert abs(grid.meta.resolution - 0.05) < 1e-9
    assert abs(grid.meta.origin_x - (-11.606540)) < 1e-5
    assert abs(grid.meta.origin_y - (-26.520793)) < 1e-5
    assert int(np.sum(grid.free_mask)) == 107954
    assert grid.num_occupied_components == 2

    # Start pose (2.2933, -17.2435) lies well inside free track with >0.95m clr
    start_clr = grid.clearance_at(2.2933, -17.2435)
    assert start_clr > 0.95


def test_world_grid_transform_inverse_precision():
    grid = OccupancyGridMap(str(PKG_ROOT / 'maps' / 'berlin.yaml'))
    rng = np.random.default_rng(42)
    xs = rng.uniform(-10.0, 15.0, size=50)
    ys = rng.uniform(-25.0, 2.0, size=50)
    rows, cols = grid.world_to_grid_float(xs, ys)
    x_rec, y_rec = grid.grid_to_world(rows, cols)
    assert np.max(np.abs(x_rec - xs)) < 1e-12
    assert np.max(np.abs(y_rec - ys)) < 1e-12


def test_all_four_shipped_berlin_paths_pass_strict_validation():
    grid = OccupancyGridMap(str(PKG_ROOT / 'maps' / 'berlin.yaml'))
    for fname in [
        'berlin_safe_centerline.csv',
        'berlin_optimal_raceline.csv',
        'berlin_apex_raceline.csv',
        'berlin_giu_reference.csv',
    ]:
        wps = parse_waypoints_csv_file(str(PKG_ROOT / 'paths' / fname))
        report = validate_path(
            wps, grid_map=grid, closed_loop=True, vehicle_width_m=0.31
        )
        assert (
            report.is_valid
        ), f'{fname} failed validation: {report.errors}'
        assert report.min_wall_clearance_m > 0.90
        assert 64.0 < report.total_length_m < 73.0


def test_active_planner_modes_and_invalid_path_rejection():
    yaml_path = str(PKG_ROOT / 'maps' / 'berlin.yaml')
    wps, grid = plan_berlin_path(yaml_path=yaml_path, mode='min_curvature')
    report = validate_path(wps, grid_map=grid, closed_loop=True)
    assert report.is_valid
    assert report.min_wall_clearance_m > 0.90

    # Empty / <3 points rejected
    assert not validate_path([]).is_valid
    assert not validate_path([(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)]).is_valid

    # Duplicate point deduplication works cleanly
    dedup = deduplicate_waypoints(
        [(0.0, 0.0, 2.0), (0.0, 0.0, 2.0), (1.0, 0.0, 2.0)],
        closed_loop=False,
    )
    assert len(dedup) == 2

    # Node instantiation & message publishing
    node = SimplePathNode()
    node.publish_path()
    assert len(node.waypoints) > 300
    node.destroy_node()
