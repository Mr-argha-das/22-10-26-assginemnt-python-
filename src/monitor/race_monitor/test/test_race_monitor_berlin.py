#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Unit tests for RaceMonitor Berlin finish line and EVO trajectory metrics."""

import csv
from pathlib import Path
import sys

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[2]
for p in (PKG_ROOT, REPO_ROOT / 'tools'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    GLOBAL_BUS,
    ensure_ros2_environment,
)

ensure_ros2_environment(str(REPO_ROOT))

from evo.core import metrics as evo_metrics  # noqa: E402
from evo.core.trajectory import PosePath3D  # noqa: E402
from race_monitor.lap_detector import LapDetector  # noqa: E402
from rclpy.node import Node  # noqa: E402


def test_berlin_finish_line_crossing_and_evo_reference():
    GLOBAL_BUS.reset()
    helper = Node('lap_test_helper')
    detector = LapDetector(
        logger=helper.get_logger(), clock=helper.get_clock()
    )
    detector.start_line_p1 = [0.80, -17.25]
    detector.start_line_p2 = [3.80, -17.25]
    detector.debounce_time = 1.0

    # Cross Berlin start line southward (y: -16.8 -> -17.7 at x = 2.29)
    detector.update_position(2.29, -16.80, velocity=3.5, heading=-1.50)
    GLOBAL_BUS.step_by_ns(100_000_000)
    detector.update_position(2.29, -17.70, velocity=3.5, heading=-1.50)
    assert detector.race_started is True

    # Complete full lap 20 s later
    GLOBAL_BUS.step_by_ns(20_000_000_000)
    detector.update_position(2.29, -16.80, velocity=3.5, heading=-1.50)
    GLOBAL_BUS.step_by_ns(100_000_000)
    detector.update_position(2.29, -17.70, velocity=3.5, heading=-1.50)
    assert detector.current_lap >= 1

    # Validate berlin_reference.csv with real EVO PosePath3D
    ref_csv = PKG_ROOT / 'ref_trajectory' / 'berlin_reference.csv'
    assert ref_csv.is_file()
    rows = []
    with open(ref_csv, 'r', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            rows.append([float(r['x']), float(r['y']), float(r['z'])])
    xyz = np.asarray(rows)
    assert len(xyz) > 300
    quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (len(xyz), 1))
    traj_ref = PosePath3D(positions_xyz=xyz, orientations_quat_wxyz=quat)
    traj_est = PosePath3D(
        positions_xyz=xyz + np.array([0.05, 0.0, 0.0]),
        orientations_quat_wxyz=quat,
    )
    ape = evo_metrics.APE(evo_metrics.PoseRelation.translation_part)
    ape.process_data((traj_ref, traj_est))
    assert (
        abs(ape.get_statistic(evo_metrics.StatisticsType.rmse) - 0.05)
        < 1e-6
    )
    helper.destroy_node()
