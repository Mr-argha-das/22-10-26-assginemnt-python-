#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Full-stack AROLA configuration, launch, rosbag2, and closed-loop tests."""

import importlib.util
from pathlib import Path
import sys

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[1]
for p in (
    REPO_ROOT / 'tools',
    PKG_ROOT,
    REPO_ROOT / 'src' / 'planning' / 'simple_planner',
    REPO_ROOT / 'src' / 'control' / 'pure_pursuit',
    REPO_ROOT / 'src' / 'monitor' / 'race_monitor',
    REPO_ROOT / 'src' / 'watchdog' / 'watchdog',
    REPO_ROOT / 'f1tenth_gym_ros',
):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    ensure_ros2_environment,
)

ensure_ros2_environment(str(REPO_ROOT))

from rosbags.rosbag2 import Reader  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402
from run_berlin_simulation import run_closed_loop_episode  # noqa: E402
from simple_planner.map_utils import OccupancyGridMap  # noqa: E402


def test_all_launch_files_generate_valid_descriptions():
    launch_files = [
        REPO_ROOT / 'src' / 'core' / 'launch' / 'core.launch.py',
        REPO_ROOT / 'src' / 'core' / 'launch' / 'simulation.core.launch.py',
        REPO_ROOT / 'src' / 'core' / 'launch' / 'berlin_full_sim.launch.py',
        REPO_ROOT
        / 'src'
        / 'planning'
        / 'simple_planner'
        / 'launch'
        / 'simple_planner.launch.py',
        REPO_ROOT
        / 'src'
        / 'control'
        / 'pure_pursuit'
        / 'launch'
        / 'pure_pursuit.launch.py',
        REPO_ROOT
        / 'src'
        / 'watchdog'
        / 'watchdog'
        / 'launch'
        / 'watchdog.launch.py',
        REPO_ROOT
        / 'f1tenth_gym_ros'
        / 'launch'
        / 'berlin_gym_bridge.launch.py',
    ]
    for lf in launch_files:
        assert lf.is_file(), f'Missing launch file: {lf}'
        spec = importlib.util.spec_from_file_location(lf.stem, lf)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        assert ld is not None


def test_rosbag2_artifact_deserializes_cleanly():
    bag_dir = REPO_ROOT / 'results' / 'rosbag2_berlin_race'
    assert bag_dir.is_dir()
    store = get_typestore(Stores.ROS2_HUMBLE)
    counts = {}
    with Reader(bag_dir) as reader:
        for conn, _ts, raw in reader.messages():
            msg = store.deserialize_cdr(raw, conn.msgtype)
            assert msg is not None
            counts[conn.topic] = counts.get(conn.topic, 0) + 1
    assert counts.get('/pp_path', 0) >= 1
    assert counts.get('/ego_racecar/odom', 0) > 100
    assert counts.get('/scan', 0) > 20


def test_closed_loop_single_lap_berlin_integration():
    import warnings

    warnings.filterwarnings('ignore', message='Chosen integrator is RK4.*')
    berlin_map = OccupancyGridMap(
        str(REPO_ROOT / 'f1tenth_gym_ros' / 'maps' / 'berlin.yaml')
    )
    res = run_closed_loop_episode(
        profile_yaml=REPO_ROOT / 'src' / 'core' / 'config' / 'params.yaml',
        berlin_map=berlin_map,
        target_laps=1,
        max_sim_time_s=35.0,
        label='Pytest-ClosedLoop-1Lap',
    )
    assert res['completed_target'] is True
    assert res['collisions'] == 0
    assert res['track_departures'] == 0
    assert res['drive_publishers'] == ['pure_pursuit_node']
    assert res['tf_parent_conflicts'] == []
    assert res['best_lap_s'] < 25.0
