#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Unit tests for WatchdogNode sensor health checks and emergency `/pause`."""

from pathlib import Path
import sys
from types import SimpleNamespace

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

from sensor_msgs.msg import LaserScan  # noqa: E402
from watchdog.watchdog_node import WatchdogNode  # noqa: E402


def test_watchdog_lidar_silence_triggers_critical_and_pause():
    GLOBAL_BUS.reset()
    node = WatchdogNode()

    # Initial live scan arrives at t = 0.1 s
    GLOBAL_BUS.step_by_ns(100_000_000)
    node._lidar_callback(LaserScan())
    assert node.lidar_status is True
    assert node.is_critical is False

    # Advance 1.6 s with zero further scans -> exceeds lidar_critical_timeout
    GLOBAL_BUS.step_by_ns(1_600_000_000)
    assert node.lidar_status is False
    assert node.is_critical is True
    assert GLOBAL_BUS.last_msg['/tmp/watchdog/critical'].data is True
    assert GLOBAL_BUS.last_msg['/pause'].data is True
    node.destroy_node()


def test_watchdog_vesc_voltage_and_temperature_faults():
    GLOBAL_BUS.reset()
    node = WatchdogNode()
    mock_vesc = SimpleNamespace(
        state=SimpleNamespace(
            voltage_input=10.5, current_motor=12.0, temp_motor=95.0
        )
    )
    node._core_callback(mock_vesc)
    assert node.is_critical is True
    status_str = node._generate_status_message()
    assert 'CRITICAL' in status_str
    node.destroy_node()
