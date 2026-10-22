#!/usr/bin/env python3
"""
AROLA Hardware Core System Launch File (GIU Berlin RoboRacer Vehicle).

Defaults to `hardware.params.yaml` with conservative speeds and physical
joystick deadman gating (`autonomous_vel_enabled: false`).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the F1TENTH AROLA hardware core launch description."""
    core_share_dir = get_package_share_directory('core')
    config_file = os.path.join(
        core_share_dir, 'config', 'hardware.params.yaml'
    )

    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to centralized YAML parameter file',
    )
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all AROLA nodes',
    )

    config_file_path = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')

    lidar_driver_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )
    simple_planner_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_planner',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )
    watchdog_node = Node(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[config_file_path],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            declare_config_file_arg,
            declare_log_level_arg,
            LogInfo(msg='Hardware Configuration file: ' + config_file),
            lidar_driver_node,
            amcl_node,
            simple_planner_node,
            pure_pursuit_node,
            watchdog_node,
            LogInfo(msg='AROLA Hardware System launch configured.'),
        ]
    )
