#!/usr/bin/env python3

"""
Basic AROLA Simulation Core System Launch File

Author: Fam Shihata
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the complete F1TENTH core system launch description."""

    # Get the core package share directory and config file path
    core_share_dir = get_package_share_directory('core')
    config_file = os.path.join(core_share_dir, 'config', 'params.yaml')

    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file containing all node parameters'
    )
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes'
    )

    config_file_path = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            config_file_path,
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    lidar_driver_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[
            config_file_path
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            config_file_path,
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    simple_planner_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_planner',
        output='screen',
        parameters=[
            config_file_path,
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    watchdog_node = Node(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[
            config_file_path,
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        # Launch arguments
        declare_config_file_arg,
        declare_log_level_arg,
        LogInfo(msg="Configuration file: " + config_file),
        lidar_driver_node,
        amcl_node,
        simple_planner_node,
        pure_pursuit_node,
        watchdog_node,
        LogInfo(msg="Basic AROLA Simulation System launch completed!"),
    ])
