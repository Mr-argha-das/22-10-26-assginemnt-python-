#!/usr/bin/env python3

"""
Watchdog Launch File

This launch file starts the F1TENTH watchdog node with its YAML configuration.

Author: Fam Shihata <fam@awadlouis.com>
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for the watchdog node."""

    # Get package directory
    pkg_dir = get_package_share_directory('watchdog')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'watchdog_params.yaml'),
        description='Path to the configuration file',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for the watchdog node',
    )

    # Define the watchdog node
    watchdog_node = Node(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            config_file_arg,
            log_level_arg,
            watchdog_node,
        ]
    )
