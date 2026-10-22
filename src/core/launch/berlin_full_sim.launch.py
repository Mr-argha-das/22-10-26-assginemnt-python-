#!/usr/bin/env python3
"""
Unified One-Command Berlin Simulator + AROLA Autonomous Racing Stack Launch.

Starts `f1tenth_gym_ros` (Berlin map + vehicle starting pose) AND the AROLA
`simulation.core.launch.py` nodes (`simple_planner`, `pure_pursuit_node`,
`watchdog_node`, `race_monitor`) in a single ROS 2 Humble session.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    core_share = get_package_share_directory('core')
    default_cfg = os.path.join(core_share, 'config', 'params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_cfg,
        description='AROLA parameter YAML profile path',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 window',
    )
    required_laps_arg = DeclareLaunchArgument(
        'required_laps',
        default_value='5',
        description='Target laps for race_monitor',
    )

    sim_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('f1tenth_gym_ros'),
                        'launch',
                        'berlin_gym_bridge.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items(),
    )

    arola_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('core'),
                        'launch',
                        'simulation.core.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'required_laps': LaunchConfiguration('required_laps'),
        }.items(),
    )

    return LaunchDescription(
        [
            config_arg,
            use_rviz_arg,
            required_laps_arg,
            sim_bridge_launch,
            arola_stack_launch,
        ]
    )
