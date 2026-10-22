#!/usr/bin/env python3
"""
Launch file for Simple Planner in AROLA (Autonomous Racing Open Layered Architecture)

This launch file starts the simple path node which reads waypoints from a CSV file
and publishes them as a Path message with velocity encoded in the angular z field.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for simple planner"""

    # Declare launch arguments
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='src/control/pure_pursuit/path/example_path.csv',
        description='Path to CSV file containing waypoints (x,y,v format)'
    )

    invert_arg = DeclareLaunchArgument(
        'invert',
        default_value='false',
        description='Invert the order of waypoints'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the waypoints'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publish rate for the path messages (Hz)'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('simple_planner'),
        'config',
        'params.yaml'
    ])

    # Simple path node
    simple_path_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_path_node',
        parameters=[
            config_file,
            {
                'csv_path': LaunchConfiguration('csv_path'),
                'invert': LaunchConfiguration('invert'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        output='screen',
        remappings=[
            ('~/planned_path', '/planned_path')
        ]
    )

    return LaunchDescription([
        csv_path_arg,
        invert_arg,
        frame_id_arg,
        publish_rate_arg,
        simple_path_node,
    ])
