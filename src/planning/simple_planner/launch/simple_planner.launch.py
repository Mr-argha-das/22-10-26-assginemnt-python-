#!/usr/bin/env python3
"""Launch file for AROLA Simple Planner configured for the GIU Berlin map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for simple_planner node."""
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='paths/berlin_optimal_raceline.csv',
        description='Path to CSV file containing waypoints (x,y,v format)',
    )
    planner_mode_arg = DeclareLaunchArgument(
        'planner_mode',
        default_value='csv',
        description='Planner mode: csv, min_curvature, or harmonic_centerline',
    )
    invert_arg = DeclareLaunchArgument(
        'invert',
        default_value='false',
        description='Invert the order of waypoints',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the waypoints',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Publish rate for the path messages (Hz)',
    )

    config_file = PathJoinSubstitution(
        [FindPackageShare('simple_planner'), 'config', 'params.yaml']
    )

    simple_path_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_planner',
        parameters=[
            config_file,
            {
                'planner_mode': LaunchConfiguration('planner_mode'),
                'csv_path': LaunchConfiguration('csv_path'),
                'invert': LaunchConfiguration('invert'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            csv_path_arg,
            planner_mode_arg,
            invert_arg,
            frame_id_arg,
            publish_rate_arg,
            simple_path_node,
        ]
    )
