#!/usr/bin/env python3
"""
Launch file for F1TENTH Gym ROS 2 Bridge configured for the GIU Berlin map.

Supports headless mode (use_rviz:=false) for automated simulation and CI,
as well as full RViz2 + Nav2 map_server launch on Ubuntu 22.04 ROS 2 Humble.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map_stem(raw_map_path: str, share_dir: str) -> str:
    if os.path.isabs(raw_map_path) and os.path.exists(raw_map_path + '.yaml'):
        return raw_map_path
    candidate = os.path.join(share_dir, raw_map_path)
    if os.path.exists(candidate + '.yaml'):
        return candidate
    return raw_map_path


def _launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('f1tenth_gym_ros')
    sim_config = LaunchConfiguration('sim_config').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_map_server = LaunchConfiguration('use_map_server')

    with open(sim_config, 'r', encoding='utf-8') as cfg_file:
        config_dict = yaml.safe_load(cfg_file)

    params = config_dict['bridge']['ros__parameters']
    has_opp = int(params.get('num_agent', 1)) > 1
    map_stem = _resolve_map_stem(params['map_path'], share_dir)

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        output='screen',
        parameters=[sim_config, {'map_path': map_stem}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d',
            os.path.join(share_dir, 'launch', 'gym_bridge.rviz'),
        ],
        condition=IfCondition(use_rviz),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_stem + '.yaml'},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'use_sim_time': False},
        ],
        condition=IfCondition(use_map_server),
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
        condition=IfCondition(use_map_server),
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[
            {
                'robot_description': Command(
                    [
                        'xacro ',
                        os.path.join(
                            share_dir, 'launch', 'ego_racecar.xacro'
                        ),
                    ]
                )
            }
        ],
        remappings=[('/robot_description', 'ego_robot_description')],
    )

    actions = [
        bridge_node,
        rviz_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
    ]

    if has_opp:
        opp_robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[
                {
                    'robot_description': Command(
                        [
                            'xacro ',
                            os.path.join(
                                share_dir, 'launch', 'opp_racecar.xacro'
                            ),
                        ]
                    )
                }
            ],
            remappings=[('/robot_description', 'opp_robot_description')],
        )
        actions.append(opp_robot_publisher)

    return actions


def generate_launch_description():
    share_dir = get_package_share_directory('f1tenth_gym_ros')
    default_cfg = os.path.join(share_dir, 'config', 'berlin_sim.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'sim_config',
                default_value=default_cfg,
                description='Path to simulator bridge YAML configuration',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz2 visualization window',
            ),
            DeclareLaunchArgument(
                'use_map_server',
                default_value='true',
                description='Launch Nav2 map_server and lifecycle manager',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
