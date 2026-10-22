# MIT License
# Copyright (c) 2020 Hongrui Zheng

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('f1tenth_gym_ros')
    config = os.path.join(share_dir, 'config', 'sim.yaml')
    with open(config, 'r', encoding='utf-8') as cfg_file:
        config_dict = yaml.safe_load(cfg_file)
    params = config_dict['bridge']['ros__parameters']
    has_opp = params['num_agent'] > 1

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d',
            os.path.join(share_dir, 'launch', 'gym_bridge.rviz'),
        ],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': params['map_path'] + '.yaml'},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True},
        ],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )
    ego_xacro = os.path.join(share_dir, 'launch', 'ego_racecar.xacro')
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', ego_xacro])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )
    opp_xacro = os.path.join(share_dir, 'launch', 'opp_racecar.xacro')
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', opp_xacro])}],
        remappings=[('/robot_description', 'opp_robot_description')],
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
