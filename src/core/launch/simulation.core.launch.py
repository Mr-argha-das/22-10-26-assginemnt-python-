#!/usr/bin/env python3

"""
Basic AROLA Simulation Core System Launch File

Author: Fam Shihata
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the F1TENTH AROLA simulation system launch description."""

    # Get the core package share directory and config file path
    core_share_dir = get_package_share_directory('core')
    config_file = os.path.join(core_share_dir, 'config', 'params.yaml')

    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file containing all node parameters'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Changed to true for simulation
        description='Use simulation time if true (default: true for simulation)'
    )
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',  # Can be changed to 'debug' for development
        description='Logging level for all nodes'
    )
    declare_use_amcl_arg = DeclareLaunchArgument(
        'use_amcl',
        default_value='true',
        description='Enable AMCL localization (false uses ground truth from simulation)'
    )
    declare_use_watchdog_arg = DeclareLaunchArgument(
        'use_watchdog',
        default_value='false',  # Often disabled in simulation for simplicity
        description='Enable system watchdog monitoring'
    )
    declare_use_race_monitor_arg = DeclareLaunchArgument(
        'use_race_monitor',
        default_value='true',
        description='Enable race monitoring and performance tracking'
    )
    declare_race_mode_arg = DeclareLaunchArgument(
        'race_mode',
        default_value='lap_complete',
        description='Race ending mode: lap_complete, crash, or manual'
    )
    declare_required_laps_arg = DeclareLaunchArgument(
        'required_laps',
        default_value='20',
        description='Number of laps required to complete the race (lap_complete mode only)'
    )
    declare_controller_name_arg = DeclareLaunchArgument(
        'controller_name',
        default_value='custom_controller',
        description='Name of the controller being tested'
    )
    declare_experiment_id_arg = DeclareLaunchArgument(
        'experiment_id',
        default_value='exp_001',
        description='Experiment identifier'
    )

    config_file_path = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_amcl = LaunchConfiguration('use_amcl')
    use_watchdog = LaunchConfiguration('use_watchdog')
    use_race_monitor = LaunchConfiguration('use_race_monitor')
    race_mode = LaunchConfiguration('race_mode')
    required_laps = LaunchConfiguration('required_laps')
    controller_name = LaunchConfiguration('controller_name')
    experiment_id = LaunchConfiguration('experiment_id')

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
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
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_amcl)
    )

    simple_planner_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_planner',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
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
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_watchdog)
    )

    odom_to_base_link_node = Node(
        package='core',
        executable='odom_to_base_link_node',
        name='odom_to_base_link_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['amcl']},
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_amcl)
    )

    race_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('race_monitor'),
                'launch',
                'race_monitor.launch.py'
            ])
        ]),
        launch_arguments={
            'race_mode': race_mode,
            'required_laps': required_laps,
            'controller_name': controller_name,
            'experiment_id': experiment_id,
        }.items(),
        condition=IfCondition(use_race_monitor)
    )

    return LaunchDescription([
        declare_config_file_arg,
        declare_use_sim_time_arg,
        declare_log_level_arg,
        declare_use_amcl_arg,
        declare_use_watchdog_arg,
        declare_use_race_monitor_arg,
        declare_race_mode_arg,
        declare_required_laps_arg,
        declare_controller_name_arg,
        declare_experiment_id_arg,
        LogInfo(msg="Simulation mode: ENABLED"),
        LogInfo(msg="Configuration file: " + config_file),
        simple_planner_node,
        amcl_node,
        pure_pursuit_node,
        watchdog_node,
        odom_to_base_link_node,
        lifecycle_manager_node,
        race_monitor_launch,
        LogInfo(msg="Basic AROLA Simulation System launch completed!"),
    ])
