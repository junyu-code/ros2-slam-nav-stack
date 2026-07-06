#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mission_behavior')
    default_config = os.path.join(package_share, 'config', 'mission_behavior.yaml')

    auto_start = LaunchConfiguration('auto_start')
    frame_id = LaunchConfiguration('frame_id')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    max_navigation_retries = LaunchConfiguration('max_navigation_retries')
    backup_distance = LaunchConfiguration('backup_distance')
    backup_speed = LaunchConfiguration('backup_speed')
    recovery_strategy = LaunchConfiguration('recovery_strategy')
    navigate_timeout_sec = LaunchConfiguration('navigate_timeout_sec')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('goal_x', default_value='8.0'),
        DeclareLaunchArgument('goal_y', default_value='4.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('max_navigation_retries', default_value='1'),
        DeclareLaunchArgument('backup_distance', default_value='0.45'),
        DeclareLaunchArgument('backup_speed', default_value='0.12'),
        DeclareLaunchArgument('recovery_strategy', default_value='free_space'),
        DeclareLaunchArgument('navigate_timeout_sec', default_value='180.0'),
        Node(
            package='mission_behavior',
            executable='mission_behavior_node.py',
            name='mission_behavior_node',
            output='screen',
            parameters=[config_file, {
                'use_sim_time': use_sim_time,
                'auto_start': auto_start,
                'frame_id': frame_id,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_yaw': goal_yaw,
                'max_navigation_retries': max_navigation_retries,
                'backup_distance': backup_distance,
                'backup_speed': backup_speed,
                'recovery_strategy': recovery_strategy,
                'navigate_timeout_sec': navigate_timeout_sec,
            }],
        ),
    ])
