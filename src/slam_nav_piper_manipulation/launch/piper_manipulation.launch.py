#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_manipulation')
    default_config = os.path.join(package_share, 'config', 'piper_manipulation.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fake_execution = LaunchConfiguration('fake_execution')
    real_backend_connected = LaunchConfiguration('real_backend_connected')
    publish_base_stop = LaunchConfiguration('publish_base_stop')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fake_execution', default_value='true'),
        DeclareLaunchArgument('real_backend_connected', default_value='false'),
        DeclareLaunchArgument('publish_base_stop', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='slam_nav_piper_manipulation',
            executable='piper_task_server_node.py',
            name='piper_task_server_node',
            namespace='piper',
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'fake_execution': ParameterValue(fake_execution, value_type=bool),
                    'real_backend_connected': ParameterValue(real_backend_connected, value_type=bool),
                    'publish_base_stop': ParameterValue(publish_base_stop, value_type=bool),
                },
            ],
            output='screen',
        ),
    ])
