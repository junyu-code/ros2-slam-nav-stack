#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_control')
    default_config = os.path.join(package_share, 'config', 'piper_control.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    backend = LaunchConfiguration('backend')
    initial_owner = LaunchConfiguration('initial_owner')
    auto_enable = LaunchConfiguration('auto_enable')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('backend', default_value='moveit'),
        DeclareLaunchArgument('initial_owner', default_value='disabled'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='slam_nav_piper_control',
            executable='piper_control_bridge_node.py',
            name='piper_control_bridge_node',
            namespace='piper',
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'backend': backend,
                    'initial_owner': initial_owner,
                    'auto_enable': ParameterValue(auto_enable, value_type=bool),
                },
            ],
            output='screen',
        ),
    ])
