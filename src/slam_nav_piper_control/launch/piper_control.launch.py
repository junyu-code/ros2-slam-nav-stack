#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    backend = LaunchConfiguration('backend')
    initial_owner = LaunchConfiguration('initial_owner')
    auto_enable = LaunchConfiguration('auto_enable')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('backend', default_value='moveit'),
        DeclareLaunchArgument('initial_owner', default_value='disabled'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        Node(
            package='slam_nav_piper_control',
            executable='piper_control_bridge_node.py',
            name='piper_control_bridge_node',
            namespace='piper',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'backend': backend,
                'initial_owner': initial_owner,
                'auto_enable': ParameterValue(auto_enable, value_type=bool),
            }],
            output='screen',
        ),
    ])
