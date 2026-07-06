#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fake_execution = LaunchConfiguration('fake_execution')
    publish_base_stop = LaunchConfiguration('publish_base_stop')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fake_execution', default_value='true'),
        DeclareLaunchArgument('publish_base_stop', default_value='false'),
        Node(
            package='slam_nav_piper_manipulation',
            executable='piper_task_server_node.py',
            name='piper_task_server_node',
            namespace='piper',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'fake_execution': ParameterValue(fake_execution, value_type=bool),
                'publish_base_stop': ParameterValue(publish_base_stop, value_type=bool),
            }],
            output='screen',
        ),
    ])
