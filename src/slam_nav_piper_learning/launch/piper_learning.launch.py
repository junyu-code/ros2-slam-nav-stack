#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_learning')
    default_config = os.path.join(package_share, 'config', 'piper_learning.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_learning = LaunchConfiguration('enable_learning')
    policy_backend = LaunchConfiguration('policy_backend')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_learning', default_value='false'),
        DeclareLaunchArgument('policy_backend', default_value='disabled'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        LogInfo(
            condition=UnlessCondition(enable_learning),
            msg='Piper 学习层未启用：不会发布 ranked grasp candidates。',
        ),
        Node(
            package='slam_nav_piper_learning',
            executable='grasp_candidate_ranker_node.py',
            name='grasp_candidate_ranker_node',
            namespace='piper',
            condition=IfCondition(enable_learning),
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'policy_backend': policy_backend,
                },
            ],
            output='screen',
        ),
    ])
