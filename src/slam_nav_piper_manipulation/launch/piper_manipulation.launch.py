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
    base_stop_confirmed = LaunchConfiguration('base_stop_confirmed')
    hand_eye_calibrated = LaunchConfiguration('hand_eye_calibrated')
    hand_eye_result_must_exist = LaunchConfiguration('hand_eye_result_must_exist')
    hand_eye_result_path = LaunchConfiguration('hand_eye_result_path')
    use_ranked_grasp_candidates = LaunchConfiguration('use_ranked_grasp_candidates')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fake_execution', default_value='true'),
        DeclareLaunchArgument('real_backend_connected', default_value='false'),
        DeclareLaunchArgument('publish_base_stop', default_value='false'),
        DeclareLaunchArgument('base_stop_confirmed', default_value='false'),
        DeclareLaunchArgument('hand_eye_calibrated', default_value='false'),
        DeclareLaunchArgument('hand_eye_result_must_exist', default_value='true'),
        DeclareLaunchArgument('use_ranked_grasp_candidates', default_value='false'),
        DeclareLaunchArgument(
            'hand_eye_result_path',
            default_value='datasets/piper_hand_eye/piper_eye_in_hand.yaml',
        ),
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
                    'base_stop_confirmed': ParameterValue(base_stop_confirmed, value_type=bool),
                    'hand_eye_calibrated': ParameterValue(hand_eye_calibrated, value_type=bool),
                    'hand_eye_result_must_exist': ParameterValue(hand_eye_result_must_exist, value_type=bool),
                    'hand_eye_result_path': hand_eye_result_path,
                    'use_ranked_grasp_candidates': ParameterValue(use_ranked_grasp_candidates, value_type=bool),
                },
            ],
            output='screen',
        ),
    ])
