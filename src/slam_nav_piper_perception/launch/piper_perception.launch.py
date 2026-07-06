#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_perception')
    default_config = os.path.join(package_share, 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fake_camera = LaunchConfiguration('fake_camera')
    target_frame = LaunchConfiguration('target_frame')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fake_camera', default_value='false'),
        DeclareLaunchArgument('target_frame', default_value='piper_base_link'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='slam_nav_piper_perception',
            executable='arm_camera_fake_node.py',
            name='arm_camera_fake_node',
            namespace='piper',
            condition=IfCondition(fake_camera),
            parameters=[
                config_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
            output='screen',
        ),
        Node(
            package='slam_nav_piper_perception',
            executable='target_pose_estimator_node.py',
            name='target_pose_estimator_node',
            namespace='piper',
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'target_frame': target_frame,
                },
            ],
            output='screen',
        ),
    ])
