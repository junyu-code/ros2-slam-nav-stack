#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fake_camera = LaunchConfiguration('fake_camera')
    target_frame = LaunchConfiguration('target_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fake_camera', default_value='false'),
        DeclareLaunchArgument('target_frame', default_value='piper_base_link'),
        Node(
            package='slam_nav_piper_perception',
            executable='arm_camera_fake_node.py',
            name='arm_camera_fake_node',
            namespace='piper',
            condition=IfCondition(fake_camera),
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
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
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'target_frame': target_frame,
            }],
            output='screen',
        ),
    ])
