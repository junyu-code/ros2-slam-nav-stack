#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    target_frame = LaunchConfiguration('target_frame')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')
    target_yaw = LaunchConfiguration('target_yaw')
    object_id = LaunchConfiguration('object_id')
    object_class = LaunchConfiguration('object_class')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('target_frame', default_value='piper_base_link'),
        DeclareLaunchArgument('target_x', default_value='0.30'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='0.22'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('object_id', default_value='mission_demo_target'),
        DeclareLaunchArgument('object_class', default_value='demo'),
        Node(
            package='mission_behavior',
            executable='mission_piper_pick_place_demo_node.py',
            name='mission_piper_pick_place_demo_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'auto_start': auto_start,
                'target_frame': target_frame,
                'target_x': target_x,
                'target_y': target_y,
                'target_z': target_z,
                'target_yaw': target_yaw,
                'object_id': object_id,
                'object_class': object_class,
            }],
        ),
    ])
