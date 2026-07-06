#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_description')
    default_xacro = os.path.join(package_share, 'urdf', 'piper_mobile_mount.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    mount_xyz = LaunchConfiguration('mount_xyz')
    mount_rpy = LaunchConfiguration('mount_rpy')
    camera_xyz = LaunchConfiguration('camera_xyz')
    camera_rpy = LaunchConfiguration('camera_rpy')
    publish_joint_states = LaunchConfiguration('publish_joint_states')

    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' mount_xyz:="', mount_xyz, '"',
            ' mount_rpy:="', mount_rpy, '"',
            ' camera_xyz:="', camera_xyz, '"',
            ' camera_rpy:="', camera_rpy, '"',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model', default_value=default_xacro),
        DeclareLaunchArgument('mount_xyz', default_value='0.16 0.0 0.22'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('camera_xyz', default_value='0.04 0.0 0.04'),
        DeclareLaunchArgument('camera_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('publish_joint_states', default_value='false'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='piper_joint_state_publisher',
            namespace='piper',
            condition=IfCondition(publish_joint_states),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='piper_robot_state_publisher',
            namespace='piper',
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
    ])
