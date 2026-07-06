#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, relative_path, arguments, condition=None):
    package_share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, relative_path)),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_model = LaunchConfiguration('arm_model')
    start_description = LaunchConfiguration('start_description')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    fake_camera = LaunchConfiguration('fake_camera')
    fake_execution = LaunchConfiguration('fake_execution')
    real_backend_connected = LaunchConfiguration('real_backend_connected')
    publish_base_stop = LaunchConfiguration('publish_base_stop')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('arm_model', default_value='official', choices=['official', 'placeholder']),
        DeclareLaunchArgument(
            'start_description',
            default_value='false',
            description='已有整车仿真/robot_state_publisher 时保持 false，避免重复 TF。',
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='true',
            description='仅 start_description:=true 时使用，用于独立冒烟补齐 Piper 关节状态。',
        ),
        DeclareLaunchArgument('fake_camera', default_value='false'),
        DeclareLaunchArgument('fake_execution', default_value='true'),
        DeclareLaunchArgument('real_backend_connected', default_value='false'),
        DeclareLaunchArgument('publish_base_stop', default_value='false'),
        include(
            'slam_nav_piper_description',
            'launch/piper_description.launch.py',
            {
                'use_sim_time': use_sim_time,
                'arm_model': arm_model,
                'publish_joint_states': publish_joint_states,
            },
            condition=IfCondition(start_description),
        ),
        include(
            'slam_nav_piper_perception',
            'launch/piper_perception.launch.py',
            {'use_sim_time': use_sim_time, 'fake_camera': fake_camera},
        ),
        include(
            'slam_nav_piper_control',
            'launch/piper_control.launch.py',
            {
                'use_sim_time': use_sim_time,
                'backend': 'moveit',
                'initial_owner': 'disabled',
                'auto_enable': 'false',
            },
        ),
        include(
            'slam_nav_piper_manipulation',
            'launch/piper_manipulation.launch.py',
            {
                'use_sim_time': use_sim_time,
                'fake_execution': fake_execution,
                'real_backend_connected': real_backend_connected,
                'publish_base_stop': publish_base_stop,
            },
        ),
    ])
