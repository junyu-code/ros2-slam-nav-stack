#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, relative_path, arguments):
    package_share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, relative_path)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    backend = LaunchConfiguration('backend')
    arm_model = LaunchConfiguration('arm_model')
    real_backend_connected = LaunchConfiguration('real_backend_connected')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('backend', default_value='moveit'),
        DeclareLaunchArgument('arm_model', default_value='placeholder', choices=['placeholder', 'official']),
        DeclareLaunchArgument('real_backend_connected', default_value='false'),
        include(
            'slam_nav_piper_description',
            'launch/piper_description.launch.py',
            {'use_sim_time': use_sim_time, 'arm_model': arm_model},
        ),
        include(
            'slam_nav_piper_perception',
            'launch/piper_perception.launch.py',
            {'use_sim_time': use_sim_time, 'fake_camera': 'false'},
        ),
        include(
            'slam_nav_piper_control',
            'launch/piper_control.launch.py',
            {
                'use_sim_time': use_sim_time,
                'backend': backend,
                'initial_owner': 'disabled',
                'auto_enable': 'false',
            },
        ),
        include(
            'slam_nav_piper_manipulation',
            'launch/piper_manipulation.launch.py',
            {
                'use_sim_time': use_sim_time,
                'fake_execution': 'false',
                'real_backend_connected': real_backend_connected,
                'publish_base_stop': 'false',
            },
        ),
    ])
