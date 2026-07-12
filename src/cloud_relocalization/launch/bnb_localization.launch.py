#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('cloud_relocalization'),
        'config',
        'bnb_localization.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_pcd_path', default_value=''),
        Node(
            package='cloud_relocalization',
            executable='bnb_localization_node',
            name='bnb_localization_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                    'map_topic': LaunchConfiguration('map_topic'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'map_pcd_path': LaunchConfiguration('map_pcd_path'),
                },
            ],
        ),
    ])
