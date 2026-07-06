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
    bringup_dir = get_package_share_directory('slam_nav_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config = LaunchConfiguration('fast_lio_config')
    slam_params = LaunchConfiguration('slam_params')
    rviz = LaunchConfiguration('rviz')
    auto_explore = LaunchConfiguration('auto_explore')
    auto_explore_max_runtime_sec = LaunchConfiguration('auto_explore_max_runtime_sec')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('auto_explore', default_value='false'),
        DeclareLaunchArgument('auto_explore_max_runtime_sec', default_value='0.0'),
        DeclareLaunchArgument(
            'fast_lio_config',
            default_value=os.path.join(fast_lio_dir, 'config', 'mid360.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml'),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[
                fast_lio_config,
                {
                    'use_sim_time': use_sim_time,
                    'feature_extract_enable': False,
                    'point_filter_num': 3,
                    'max_iteration': 3,
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.5,
                    'cube_side_length': 1000.0,
                    'runtime_pos_log_enable': False,
                },
            ],
            output='screen',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.05,
                'min_height': -0.30,
                'max_height': 0.45,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.2,
                'range_min': 0.35,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(rviz),
            output='screen',
        ),
        Node(
            package='slam_nav_bringup',
            executable='auto_explore_mapping.py',
            name='auto_explore_mapper',
            condition=IfCondition(auto_explore),
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_runtime_sec': ParameterValue(
                    auto_explore_max_runtime_sec,
                    value_type=float,
                ),
            }],
            output='screen',
        ),
    ])
