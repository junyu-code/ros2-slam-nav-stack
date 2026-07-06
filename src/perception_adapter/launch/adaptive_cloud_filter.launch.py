#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_adapter',
            executable='adaptive_cloud_filter',
            name='adaptive_cloud_filter',
            parameters=[{
                'input_cloud_topic': '/cloud_registered',
                'odom_topic': '/Odometry',
                'output_cloud_topic': '/cloud_nav_filtered',
                'obstacle_cloud_topic': '/nav_obstacle_cloud',
                'ground_cloud_topic': '/nav_ground_cloud',
                'mode_topic': '/perception_mode',
                'low_speed_threshold': 0.15,
                'high_speed_threshold': 0.55,
                'detail_voxel_size': 0.03,
                'normal_voxel_size': 0.08,
                'fast_voxel_size': 0.18,
                'safe_voxel_size': 0.05,
                'min_height': -0.30,
                'max_height': 0.60,
                'min_range': 0.35,
                'max_range': 15.0,
                'roi_filter_enabled': False,
                'publish_split_clouds': True,
                'ground_min_height': -0.15,
                'ground_max_height': 0.06,
                'obstacle_min_height': 0.08,
                'obstacle_max_height': 1.20,
                'obstacle_check_enabled': False,
            }],
            output='screen',
        ),
    ])
