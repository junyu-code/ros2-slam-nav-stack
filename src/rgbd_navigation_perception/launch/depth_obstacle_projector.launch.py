#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enabled = LaunchConfiguration('enabled')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    output_cloud_topic = LaunchConfiguration('output_cloud_topic')
    frame_id_override = LaunchConfiguration('frame_id_override')
    set_enabled_service = LaunchConfiguration('set_enabled_service')
    pixel_step = LaunchConfiguration('pixel_step')
    min_depth_m = LaunchConfiguration('min_depth_m')
    max_depth_m = LaunchConfiguration('max_depth_m')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('depth_image_topic', default_value='/nav_camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/nav_camera/depth/camera_info'),
        DeclareLaunchArgument('output_cloud_topic', default_value='/visual_obstacles'),
        DeclareLaunchArgument('frame_id_override', default_value=''),
        DeclareLaunchArgument('set_enabled_service', default_value='/rgbd_nav/set_enabled'),
        DeclareLaunchArgument('pixel_step', default_value='4'),
        DeclareLaunchArgument('min_depth_m', default_value='0.25'),
        DeclareLaunchArgument('max_depth_m', default_value='5.0'),
        Node(
            package='rgbd_navigation_perception',
            executable='depth_obstacle_projector',
            name='depth_obstacle_projector',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'enabled': ParameterValue(enabled, value_type=bool),
                'depth_image_topic': depth_image_topic,
                'camera_info_topic': camera_info_topic,
                'output_cloud_topic': output_cloud_topic,
                'frame_id_override': frame_id_override,
                'set_enabled_service': set_enabled_service,
                'pixel_step': ParameterValue(pixel_step, value_type=int),
                'min_depth_m': ParameterValue(min_depth_m, value_type=float),
                'max_depth_m': ParameterValue(max_depth_m, value_type=float),
            }],
            output='screen',
        ),
    ])
