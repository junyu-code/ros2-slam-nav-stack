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
    target_frame = LaunchConfiguration('target_frame')
    set_enabled_service = LaunchConfiguration('set_enabled_service')
    pixel_step = LaunchConfiguration('pixel_step')
    min_depth_m = LaunchConfiguration('min_depth_m')
    max_depth_m = LaunchConfiguration('max_depth_m')
    obstacle_min_height_m = LaunchConfiguration('obstacle_min_height_m')
    obstacle_max_height_m = LaunchConfiguration('obstacle_max_height_m')
    min_forward_m = LaunchConfiguration('min_forward_m')
    max_forward_m = LaunchConfiguration('max_forward_m')
    max_lateral_m = LaunchConfiguration('max_lateral_m')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('depth_image_topic', default_value='/nav_camera/d435i/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/nav_camera/d435i/depth/camera_info'),
        DeclareLaunchArgument('output_cloud_topic', default_value='/visual_obstacles'),
        DeclareLaunchArgument('frame_id_override', default_value=''),
        DeclareLaunchArgument('target_frame', default_value='base_footprint'),
        DeclareLaunchArgument('set_enabled_service', default_value='/rgbd_nav/set_enabled'),
        DeclareLaunchArgument('pixel_step', default_value='4'),
        DeclareLaunchArgument('min_depth_m', default_value='0.25'),
        DeclareLaunchArgument('max_depth_m', default_value='5.0'),
        DeclareLaunchArgument('obstacle_min_height_m', default_value='0.08'),
        DeclareLaunchArgument('obstacle_max_height_m', default_value='1.20'),
        DeclareLaunchArgument('min_forward_m', default_value='0.05'),
        DeclareLaunchArgument('max_forward_m', default_value='4.0'),
        DeclareLaunchArgument('max_lateral_m', default_value='2.0'),
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
                'target_frame': target_frame,
                'set_enabled_service': set_enabled_service,
                'pixel_step': ParameterValue(pixel_step, value_type=int),
                'min_depth_m': ParameterValue(min_depth_m, value_type=float),
                'max_depth_m': ParameterValue(max_depth_m, value_type=float),
                'obstacle_min_height_m': ParameterValue(obstacle_min_height_m, value_type=float),
                'obstacle_max_height_m': ParameterValue(obstacle_max_height_m, value_type=float),
                'min_forward_m': ParameterValue(min_forward_m, value_type=float),
                'max_forward_m': ParameterValue(max_forward_m, value_type=float),
                'max_lateral_m': ParameterValue(max_lateral_m, value_type=float),
            }],
            output='screen',
        ),
    ])
