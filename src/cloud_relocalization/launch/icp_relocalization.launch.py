#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_pcd_path = LaunchConfiguration('map_pcd_path')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    publish_tf = LaunchConfiguration('publish_tf')
    auto_align = LaunchConfiguration('auto_align')
    registration_method = LaunchConfiguration('registration_method')
    ndt_resolution = LaunchConfiguration('ndt_resolution')
    ndt_step_size = LaunchConfiguration('ndt_step_size')
    crop_map_around_guess = LaunchConfiguration('crop_map_around_guess')
    local_map_radius = LaunchConfiguration('local_map_radius')
    fitness_score_threshold = LaunchConfiguration('fitness_score_threshold')
    map_leaf_size = LaunchConfiguration('map_leaf_size')
    scan_leaf_size = LaunchConfiguration('scan_leaf_size')
    max_correspondence_distance = LaunchConfiguration('max_correspondence_distance')
    max_iterations = LaunchConfiguration('max_iterations')
    min_interval_sec = LaunchConfiguration('min_interval_sec')
    max_result_translation_jump = LaunchConfiguration('max_result_translation_jump')
    max_result_yaw_jump = LaunchConfiguration('max_result_yaw_jump')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params = PathJoinSubstitution([
        FindPackageShare('cloud_relocalization'),
        'config',
        'icp_relocalization.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map_pcd_path', default_value=''),
        DeclareLaunchArgument('input_cloud_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('auto_align', default_value='false'),
        DeclareLaunchArgument('registration_method', default_value='icp'),
        DeclareLaunchArgument('ndt_resolution', default_value='1.0'),
        DeclareLaunchArgument('ndt_step_size', default_value='0.1'),
        DeclareLaunchArgument('crop_map_around_guess', default_value='true'),
        DeclareLaunchArgument('local_map_radius', default_value='8.0'),
        DeclareLaunchArgument('fitness_score_threshold', default_value='0.45'),
        DeclareLaunchArgument('map_leaf_size', default_value='0.12'),
        DeclareLaunchArgument('scan_leaf_size', default_value='0.10'),
        DeclareLaunchArgument('max_correspondence_distance', default_value='1.0'),
        DeclareLaunchArgument('max_iterations', default_value='45'),
        DeclareLaunchArgument('min_interval_sec', default_value='2.0'),
        DeclareLaunchArgument('max_result_translation_jump', default_value='1.5'),
        DeclareLaunchArgument('max_result_yaw_jump', default_value='0.8'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='cloud_relocalization',
            executable='icp_relocalization_node',
            name='icp_relocalization_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'map_pcd_path': map_pcd_path,
                    'input_cloud_topic': input_cloud_topic,
                    'publish_tf': ParameterValue(publish_tf, value_type=bool),
                    'auto_align': ParameterValue(auto_align, value_type=bool),
                    'registration_method': registration_method,
                    'ndt_resolution': ParameterValue(ndt_resolution, value_type=float),
                    'ndt_step_size': ParameterValue(ndt_step_size, value_type=float),
                    'crop_map_around_guess': ParameterValue(
                        crop_map_around_guess,
                        value_type=bool,
                    ),
                    'local_map_radius': ParameterValue(local_map_radius, value_type=float),
                    'fitness_score_threshold': ParameterValue(
                        fitness_score_threshold,
                        value_type=float,
                    ),
                    'map_leaf_size': ParameterValue(map_leaf_size, value_type=float),
                    'scan_leaf_size': ParameterValue(scan_leaf_size, value_type=float),
                    'max_correspondence_distance': ParameterValue(
                        max_correspondence_distance,
                        value_type=float,
                    ),
                    'max_iterations': ParameterValue(max_iterations, value_type=int),
                    'min_interval_sec': ParameterValue(min_interval_sec, value_type=float),
                    'max_result_translation_jump': ParameterValue(
                        max_result_translation_jump,
                        value_type=float,
                    ),
                    'max_result_yaw_jump': ParameterValue(max_result_yaw_jump, value_type=float),
                },
            ],
        ),
    ])
