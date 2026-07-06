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
    crop_map_around_guess = LaunchConfiguration('crop_map_around_guess')
    local_map_radius = LaunchConfiguration('local_map_radius')
    fitness_score_threshold = LaunchConfiguration('fitness_score_threshold')
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
        DeclareLaunchArgument('crop_map_around_guess', default_value='true'),
        DeclareLaunchArgument('local_map_radius', default_value='8.0'),
        DeclareLaunchArgument('fitness_score_threshold', default_value='0.45'),
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
                    'crop_map_around_guess': ParameterValue(
                        crop_map_around_guess,
                        value_type=bool,
                    ),
                    'local_map_radius': ParameterValue(local_map_radius, value_type=float),
                    'fitness_score_threshold': ParameterValue(
                        fitness_score_threshold,
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
