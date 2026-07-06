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
    builder_script = os.path.join(package_share, 'scripts', 'piper_description_builder.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_model = LaunchConfiguration('arm_model')
    official_description_package = LaunchConfiguration('official_description_package')
    official_description_xacro = LaunchConfiguration('official_description_xacro')
    mount_xyz = LaunchConfiguration('mount_xyz')
    mount_rpy = LaunchConfiguration('mount_rpy')
    base_offset_xyz = LaunchConfiguration('base_offset_xyz')
    base_offset_rpy = LaunchConfiguration('base_offset_rpy')
    tcp_parent_link = LaunchConfiguration('tcp_parent_link')
    tcp_xyz = LaunchConfiguration('tcp_xyz')
    tcp_rpy = LaunchConfiguration('tcp_rpy')
    camera_xyz = LaunchConfiguration('camera_xyz')
    camera_rpy = LaunchConfiguration('camera_rpy')
    enable_piper_gazebo_camera = LaunchConfiguration('enable_piper_gazebo_camera')
    publish_joint_states = LaunchConfiguration('publish_joint_states')

    robot_description = ParameterValue(
        Command([
            'python3 ', builder_script,
            ' --placeholder-xacro ', default_xacro,
            ' --arm-model ', arm_model,
            ' --official-description-package ', official_description_package,
            ' --official-description-xacro ', official_description_xacro,
            ' --mount-xyz "', mount_xyz, '"',
            ' --mount-rpy "', mount_rpy, '"',
            ' --base-offset-xyz "', base_offset_xyz, '"',
            ' --base-offset-rpy "', base_offset_rpy, '"',
            ' --tcp-parent-link ', tcp_parent_link,
            ' --tcp-xyz "', tcp_xyz, '"',
            ' --tcp-rpy "', tcp_rpy, '"',
            ' --camera-xyz "', camera_xyz, '"',
            ' --camera-rpy "', camera_rpy, '"',
            ' --enable-piper-gazebo-camera ', enable_piper_gazebo_camera,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('arm_model', default_value='official', choices=['official', 'placeholder']),
        DeclareLaunchArgument('official_description_package', default_value='piper_description'),
        DeclareLaunchArgument('official_description_xacro', default_value='urdf/piper_description.xacro'),
        DeclareLaunchArgument('mount_xyz', default_value='0.16 0.0 0.22'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('base_offset_xyz', default_value='0 0 0.04'),
        DeclareLaunchArgument('base_offset_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_parent_link', default_value='piper_link6'),
        DeclareLaunchArgument('tcp_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('camera_xyz', default_value='0.04 0.0 0.04'),
        DeclareLaunchArgument('camera_rpy', default_value='0 0 0'),
        DeclareLaunchArgument(
            'enable_piper_gazebo_camera',
            default_value='false',
            description='显式打开时在 piper_arm_camera_link 上挂 Gazebo RGB-D 插件，话题固定为 /piper/arm_camera/*。',
        ),
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
