#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_dir = get_package_share_directory('slam_nav_simulation')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    piper_description_dir = get_package_share_directory('slam_nav_piper_description')
    robot_xacro = get_package_share_path('slam_nav_simulation') / 'urdf' / 'mobile_robot.xacro'
    piper_description_builder = os.path.join(
        piper_description_dir,
        'scripts',
        'piper_description_builder.py',
    )
    static_world = os.path.join(sim_dir, 'world', 'nav_test_world', 'nav_test_world.world')
    dynamic_world = os.path.join(sim_dir, 'world', 'nav_test_world', 'nav_test_world_dynamic.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    enable_nav_rgbd_camera = LaunchConfiguration('enable_nav_rgbd_camera')
    enable_piper_arm = LaunchConfiguration('enable_piper_arm')
    piper_arm_model = LaunchConfiguration('piper_arm_model')
    enable_piper_gazebo_camera = LaunchConfiguration('enable_piper_gazebo_camera')
    piper_mount_xyz = LaunchConfiguration('piper_mount_xyz')
    piper_mount_rpy = LaunchConfiguration('piper_mount_rpy')
    piper_tcp_parent_link = LaunchConfiguration('piper_tcp_parent_link')
    piper_camera_xyz = LaunchConfiguration('piper_camera_xyz')
    piper_camera_rpy = LaunchConfiguration('piper_camera_rpy')
    world_file = PythonExpression([
        "'", dynamic_world, "' if '", world, "' == 'dynamic' else '", static_world, "'"
    ])

    robot_description = ParameterValue(
        Command([
            'python3 ', piper_description_builder,
            ' --base-xacro ', str(robot_xacro),
            ' --enable-nav-rgbd-camera ',
            enable_nav_rgbd_camera,
            ' --enable-piper-arm ',
            enable_piper_arm,
            ' --arm-model ',
            piper_arm_model,
            ' --enable-piper-gazebo-camera ',
            enable_piper_gazebo_camera,
            ' --mount-xyz "', piper_mount_xyz, '"',
            ' --mount-rpy "', piper_mount_rpy, '"',
            ' --tcp-parent-link ', piper_tcp_parent_link,
            ' --camera-xyz "', piper_camera_xyz, '"',
            ' --camera-rpy "', piper_camera_rpy, '"',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='dynamic',
            choices=['static', 'dynamic'],
            description='Choose static or dynamic-obstacle test world.',
        ),
        DeclareLaunchArgument(
            'enable_nav_rgbd_camera',
            default_value='false',
            description='Attach the optional navigation RGB-D camera to the robot model.',
        ),
        DeclareLaunchArgument(
            'enable_piper_arm',
            default_value='false',
            description='Attach the optional Piper arm to the robot model.',
        ),
        DeclareLaunchArgument(
            'piper_arm_model',
            default_value='official',
            choices=['official', 'placeholder'],
            description='Choose AgileX official Piper URDF adapter or placeholder fallback.',
        ),
        DeclareLaunchArgument(
            'enable_piper_gazebo_camera',
            default_value='false',
            description='Attach an optional Gazebo RGB-D sensor to Piper wrist camera under /piper/arm_camera.',
        ),
        DeclareLaunchArgument(
            'piper_mount_xyz',
            default_value='0.16 0.0 0.22',
            description='Piper mount position relative to base_link.',
        ),
        DeclareLaunchArgument(
            'piper_mount_rpy',
            default_value='0 0 0',
            description='Piper mount orientation relative to base_link.',
        ),
        DeclareLaunchArgument(
            'piper_tcp_parent_link',
            default_value='piper_link6',
            description='Adapted official link used as piper_tcp parent.',
        ),
        DeclareLaunchArgument(
            'piper_camera_xyz',
            default_value='0.04 0.0 0.04',
            description='Piper wrist camera position relative to piper_tcp.',
        ),
        DeclareLaunchArgument(
            'piper_camera_rpy',
            default_value='0 0 0',
            description='Piper wrist camera orientation relative to piper_tcp.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world_file,
                'init': 'true',
                'factory': 'true',
                'force_system': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'mobile_robot',
                        '-topic', 'robot_description',
                        '-x', '-7.0',
                        '-y', '-4.2',
                        '-z', '0.06',
                        '-Y', '0.0',
                        '-timeout', '120',
                    ],
                    output='screen',
                )
            ],
        ),
    ])
