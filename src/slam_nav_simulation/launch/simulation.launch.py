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
    robot_xacro = get_package_share_path('slam_nav_simulation') / 'urdf' / 'mobile_robot.xacro'
    static_world = os.path.join(sim_dir, 'world', 'nav_test_world', 'nav_test_world.world')
    dynamic_world = os.path.join(sim_dir, 'world', 'nav_test_world', 'nav_test_world_dynamic.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    enable_nav_rgbd_camera = LaunchConfiguration('enable_nav_rgbd_camera')
    world_file = PythonExpression([
        "'", dynamic_world, "' if '", world, "' == 'dynamic' else '", static_world, "'"
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            str(robot_xacro),
            ' enable_nav_rgbd_camera:=',
            enable_nav_rgbd_camera,
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
            default_value='static',
            choices=['static', 'dynamic'],
            description='Choose static or dynamic-obstacle test world.',
        ),
        DeclareLaunchArgument(
            'enable_nav_rgbd_camera',
            default_value='false',
            description='Attach the optional navigation RGB-D camera to the robot model.',
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
