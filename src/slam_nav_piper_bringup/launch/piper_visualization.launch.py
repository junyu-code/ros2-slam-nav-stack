#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, relative_path, arguments, condition=None):
    package_share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, relative_path)),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    package_share = get_package_share_directory('slam_nav_piper_bringup')
    default_rviz_config = os.path.join(package_share, 'config', 'piper_visualization.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_model = LaunchConfiguration('arm_model')
    start_runtime = LaunchConfiguration('start_runtime')
    start_moveit_plan = LaunchConfiguration('start_moveit_plan')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('arm_model', default_value='official', choices=['official', 'placeholder']),
        DeclareLaunchArgument(
            'start_runtime',
            default_value='true',
            description='默认启动 Piper 独立假感知/假执行链路，便于直接在 RViz 里查看。',
        ),
        DeclareLaunchArgument(
            'start_moveit_plan',
            default_value='false',
            description='显式打开时只启动 MoveIt2 plan-only，不执行轨迹、不接 SDK。',
        ),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        LogInfo(
            msg=(
                '启动 Piper RViz 可视化；默认只看官方 URDF 适配链、假腕部相机、'
                '目标位姿和抓取候选，不接入 task1/Nav2/SDK。'
            )
        ),
        include(
            'slam_nav_piper_bringup',
            'launch/piper_sim.launch.py',
            {
                'use_sim_time': use_sim_time,
                'arm_model': arm_model,
                'publish_joint_states': 'true',
            },
            condition=IfCondition(start_runtime),
        ),
        include(
            'slam_nav_piper_moveit_config',
            'launch/piper_project_moveit_plan.launch.py',
            {
                'use_sim_time': use_sim_time,
                'description_mode': 'standalone',
                'publish_robot_state': 'false',
                'start_joint_state_publisher': 'false',
                'allow_trajectory_execution': 'false',
                'joint_states_topic': '/piper/joint_states',
            },
            condition=IfCondition(start_moveit_plan),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='piper_rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
