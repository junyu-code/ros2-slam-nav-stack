#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_official_gazebo(context):
    description_package = LaunchConfiguration('description_package').perform(context)
    gazebo_launch_file = LaunchConfiguration('gazebo_launch_file').perform(context)

    try:
        description_share = get_package_share_directory(description_package)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'未找到官方 Piper 描述包 {description_package}。请先导入 AgileX open class '
                    '中的 piper_description 并完成 colcon build。'
                )
            ),
            Shutdown(reason='official Piper description package is missing'),
        ]

    launch_path = os.path.join(description_share, 'launch', gazebo_launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(msg=f'官方 Piper Gazebo launch 不存在: {launch_path}'),
            Shutdown(reason='official Piper Gazebo launch is missing'),
        ]

    return [
        LogInfo(msg=f'启动官方 Piper Gazebo demo: {description_package}/{gazebo_launch_file}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
    ]


def _include_official_gazebo_moveit(context):
    moveit_config_package = LaunchConfiguration('gazebo_moveit_config_package').perform(context)
    demo_launch_file = LaunchConfiguration('demo_launch_file').perform(context)

    try:
        config_share = get_package_share_directory(moveit_config_package)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'未找到官方 Gazebo MoveIt2 配置包 {moveit_config_package}。'
                    '如果只想看 Gazebo 模型，请保持 start_moveit:=false。'
                )
            ),
            Shutdown(reason='official Piper Gazebo MoveIt2 config package is missing'),
        ]

    launch_path = os.path.join(config_share, 'launch', demo_launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(msg=f'官方 Gazebo MoveIt2 demo launch 不存在: {launch_path}'),
            Shutdown(reason='official Piper Gazebo MoveIt2 demo launch is missing'),
        ]

    return [
        LogInfo(msg=f'启动官方 Gazebo MoveIt2 demo: {moveit_config_package}/{demo_launch_file}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
    ]


def generate_launch_description():
    start_moveit = LaunchConfiguration('start_moveit')

    return LaunchDescription([
        DeclareLaunchArgument('description_package', default_value='piper_description'),
        DeclareLaunchArgument('gazebo_launch_file', default_value='piper_gazebo.launch.py'),
        DeclareLaunchArgument('start_moveit', default_value='false'),
        DeclareLaunchArgument('gazebo_moveit_config_package', default_value='piper_moveit_config_v4'),
        DeclareLaunchArgument('demo_launch_file', default_value='demo.launch.py'),
        OpaqueFunction(function=_include_official_gazebo),
        OpaqueFunction(
            function=_include_official_gazebo_moveit,
            condition=IfCondition(start_moveit),
        ),
    ])
