#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_official_moveit(context):
    config_package = LaunchConfiguration('moveit_config_package').perform(context)
    demo_launch_file = LaunchConfiguration('demo_launch_file').perform(context)

    try:
        config_share = get_package_share_directory(config_package)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'未找到官方 MoveIt2 配置包 {config_package}。请先导入 AgileX open class '
                    '中的 piper_moveit_config_v5 并完成 colcon build。'
                )
            ),
            Shutdown(reason='official Piper MoveIt2 config package is missing'),
        ]

    launch_path = os.path.join(config_share, 'launch', demo_launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(msg=f'官方 MoveIt2 demo launch 不存在: {launch_path}'),
            Shutdown(reason='official Piper MoveIt2 demo launch is missing'),
        ]

    return [
        LogInfo(msg=f'启动官方 Piper MoveIt2 demo: {config_package}/{demo_launch_file}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('moveit_config_package', default_value='piper_moveit_config_v5'),
        DeclareLaunchArgument('demo_launch_file', default_value='demo.launch.py'),
        OpaqueFunction(function=_include_official_moveit),
    ])
