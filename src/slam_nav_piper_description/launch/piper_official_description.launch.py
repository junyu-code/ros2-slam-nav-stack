#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _split_xyz(value):
    return str(value).split()


def _include_official_description(context):
    official_package = LaunchConfiguration('official_description_package').perform(context)
    official_xacro = LaunchConfiguration('official_description_xacro').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    mount_xyz = _split_xyz(LaunchConfiguration('official_base_xyz').perform(context))
    mount_rpy = _split_xyz(LaunchConfiguration('official_base_rpy').perform(context))
    tcp_parent_frame = LaunchConfiguration('official_tcp_parent_frame').perform(context)
    camera_xyz = _split_xyz(LaunchConfiguration('camera_xyz').perform(context))
    camera_rpy = _split_xyz(LaunchConfiguration('camera_rpy').perform(context))

    if len(mount_xyz) != 3 or len(mount_rpy) != 3 or len(camera_xyz) != 3 or len(camera_rpy) != 3:
        return [
            LogInfo(msg='Piper 官方后端参数错误：xyz/rpy 必须各包含 3 个数。'),
            Shutdown(reason='invalid official Piper TF arguments'),
        ]

    try:
        official_share = get_package_share_directory(official_package)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'未找到官方 Piper 描述包 {official_package}。请先用 piper_external.repos '
                    '导入 AgileX open class，构建并 source install/setup.bash。'
                )
            ),
            Shutdown(reason='official Piper description package is missing'),
        ]

    official_xacro_path = os.path.join(official_share, official_xacro)
    if not os.path.exists(official_xacro_path):
        return [
            LogInfo(msg=f'官方 Piper xacro 不存在: {official_xacro_path}'),
            Shutdown(reason='official Piper xacro is missing'),
        ]

    project_share = get_package_share_directory('slam_nav_piper_description')
    launch_path = os.path.join(project_share, 'launch', 'piper_description.launch.py')

    return [
        LogInfo(msg='使用 AgileX 官方 Piper URDF 生成项目侧 piper_* 适配 TF 链。'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'arm_model': 'official',
                'official_description_package': official_package,
                'official_description_xacro': official_xacro,
                'mount_xyz': ' '.join(mount_xyz),
                'mount_rpy': ' '.join(mount_rpy),
                'tcp_parent_link': tcp_parent_frame,
                'camera_xyz': ' '.join(camera_xyz),
                'camera_rpy': ' '.join(camera_rpy),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('official_description_package', default_value='piper_description'),
        DeclareLaunchArgument('official_description_xacro', default_value='urdf/piper_description.xacro'),
        DeclareLaunchArgument('official_base_xyz', default_value='0.16 0.0 0.26'),
        DeclareLaunchArgument('official_base_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('official_tcp_parent_frame', default_value='piper_link6'),
        DeclareLaunchArgument('camera_xyz', default_value='0.04 0.0 0.04'),
        DeclareLaunchArgument('camera_rpy', default_value='0 0 0'),
        OpaqueFunction(function=_include_official_description),
    ])
