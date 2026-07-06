#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    nav2_dir = get_package_share_directory('nav2_bringup')

    fast_lio_config = LaunchConfiguration('fast_lio_config')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    localization_mode = LaunchConfiguration('localization_mode')
    static_map_to_odom_x = LaunchConfiguration('static_map_to_odom_x')
    static_map_to_odom_y = LaunchConfiguration('static_map_to_odom_y')
    static_map_to_odom_z = LaunchConfiguration('static_map_to_odom_z')
    static_map_to_odom_yaw = LaunchConfiguration('static_map_to_odom_yaw')
    static_map_to_odom_pitch = LaunchConfiguration('static_map_to_odom_pitch')
    static_map_to_odom_roll = LaunchConfiguration('static_map_to_odom_roll')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    def normalize_python_bool(value):
        # Nav2 官方 launch 里有 PythonExpression，布尔值要用 Python 可识别的 True/False。
        lowered = str(value).strip().lower()
        if lowered in ('true', '1', 'yes', 'on'):
            return 'True'
        if lowered in ('false', '0', 'no', 'off'):
            return 'False'
        return value

    def make_localization_include(context):
        # 先在当前 launch 作用域解析路径，再传给 Nav2 官方 launch，避免嵌套默认参数覆盖。
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file.perform(context),
                    'use_sim_time': use_sim_time.perform(context),
                    'params_file': nav2_params_file.perform(context),
                    'autostart': 'true',
                    'use_composition': normalize_python_bool(use_composition.perform(context)),
                    'container_name': container_name.perform(context),
                }.items(),
            )
        ]

    def make_navigation_include(context):
        # 同上，Nav2 参数文件必须在这里解析成实际路径。
        resolved_params_file = nav2_params_file.perform(context)
        return [
            LogInfo(msg=f'Using Nav2 params file: {resolved_params_file}'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time.perform(context),
                    'params_file': resolved_params_file,
                    'autostart': 'true',
                    'use_composition': normalize_python_bool(use_composition.perform(context)),
                    'container_name': container_name.perform(context),
                }.items(),
            )
        ]

    initial_pose_publisher = Node(
        package='slam_nav_bringup',
        executable='publish_initial_pose.py',
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'x': initial_pose_x,
            'y': initial_pose_y,
            'yaw': initial_pose_yaw,
            'settle_time': 2.0,
            'publish_count': 1,
        }],
        output='screen',
    )

    static_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            },
        ],
    )

    static_map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=[
            static_map_to_odom_x,
            static_map_to_odom_y,
            static_map_to_odom_z,
            static_map_to_odom_yaw,
            static_map_to_odom_pitch,
            static_map_to_odom_roll,
            'map',
            'odom',
        ],
        output='screen',
    )

    static_localization = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'static'"])),
        actions=[
            static_map_server,
            static_map_lifecycle,
            static_map_to_odom,
        ],
    )

    amcl_localization = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        actions=[
            OpaqueFunction(function=make_localization_include),
        ],
    )

    static_navigation_start = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'static'"])),
        actions=[
            TimerAction(
                period=10.0,
                actions=[OpaqueFunction(function=make_navigation_include)],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use a Nav2 component container to reduce lifecycle service timeouts.',
        ),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='amcl',
            choices=['amcl', 'static'],
            description='Use AMCL localization or a fixed map->odom transform.',
        ),
        DeclareLaunchArgument('static_map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('static_map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('static_map_to_odom_z', default_value='0.0'),
        DeclareLaunchArgument('static_map_to_odom_yaw', default_value='0.0'),
        DeclareLaunchArgument('static_map_to_odom_pitch', default_value='0.0'),
        DeclareLaunchArgument('static_map_to_odom_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'fast_lio_config',
            default_value=os.path.join(fast_lio_dir, 'config', 'mid360.yaml'),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'map', 'nav_test_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Backward-compatible Nav2 parameter file argument.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=params_file,
            description='Nav2 parameter file passed into Nav2 bringup. Defaults to params_file.',
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_debug.rviz'),
        ),
        Node(
            # 参考成熟工程的做法，把 Nav2 组件加载进同一个容器，减少 WSL 高负载下生命周期服务握手超时。
            package='rclcpp_components',
            executable='component_container_mt',
            name=container_name,
            condition=IfCondition(use_composition),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[
                fast_lio_config,
                {
                    'use_sim_time': use_sim_time,
                    'feature_extract_enable': False,
                    'point_filter_num': 3,
                    'max_iteration': 3,
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.5,
                    'cube_side_length': 1000.0,
                    'runtime_pos_log_enable': False,
                    'pcd_save.pcd_save_en': False,
                },
            ],
            output='screen',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.20,
                'min_height': -0.30,
                'max_height': 0.45,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.2,
                'range_min': 0.35,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            output='screen',
        ),
        TimerAction(
            period=8.0,
            actions=[
                amcl_localization,
                initial_pose_publisher,
                static_localization,
            ],
        ),
        static_navigation_start,
        RegisterEventHandler(
            OnProcessExit(
                target_action=initial_pose_publisher,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[OpaqueFunction(function=make_navigation_include)],
                    ),
                ],
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
