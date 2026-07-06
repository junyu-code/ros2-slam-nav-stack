#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    terrain_analysis_dir = get_package_share_directory('terrain_analysis')
    terrain_analysis_ext_dir = get_package_share_directory('terrain_analysis_ext')

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
    fast_lio_config = LaunchConfiguration('fast_lio_config')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_rgbd_nav = LaunchConfiguration('enable_rgbd_nav')
    rgbd_depth_image_topic = LaunchConfiguration('rgbd_depth_image_topic')
    rgbd_camera_info_topic = LaunchConfiguration('rgbd_camera_info_topic')
    rgbd_output_cloud_topic = LaunchConfiguration('rgbd_output_cloud_topic')
    enable_terrain_analysis = LaunchConfiguration('enable_terrain_analysis')
    terrain_input_cloud_topic = LaunchConfiguration('terrain_input_cloud_topic')
    terrain_odometry_topic = LaunchConfiguration('terrain_odometry_topic')
    terrain_map_topic = LaunchConfiguration('terrain_map_topic')
    terrain_map_ext_topic = LaunchConfiguration('terrain_map_ext_topic')

    def normalize_python_bool(value):
        # Nav2 官方 launch 里有 PythonExpression，布尔值要用 Python 可识别的 True/False。
        lowered = str(value).strip().lower()
        if lowered in ('true', '1', 'yes', 'on'):
            return 'True'
        if lowered in ('false', '0', 'no', 'off'):
            return 'False'
        return value

    adaptive_cloud_filter = Node(
        package='perception_adapter',
        executable='adaptive_cloud_filter',
        name='adaptive_cloud_filter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud_topic': '/cloud_registered',
            'odom_topic': '/Odometry',
            'output_cloud_topic': '/cloud_nav_filtered',
            'obstacle_cloud_topic': '/nav_obstacle_cloud',
            'ground_cloud_topic': '/nav_ground_cloud',
            'mode_topic': '/perception_mode',
            'low_speed_threshold': 0.15,
            'high_speed_threshold': 0.55,
            'detail_voxel_size': 0.03,
            'normal_voxel_size': 0.08,
            'fast_voxel_size': 0.18,
            'safe_voxel_size': 0.05,
            # /cloud_registered 通常已经在全局/里程计坐标系下，ROI 先交给 costmap 和 scan 投影层处理。
            'roi_filter_enabled': False,
            'publish_split_clouds': True,
            'ground_min_height': -0.15,
            'ground_max_height': 0.06,
            'obstacle_min_height': 0.08,
            'obstacle_max_height': 1.20,
            'ground_estimation_enabled': True,
            'ground_grid_size': 0.35,
            'ground_height_margin': 0.08,
            'ground_seed_max_height': 0.25,
            'obstacle_relative_min_height': 0.12,
            'obstacle_check_enabled': False,
        }],
        output='screen',
    )

    depth_obstacle_projector = Node(
        package='rgbd_navigation_perception',
        executable='depth_obstacle_projector',
        name='depth_obstacle_projector',
        condition=IfCondition(enable_rgbd_nav),
        parameters=[{
            'use_sim_time': use_sim_time,
            'enabled': True,
            'depth_image_topic': rgbd_depth_image_topic,
            'camera_info_topic': rgbd_camera_info_topic,
            'output_cloud_topic': rgbd_output_cloud_topic,
            'set_enabled_service': '/rgbd_nav/set_enabled',
            'target_frame': 'base_footprint',
            'pixel_step': 4,
            'min_depth_m': 0.25,
            'max_depth_m': 5.0,
            'obstacle_min_height_m': 0.08,
            'obstacle_max_height_m': 1.20,
            'min_forward_m': 0.05,
            'max_forward_m': 4.0,
            'max_lateral_m': 2.0,
        }],
        output='screen',
    )

    terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_analysis_dir, 'launch', 'terrain_analysis.launch.py')
        ),
        condition=IfCondition(enable_terrain_analysis),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_cloud_topic': terrain_input_cloud_topic,
            'odometry_topic': terrain_odometry_topic,
            'output_terrain_topic': terrain_map_topic,
        }.items(),
    )

    terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_analysis_ext_dir, 'launch', 'terrain_analysis_ext.launch.py')
        ),
        condition=IfCondition(enable_terrain_analysis),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_cloud_topic': terrain_input_cloud_topic,
            'odometry_topic': terrain_odometry_topic,
            'local_terrain_topic': terrain_map_topic,
            'output_terrain_topic': terrain_map_ext_topic,
        }.items(),
    )

    def make_navigation_include(context):
        # 嵌套 launch 中存在同名参数时，先在父作用域解析成普通字符串，避免回落到子 launch 默认值。
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time.perform(context),
                    'use_composition': normalize_python_bool(use_composition.perform(context)),
                    'container_name': container_name.perform(context),
                    'localization_mode': localization_mode.perform(context),
                    'static_map_to_odom_x': static_map_to_odom_x.perform(context),
                    'static_map_to_odom_y': static_map_to_odom_y.perform(context),
                    'static_map_to_odom_z': static_map_to_odom_z.perform(context),
                    'static_map_to_odom_yaw': static_map_to_odom_yaw.perform(context),
                    'static_map_to_odom_pitch': static_map_to_odom_pitch.perform(context),
                    'static_map_to_odom_roll': static_map_to_odom_roll.perform(context),
                    'fast_lio_config': fast_lio_config.perform(context),
                    'map': map_file.perform(context),
                    'nav2_params_file': navigation_params_file.perform(context),
                    'initial_pose_x': initial_pose_x.perform(context),
                    'initial_pose_y': initial_pose_y.perform(context),
                    'initial_pose_yaw': initial_pose_yaw.perform(context),
                    'rviz': rviz.perform(context),
                    'rviz_config': rviz_config.perform(context),
                }.items(),
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('localization_mode', default_value='amcl', choices=['amcl', 'static']),
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
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params_3d.yaml'),
            description='Backward-compatible 3D Nav2 parameter file argument.',
        ),
        DeclareLaunchArgument(
            'navigation_params_file',
            default_value=params_file,
            description='3D Nav2 parameter file passed into the nested navigation launch. Defaults to params_file.',
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_debug.rviz'),
        ),
        DeclareLaunchArgument(
            'enable_rgbd_nav',
            default_value='false',
            description='Enable optional RGB-D navigation obstacle projection.',
        ),
        DeclareLaunchArgument('rgbd_depth_image_topic', default_value='/nav_camera/depth/image_raw'),
        DeclareLaunchArgument('rgbd_camera_info_topic', default_value='/nav_camera/depth/camera_info'),
        DeclareLaunchArgument('rgbd_output_cloud_topic', default_value='/visual_obstacles'),
        DeclareLaunchArgument(
            'enable_terrain_analysis',
            default_value='true',
            description='Enable two-stage LiDAR terrain analysis for 3D costmaps.',
        ),
        DeclareLaunchArgument('terrain_input_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('terrain_odometry_topic', default_value='/Odometry'),
        DeclareLaunchArgument('terrain_map_topic', default_value='/terrain_map'),
        DeclareLaunchArgument('terrain_map_ext_topic', default_value='/terrain_map_ext'),
        adaptive_cloud_filter,
        terrain_analysis,
        terrain_analysis_ext,
        depth_obstacle_projector,
        OpaqueFunction(function=make_navigation_include),
    ])
