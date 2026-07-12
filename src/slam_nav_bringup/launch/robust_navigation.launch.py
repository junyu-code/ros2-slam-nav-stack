#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from slam_nav_bringup.base_profiles import BUILTIN_PROFILES, resolve_base_profile_file


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav_bringup')
    localization_guard_dir = get_package_share_directory('localization_guard')
    safe_cmd_bridge_dir = get_package_share_directory('safe_cmd_bridge')

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
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    base_profile = LaunchConfiguration('base_profile')
    base_profile_file = LaunchConfiguration('base_profile_file')
    fast_lio_config = LaunchConfiguration('fast_lio_config')
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
    enable_localization_guard = LaunchConfiguration('enable_localization_guard')
    enable_safe_cmd_bridge = LaunchConfiguration('enable_safe_cmd_bridge')
    publish_zero_on_fault = LaunchConfiguration('publish_zero_on_fault')
    safe_input_topic = LaunchConfiguration('safe_input_topic')
    safe_output_topic = LaunchConfiguration('safe_output_topic')
    safe_enable_udp_output = LaunchConfiguration('safe_enable_udp_output')
    safe_enable_fault_stop = LaunchConfiguration('safe_enable_fault_stop')
    safe_fault_topic = LaunchConfiguration('safe_fault_topic')
    safe_enable_feedback_watchdog = LaunchConfiguration('safe_enable_feedback_watchdog')
    safe_feedback_topic = LaunchConfiguration('safe_feedback_topic')
    safe_udp_host = LaunchConfiguration('safe_udp_host')
    safe_udp_port = LaunchConfiguration('safe_udp_port')

    navigation_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_3d.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_composition': use_composition,
            'container_name': container_name,
            'localization_mode': localization_mode,
            'static_map_to_odom_x': static_map_to_odom_x,
            'static_map_to_odom_y': static_map_to_odom_y,
            'static_map_to_odom_z': static_map_to_odom_z,
            'static_map_to_odom_yaw': static_map_to_odom_yaw,
            'static_map_to_odom_pitch': static_map_to_odom_pitch,
            'static_map_to_odom_roll': static_map_to_odom_roll,
            'map': map_file,
            'navigation_params_file': params_file,
            'base_profile': base_profile,
            'base_profile_file': base_profile_file,
            'fast_lio_config': fast_lio_config,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'rviz': rviz,
            'rviz_config': rviz_config,
            'enable_rgbd_nav': enable_rgbd_nav,
            'rgbd_depth_image_topic': rgbd_depth_image_topic,
            'rgbd_camera_info_topic': rgbd_camera_info_topic,
            'rgbd_output_cloud_topic': rgbd_output_cloud_topic,
            'enable_terrain_analysis': enable_terrain_analysis,
            'terrain_input_cloud_topic': terrain_input_cloud_topic,
            'terrain_odometry_topic': terrain_odometry_topic,
            'terrain_map_topic': terrain_map_topic,
            'terrain_map_ext_topic': terrain_map_ext_topic,
        }.items(),
    )

    localization_guard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_guard_dir, 'launch', 'localization_guard.launch.py')
        ),
        condition=IfCondition(enable_localization_guard),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_zero_on_fault': publish_zero_on_fault,
        }.items(),
    )

    def make_safe_cmd_bridge(context):
        resolved_profile = resolve_base_profile_file(
            bringup_dir,
            base_profile.perform(context),
            base_profile_file.perform(context),
        )
        return [
            LogInfo(msg=f'Using safe command limits from: {resolved_profile}'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(safe_cmd_bridge_dir, 'launch', 'safe_cmd_bridge.launch.py')
                ),
                launch_arguments={
                    'params_file': str(resolved_profile),
                    'input_topic': safe_input_topic.perform(context),
                    'output_topic': safe_output_topic.perform(context),
                    'enable_udp_output': safe_enable_udp_output.perform(context),
                    'enable_fault_stop': safe_enable_fault_stop.perform(context),
                    'fault_topic': safe_fault_topic.perform(context),
                    'enable_feedback_watchdog': safe_enable_feedback_watchdog.perform(context),
                    'feedback_topic': safe_feedback_topic.perform(context),
                    'udp_host': safe_udp_host.perform(context),
                    'udp_port': safe_udp_port.perform(context),
                }.items(),
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
            'map',
            default_value=os.path.join(bringup_dir, 'map', 'nav_test_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params_3d.yaml'),
        ),
        DeclareLaunchArgument(
            'base_profile',
            default_value='omni',
            choices=list(BUILTIN_PROFILES),
            description='Chassis kinematics, footprint, and velocity limit profile.',
        ),
        DeclareLaunchArgument(
            'base_profile_file',
            default_value='',
            description='Optional custom chassis profile YAML; overrides base_profile when set.',
        ),
        DeclareLaunchArgument(
            'fast_lio_config',
            default_value=os.path.join(get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'),
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_debug.rviz'),
        ),
        DeclareLaunchArgument('enable_rgbd_nav', default_value='false'),
        DeclareLaunchArgument('rgbd_depth_image_topic', default_value='/nav_camera/depth/image_raw'),
        DeclareLaunchArgument('rgbd_camera_info_topic', default_value='/nav_camera/depth/camera_info'),
        DeclareLaunchArgument('rgbd_output_cloud_topic', default_value='/visual_obstacles'),
        DeclareLaunchArgument('enable_terrain_analysis', default_value='true'),
        DeclareLaunchArgument('terrain_input_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('terrain_odometry_topic', default_value='/Odometry'),
        DeclareLaunchArgument('terrain_map_topic', default_value='/terrain_map'),
        DeclareLaunchArgument('terrain_map_ext_topic', default_value='/terrain_map_ext'),
        DeclareLaunchArgument('enable_localization_guard', default_value='true'),
        DeclareLaunchArgument('enable_safe_cmd_bridge', default_value='true'),
        DeclareLaunchArgument('publish_zero_on_fault', default_value='false'),
        DeclareLaunchArgument('safe_input_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('safe_output_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('safe_enable_udp_output', default_value='false'),
        DeclareLaunchArgument('safe_enable_fault_stop', default_value='true'),
        DeclareLaunchArgument('safe_fault_topic', default_value='/localization_fault'),
        DeclareLaunchArgument('safe_enable_feedback_watchdog', default_value='false'),
        DeclareLaunchArgument('safe_feedback_topic', default_value='/base/odom'),
        DeclareLaunchArgument('safe_udp_host', default_value='192.168.123.22'),
        DeclareLaunchArgument('safe_udp_port', default_value='15000'),
        navigation_3d,
        localization_guard,
        OpaqueFunction(
            function=make_safe_cmd_bridge,
            condition=IfCondition(enable_safe_cmd_bridge),
        ),
    ])
