#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav_bringup')
    cloud_relocalization_dir = get_package_share_directory('cloud_relocalization')
    localization_guard_dir = get_package_share_directory('localization_guard')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    map_pcd_path = LaunchConfiguration('map_pcd_path')
    relocalization_method = LaunchConfiguration('relocalization_method')
    relocalization_input_cloud = LaunchConfiguration('relocalization_input_cloud')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'localization_mode': 'amcl',
            'map': map_file,
            'nav2_params_file': params_file,
            # 大场地地图来自建图原点，AMCL 初始位姿使用 map 坐标，不使用 Gazebo mesh 坐标。
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'initial_pose_xy_stddev': '1.00',
            'initial_pose_yaw_stddev': '0.90',
            # AMCL 的 /scan 从原始雷达 PointCloud2 投影，避免用已配准点云反向污染定位。
            'scan_cloud_topic': scan_cloud_topic,
            'scan_target_frame': 'livox_frame',
            'require_amcl_convergence': 'false',
            'amcl_convergence_timeout': '30.0',
            'continue_on_amcl_convergence_timeout': 'true',
        }.items(),
    )

    localization_guard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_guard_dir, 'launch', 'localization_guard.launch.py')
        ),
        launch_arguments={
            'publish_zero_on_fault': 'true',
        }.items(),
    )

    relocalization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cloud_relocalization_dir, 'launch', 'icp_relocalization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_pcd_path': map_pcd_path,
            'input_cloud_topic': relocalization_input_cloud,
            'registration_method': relocalization_method,
            'publish_tf': 'false',
            'auto_align': 'false',
            'crop_map_around_guess': 'true',
            'local_map_radius': '12.0',
            'fitness_score_threshold': '0.65',
            'map_leaf_size': '0.10',
            'scan_leaf_size': '0.08',
            'max_correspondence_distance': '1.5',
            'max_iterations': '60',
            'min_interval_sec': '4.0',
            'max_result_translation_jump': '2.5',
            'max_result_yaw_jump': '1.2',
            'ndt_resolution': '1.0',
            'ndt_step_size': '0.1',
        }.items(),
    )


    amcl_convergence_monitor = Node(
        package='slam_nav_bringup',
        executable='amcl_convergence_monitor.py',
        name='amcl_convergence_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'amcl_pose_topic': '/amcl_pose',
            'particle_cloud_topic': '/particle_cloud',
            'scan_topic': '/scan',
            'map_topic': '/map',
            # covariance ????????????????? TF????? scan-map ???
            'covariance_xy_threshold': 0.080,
            'covariance_yaw_threshold': 0.120,
            'tf_translation_threshold': 0.10,
            'tf_yaw_threshold': 0.07,
            'require_particle_cloud': False,
            'particle_rms_threshold': 0.60,
            'particle_max_radius_threshold': 1.50,
            'scan_mean_residual_threshold': 0.60,
            'scan_p90_residual_threshold': 1.20,
            'score_threshold': 75.0,
            'stable_required_sec': 1.5,
            'max_pose_age_sec': 0.0,
            'max_particle_age_sec': 0.0,
        }],
    )

    relocalization_amcl_bridge = Node(
        package='cloud_relocalization',
        executable='relocalization_amcl_bridge.py',
        name='relocalization_amcl_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'relocalization_pose_topic': '/relocalization/pose',
            'relocalization_status_topic': '/relocalization/status',
            'trigger_service': '/relocalization/trigger',
            'fault_topic': '/localization_fault',
            'initialpose_topic': '/initialpose',
            'odom_topic': '/Odometry',
            'amcl_converged_topic': '/amcl_converged',
            'amcl_convergence_score_topic': '/amcl_convergence_score',
            # AMCL 只负责启动期全局对齐；收敛后 bridge 接管并冻结 map->odom。
            # 正常行驶依赖 FAST-LIO/odom，只有定位守护报警且车体/雷达近似静止后才触发 GICP 恢复。
            'output_mode': 'tf',
            'manage_map_to_odom_tf': True,
            'bootstrap_requires_amcl_convergence': False,
            'bootstrap_min_age_sec': 15.0,
            'require_still_for_bootstrap_handoff': True,
            'require_sensor_still_for_bootstrap': False,
            'bootstrap_linear_threshold': 0.03,
            'bootstrap_angular_threshold': 0.05,
            'deactivate_amcl_after_bootstrap': True,
            'amcl_node_name': '/amcl',
            'tf_publish_rate_hz': 20.0,
            'use_amcl_quality_for_low_speed': False,
            'trigger_period_sec': 0.0,
            'trigger_on_fault': False,
            'publish_on_fault': True,
            'publish_on_correction': True,
            'correction_translation_threshold': 0.35,
            'correction_yaw_threshold': 0.20,
            'max_correction_translation': 2.5,
            'max_correction_yaw': 1.2,
            'min_publish_interval_sec': 8.0,
            'trigger_on_low_speed': True,
            'low_speed_linear_threshold': 0.12,
            'low_speed_angular_threshold': 0.20,
            'low_speed_hold_sec': 1.5,
            'low_speed_cooldown_sec': 12.0,
            'low_speed_score_threshold': 85.0,
            'low_speed_start_delay_sec': 12.0,
            'sensor_frame': 'livox_frame',
            'sensor_still_reference_frame': 'odom',
            'require_sensor_still_for_low_speed': True,
            'sensor_still_window_sec': 1.5,
            'sensor_still_translation_threshold': 0.025,
            'sensor_still_yaw_threshold': 0.025,
            'sensor_still_min_samples': 3,
            'require_known_area_for_relocalization': True,
            'known_area_check_radius_m': 0.80,
            'known_area_min_known_fraction': 0.75,
            'xy_stddev': 0.30,
            'yaw_stddev': 0.45,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'map', 'large_arena.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params_large_arena.yaml'),
        ),
        DeclareLaunchArgument(
            'map_pcd_path',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'slam_nav_ws',
                'src',
                'FAST_LIO',
                'PCD',
                'scan.pcd',
            ),
        ),
        DeclareLaunchArgument('relocalization_method', default_value='gicp'),
        DeclareLaunchArgument('relocalization_input_cloud', default_value='/cloud_registered'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar/pointcloud'),
        navigation,
        # 给 FAST-LIO、/scan 和 AMCL 留一点启动时间，再启动定位守护和点云重定位救援链路。
        TimerAction(period=8.0, actions=[localization_guard, amcl_convergence_monitor, relocalization, relocalization_amcl_bridge]),
    ])
