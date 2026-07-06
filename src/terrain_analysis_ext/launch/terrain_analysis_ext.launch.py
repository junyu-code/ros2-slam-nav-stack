#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    odometry_topic = LaunchConfiguration('odometry_topic')
    local_terrain_topic = LaunchConfiguration('local_terrain_topic')
    output_terrain_topic = LaunchConfiguration('output_terrain_topic')
    joy_topic = LaunchConfiguration('joy_topic')
    clearing_topic = LaunchConfiguration('clearing_topic')
    check_terrain_conn = LaunchConfiguration('check_terrain_conn')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('odometry_topic', default_value='/Odometry'),
        DeclareLaunchArgument('local_terrain_topic', default_value='/terrain_map'),
        DeclareLaunchArgument('output_terrain_topic', default_value='/terrain_map_ext'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('clearing_topic', default_value='/cloud_clearing'),
        DeclareLaunchArgument('check_terrain_conn', default_value='false'),
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrain_analysis_ext',
            parameters=[{
                'use_sim_time': use_sim_time,
                'inputCloudTopic': input_cloud_topic,
                'odometryTopic': odometry_topic,
                'localTerrainTopic': local_terrain_topic,
                'outputTerrainTopic': output_terrain_topic,
                'joyTopic': joy_topic,
                'clearingTopic': clearing_topic,
                # 二阶段融合一阶段近场地形，维护更大范围滚动地形图。
                'scanVoxelSize': 0.05,
                'decayTime': 0.3,
                'noDecayDis': 0.0,
                'clearingDis': 20.0,
                'useSorting': True,
                'quantileZ': 0.55,
                'vehicleHeight': 0.5,
                'voxelPointUpdateThre': 200,
                'voxelTimeUpdateThre': 2.0,
                'lowerBoundZ': -1.0,
                'upperBoundZ': 2.0,
                'disRatioZ': 0.3,
                'checkTerrainConn': ParameterValue(check_terrain_conn, value_type=bool),
                'terrainConnThre': 0.5,
                'terrainUnderVehicle': -0.75,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 4.0,
            }],
            output='screen',
        ),
    ])
