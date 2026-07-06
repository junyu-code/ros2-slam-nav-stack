#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    odometry_topic = LaunchConfiguration('odometry_topic')
    output_terrain_topic = LaunchConfiguration('output_terrain_topic')
    joy_topic = LaunchConfiguration('joy_topic')
    clearing_topic = LaunchConfiguration('clearing_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('odometry_topic', default_value='/Odometry'),
        DeclareLaunchArgument('output_terrain_topic', default_value='/terrain_map'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('clearing_topic', default_value='/map_clearing'),
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrain_analysis',
            parameters=[{
                'use_sim_time': use_sim_time,
                'inputCloudTopic': input_cloud_topic,
                'odometryTopic': odometry_topic,
                'outputTerrainTopic': output_terrain_topic,
                'joyTopic': joy_topic,
                'clearingTopic': clearing_topic,
                # 一阶段保留近场滚动地形估计，输出带 intensity 的局部地形代价点云。
                'scanVoxelSize': 0.05,
                'decayTime': 0.6,
                'noDecayDis': 0.0,
                'clearingDis': 0.0,
                'useSorting': True,
                'quantileZ': 0.6,
                'considerDrop': False,
                'limitGroundLift': False,
                'maxGroundLift': 0.3,
                'clearDyObs': False,
                'minDyObsDis': 0.4,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': -0.3,
                'absDyObsRelZThre': 0.2,
                'minDyObsVFOV': -28.0,
                'maxDyObsVFOV': 33.0,
                'minDyObsPointNum': 1,
                'noDataObstacle': False,
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 10,
                'vehicleHeight': 0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 1.0,
                'minRelZ': -1.0,
                'maxRelZ': 2.0,
                'disRatioZ': 0.3,
            }],
            output='screen',
        ),
    ])
