#!/usr/bin/env python3

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _load_text(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return handle.read()


def _robot_description_command(context, package_share):
    description_share = get_package_share_directory('slam_nav_piper_description')
    simulation_share = get_package_share_path('slam_nav_simulation')
    builder = os.path.join(description_share, 'scripts', 'piper_description_builder.py')
    base_xacro = str(simulation_share / 'urdf' / 'mobile_robot.xacro')
    description_mode = context.perform_substitution(LaunchConfiguration('description_mode'))
    command = [
        'python3 ', builder,
        ' --arm-model official',
        ' --mount-xyz "', LaunchConfiguration('piper_mount_xyz'), '"',
        ' --mount-rpy "', LaunchConfiguration('piper_mount_rpy'), '"',
        ' --tcp-parent-link ', LaunchConfiguration('piper_tcp_parent_link'),
        ' --camera-xyz "', LaunchConfiguration('piper_camera_xyz'), '"',
        ' --camera-rpy "', LaunchConfiguration('piper_camera_rpy'), '"',
    ]

    if description_mode == 'mobile':
        command.extend([
            ' --base-xacro ', base_xacro,
            ' --enable-nav-rgbd-camera false',
            ' --enable-piper-arm true',
        ])

    return ParameterValue(Command(command), value_type=str)


def _create_nodes(context):
    package_share = Path(get_package_share_directory('slam_nav_piper_moveit_config'))
    config_dir = package_share / 'config'
    use_sim_time = _as_bool(context.perform_substitution(LaunchConfiguration('use_sim_time')))
    publish_robot_state = _as_bool(context.perform_substitution(LaunchConfiguration('publish_robot_state')))
    start_joint_state_publisher = _as_bool(
        context.perform_substitution(LaunchConfiguration('start_joint_state_publisher'))
    )
    allow_trajectory_execution = context.perform_substitution(
        LaunchConfiguration('allow_trajectory_execution')
    )
    joint_states_topic = LaunchConfiguration('joint_states_topic')

    robot_description = {'robot_description': _robot_description_command(context, package_share)}
    moveit_params = {
        'robot_description_semantic': _load_text(config_dir / 'piper.srdf'),
        'robot_description_kinematics': _load_yaml(config_dir / 'kinematics.yaml'),
        'robot_description_planning': {
            **_load_yaml(config_dir / 'joint_limits.yaml'),
            **_load_yaml(config_dir / 'pilz_cartesian_limits.yaml'),
        },
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': _load_yaml(config_dir / 'ompl_planning.yaml'),
        'moveit_manage_controllers': False,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': False,
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': ParameterValue(allow_trajectory_execution, value_type=bool),
        'monitor_dynamics': False,
        # 当前入口只做项目侧 MoveIt2 plan-only 验证，不连接 ros2_control/SDK 执行后端。
        'capabilities': '',
        'disable_capabilities': '',
    }
    moveit_params.update(_load_yaml(config_dir / 'moveit_controllers.yaml'))

    nodes = [
        LogInfo(msg='启动 Piper 项目侧 MoveIt2 plan-only 配置；不会接入 task1 或厂家 SDK。'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            namespace='piper',
            output='screen',
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
                ('joint_states', joint_states_topic),
            ],
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description,
                moveit_params,
            ],
        ),
    ]

    if publish_robot_state:
        if start_joint_state_publisher:
            nodes.append(
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='piper_joint_state_publisher',
                    namespace='piper',
                    remappings=[
                        ('joint_states', joint_states_topic),
                    ],
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        robot_description,
                    ],
                    output='screen',
                )
            )
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='piper_robot_state_publisher',
                namespace='piper',
                remappings=[
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                    ('joint_states', joint_states_topic),
                ],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    robot_description,
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'description_mode',
            default_value='standalone',
            choices=['standalone', 'mobile'],
            description='standalone 只发布 Piper 挂载链；mobile 使用移动底盘组合模型。',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='已有仿真 robot_state_publisher 时应设为 false，避免重复 TF。',
        ),
        DeclareLaunchArgument(
            'start_joint_state_publisher',
            default_value='true',
            description='plan-only 冒烟时发布假关节状态；接入 ros2_control 后关闭。',
        ),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value='false',
            description='默认只规划不执行；真实/仿真执行后端验证后再显式打开。',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/piper/joint_states',
            description='独立 plan-only 用 /piper/joint_states；和整车仿真共用时可设为 /joint_states。',
        ),
        DeclareLaunchArgument('piper_mount_xyz', default_value='0.16 0.0 0.22'),
        DeclareLaunchArgument('piper_mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('piper_tcp_parent_link', default_value='piper_link6'),
        DeclareLaunchArgument('piper_camera_xyz', default_value='0.04 0.0 0.04'),
        DeclareLaunchArgument('piper_camera_rpy', default_value='0 0 0'),
        OpaqueFunction(function=_create_nodes),
    ])
