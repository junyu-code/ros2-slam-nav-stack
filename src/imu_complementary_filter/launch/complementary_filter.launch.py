from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    raw_imu_topic = LaunchConfiguration('raw_imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument('raw_imu_topic', default_value='/livox/imu'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
                remappings=[
                    ('/imu/data_raw', raw_imu_topic),
                ]
            )
        ]
    )
