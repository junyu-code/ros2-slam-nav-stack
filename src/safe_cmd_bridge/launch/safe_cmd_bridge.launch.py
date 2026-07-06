from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    enable_topic_output = LaunchConfiguration('enable_topic_output')
    enable_udp_output = LaunchConfiguration('enable_udp_output')
    enable_fault_stop = LaunchConfiguration('enable_fault_stop')
    fault_topic = LaunchConfiguration('fault_topic')
    udp_host = LaunchConfiguration('udp_host')
    udp_port = LaunchConfiguration('udp_port')

    default_params = PathJoinSubstitution([
        FindPackageShare('safe_cmd_bridge'),
        'config',
        'safe_cmd_bridge.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('output_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('enable_topic_output', default_value='true'),
        DeclareLaunchArgument('enable_udp_output', default_value='false'),
        DeclareLaunchArgument('enable_fault_stop', default_value='true'),
        DeclareLaunchArgument('fault_topic', default_value='/localization_fault'),
        DeclareLaunchArgument('udp_host', default_value='192.168.123.22'),
        DeclareLaunchArgument('udp_port', default_value='15000'),
        Node(
            package='safe_cmd_bridge',
            executable='safe_cmd_bridge_node.py',
            name='safe_cmd_bridge_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'input_topic': input_topic,
                    'output_topic': output_topic,
                    'enable_topic_output': ParameterValue(enable_topic_output, value_type=bool),
                    'enable_udp_output': ParameterValue(enable_udp_output, value_type=bool),
                    'enable_fault_stop': ParameterValue(enable_fault_stop, value_type=bool),
                    'fault_topic': fault_topic,
                    'udp_host': udp_host,
                    'udp_port': ParameterValue(udp_port, value_type=int),
                },
            ],
        ),
    ])
