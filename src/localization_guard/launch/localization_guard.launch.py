from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    publish_zero_on_fault = LaunchConfiguration('publish_zero_on_fault')

    default_params = PathJoinSubstitution([
        FindPackageShare('localization_guard'),
        'config',
        'localization_guard.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('publish_zero_on_fault', default_value='false'),
        Node(
            package='localization_guard',
            executable='localization_guard_node.py',
            name='localization_guard_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'publish_zero_on_fault': ParameterValue(
                        publish_zero_on_fault,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
