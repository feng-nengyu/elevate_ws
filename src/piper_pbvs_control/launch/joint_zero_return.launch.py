"""Launch only the opt-in seven-joint zero-return service node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the isolated all-joint zero-return launch description."""
    config = PathJoinSubstitution([
        FindPackageShare('piper_pbvs_control'),
        'config',
        'joint_zero_return.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument(
            'zero_velocity_scaling_factor',
            default_value='0.10',
        ),
        DeclareLaunchArgument(
            'zero_acceleration_scaling_factor',
            default_value='0.10',
        ),
        Node(
            package='piper_pbvs_control',
            executable='joint_zero_return',
            name='joint_zero_return',
            output='screen',
            parameters=[
                config,
                {
                    'enable_motion': ParameterValue(
                        LaunchConfiguration('enable_motion'),
                        value_type=bool,
                    ),
                    'zero_velocity_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'zero_velocity_scaling_factor'
                        ),
                        value_type=float,
                    ),
                    'zero_acceleration_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'zero_acceleration_scaling_factor'
                        ),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
