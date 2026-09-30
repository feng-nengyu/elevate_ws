"""Launch the eye-in-hand static TF bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the hand-eye TF launch description."""
    config_file = PathJoinSubstitution([
        FindPackageShare('piper_tf'),
        'config',
        'handeye.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('parent_frame', default_value='link6'),
        DeclareLaunchArgument(
            'camera_link_frame',
            default_value='camera_link',
        ),
        DeclareLaunchArgument(
            'calibrated_frame',
            default_value='camera_link',
        ),
        Node(
            package='piper_tf',
            executable='handeye_static_tf',
            name='handeye_static_tf',
            output='screen',
            parameters=[
                config_file,
                {
                    'parent_frame': LaunchConfiguration('parent_frame'),
                    'camera_link_frame': LaunchConfiguration(
                        'camera_link_frame',
                    ),
                    'calibrated_frame': LaunchConfiguration(
                        'calibrated_frame',
                    ),
                },
            ],
        ),
    ])
