from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    handeye_config = PathJoinSubstitution([
        FindPackageShare('piper_tf'),
        'config',
        'handeye.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('reference_frame', default_value='base_link'),
        DeclareLaunchArgument('marker_frame', default_value='camera_marker'),
        DeclareLaunchArgument('enable_handeye_tf', default_value='true'),
        DeclareLaunchArgument('handeye_parent_frame', default_value='link6'),
        DeclareLaunchArgument(
            'camera_link_frame',
            default_value='camera_link',
        ),
        DeclareLaunchArgument(
            'calibrated_frame',
            default_value='camera_link',
        ),
        DeclareLaunchArgument('board_squares_x', default_value='7'),
        DeclareLaunchArgument('board_squares_y', default_value='5'),
        DeclareLaunchArgument('square_length', default_value='0.035'),
        DeclareLaunchArgument('marker_length', default_value='0.026'),
        DeclareLaunchArgument('min_charuco_corners', default_value='4'),
        Node(
            package='piper_tf',
            executable='handeye_static_tf',
            name='handeye_static_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_handeye_tf')),
            parameters=[
                handeye_config,
                {
                    'parent_frame': LaunchConfiguration(
                        'handeye_parent_frame',
                    ),
                    'camera_link_frame': LaunchConfiguration(
                        'camera_link_frame',
                    ),
                    'calibrated_frame': LaunchConfiguration(
                        'calibrated_frame',
                    ),
                },
            ],
        ),
        Node(
            package='piper_vision',
            executable='charuco_detector',
            name='charuco_detector',
            parameters=[{
                'camera_frame': LaunchConfiguration('camera_frame'),
                'reference_frame': LaunchConfiguration('reference_frame'),
                'marker_frame': LaunchConfiguration('marker_frame'),
                'board_squares_x': LaunchConfiguration('board_squares_x'),
                'board_squares_y': LaunchConfiguration('board_squares_y'),
                'square_length': LaunchConfiguration('square_length'),
                'marker_length': LaunchConfiguration('marker_length'),
                'min_charuco_corners': LaunchConfiguration(
                    'min_charuco_corners',
                ),
            }],
            remappings=[
                ('/camera_info', '/camera/color/camera_info'),
                ('/image', '/camera/color/image_raw'),
            ],
        ),
    ])
