"""Launch YOLO RGB-D localization with the eye-in-hand TF bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the YOLO and hand-eye launch description."""
    handeye_config = PathJoinSubstitution([
        FindPackageShare('piper_tf'),
        'config',
        'handeye.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to a YOLO11 detect .pt model',
        ),
        DeclareLaunchArgument('conda_env', default_value='yolo11'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('interest', default_value='all'),
        DeclareLaunchArgument('depth_threshold', default_value='2.0'),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('box_roi_inset', default_value='0.25'),
        DeclareLaunchArgument('plane_outer_scale', default_value='2.0'),
        DeclareLaunchArgument('plane_inner_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'plane_ransac_threshold',
            default_value='0.003',
        ),
        DeclareLaunchArgument('plane_min_points', default_value='100'),
        DeclareLaunchArgument(
            'plane_min_inlier_ratio',
            default_value='0.6',
        ),
        DeclareLaunchArgument('plane_max_rms', default_value='0.004'),
        DeclareLaunchArgument(
            'plane_max_depth_deviation',
            default_value='0.03',
        ),
        DeclareLaunchArgument('plane_sample_step', default_value='3'),
        DeclareLaunchArgument('plane_lock_sample_count', default_value='5'),
        DeclareLaunchArgument(
            'plane_lock_max_offset_spread',
            default_value='0.005',
        ),
        DeclareLaunchArgument(
            'plane_lock_max_angle_spread',
            default_value='0.052359878',
        ),
        DeclareLaunchArgument('conf_threshold', default_value='0.7'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('bg_removal', default_value='false'),
        DeclareLaunchArgument('target_frame_id', default_value='base_link'),
        DeclareLaunchArgument('camera_frame_id', default_value=''),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
        ),
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/depth/image_raw',
        ),
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
            executable='yolo_detect_3d',
            name='yolo_ros2',
            output='screen',
            prefix=[
                'conda run --no-capture-output -n ',
                LaunchConfiguration('conda_env'),
                ' python',
            ],
            parameters=[{
                'device': LaunchConfiguration('device'),
                'model_path': LaunchConfiguration('model_path'),
                'interest': LaunchConfiguration('interest'),
                'depth_threshold': ParameterValue(
                    LaunchConfiguration('depth_threshold'),
                    value_type=float,
                ),
                'depth_scale': ParameterValue(
                    LaunchConfiguration('depth_scale'),
                    value_type=float,
                ),
                'box_roi_inset': ParameterValue(
                    LaunchConfiguration('box_roi_inset'),
                    value_type=float,
                ),
                'plane_outer_scale': ParameterValue(
                    LaunchConfiguration('plane_outer_scale'),
                    value_type=float,
                ),
                'plane_inner_scale': ParameterValue(
                    LaunchConfiguration('plane_inner_scale'),
                    value_type=float,
                ),
                'plane_ransac_threshold': ParameterValue(
                    LaunchConfiguration('plane_ransac_threshold'),
                    value_type=float,
                ),
                'plane_min_points': ParameterValue(
                    LaunchConfiguration('plane_min_points'),
                    value_type=int,
                ),
                'plane_min_inlier_ratio': ParameterValue(
                    LaunchConfiguration('plane_min_inlier_ratio'),
                    value_type=float,
                ),
                'plane_max_rms': ParameterValue(
                    LaunchConfiguration('plane_max_rms'),
                    value_type=float,
                ),
                'plane_max_depth_deviation': ParameterValue(
                    LaunchConfiguration('plane_max_depth_deviation'),
                    value_type=float,
                ),
                'plane_sample_step': ParameterValue(
                    LaunchConfiguration('plane_sample_step'),
                    value_type=int,
                ),
                'plane_lock_sample_count': ParameterValue(
                    LaunchConfiguration('plane_lock_sample_count'),
                    value_type=int,
                ),
                'plane_lock_max_offset_spread': ParameterValue(
                    LaunchConfiguration('plane_lock_max_offset_spread'),
                    value_type=float,
                ),
                'plane_lock_max_angle_spread': ParameterValue(
                    LaunchConfiguration('plane_lock_max_angle_spread'),
                    value_type=float,
                ),
                'conf_threshold': ParameterValue(
                    LaunchConfiguration('conf_threshold'),
                    value_type=float,
                ),
                'iou_threshold': ParameterValue(
                    LaunchConfiguration('iou_threshold'),
                    value_type=float,
                ),
                'bg_removal': ParameterValue(
                    LaunchConfiguration('bg_removal'),
                    value_type=bool,
                ),
                'target_frame_id': LaunchConfiguration('target_frame_id'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'camera_info_topic': LaunchConfiguration(
                    'camera_info_topic',
                ),
                'color_image_topic': LaunchConfiguration(
                    'color_image_topic',
                ),
                'depth_image_topic': LaunchConfiguration(
                    'depth_image_topic',
                ),
            }],
        ),
    ])
