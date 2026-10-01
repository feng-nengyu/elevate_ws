"""Launch the guarded Piper elevator-button stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, condition, arguments=None):
    """Create a conditional include for a package launch file."""
    launch_path = PathJoinSubstitution([
        FindPackageShare(package),
        'launch',
        launch_file,
    ])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        condition=IfCondition(LaunchConfiguration(condition)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    """Build the safe-by-default elevator task launch description."""
    arguments = [
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'PIPER_MODEL_PATH',
                default_value='',
            ),
            description=(
                'Absolute path to the YOLO detect model. Set this argument '
                'or the PIPER_MODEL_PATH environment variable.'
            ),
        ),
        DeclareLaunchArgument('conda_env', default_value='yolo11'),
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='YOLO device: empty, cpu, cuda:0, and so on.',
        ),
        DeclareLaunchArgument(
            'bg_removal',
            default_value='false',
            description='Keep RGB visible when close-range depth is invalid.',
        ),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument(
            'auto_enable',
            default_value='false',
            description='Keep false until all dry-run checks pass.',
        ),
        DeclareLaunchArgument('gripper_exist', default_value='true'),
        DeclareLaunchArgument('gripper_val_mutiple', default_value='2'),
        DeclareLaunchArgument('tcp_offset_x', default_value='0.0'),
        DeclareLaunchArgument('tcp_offset_y', default_value='0.0'),
        DeclareLaunchArgument('tcp_offset_z', default_value='0.1358'),
        DeclareLaunchArgument(
            'boundary_recovery_tolerance',
            default_value='0.08',
            description='Joint2/joint3 startup recovery window in radians.',
        ),
        DeclareLaunchArgument(
            'require_arm_initialization',
            default_value='true',
            description=(
                'Require /initialize_arm before real MoveIt execution.'
            ),
        ),
        DeclareLaunchArgument(
            'initialization_duration',
            default_value='6.0',
            description='Minimum direct Ready motion duration in seconds.',
        ),
        DeclareLaunchArgument(
            'initialization_speed_percent',
            default_value='20',
            description='Piper speed percentage during initialization.',
        ),
        DeclareLaunchArgument(
            'initialization_max_step',
            default_value='0.006',
            description='Maximum nominal 50 Hz initialization step.',
        ),
        DeclareLaunchArgument(
            'trajectory_speed_percent',
            default_value='20',
            description='Piper speed percentage for MoveIt trajectories.',
        ),
        DeclareLaunchArgument(
            'arm_goal_tolerance',
            default_value='0.01',
            description='Final real arm joint tolerance in radians.',
        ),
        DeclareLaunchArgument(
            'trajectory_settle_cycles',
            default_value='1',
            description='Required consecutive final in-tolerance samples.',
        ),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_serial_number', default_value=''),
        DeclareLaunchArgument('camera_usb_port', default_value=''),
        DeclareLaunchArgument(
            'camera_depth_width',
            default_value='640',
            description='Native depth width; 640x360 improves close range.',
        ),
        DeclareLaunchArgument(
            'camera_depth_height',
            default_value='360',
            description='Native depth height; use with width 640.',
        ),
        DeclareLaunchArgument('camera_depth_fps', default_value='30'),
        DeclareLaunchArgument(
            'camera_device_preset',
            default_value='High Density',
            description='Gemini preset used to improve valid-depth fill rate.',
        ),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_piper', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('start_pbvs', default_value='true'),
        DeclareLaunchArgument(
            'start_joint_zero_return',
            default_value='true',
            description=(
                'Expose the opt-in /return_all_joints_zero service.'
            ),
        ),
        DeclareLaunchArgument(
            'zero_velocity_scaling_factor',
            default_value='0.10',
            description='MoveIt velocity scaling for all-joint zero return.',
        ),
        DeclareLaunchArgument(
            'zero_acceleration_scaling_factor',
            default_value='0.10',
            description=(
                'MoveIt acceleration scaling for all-joint zero return.'
            ),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_handeye_tf', default_value='true'),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Allow physical MoveIt coarse positioning.',
        ),
        DeclareLaunchArgument(
            'floor_number',
            default_value='1',
            description=(
                'Default one- or two-digit floor used by a sequence goal.'
            ),
        ),

        # 设置返回 OK 按钮的位置和速度。
        DeclareLaunchArgument(
            'home_joint_positions',
            default_value=(
                '[-1.557051440, 0.234953236, -0.324353736, '
                '0.013745872, 0.203885472, -0.018106872]'
            ),
            description='Six arm home joint positions in radians.',
        ),
        DeclareLaunchArgument(
            'home_joint_tolerance',
            default_value='0.012',
        ),
        DeclareLaunchArgument(
            'home_velocity_scaling_factor',
            default_value='0.07',
            description='MoveIt velocity scaling used only for home return.',
        ),
        DeclareLaunchArgument(
            'home_acceleration_scaling_factor',
            default_value='0.07',
            description=(
                'MoveIt acceleration scaling used only for home return.'
            ),
        ),
        DeclareLaunchArgument('home_timeout', default_value='30.0'),
        DeclareLaunchArgument('press_timeout', default_value='120.0'),
        DeclareLaunchArgument(
            'moveit_velocity_scaling_factor',
            default_value='0.09',
            description=(
                'MoveIt velocity scaling for coarse and panel-normal moves.'
            ),
        ),
        DeclareLaunchArgument(
            'moveit_acceleration_scaling_factor',
            default_value='0.09',
            description=(
                'MoveIt acceleration scaling for coarse and panel-normal moves.'
            ),
        ),
        DeclareLaunchArgument(
            'orientation_mode',
            default_value='preserve_current_roll',
            description=(
                'TCP orientation policy: preserve_current_roll or world_up.'
            ),
        ),
        DeclareLaunchArgument('close_panel_sequence', default_value='false'),
        DeclareLaunchArgument('preplan_sequence', default_value='true'),
        DeclareLaunchArgument('preplan_retry_attempts', default_value='2'),
        DeclareLaunchArgument('transition_plan_candidates', default_value='3'),
        DeclareLaunchArgument('preplan_retry_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('sequence_snapshot_acquire_timeout', default_value='15.0'),
        DeclareLaunchArgument('sequence_retract_distance_mm', default_value='15.0'),
        DeclareLaunchArgument('transition_velocity_scaling_factor', default_value='0.35'),
        DeclareLaunchArgument('transition_acceleration_scaling_factor', default_value='0.35'),
        DeclareLaunchArgument('press_velocity_scaling_factor', default_value='0.17'),
        DeclareLaunchArgument('press_acceleration_scaling_factor', default_value='0.17'),
        DeclareLaunchArgument('retract_velocity_scaling_factor', default_value='0.2'),
        DeclareLaunchArgument('retract_acceleration_scaling_factor', default_value='0.2'),
        DeclareLaunchArgument('coarse_standoff', default_value='0.08'),
        DeclareLaunchArgument(
            'coarse_horizontal_offset',
            default_value='0.026',
            description=(
                'Coarse target horizontal compensation in metres; '
                'positive is left facing the panel.'
            ),
        ),
        DeclareLaunchArgument(
            'coarse_vertical_offset',
            default_value='0.007',
            description=(
                'Coarse target vertical compensation in metres; '
                'positive is up facing the panel.'
            ),
        ),
        DeclareLaunchArgument(
            'coarse_lateral_error_min',
            default_value='0.021',
            description='Minimum accepted panel lateral error in metres.',
        ),
        DeclareLaunchArgument(
            'coarse_lateral_error_max',
            default_value='0.032',
            description='Maximum accepted panel lateral error in metres.',
        ),
        DeclareLaunchArgument(
            'coarse_axial_tolerance',
            default_value='0.01',
            description='Allowed error around the coarse standoff.',
        ),
        DeclareLaunchArgument(
            'coarse_correction_attempts',
            default_value='0',
            description='MoveIt corrections after first coarse arrival.',
        ),
        DeclareLaunchArgument(
            'distance_mm',
            default_value='67.0',
            description=(
                'Optional post-coarse advance displacement in mm.'
            ),
        ),
        DeclareLaunchArgument(
            'x_advance_axis_mode',
            default_value='panel_normal',
            description='Advance axis: base_x or panel_normal.',
        ),
        DeclareLaunchArgument('plane_outer_scale', default_value='2.0'),
        DeclareLaunchArgument('plane_inner_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'plane_ransac_threshold',
            default_value='0.003',
        ),
        DeclareLaunchArgument('plane_min_points', default_value='60'),
        DeclareLaunchArgument(
            'plane_min_inlier_ratio',
            default_value='0.6',
        ),
        DeclareLaunchArgument('plane_max_rms', default_value='0.004'),
        DeclareLaunchArgument(
            'plane_max_depth_deviation',
            default_value='0.03',
        ),
        DeclareLaunchArgument('plane_sample_step', default_value='2'),
        DeclareLaunchArgument('plane_lock_sample_count', default_value='5'),
        DeclareLaunchArgument(
            'plane_lock_max_offset_spread',
            default_value='0.005',
        ),
        DeclareLaunchArgument(
            'plane_lock_max_angle_spread',
            default_value='0.052359878',
        ),
    ]

    camera = _include(
        'orbbec_camera',
        'gemini_330_series.launch.py',
        'start_camera',
        {
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_number': LaunchConfiguration(
                'camera_serial_number'
            ),
            'usb_port': LaunchConfiguration('camera_usb_port'),
            'depth_registration': 'true',
            'depth_format': 'Y16',
            'depth_width': LaunchConfiguration('camera_depth_width'),
            'depth_height': LaunchConfiguration('camera_depth_height'),
            'depth_fps': LaunchConfiguration('camera_depth_fps'),
            'device_preset': LaunchConfiguration('camera_device_preset'),
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
        },
    )
    piper = _include(
        'piper',
        'start_single_piper.launch.py',
        'start_piper',
        {
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'gripper_val_mutiple': LaunchConfiguration(
                'gripper_val_mutiple'
            ),
            'tcp_offset_x': LaunchConfiguration('tcp_offset_x'),
            'tcp_offset_y': LaunchConfiguration('tcp_offset_y'),
            'tcp_offset_z': LaunchConfiguration('tcp_offset_z'),
        },
    )
    moveit = _include(
        'piper_with_gripper_moveit',
        'real_feedback_demo.launch.py',
        'start_moveit',
        {
            'use_rviz': LaunchConfiguration('use_rviz'),
            'can_port': LaunchConfiguration('can_port'),
            'allow_trajectory_execution': LaunchConfiguration(
                'enable_motion'
            ),
            'require_arm_initialization': LaunchConfiguration(
                'require_arm_initialization'
            ),
            'boundary_recovery_tolerance': LaunchConfiguration(
                'boundary_recovery_tolerance'
            ),
            'initialization_duration': LaunchConfiguration(
                'initialization_duration'
            ),
            'initialization_speed_percent': LaunchConfiguration(
                'initialization_speed_percent'
            ),
            'initialization_max_step': LaunchConfiguration(
                'initialization_max_step'
            ),
            'trajectory_speed_percent': LaunchConfiguration(
                'trajectory_speed_percent'
            ),
            'arm_goal_tolerance': LaunchConfiguration(
                'arm_goal_tolerance'
            ),
            'trajectory_settle_cycles': LaunchConfiguration(
                'trajectory_settle_cycles'
            ),
        },
    )
    pbvs = _include(
        'piper_pbvs_control',
        'elevator_press.launch.py',
        'start_pbvs',
        {
            'model_path': LaunchConfiguration('model_path'),
            'conda_env': LaunchConfiguration('conda_env'),
            'device': LaunchConfiguration('device'),
            'bg_removal': LaunchConfiguration('bg_removal'),
            'enable_handeye_tf': LaunchConfiguration(
                'enable_handeye_tf'
            ),
            'enable_motion': LaunchConfiguration('enable_motion'),
            'floor_number': LaunchConfiguration('floor_number'),
            'home_joint_positions': LaunchConfiguration(
                'home_joint_positions'
            ),
            'home_joint_tolerance': LaunchConfiguration(
                'home_joint_tolerance'
            ),
            'home_velocity_scaling_factor': LaunchConfiguration(
                'home_velocity_scaling_factor'
            ),
            'home_acceleration_scaling_factor': LaunchConfiguration(
                'home_acceleration_scaling_factor'
            ),
            'home_timeout': LaunchConfiguration('home_timeout'),
            'press_timeout': LaunchConfiguration('press_timeout'),
            'moveit_velocity_scaling_factor': LaunchConfiguration(
                'moveit_velocity_scaling_factor'
            ),
            'moveit_acceleration_scaling_factor': LaunchConfiguration(
                'moveit_acceleration_scaling_factor'
            ),
            'orientation_mode': LaunchConfiguration('orientation_mode'),
            'close_panel_sequence': LaunchConfiguration('close_panel_sequence'),
            'preplan_sequence': LaunchConfiguration('preplan_sequence'),
            'preplan_retry_attempts': LaunchConfiguration('preplan_retry_attempts'),
            'transition_plan_candidates': LaunchConfiguration('transition_plan_candidates'),
            'preplan_retry_timeout_sec': LaunchConfiguration('preplan_retry_timeout_sec'),
            'sequence_snapshot_acquire_timeout': LaunchConfiguration('sequence_snapshot_acquire_timeout'),
            'sequence_retract_distance_mm': LaunchConfiguration('sequence_retract_distance_mm'),
            'transition_velocity_scaling_factor': LaunchConfiguration('transition_velocity_scaling_factor'),
            'transition_acceleration_scaling_factor': LaunchConfiguration('transition_acceleration_scaling_factor'),
            'press_velocity_scaling_factor': LaunchConfiguration('press_velocity_scaling_factor'),
            'press_acceleration_scaling_factor': LaunchConfiguration('press_acceleration_scaling_factor'),
            'retract_velocity_scaling_factor': LaunchConfiguration('retract_velocity_scaling_factor'),
            'retract_acceleration_scaling_factor': LaunchConfiguration('retract_acceleration_scaling_factor'),
            'coarse_standoff': LaunchConfiguration('coarse_standoff'),
            'coarse_horizontal_offset': LaunchConfiguration(
                'coarse_horizontal_offset'
            ),
            'coarse_vertical_offset': LaunchConfiguration(
                'coarse_vertical_offset'
            ),
            'coarse_lateral_error_min': LaunchConfiguration(
                'coarse_lateral_error_min'
            ),
            'coarse_lateral_error_max': LaunchConfiguration(
                'coarse_lateral_error_max'
            ),
            'coarse_axial_tolerance': LaunchConfiguration(
                'coarse_axial_tolerance'
            ),
            'coarse_correction_attempts': LaunchConfiguration(
                'coarse_correction_attempts'
            ),
            'distance_mm': LaunchConfiguration('distance_mm'),
            'x_advance_axis_mode': LaunchConfiguration(
                'x_advance_axis_mode'
            ),
            'plane_outer_scale': LaunchConfiguration('plane_outer_scale'),
            'plane_inner_scale': LaunchConfiguration('plane_inner_scale'),
            'plane_ransac_threshold': LaunchConfiguration(
                'plane_ransac_threshold'
            ),
            'plane_min_points': LaunchConfiguration(
                'plane_min_points'
            ),
            'plane_min_inlier_ratio': LaunchConfiguration(
                'plane_min_inlier_ratio'
            ),
            'plane_max_rms': LaunchConfiguration('plane_max_rms'),
            'plane_max_depth_deviation': LaunchConfiguration(
                'plane_max_depth_deviation'
            ),
            'plane_sample_step': LaunchConfiguration('plane_sample_step'),
            'plane_lock_sample_count': LaunchConfiguration(
                'plane_lock_sample_count'
            ),
            'plane_lock_max_offset_spread': LaunchConfiguration(
                'plane_lock_max_offset_spread'
            ),
            'plane_lock_max_angle_spread': LaunchConfiguration(
                'plane_lock_max_angle_spread'
            ),
        },
    )
    joint_zero_return = _include(
        'piper_pbvs_control',
        'joint_zero_return.launch.py',
        'start_joint_zero_return',
        {
            'enable_motion': LaunchConfiguration('enable_motion'),
            'zero_velocity_scaling_factor': LaunchConfiguration(
                'zero_velocity_scaling_factor'
            ),
            'zero_acceleration_scaling_factor': LaunchConfiguration(
                'zero_acceleration_scaling_factor'
            ),
        },
    )

    return LaunchDescription(
        arguments + [camera, piper, moveit, pbvs, joint_zero_return]
    )
