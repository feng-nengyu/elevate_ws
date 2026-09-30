"""Launch MoveIt with Piper feedback as the only /joint_states source."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build MoveIt with real Piper feedback and trajectory execution."""
    moveit_config = MoveItConfigsBuilder(
        'piper',
        package_name='piper_with_gripper_moveit',
    ).to_moveit_configs()
    package_path = moveit_config.package_path

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_path / 'launch/static_virtual_joint_tfs.launch.py')
        )
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_path / 'launch/rsp.launch.py')
        )
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_path / 'launch/move_group.launch.py')
        ),
        launch_arguments={
            'allow_trajectory_execution': LaunchConfiguration(
                'allow_trajectory_execution'
            ),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_path / 'launch/moveit_rviz.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    trajectory_controller = Node(
        package='piper',
        executable='piper_trajectory_controller',
        name='piper_trajectory_controller',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('start_trajectory_controller')
        ),
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'speed_percent': ParameterValue(
                LaunchConfiguration('trajectory_speed_percent'),
                value_type=int,
            ),
            'arm_goal_tolerance': ParameterValue(
                LaunchConfiguration('arm_goal_tolerance'),
                value_type=float,
            ),
            'arm_goal_settle_cycles': ParameterValue(
                LaunchConfiguration('trajectory_settle_cycles'),
                value_type=int,
            ),
            'require_initialization': ParameterValue(
                LaunchConfiguration('require_arm_initialization'),
                value_type=bool,
            ),
            'boundary_recovery_tolerance': ParameterValue(
                LaunchConfiguration('boundary_recovery_tolerance'),
                value_type=float,
            ),
            'initialization_duration': ParameterValue(
                LaunchConfiguration('initialization_duration'),
                value_type=float,
            ),
            'initialization_speed_percent': ParameterValue(
                LaunchConfiguration('initialization_speed_percent'),
                value_type=int,
            ),
            'initialization_max_step': ParameterValue(
                LaunchConfiguration('initialization_max_step'),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument(
            'trajectory_speed_percent',
            default_value='10',
            description='Piper global speed percentage for MoveIt motion.',
        ),
        DeclareLaunchArgument(
            'arm_goal_tolerance',
            default_value='0.01',
            description='Final real arm joint tolerance in radians.',
        ),
        DeclareLaunchArgument(
            'trajectory_settle_cycles',
            default_value='5',
            description='Required consecutive final in-tolerance samples.',
        ),
        DeclareLaunchArgument(
            'require_arm_initialization',
            default_value='true',
            description=(
                'Reject real trajectories until /initialize_arm succeeds.'
            ),
        ),
        DeclareLaunchArgument(
            'boundary_recovery_tolerance',
            default_value='0.08',
            description='Joint2/joint3 startup recovery window in radians.',
        ),
        DeclareLaunchArgument(
            'initialization_duration',
            default_value='12.0',
            description='Minimum direct Ready motion duration in seconds.',
        ),
        DeclareLaunchArgument(
            'initialization_speed_percent',
            default_value='12',
            description='Piper speed percentage during direct Ready motion.',
        ),
        DeclareLaunchArgument(
            'initialization_max_step',
            default_value='0.002',
            description='Maximum nominal 50 Hz joint step in radians.',
        ),
        DeclareLaunchArgument(
            'start_trajectory_controller',
            default_value='true',
            description='Start the real Piper trajectory bridge.',
        ),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value='false',
            description=(
                'Allow MoveIt to execute through the real Piper bridge.'
            ),
        ),
        static_tf,
        robot_state_publisher,
        move_group,
        rviz,
        trajectory_controller,
    ])
