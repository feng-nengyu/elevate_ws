from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare the launch arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the Piper node.'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper node.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Start rviz flag.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='gripper'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='gripper'
    )

    tcp_offset_x_arg = DeclareLaunchArgument(
        'tcp_offset_x', default_value='0.0',
        description='TCP X offset in the J6 frame, in metres.'
    )
    tcp_offset_y_arg = DeclareLaunchArgument(
        'tcp_offset_y', default_value='0.0',
        description='TCP Y offset in the J6 frame, in metres.'
    )
    #修改的参数
    tcp_offset_z_arg = DeclareLaunchArgument(
        'tcp_offset_z', default_value='0.1358',
        description='TCP Z offset in the J6 frame, in metres.'
    )
    # Define the node
    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'tcp_offset_x': ParameterValue(LaunchConfiguration('tcp_offset_x'), value_type=float),
            'tcp_offset_y': ParameterValue(LaunchConfiguration('tcp_offset_y'), value_type=float),
            'tcp_offset_z': ParameterValue(LaunchConfiguration('tcp_offset_z'), value_type=float),
        }]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        tcp_offset_x_arg,
        tcp_offset_y_arg,
        tcp_offset_z_arg,
        piper_node
    ])
