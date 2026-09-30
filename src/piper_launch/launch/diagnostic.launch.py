"""Inspect the key topic rates of a running Piper elevator stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def _topic_rate(topic):
    """Run a bounded ros2 topic frequency measurement."""
    return ExecuteProcess(
        cmd=[
            'timeout',
            LaunchConfiguration('duration'),
            'ros2',
            'topic',
            'hz',
            topic,
        ],
        output='screen',
    )


def generate_launch_description():
    """Measure camera, hardware-feedback, and perception topic rates."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'duration',
            default_value='8',
            description='Measurement duration in seconds.',
        ),
        LogInfo(msg='Measuring Piper elevator stack topic rates...'),
        _topic_rate('/camera/color/image_raw'),
        _topic_rate('/camera/depth/image_raw'),
        _topic_rate('/arm_status'),
        _topic_rate('/joint_states'),
        _topic_rate('/tcp_pose'),
        _topic_rate('/piper_vision/button_pose'),
    ])
