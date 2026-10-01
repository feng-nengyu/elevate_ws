"""MoveIt launch with an opt-in acceleration model for wrist joints."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def apply_wrist_acceleration_limit(moveit_config, value):
    """Keep legacy limits at zero; override only joint4/joint6 when explicit."""
    limit = float(value)
    if not math.isfinite(limit) or not 0.0 <= limit <= 3.0:
        raise ValueError('wrist_joint_acceleration_limit must be finite in 0..3 rad/s^2')
    if limit == 0.0:
        return
    joints = moveit_config.joint_limits['robot_description_planning']['joint_limits']
    for name in ('joint4', 'joint6'):
        joints[name]['has_acceleration_limits'] = True
        joints[name]['max_acceleration'] = limit


def launch_move_group(context):
    moveit_config = MoveItConfigsBuilder(
        'piper', package_name='piper_with_gripper_moveit'
    ).to_moveit_configs()
    moveit_config.planning_pipelines['ompl'].update({
        'path_tolerance': 0.001,
        'resample_dt': 0.02,
    })
    apply_wrist_acceleration_limit(
        moveit_config, LaunchConfiguration('wrist_joint_acceleration_limit').perform(context)
    )
    return list(generate_move_group_launch(moveit_config).entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wrist_joint_acceleration_limit', default_value='0.0',
            description='Opt-in joint4/joint6 planning acceleration in rad/s^2; 0 keeps legacy limits; maximum 3.',
        ),
        OpaqueFunction(function=launch_move_group),
    ])
