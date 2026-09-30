"""Tests for coarse-positioning diagnostic helpers."""

import numpy as np
import pytest

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectoryPoint

from piper_pbvs_control.pbvs_controller import PiperPbvsController


def test_moveit_goal_explicitly_sets_seven_percent_scaling():
    class Controller:
        base_frame = 'base_link'
        tcp_frame = 'tcp_link'
        move_group_name = 'arm'
        moveit_position_tolerance = 0.002
        moveit_orientation_tolerance = 0.05
        moveit_velocity_scaling_factor = 0.07
        moveit_acceleration_scaling_factor = 0.07

    target = PoseStamped()
    target.pose.orientation.w = 1.0

    goal = PiperPbvsController._moveit_goal(Controller(), target, False)

    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.07)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.07)


def _set_final_point(robot_trajectory, names, positions):
    trajectory = robot_trajectory.joint_trajectory
    trajectory.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    trajectory.points = [point]


def test_final_target_prefers_executed_trajectory_and_reorders_joints():
    result = MoveGroup.Result()
    _set_final_point(
        result.executed_trajectory,
        ['joint3', 'joint1', 'joint6', 'joint2', 'joint5', 'joint4'],
        [0.3, 0.1, 0.6, 0.2, 0.5, 0.4],
    )
    _set_final_point(
        result.planned_trajectory,
        PiperPbvsController.ARM_JOINT_NAMES,
        [1.0] * 6,
    )

    target = PiperPbvsController._final_moveit_arm_target(result)

    assert target == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def test_final_target_falls_back_to_planned_trajectory():
    result = MoveGroup.Result()
    expected = (-0.1, 0.2, -0.3, 0.4, -0.5, 0.6)
    _set_final_point(
        result.planned_trajectory,
        PiperPbvsController.ARM_JOINT_NAMES,
        expected,
    )

    target = PiperPbvsController._final_moveit_arm_target(result)

    assert target == expected


def test_final_target_returns_none_for_incomplete_joint_data():
    result = MoveGroup.Result()
    _set_final_point(
        result.executed_trajectory,
        ['joint1', 'joint2'],
        [0.1, 0.2],
    )

    assert PiperPbvsController._final_moveit_arm_target(result) is None


def test_arm_joint_formatter_includes_names_and_radian_values():
    text = PiperPbvsController._format_arm_joints(
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
    )

    assert 'joint1=+0.100000' in text
    assert 'joint2=-0.200000' in text
    assert 'joint6=-0.600000' in text


def test_x_advance_has_a_distinct_action_feedback_state():
    assert PiperPbvsController.STATE_LABELS['X_ADVANCE'] == 'MoveIt 按压移动'
    assert PiperPbvsController.X_POSITION_TOLERANCE == 0.006
    assert PiperPbvsController.X_ORIENTATION_TOLERANCE == 0.075


def test_x_advance_uses_measured_t0_and_only_offsets_base_x():
    """The added MoveIt request starts from measured T0, not desired C0."""
    class Publisher:
        message = None

        def publish(self, message):
            self.message = message

    class Logger:
        def info(self, _message):
            pass

    class Controller:
        distance_m = 0.08
        x_advance_axis_mode = 'base_x'
        desired_tcp_pub = Publisher()

        def __init__(self):
            self.state = None
            self.moveit_call = None

        def _set_state(self, state):
            self.state = state

        def _pose_message(self, position, quaternion):
            message = PoseStamped()
            message.pose.position.x = float(position[0])
            message.pose.position.y = float(position[1])
            message.pose.position.z = float(position[2])
            message.pose.orientation.x = float(quaternion[0])
            message.pose.orientation.y = float(quaternion[1])
            message.pose.orientation.z = float(quaternion[2])
            message.pose.orientation.w = float(quaternion[3])
            return message

        def get_logger(self):
            return Logger()

        def _format_xyz(self, position):
            return str(tuple(position))

        def _run_moveit(self, pose, goal_handle, stage):
            self.moveit_call = pose, goal_handle, stage

        def _verify_target_pose(
            self,
            _goal,
            position,
            quaternion,
            _movement_label,
        ):
            return position, quaternion

        def _stage_success(self, _state, _message):
            pass

    controller = Controller()
    goal_handle = object()
    measured_position = np.array([0.38, -0.02, 0.54])
    measured_quaternion = np.array([0.0, 0.0, 0.0, 1.0])

    PiperPbvsController._run_x_advance(
        controller,
        goal_handle,
        measured_position,
        measured_quaternion,
        measured_quaternion,
    )

    target_pose, called_goal, stage = controller.moveit_call
    assert controller.state == 'X_ADVANCE'
    assert called_goal is goal_handle
    assert stage == 'base-link X movement'
    assert np.allclose(
        [
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ],
        [0.46, -0.02, 0.54],
    )
    assert target_pose.pose.orientation.w == 1.0

    controller = Controller()
    controller.x_advance_axis_mode = 'panel_normal'
    panel_press_quaternion = np.array([
        np.sqrt(0.5),
        0.0,
        0.0,
        np.sqrt(0.5),
    ])

    PiperPbvsController._run_x_advance(
        controller,
        goal_handle,
        measured_position,
        measured_quaternion,
        panel_press_quaternion,
    )

    target_pose, called_goal, stage = controller.moveit_call
    assert called_goal is goal_handle
    assert stage == 'panel-normal movement'
    assert np.allclose(
        [
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ],
        [0.38, -0.10, 0.54],
    )
    assert target_pose.pose.orientation.w == 1.0
