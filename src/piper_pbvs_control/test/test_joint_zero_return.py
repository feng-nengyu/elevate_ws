"""Tests for the isolated seven-joint zero-return helpers."""

import math

import pytest
from sensor_msgs.msg import JointState

from piper_pbvs_control.joint_zero_return import (
    ARM_JOINT_NAMES,
    COMMAND_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
    make_zero_moveit_goal,
    zero_joint_errors,
)


@pytest.mark.parametrize(
    ('joint_names', 'group_name', 'tolerance'),
    [
        (ARM_JOINT_NAMES, 'arm', 0.01),
        (GRIPPER_JOINT_NAMES, 'gripper', 0.003),
    ],
)
def test_zero_goal_targets_every_group_joint(joint_names, group_name, tolerance):
    """The MoveIt goals contain only zero constraints for their group."""
    goal = make_zero_moveit_goal(
        joint_names,
        group_name,
        tolerance,
        False,
        0.02,
        0.03,
    )
    constraints = goal.request.goal_constraints[0].joint_constraints
    assert goal.request.group_name == group_name
    assert [item.joint_name for item in constraints] == list(joint_names)
    assert [item.position for item in constraints] == [0.0] * len(joint_names)
    assert all(item.tolerance_above == tolerance for item in constraints)
    assert goal.planning_options.plan_only is False
    assert goal.planning_options.replan is True


def test_zero_feedback_maps_all_seven_joints_by_name():
    """Feedback order does not affect seven-joint zero error checks."""
    message = JointState()
    message.name = list(reversed(COMMAND_JOINT_NAMES))
    message.position = [0.007, -0.006, 0.005, -0.004, 0.003, -0.002, 0.001]
    errors = zero_joint_errors(message)
    expected_by_name = dict(zip(message.name, message.position))
    assert errors == tuple(
        abs(expected_by_name[name]) for name in COMMAND_JOINT_NAMES
    )


@pytest.mark.parametrize(
    ('names', 'positions'),
    [
        (COMMAND_JOINT_NAMES[:-1], [0.0] * 6),
        (COMMAND_JOINT_NAMES, [0.0] * 6),
        (COMMAND_JOINT_NAMES, [0.0] * 6 + [math.nan]),
    ],
)
def test_zero_feedback_rejects_incomplete_or_non_finite_samples(
    names,
    positions,
):
    """Verification never treats malformed feedback as a zero state."""
    message = JointState()
    message.name = list(names)
    message.position = list(positions)
    assert zero_joint_errors(message) is None
