"""Tests for Piper-to-MoveIt joint feedback mapping."""

import pytest

from piper.joint_state_mapping import (
    MOVEIT_JOINT_NAMES,
    moveit_joint_positions,
)


def test_moveit_joint_names_match_robot_model():
    """Feedback contains the six axes and one independent gripper joint."""
    assert MOVEIT_JOINT_NAMES == tuple(
        f'joint{index}' for index in range(1, 8)
    )


def test_moveit_joint_positions_map_arm_and_gripper():
    """Six arm joints pass through and total opening becomes joint7."""
    result = moveit_joint_positions(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.04],
        gripper_multiplier=2,
    )

    assert result[:6] == pytest.approx(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    )
    assert result[6] == pytest.approx(0.02)


def test_moveit_joint_positions_clamp_to_urdf_limit():
    """Out-of-range gripper feedback must stay inside URDF limits."""
    result = moveit_joint_positions(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08],
        gripper_multiplier=2,
    )

    assert result[6] == pytest.approx(0.035)


@pytest.mark.parametrize('multiplier', [0, -1, float('nan')])
def test_moveit_joint_positions_reject_invalid_multiplier(multiplier):
    """An invalid command scaling value must not corrupt feedback."""
    with pytest.raises(ValueError):
        moveit_joint_positions([0.0] * 7, multiplier)
