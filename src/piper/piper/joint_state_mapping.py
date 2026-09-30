"""Map Piper SDK feedback to the joint names used by MoveIt."""

import math


MOVEIT_JOINT_NAMES = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'joint7',
)


def moveit_joint_positions(
    piper_positions,
    gripper_multiplier,
    finger_limit=0.035,
):
    """Return six arm joints and one independent gripper position."""
    positions = tuple(float(value) for value in piper_positions)
    if len(positions) != 7:
        raise ValueError('Piper feedback must contain six joints and gripper')
    if not all(math.isfinite(value) for value in positions):
        raise ValueError('Piper joint feedback must be finite')

    multiplier = float(gripper_multiplier)
    if not math.isfinite(multiplier) or multiplier <= 0.0:
        raise ValueError('gripper_multiplier must be positive and finite')

    finger_position = min(
        max(positions[6] / multiplier, 0.0),
        float(finger_limit),
    )
    return (
        *positions[:6],
        finger_position,
    )
