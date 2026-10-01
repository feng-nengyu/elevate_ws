"""Tests for real Piper trajectory validation and interpolation."""

from types import SimpleNamespace

import pytest

from piper.trajectory_execution import (
    ARM_JOINTS,
    PiperCanFeedbackState,
    READY_POSITIONS,
    TrajectoryValidationError,
    normalize_trajectory,
    position_tolerances,
    sample_linear_trajectory,
    startup_state_error,
    updated_settle_count,
    violating_joint,
)


def test_ready_pose_contains_requested_arm_and_gripper_targets():
    """Initialization Ready pose contains all seven requested joints."""
    assert READY_POSITIONS == pytest.approx(
        (
            -1.557051440,
            0.234953236,
            -0.324353736,
            0.013745872,
            0.203885472,
            -0.018106872,
            0.0020,
        )
    )


def duration(seconds):
    """Create a ROS-like duration."""
    whole_seconds = int(seconds)
    return SimpleNamespace(
        sec=whole_seconds,
        nanosec=int((seconds - whole_seconds) * 1e9),
    )


def point(positions, seconds, velocities=()):
    """Create a ROS-like trajectory point."""
    return SimpleNamespace(
        positions=list(positions),
        velocities=list(velocities),
        accelerations=[],
        effort=[],
        time_from_start=duration(seconds),
    )


def trajectory(joint_names, points):
    """Create a ROS-like joint trajectory."""
    return SimpleNamespace(
        joint_names=list(joint_names),
        points=list(points),
    )


def test_normalize_reorders_moveit_joint_names():
    """Goals are executed in the controller's fixed joint order."""
    reversed_names = tuple(reversed(ARM_JOINTS))
    normalized = normalize_trajectory(
        trajectory(
            reversed_names,
            [point((0.4, 0.3, 0.2, -0.5, 0.5, 0.1), 1.0)],
        ),
        ARM_JOINTS,
    )

    assert normalized.joint_names == ARM_JOINTS
    assert normalized.positions == (
        (0.1, 0.5, -0.5, 0.2, 0.3, 0.4),
    )


@pytest.mark.parametrize(
    'bad_trajectory',
    (
        trajectory(
            ARM_JOINTS[:-1],
            [point((0.0,) * 5, 1.0)],
        ),
        trajectory(
            ARM_JOINTS,
            [
                point((0.0, 0.2, -0.2, 0.0, 0.0, 0.0), 1.0),
                point((0.0, 0.3, -0.3, 0.0, 0.0, 0.0), 1.0),
            ],
        ),
        trajectory(
            ARM_JOINTS,
            [point((0.0, -0.1, -0.2, 0.0, 0.0, 0.0), 1.0)],
        ),
    ),
)
def test_normalize_rejects_unsafe_goals(bad_trajectory):
    """Missing joints, repeated times, and limit violations are rejected."""
    with pytest.raises(TrajectoryValidationError):
        normalize_trajectory(bad_trajectory, ARM_JOINTS)


def test_linear_sampling_starts_from_measured_state():
    """A delayed first point is approached continuously from feedback."""
    normalized = normalize_trajectory(
        trajectory(
            ARM_JOINTS,
            [
                point((0.2, 0.4, -0.4, 0.2, 0.2, 0.2), 2.0),
                point((0.4, 0.6, -0.6, 0.4, 0.4, 0.4), 4.0),
            ],
        ),
        ARM_JOINTS,
    )
    sampled, velocities = sample_linear_trajectory(
        normalized,
        (0.0, 0.2, -0.2, 0.0, 0.0, 0.0),
        1.0,
    )

    assert sampled == pytest.approx(
        (0.1, 0.3, -0.3, 0.1, 0.1, 0.1)
    )
    assert velocities == pytest.approx(
        (0.1, 0.1, -0.1, 0.1, 0.1, 0.1)
    )


def test_requested_tolerance_overrides_default():
    """MoveIt may tighten an individual joint tolerance."""
    requested = [
        SimpleNamespace(name='joint3', position=0.02),
    ]
    tolerances = position_tolerances(requested, ARM_JOINTS, 0.5)

    assert tolerances[2] == 0.02
    assert violating_joint(
        ARM_JOINTS,
        (0.0, 0.0, 0.03, 0.0, 0.0, 0.0),
        tolerances,
    ) == 'joint3'


def test_settle_count_requires_consecutive_in_tolerance_samples():
    """One out-of-tolerance sample resets final-pose stabilization."""
    tolerances = (0.01,) * len(ARM_JOINTS)
    inside = (0.005,) * len(ARM_JOINTS)
    outside = (0.005, 0.005, 0.011, 0.005, 0.005, 0.005)

    count = 0
    for _ in range(4):
        count = updated_settle_count(
            ARM_JOINTS,
            inside,
            tolerances,
            count,
        )
    assert count == 4

    count = updated_settle_count(
        ARM_JOINTS,
        outside,
        tolerances,
        count,
    )
    assert count == 0

    for _ in range(5):
        count = updated_settle_count(
            ARM_JOINTS,
            inside,
            tolerances,
            count,
        )
    assert count == 5


def test_can_feedback_requires_all_six_fresh_enabled_frames():
    """One missing or disabled low-speed frame prevents real execution."""
    state = PiperCanFeedbackState()
    now = 10.0
    for can_id in state.ARM_POSITION_CAN_IDS:
        state.observe(can_id, bytes(8), now)
    for can_id in state.LOW_SPEED_CAN_IDS:
        state.observe(can_id, bytes([0, 0, 0, 0, 0, 0x40, 0, 0]), now)

    assert state.arm_ready(now, 0.5)

    state.observe(
        state.LOW_SPEED_CAN_IDS[-1],
        bytes(8),
        now + 0.1,
    )
    assert not state.arm_ready(now + 0.1, 0.5)


def test_stale_can_feedback_prevents_execution():
    """Cached CAN states are rejected even when their values look enabled."""
    state = PiperCanFeedbackState()
    for can_id in state.ARM_POSITION_CAN_IDS:
        state.observe(can_id, bytes(8), 1.0)
    for can_id in state.LOW_SPEED_CAN_IDS:
        state.observe(can_id, bytes([0, 0, 0, 0, 0, 0x40, 0, 0]), 1.0)

    assert not state.arm_ready(2.0, 0.5)


def test_startup_state_allows_only_small_joint2_joint3_boundary_error():
    """Only the two known zero-boundary errors receive recovery margin."""
    assert startup_state_error(
        [0.1, -0.04, 0.034, -0.1, 0.3, 0.0],
        0.08,
    ) is None
    assert 'joint2' in startup_state_error(
        [0.0, -0.081, 0.0, 0.0, 0.0, 0.0],
        0.08,
    )
    assert 'joint3' in startup_state_error(
        [0.0, 0.0, 0.081, 0.0, 0.0, 0.0],
        0.08,
    )
    assert 'joint1' in startup_state_error(
        [-2.7, 0.0, 0.0, 0.0, 0.0, 0.0],
        0.08,
    )


def test_smooth_interpolation_is_continuous_and_bounded():
    from piper.trajectory_execution import NormalizedTrajectory,prepare_smooth_trajectory,sample_smooth_trajectory
    t=NormalizedTrajectory(('joint1',),(0.,.1,.2),((0.,),(.02,),(.01,)),((0.,),(2.,),(0.,)))
    t=prepare_smooth_trajectory(t,(0.,))
    for i in range(200):
        stamp=i/1000
        p,v=sample_smooth_trajectory(t,(0.,),stamp)
        low,high=(0.,.02) if stamp<.1 else (.01,.02)
        assert low-1e-12<=p[0]<=high+1e-12
    left=sample_smooth_trajectory(t,(0.,),.1-1e-8)[1][0]
    right=sample_smooth_trajectory(t,(0.,),.1+1e-8)[1][0]
    assert left==pytest.approx(right,abs=1e-6)
    assert sample_smooth_trajectory(t,(0.,),0)[1]==(0.,)
    assert sample_smooth_trajectory(t,(0.,),.2)[1]==(0.,)
