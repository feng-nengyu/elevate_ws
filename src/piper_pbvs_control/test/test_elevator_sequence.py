"""Tests for elevator sequence input and home-goal helpers."""

import math
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import JointState

from piper_pbvs_control.elevator_sequence import (
    ARM_JOINT_NAMES,
    DEFAULT_HOME_JOINT_POSITIONS,
    HOME_JOINT_ACCEPTANCE_SLACK,
    SequenceFailure,
    home_joint_error_accepted,
    home_joint_errors,
    make_home_moveit_goal,
    normalize_floor_target,
    normalize_floor_targets,
    validate_home_joint_positions,
)


@pytest.mark.parametrize(
    ('requested', 'default_floor', 'expected'),
    [
        ('', 1, 'key_1'),
        ('3', 1, 'key_3'),
        ('key_8', 1, 'key_8'),
        (' 0 ', 1, 'key_0'),
    ],
)
def test_floor_target_uses_default_or_request(
    requested,
    default_floor,
    expected,
):
    """Empty input uses the parameter and valid overrides are normalized."""
    assert normalize_floor_target(requested, default_floor) == expected


@pytest.mark.parametrize('target', ['-1', 'ok', 'key_ok', 'floor_3'])
def test_floor_target_rejects_non_numeric_buttons(target):
    """The sequence API only accepts one numeric elevator key."""
    with pytest.raises(ValueError):
        normalize_floor_target(target, 1)


@pytest.mark.parametrize(
    ('requested', 'default_floor', 'expected'),
    [
        ('10', 1, ('key_1', 'key_0')),
        ('key_10', 1, ('key_1', 'key_0')),
        ('', 10, ('key_1', 'key_0')),
    ],
)
def test_two_digit_floor_is_split_into_two_button_targets(
    requested,
    default_floor,
    expected,
):
    assert normalize_floor_targets(requested, default_floor) == expected


@pytest.mark.parametrize('target', ['100', 'key_100', '1.0'])
def test_floor_target_rejects_more_than_two_or_invalid_digits(target):
    with pytest.raises(ValueError):
        normalize_floor_targets(target, 1)


def test_home_positions_require_six_finite_values():
    """Malformed home parameters are rejected before the node starts."""
    assert validate_home_joint_positions(
        DEFAULT_HOME_JOINT_POSITIONS
    ) == DEFAULT_HOME_JOINT_POSITIONS
    with pytest.raises(ValueError):
        validate_home_joint_positions([0.0] * 5)
    with pytest.raises(ValueError):
        validate_home_joint_positions([0.0] * 5 + [math.nan])


def test_default_home_matches_initial_arm_pose():
    """Button sequences return the arm to the requested initial pose."""
    assert DEFAULT_HOME_JOINT_POSITIONS == pytest.approx(
        (
            -1.557051440,
            0.234953236,
            -0.324353736,
            0.013745872,
            0.203885472,
            -0.018106872,
        )
    )


def test_home_errors_map_joint_state_by_name():
    """Joint feedback order does not affect home verification."""
    message = JointState()
    message.name = [
        'joint3',
        'joint1',
        'joint6',
        'joint2',
        'joint5',
        'joint4',
    ]
    message.position = [
        -0.313453736,
        -1.547051440,
        -0.038106872,
        0.238553236,
        0.233885472,
        -0.026254128,
    ]
    errors = home_joint_errors(message, DEFAULT_HOME_JOINT_POSITIONS)
    assert errors == pytest.approx((0.01, 0.0036, 0.0109, 0.04, 0.03, 0.02))


def test_home_errors_reject_incomplete_feedback():
    """Feedback missing any arm joint cannot pass home verification."""
    message = JointState()
    message.name = ['joint1', 'joint2']
    message.position = [0.0, 0.4164]
    assert home_joint_errors(message, DEFAULT_HOME_JOINT_POSITIONS) is None


@pytest.mark.parametrize(
    ('max_error', 'expected'),
    [
        (0.016999, True),
        (0.017, True),
        (0.017363, True),
        (0.087363, False),
    ],
)
def test_home_feedback_accepts_small_tolerance_overrun(
    max_error,
    expected,
):
    """Less than 0.001 rad overrun passes final home verification."""
    assert home_joint_error_accepted(max_error, 0.017) is expected


def test_home_feedback_rejects_exact_slack_boundary():
    """An overrun equal to the fixed slack remains a failure."""
    max_error = 0.017 + HOME_JOINT_ACCEPTANCE_SLACK
    assert home_joint_error_accepted(max_error, 0.017) is False


def test_home_moveit_goal_contains_six_joint_constraints():
    """The return goal targets every arm joint and respects dry-run."""
    goal = make_home_moveit_goal(
        DEFAULT_HOME_JOINT_POSITIONS,
        tolerance=0.01,
        group_name='arm',
        plan_only=True,
        velocity_scaling_factor=0.01,
        acceleration_scaling_factor=0.02,
    )
    assert goal.request.group_name == 'arm'
    constraints = goal.request.goal_constraints[0].joint_constraints
    assert tuple(item.joint_name for item in constraints) == ARM_JOINT_NAMES
    assert tuple(item.position for item in constraints) == pytest.approx(
        DEFAULT_HOME_JOINT_POSITIONS
    )
    assert all(item.tolerance_above == pytest.approx(0.01)
               for item in constraints)
    assert goal.planning_options.plan_only is True
    assert goal.planning_options.replan is False
    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.01)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.02)


def test_sequence_order_is_number_home_ok_home():
    """A successful task cannot reorder either press or home operation."""
    from piper_pbvs_control.elevator_sequence import ElevatorSequence

    calls = []

    class FakeSequence:
        floor_number = 1
        current_state = 'IDLE'
        active_press_goal = None
        active_move_goal = None
        active_lock = SimpleNamespace(
            __enter__=lambda self: self,
            __exit__=lambda self, *args: None,
        )
        task_active = True

        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None, error=lambda *_: None)

        def _run_press(self, _goal, target, state):
            calls.append((state, target))

        def _run_home(self, _goal, state):
            calls.append((state, 'home'))

        def _set_state(self, state):
            self.current_state = state

        def _result(self, success, message):
            return SimpleNamespace(success=success, message=message)

    class FakeGoal:
        request = SimpleNamespace(target_name='4')

        def succeed(self):
            calls.append(('RESULT', 'success'))

        def abort(self):
            calls.append(('RESULT', 'abort'))

        def canceled(self):
            calls.append(('RESULT', 'canceled'))

    fake = FakeSequence()
    # Use a real lock for the cleanup path.
    import threading
    fake.active_lock = threading.Lock()
    result = ElevatorSequence._execute_sequence(fake, FakeGoal())
    assert result.success is True
    assert calls == [
        ('PRESS_NUMBER', 'key_4'),
        ('RETURN_AFTER_NUMBER', 'home'),
        ('PRESS_OK', 'key_ok'),
        ('RETURN_AFTER_OK', 'home'),
        ('RESULT', 'success'),
    ]


def test_two_digit_sequence_returns_home_after_each_digit():
    """Floor 10 is key_1, home, key_0, home, key_ok, and home."""
    import threading

    from piper_pbvs_control.elevator_sequence import ElevatorSequence

    calls = []

    class FakeSequence:
        floor_number = 1
        current_state = 'IDLE'
        active_press_goal = None
        active_move_goal = None
        active_lock = threading.Lock()
        task_active = True

        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None, error=lambda *_: None)

        def _run_press(self, _goal, target, state):
            calls.append((state, target))

        def _run_home(self, _goal, state):
            calls.append((state, 'home'))

        def _set_state(self, state):
            self.current_state = state

        def _result(self, success, message):
            return SimpleNamespace(success=success, message=message)

    class FakeGoal:
        request = SimpleNamespace(target_name='10')

        def succeed(self):
            calls.append(('RESULT', 'success'))

        def abort(self):
            calls.append(('RESULT', 'abort'))

        def canceled(self):
            calls.append(('RESULT', 'canceled'))

    result = ElevatorSequence._execute_sequence(FakeSequence(), FakeGoal())
    assert result.success is True
    assert calls == [
        ('PRESS_NUMBER', 'key_1'),
        ('RETURN_AFTER_NUMBER', 'home'),
        ('PRESS_NUMBER', 'key_0'),
        ('RETURN_AFTER_NUMBER', 'home'),
        ('PRESS_OK', 'key_ok'),
        ('RETURN_AFTER_OK', 'home'),
        ('RESULT', 'success'),
    ]


def test_failed_first_home_prevents_ok_press():
    """A failed return after the number must stop the sequence."""
    import threading

    from piper_pbvs_control.elevator_sequence import ElevatorSequence

    calls = []

    class FakeSequence:
        floor_number = 1
        current_state = 'IDLE'
        active_press_goal = None
        active_move_goal = None
        active_lock = threading.Lock()
        task_active = True

        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None, error=lambda *_: None)

        def _run_press(self, _goal, target, state):
            calls.append((state, target))

        def _run_home(self, _goal, state):
            calls.append((state, 'home'))
            raise SequenceFailure('home failed')

        def _set_state(self, state):
            self.current_state = state

        def _result(self, success, message):
            return SimpleNamespace(success=success, message=message)

    class FakeGoal:
        request = SimpleNamespace(target_name='2')

        def succeed(self):
            calls.append(('RESULT', 'success'))

        def abort(self):
            calls.append(('RESULT', 'abort'))

        def canceled(self):
            calls.append(('RESULT', 'canceled'))

    result = ElevatorSequence._execute_sequence(FakeSequence(), FakeGoal())
    assert result.success is False
    assert calls == [
        ('PRESS_NUMBER', 'key_2'),
        ('RETURN_AFTER_NUMBER', 'home'),
        ('RESULT', 'abort'),
    ]


@pytest.mark.parametrize('motion_state', ['COARSE_APPROACH', 'X_ADVANCE'])
def test_motion_stage_failure_attempts_recovery_home(motion_state):
    """Coarse and X failures both trigger the guarded recovery home."""
    import threading

    from piper_pbvs_control.elevator_sequence import ElevatorSequence

    calls = []

    class FakeSequence:
        data_lock = threading.Lock()
        last_press_motion_state = motion_state

        def get_logger(self):
            return SimpleNamespace(warning=lambda *_: None)

        def _run_home(self, _goal, state):
            calls.append(state)

    with pytest.raises(SequenceFailure, match='recovered to home'):
        ElevatorSequence._recover_after_press_failure(
            FakeSequence(),
            SimpleNamespace(),
            'key_3',
            'MoveIt failed',
        )
    assert calls == ['RECOVERY_HOME']


def test_detection_failure_does_not_send_recovery_motion():
    """WAIT_TARGET failure preserves the no-recognition/no-motion rule."""
    import threading

    from piper_pbvs_control.elevator_sequence import ElevatorSequence

    class FakeSequence:
        data_lock = threading.Lock()
        last_press_motion_state = None

    with pytest.raises(SequenceFailure, match='target not found'):
        ElevatorSequence._recover_after_press_failure(
            FakeSequence(),
            SimpleNamespace(),
            'key_3',
            'target not found',
        )
