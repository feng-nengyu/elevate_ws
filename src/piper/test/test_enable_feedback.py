"""Tests for low-speed feedback freshness checks."""

import threading
from collections import defaultdict

import pytest

from piper.enable_feedback import (
    LOW_SPEED_FEEDBACK_COUNTER_NAMES,
    LowSpeedFeedbackCounterUnavailable,
    all_low_speed_feedback_is_fresh,
    fresh_low_speed_feedback_joints,
    read_low_speed_feedback_counts,
    requested_state_is_confirmed,
)


class FakeFpsCounter:
    """Minimal representation of the Piper SDK frame counter."""

    def __init__(self, counts):
        self.lock = threading.Lock()
        self.fps_data = defaultdict(int)
        self.fps_data.update(
            zip(LOW_SPEED_FEEDBACK_COUNTER_NAMES, counts)
        )


def fake_piper(counts, sdk_v2=False):
    """Create a fake SDK interface containing private frame counters."""
    piper = object.__new__(type("FakePiper", (), {}))
    class_name = "C_PiperInterface_V2" if sdk_v2 else "C_PiperInterface"
    setattr(
        piper,
        f"_{class_name}__fps_counter",
        FakeFpsCounter(counts),
    )
    return piper


def test_reads_all_six_sdk_frame_counts():
    """The tracker reads an individual counter for every joint."""
    assert read_low_speed_feedback_counts(
        fake_piper((10, 11, 12, 13, 14, 15))
    ) == (10, 11, 12, 13, 14, 15)


def test_supports_v2_sdk_counter_name():
    """The V2 SDK's name-mangled counter is also supported."""
    assert read_low_speed_feedback_counts(
        fake_piper((1, 2, 3, 4, 5, 6), sdk_v2=True)
    ) == (1, 2, 3, 4, 5, 6)


def test_requires_every_low_speed_frame_to_advance():
    """Five new frames cannot make a cached sixth status look fresh."""
    baseline = (20, 20, 20, 20, 20, 20)

    assert not all_low_speed_feedback_is_fresh(
        baseline,
        (21, 21, 21, 21, 21, 20),
    )
    assert fresh_low_speed_feedback_joints(
        baseline,
        (21, 21, 21, 21, 21, 20),
    ) == (1, 2, 3, 4, 5)
    assert all_low_speed_feedback_is_fresh(
        baseline,
        (21, 21, 21, 21, 21, 21),
    )


def test_missing_sdk_counter_fails_closed():
    """A changed SDK layout must not silently accept cached states."""
    with pytest.raises(LowSpeedFeedbackCounterUnavailable):
        read_low_speed_feedback_counts(object())


def test_cached_enabled_states_do_not_confirm_enable_request():
    """Six cached True values are rejected until all six frames advance."""
    baseline = (30, 30, 30, 30, 30, 30)

    assert not requested_state_is_confirmed(
        True,
        [True] * 6,
        baseline,
        baseline,
    )
    assert requested_state_is_confirmed(
        True,
        [True] * 6,
        baseline,
        (31, 31, 31, 31, 31, 31),
    )


def test_disable_request_needs_fresh_disabled_states():
    """A disable request also requires six fresh False states."""
    baseline = (40, 40, 40, 40, 40, 40)
    current = (41, 41, 41, 41, 41, 41)

    assert requested_state_is_confirmed(
        False,
        [False] * 6,
        baseline,
        current,
    )
    assert not requested_state_is_confirmed(
        False,
        [False, False, False, False, False, True],
        baseline,
        current,
    )
