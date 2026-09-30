"""Helpers for confirming fresh Piper low-speed motor feedback."""

LOW_SPEED_FEEDBACK_COUNTER_NAMES = tuple(
    f"ArmMotorDriverInfoLowSpd_{joint_index}"
    for joint_index in range(1, 7)
)

_SDK_FPS_COUNTER_ATTRIBUTES = (
    "_C_PiperInterface__fps_counter",
    "_C_PiperInterface_V2__fps_counter",
)


class LowSpeedFeedbackCounterUnavailable(RuntimeError):
    """Raised when the SDK does not expose its per-frame receive counters."""


def read_low_speed_feedback_counts(piper):
    """Return the SDK receive count for each of the six low-speed CAN frames."""
    fps_counter = None
    for attribute_name in _SDK_FPS_COUNTER_ATTRIBUTES:
        fps_counter = getattr(piper, attribute_name, None)
        if fps_counter is not None:
            break

    if fps_counter is None:
        raise LowSpeedFeedbackCounterUnavailable(
            "Piper SDK per-frame counter is unavailable"
        )

    try:
        with fps_counter.lock:
            return tuple(
                int(fps_counter.fps_data[counter_name])
                for counter_name in LOW_SPEED_FEEDBACK_COUNTER_NAMES
            )
    except (AttributeError, KeyError, TypeError) as error:
        raise LowSpeedFeedbackCounterUnavailable(
            "Piper SDK per-frame counter has an unsupported layout"
        ) from error


def fresh_low_speed_feedback_joints(baseline_counts, current_counts):
    """Return one-based joint numbers whose frame count advanced."""
    if len(baseline_counts) != 6 or len(current_counts) != 6:
        return ()

    return tuple(
        joint_index
        for joint_index, (baseline, current) in enumerate(
            zip(baseline_counts, current_counts),
            start=1,
        )
        if current > baseline
    )


def all_low_speed_feedback_is_fresh(baseline_counts, current_counts):
    """Return whether all six low-speed frames arrived after the baseline."""
    return fresh_low_speed_feedback_joints(
        baseline_counts,
        current_counts,
    ) == (1, 2, 3, 4, 5, 6)


def requested_state_is_confirmed(
    enable_request,
    enable_states,
    baseline_counts,
    current_counts,
):
    """Confirm the requested state using six fresh per-joint frames."""
    if len(enable_states) != 6:
        return False

    state_matches_request = (
        all(enable_states)
        if enable_request
        else not any(enable_states)
    )
    return (
        state_matches_request
        and all_low_speed_feedback_is_fresh(
            baseline_counts,
            current_counts,
        )
    )
