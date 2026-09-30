"""Unit tests for MoveIt coarse-positioning pose mathematics."""

import math

import numpy as np

from piper_pbvs_control.control_math import (
    align_tool_z_preserve_roll,
    average_stable_poses,
    coarse_lateral_error_in_range,
    coarse_pose_is_acceptable,
    coarse_standoff_errors,
    coarse_total_attempts,
    matrix_to_quaternion,
    offset_along_panel_horizontal,
    offset_along_panel_vertical,
    offset_along_press_axis,
    quaternion_to_matrix,
    rotation_vector,
    translated_base_x,
    x_distance_metres,
)


def test_x_distance_converts_signed_millimetres_to_metres():
    """Post-coarse X distance keeps its sign and uses millimetres."""
    assert np.isclose(x_distance_metres(80.0), 0.08)
    assert np.isclose(x_distance_metres(-25.0), -0.025)
    assert x_distance_metres(0.0) == 0.0


def test_x_distance_rejects_non_finite_and_out_of_range_values():
    """Post-coarse X requests use the same guarded 100 mm range."""
    for value in (100.001, -100.001, math.inf, -math.inf, math.nan):
        with np.testing.assert_raises(ValueError):
            x_distance_metres(value)


def test_translated_base_x_only_changes_the_first_coordinate():
    """The optional displacement is expressed in base_link, not tool Z."""
    start = np.array([0.38, -0.02, 0.54])

    assert np.allclose(
        translated_base_x(start, 0.08),
        [0.46, -0.02, 0.54],
    )
    assert np.allclose(start, [0.38, -0.02, 0.54])


def test_align_tool_z_keeps_pose_when_axis_is_already_aligned():
    """An aligned reference pose must not acquire an extra tool roll."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))

    result = align_tool_z_preserve_roll(reference, [0.0, 0.0, 1.0])

    assert np.allclose(
        quaternion_to_matrix(result),
        quaternion_to_matrix(reference),
        atol=1e-9,
    )


def test_align_tool_z_uses_shortest_rotation():
    """A perpendicular press axis must need only a 90 degree swing."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))

    result = align_tool_z_preserve_roll(reference, [1.0, 0.0, 0.0])
    rotation = quaternion_to_matrix(result)
    angular_change = np.linalg.norm(rotation_vector(result, reference))

    assert np.allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.isclose(angular_change, math.pi / 2.0)


def test_align_tool_z_handles_opposite_axis_deterministically():
    """An opposite axis must use the reference tool X as the flip axis."""
    reference = np.array([0.0, 0.0, 0.0, 1.0])

    first = align_tool_z_preserve_roll(reference, [0.0, 0.0, -1.0])
    second = align_tool_z_preserve_roll(reference, [0.0, 0.0, -1.0])
    rotation = quaternion_to_matrix(first)

    assert np.allclose(rotation[:, 2], [0.0, 0.0, -1.0], atol=1e-9)
    assert np.allclose(rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(first, second)


def test_align_tool_z_changes_continuously_for_nearby_axes():
    """Nearby observed normals must not introduce a roll discontinuity."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))
    first_axis = np.array([0.20, -0.10, 1.0])
    second_axis = np.array([0.201, -0.099, 1.0])
    first_axis /= np.linalg.norm(first_axis)
    second_axis /= np.linalg.norm(second_axis)

    first = align_tool_z_preserve_roll(reference, first_axis)
    second = align_tool_z_preserve_roll(reference, second_axis)

    assert np.allclose(
        quaternion_to_matrix(first)[:, 2],
        first_axis,
        atol=1e-9,
    )
    assert np.allclose(
        quaternion_to_matrix(second)[:, 2],
        second_axis,
        atol=1e-9,
    )
    assert np.linalg.norm(rotation_vector(second, first)) < 0.002


def test_align_tool_z_rejects_invalid_target_axis():
    """A missing press direction must not produce a motion pose."""
    with np.testing.assert_raises(ValueError):
        align_tool_z_preserve_roll(
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
        )


def test_offset_uses_tool_positive_z_press_axis():
    """Negative standoff must move away from the panel along tool Z."""
    result = offset_along_press_axis(
        [0.4, 0.1, 0.3],
        [0.0, 0.0, 0.0, 1.0],
        -0.08,
    )

    assert np.allclose(result, [0.4, 0.1, 0.22])


def test_horizontal_offset_uses_button_positive_x_as_panel_left():
    """Positive compensation must move left in the detected panel frame."""
    button_quaternion = matrix_to_quaternion(np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]))

    left = offset_along_panel_horizontal(
        [0.4, 0.1, 0.3],
        button_quaternion,
        0.03,
    )
    right = offset_along_panel_horizontal(
        [0.4, 0.1, 0.3],
        button_quaternion,
        -0.02,
    )

    assert np.allclose(left, [0.4, 0.13, 0.3])
    assert np.allclose(right, [0.4, 0.08, 0.3])


def test_coarse_errors_use_button_local_normal_and_tangent_plane():
    """Coarse XY/axial acceptance must rotate with the button frame."""
    control_quaternion = matrix_to_quaternion(np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]))

    axial_distance, axial_error, lateral, lateral_error = (
        coarse_standoff_errors(
            [0.08, 0.004, 0.0],
            [0.0, 0.0, 0.0],
            control_quaternion,
            0.08,
        )
    )

    assert np.isclose(axial_distance, 0.08)
    assert np.isclose(axial_error, 0.0)
    assert np.allclose(lateral, [0.0, 0.004, 0.0])
    assert np.isclose(lateral_error, 0.004)


def test_coarse_axial_error_reports_distance_from_requested_standoff():
    """Positive axial error means TCP is farther from the button."""
    _, axial_error, _, lateral_error = coarse_standoff_errors(
        [0.0, 0.0, 0.091],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        0.08,
    )

    assert np.isclose(axial_error, 0.011)
    assert np.isclose(lateral_error, 0.0)


def test_coarse_lateral_error_range_is_closed():
    """The configured 25-35 mm lateral band includes both boundaries."""
    assert coarse_lateral_error_in_range(0.025, 0.025, 0.035)
    assert coarse_lateral_error_in_range(0.030, 0.025, 0.035)
    assert coarse_lateral_error_in_range(0.035, 0.025, 0.035)


def test_coarse_lateral_error_range_rejects_values_outside_band():
    """Values just outside the lateral band must trigger correction."""
    assert not coarse_lateral_error_in_range(0.0249, 0.025, 0.035)
    assert not coarse_lateral_error_in_range(0.0351, 0.025, 0.035)


def test_coarse_acceptance_keeps_axial_and_angular_guards():
    """A valid lateral error cannot bypass axial or angular protection."""
    assert coarse_pose_is_acceptable(
        0.009, 0.030, 0.010, 0.010, 0.025, 0.035, 0.075
    )
    assert not coarse_pose_is_acceptable(
        0.011, 0.030, 0.010, 0.010, 0.025, 0.035, 0.075
    )
    assert not coarse_pose_is_acceptable(
        0.009, 0.030, 0.080, 0.010, 0.025, 0.035, 0.075
    )


def test_coarse_attempt_count_is_four_only_for_physical_motion():
    """Three corrections mean four physical attempts and one dry-run plan."""
    assert coarse_total_attempts(True, 3) == 4
    assert coarse_total_attempts(False, 3) == 1


def test_pose_stability_handles_quaternion_sign():
    """Equivalent positive and negative quaternions must average together."""
    result = average_stable_poses(
        [[0.4, 0.1, 0.3], [0.401, 0.1, 0.3]],
        [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0]],
        0.003,
        math.radians(3.0),
    )

    assert result is not None
    position, quaternion = result
    assert np.allclose(position, [0.4005, 0.1, 0.3])
    assert np.allclose(quaternion_to_matrix(quaternion), np.eye(3))


def test_panel_vertical_offset_moves_up_in_button_frame():
    """A positive vertical correction follows the detector's up axis."""
    quaternion = matrix_to_quaternion(np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]))
    start = [0.4, 0.1, 0.3]
    assert np.allclose(
        offset_along_panel_vertical(start, quaternion, 0.0075),
        [0.4, 0.1, 0.3075],
    )
