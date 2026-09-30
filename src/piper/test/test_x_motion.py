"""Tests for guarded Cartesian X-motion calculations."""

import math

import pytest

from piper.x_motion import (
    MAX_STEP_M,
    distance_metres,
    interpolated_displacements,
    normalize_motion_algorithm,
    translated_x,
)


@pytest.mark.parametrize(
    'distance_mm, expected_m',
    [
        (0.0, 0.0),
        (10.0, 0.01),
        (-10.0, -0.01),
        (100.0, 0.1),
        (-100.0, -0.1),
    ],
)
def test_distance_accepts_signed_range(distance_mm, expected_m):
    """Distances inside the configured safety range convert to metres."""
    assert distance_metres(distance_mm) == pytest.approx(expected_m)


@pytest.mark.parametrize(
    'distance_mm',
    [100.001, -100.001, math.inf, -math.inf, math.nan],
)
def test_distance_rejects_unsafe_values(distance_mm):
    """Non-finite and out-of-range distances fail closed."""
    with pytest.raises(ValueError):
        distance_metres(distance_mm)


@pytest.mark.parametrize('distance_m', [0.011, -0.011, 0.1, -0.1])
def test_interpolation_is_bounded_and_finishes_exactly(distance_m):
    """Every command increment is bounded and the final target is exact."""
    offsets = interpolated_displacements(distance_m)
    increments = [
        current - previous
        for previous, current in zip((0.0,) + offsets[:-1], offsets)
    ]

    assert offsets[-1] == pytest.approx(distance_m)
    assert all(
        abs(increment) <= MAX_STEP_M + 1e-12
        for increment in increments
    )


def test_zero_distance_has_no_motion_steps():
    """The safe default must never produce a motion command."""
    assert interpolated_displacements(0.0) == ()


def test_translation_only_changes_x():
    """Base-frame X movement preserves the other Cartesian coordinates."""
    assert translated_x((0.4, -0.2, 0.3), 0.025) == pytest.approx(
        (0.425, -0.2, 0.3)
    )


@pytest.mark.parametrize(
    'value, expected',
    [
        ('cartesian', 'cartesian'),
        ('moveit', 'moveit'),
        (' MOVEIT ', 'moveit'),
    ],
)
def test_motion_algorithm_accepts_supported_names(value, expected):
    """Algorithm selection is explicit and insensitive to case/spacing."""
    assert normalize_motion_algorithm(value) == expected


@pytest.mark.parametrize('value', ['', 'linear', 'move_l', None])
def test_motion_algorithm_rejects_unknown_names(value):
    """Unknown algorithms must fail before any motion interface is used."""
    with pytest.raises(ValueError):
        normalize_motion_algorithm(value)
