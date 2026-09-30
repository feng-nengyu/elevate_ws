"""Pure helpers for a guarded relative Cartesian X movement."""

import math


MAX_DISTANCE_MM = 100.0
MAX_STEP_M = 0.002
MOTION_ALGORITHMS = ('cartesian', 'moveit')


def normalize_motion_algorithm(value):
    """Return a supported lower-case motion algorithm name."""
    algorithm = str(value).strip().lower()
    if algorithm not in MOTION_ALGORITHMS:
        choices = ', '.join(MOTION_ALGORITHMS)
        raise ValueError(
            f'motion_algorithm must be one of: {choices}'
        )
    return algorithm


def distance_metres(distance_mm):
    """Validate a signed millimetre displacement and return metres."""
    distance = float(distance_mm)
    if not math.isfinite(distance):
        raise ValueError('distance_mm must be finite')
    if abs(distance) > MAX_DISTANCE_MM:
        raise ValueError(
            f'distance_mm must be within +/-{MAX_DISTANCE_MM:g} mm'
        )
    return distance / 1000.0


def interpolated_displacements(distance_m, max_step_m=MAX_STEP_M):
    """Return signed offsets whose adjacent spacing is safely bounded."""
    distance = float(distance_m)
    max_step = float(max_step_m)
    if not math.isfinite(distance):
        raise ValueError('distance_m must be finite')
    if not math.isfinite(max_step) or max_step <= 0.0:
        raise ValueError('max_step_m must be finite and positive')
    if distance == 0.0:
        return ()

    step_count = int(math.ceil(abs(distance) / max_step))
    return tuple(
        distance * step_index / step_count
        for step_index in range(1, step_count + 1)
    )


def translated_x(position, displacement_m):
    """Translate a three-dimensional position along base-frame X."""
    values = tuple(float(value) for value in position)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('position must contain three finite values')
    displacement = float(displacement_m)
    if not math.isfinite(displacement):
        raise ValueError('displacement_m must be finite')
    return (values[0] + displacement, values[1], values[2])
