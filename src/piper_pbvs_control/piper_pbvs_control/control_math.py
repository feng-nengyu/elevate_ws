"""Pure pose mathematics for MoveIt coarse positioning."""

import math

import numpy as np


MAX_X_DISTANCE_MM = 100.0


def x_distance_metres(distance_mm):
    """Validate a signed base-X displacement and return metres."""
    distance = float(distance_mm)
    if not math.isfinite(distance):
        raise ValueError('distance_mm must be finite')
    if abs(distance) > MAX_X_DISTANCE_MM:
        raise ValueError(
            f'distance_mm must be within +/-{MAX_X_DISTANCE_MM:g} mm'
        )
    return distance / 1000.0


def translated_base_x(position, displacement_m):
    """Translate a finite three-dimensional position along base-frame X."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    displacement = float(displacement_m)
    if not np.all(np.isfinite(position)):
        raise ValueError('position must contain three finite values')
    if not math.isfinite(displacement):
        raise ValueError('displacement_m must be finite')
    result = position.copy()
    result[0] += displacement
    return result


def normalize_quaternion(quaternion):
    """Return a unit ``[x, y, z, w]`` quaternion."""
    quaternion = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12 or not np.isfinite(norm):
        raise ValueError('quaternion must be finite and non-zero')
    return quaternion / norm


def quaternion_to_matrix(quaternion):
    """Convert an ``[x, y, z, w]`` quaternion to a rotation matrix."""
    x_value, y_value, z_value, w_value = normalize_quaternion(quaternion)
    return np.array([
        [
            1.0 - 2.0 * (y_value ** 2 + z_value ** 2),
            2.0 * (x_value * y_value - z_value * w_value),
            2.0 * (x_value * z_value + y_value * w_value),
        ],
        [
            2.0 * (x_value * y_value + z_value * w_value),
            1.0 - 2.0 * (x_value ** 2 + z_value ** 2),
            2.0 * (y_value * z_value - x_value * w_value),
        ],
        [
            2.0 * (x_value * z_value - y_value * w_value),
            2.0 * (y_value * z_value + x_value * w_value),
            1.0 - 2.0 * (x_value ** 2 + y_value ** 2),
        ],
    ])


def matrix_to_quaternion(rotation):
    """Convert a rotation matrix to a unit ``[x, y, z, w]`` quaternion."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0]
                - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1]
                - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2]
                - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return normalize_quaternion(quaternion)


def rotation_vector(desired_quaternion, current_quaternion):
    """Return the base-frame rotation vector from current to desired."""
    desired = quaternion_to_matrix(desired_quaternion)
    current = quaternion_to_matrix(current_quaternion)
    error_rotation = desired @ current.T
    cosine = np.clip((np.trace(error_rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        eigenvalues, eigenvectors = np.linalg.eig(error_rotation)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array([
        error_rotation[2, 1] - error_rotation[1, 2],
        error_rotation[0, 2] - error_rotation[2, 0],
        error_rotation[1, 0] - error_rotation[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle


def rotation_from_vector(vector):
    """Return a rotation matrix from an axis-angle vector."""
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    cross = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(angle) * cross
        + (1.0 - math.cos(angle)) * (cross @ cross)
    )


def align_tool_z_preserve_roll(reference_quaternion, target_axis):
    """Align tool +Z by the shortest rotation without adding tool roll."""
    reference_rotation = quaternion_to_matrix(reference_quaternion)
    current_axis = reference_rotation[:, 2]
    target_axis = np.asarray(target_axis, dtype=np.float64).reshape(3)
    target_norm = np.linalg.norm(target_axis)
    if target_norm < 1e-12 or not np.isfinite(target_norm):
        raise ValueError('target_axis must be finite and non-zero')
    target_axis /= target_norm

    cross_axis = np.cross(current_axis, target_axis)
    sine = np.linalg.norm(cross_axis)
    cosine = float(np.clip(np.dot(current_axis, target_axis), -1.0, 1.0))
    if sine < 1e-9:
        if cosine > 0.0:
            alignment = np.eye(3)
        else:
            alignment = rotation_from_vector(
                reference_rotation[:, 0] * math.pi
            )
    else:
        angle = math.atan2(sine, cosine)
        alignment = rotation_from_vector(cross_axis / sine * angle)

    return matrix_to_quaternion(alignment @ reference_rotation)


def offset_along_press_axis(position, quaternion, offset):
    """Offset a position along the pose's tool +Z press axis."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    press_axis = quaternion_to_matrix(quaternion)[:, 2]
    return position + float(offset) * press_axis


def offset_along_panel_horizontal(position, button_quaternion, offset):
    """Offset along button-frame +X; positive is left facing the panel."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    horizontal_axis = quaternion_to_matrix(button_quaternion)[:, 0]
    return position + float(offset) * horizontal_axis


def offset_along_panel_vertical(position, button_quaternion, offset):
    """Offset along button-frame +Y; positive is up facing the panel."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    vertical_axis = quaternion_to_matrix(button_quaternion)[:, 1]
    return position + float(offset) * vertical_axis


def coarse_standoff_errors(
    button_position,
    tcp_position,
    control_quaternion,
    desired_standoff,
):
    """Decompose coarse TCP error into panel-normal and lateral parts."""
    button_position = np.asarray(
        button_position,
        dtype=np.float64,
    ).reshape(3)
    tcp_position = np.asarray(tcp_position, dtype=np.float64).reshape(3)
    press_axis = quaternion_to_matrix(control_quaternion)[:, 2]
    button_from_tcp = button_position - tcp_position
    axial_distance = float(np.dot(button_from_tcp, press_axis))
    lateral_vector = button_from_tcp - axial_distance * press_axis
    lateral_error = float(np.linalg.norm(lateral_vector))
    axial_error = axial_distance - float(desired_standoff)
    return axial_distance, axial_error, lateral_vector, lateral_error


def coarse_lateral_error_in_range(error, minimum, maximum):
    """Return whether a coarse lateral-error magnitude is in a closed range."""
    error = float(error)
    minimum = float(minimum)
    maximum = float(maximum)
    return minimum <= error <= maximum


def coarse_pose_is_acceptable(
    axial_error,
    lateral_error,
    angular_error,
    axial_tolerance,
    lateral_minimum,
    lateral_maximum,
    angular_tolerance,
):
    """Apply all measured coarse-positioning acceptance limits."""
    return (
        abs(float(axial_error)) <= float(axial_tolerance)
        and coarse_lateral_error_in_range(
            lateral_error,
            lateral_minimum,
            lateral_maximum,
        )
        and float(angular_error) <= float(angular_tolerance)
    )


def coarse_total_attempts(enable_motion, correction_attempts):
    """Return one dry-run plan or initial motion plus configured retries."""
    return 1 + int(correction_attempts) if enable_motion else 1


def pose_error(target_position, target_quaternion, current_position,
               current_quaternion):
    """Return translation vector, rotation vector, and their magnitudes."""
    translation = (
        np.asarray(target_position, dtype=np.float64)
        - np.asarray(current_position, dtype=np.float64)
    )
    rotation = rotation_vector(target_quaternion, current_quaternion)
    return translation, rotation, np.linalg.norm(translation), np.linalg.norm(
        rotation
    )


def average_stable_poses(
    positions,
    quaternions,
    max_position_spread,
    max_angle_spread,
):
    """Average poses if all samples lie within configured spreads."""
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    quaternions = np.asarray(quaternions, dtype=np.float64).reshape(-1, 4)
    if positions.shape[0] == 0 or positions.shape[0] != quaternions.shape[0]:
        return None
    reference = normalize_quaternion(quaternions[0])
    aligned = []
    for quaternion in quaternions:
        quaternion = normalize_quaternion(quaternion)
        if np.dot(quaternion, reference) < 0.0:
            quaternion = -quaternion
        aligned.append(quaternion)
    average_position = np.mean(positions, axis=0)
    average_quaternion = normalize_quaternion(np.mean(aligned, axis=0))
    position_spread = np.max(
        np.linalg.norm(positions - average_position, axis=1)
    )
    angle_spread = max(
        np.linalg.norm(rotation_vector(average_quaternion, quaternion))
        for quaternion in aligned
    )
    if (
        position_spread > max_position_spread
        or angle_spread > max_angle_spread
    ):
        return None
    return average_position, average_quaternion


def sequence_button_positions(position, control_quaternion, button_quaternion,
                              standoff, advance, retract, horizontal, vertical, first):
    """Compute near-panel stages; retreat is measured from the press endpoint."""
    if not 0 < retract <= 0.1 or not 0 <= advance <= 0.1:
        raise ValueError('advance/retract distances outside 100mm bounds')
    coarse = offset_along_press_axis(position, control_quaternion, -standoff)
    coarse = offset_along_panel_horizontal(coarse, button_quaternion, horizontal)
    coarse = offset_along_panel_vertical(coarse, button_quaternion, vertical)
    press = offset_along_press_axis(coarse, control_quaternion, advance)
    retreat = offset_along_press_axis(press, control_quaternion, -retract)
    approach = coarse if first else retreat
    return approach, press, retreat
