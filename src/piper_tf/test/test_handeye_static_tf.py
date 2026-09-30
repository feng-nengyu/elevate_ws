"""Unit tests for the hand-eye static TF bridge."""

import numpy as np

from piper_tf.handeye_static_tf import (
    DEFAULT_HAND_EYE_MATRIX,
    camera_link_transform,
    normalize_homogeneous_matrix,
    quaternion_from_rotation_matrix,
    rigid_inverse,
    rotation_matrix_from_quaternion,
)


def test_supplied_handeye_matrix_is_projected_to_so3():
    """The rounded calibration must become a valid rigid transform."""
    raw_matrix = np.asarray(DEFAULT_HAND_EYE_MATRIX).reshape(4, 4)
    normalized = normalize_homogeneous_matrix(DEFAULT_HAND_EYE_MATRIX)

    assert np.allclose(normalized[:3, 3], [-0.068, 0.024, 0.052])
    assert np.allclose(normalized[:3, :3].T @ normalized[:3, :3], np.eye(3))
    assert np.isclose(np.linalg.det(normalized[:3, :3]), 1.0)
    assert np.linalg.norm(
        normalized[:3, :3] - raw_matrix[:3, :3],
        ord='fro',
    ) < 1e-3


def test_camera_link_bridge_preserves_calibrated_transform():
    """The bridge must preserve an arbitrary calibrated child frame."""
    link6_to_calibrated = normalize_homogeneous_matrix(
        DEFAULT_HAND_EYE_MATRIX,
    )
    camera_link_to_optical = np.eye(4)
    camera_link_to_optical[:3, :3] = rotation_matrix_from_quaternion(
        [0.5, -0.5, 0.5, -0.5],
    )
    camera_link_to_optical[:3, 3] = [0.01, -0.02, 0.03]

    link6_to_camera_link = camera_link_transform(
        link6_to_calibrated,
        camera_link_to_optical,
    )

    assert np.allclose(
        link6_to_camera_link @ camera_link_to_optical,
        link6_to_calibrated,
    )
    assert np.allclose(
        rigid_inverse(link6_to_camera_link) @ link6_to_camera_link,
        np.eye(4),
    )


def test_camera_optical_z_is_parallel_to_link6_z_after_driver_tf():
    """The saved camera-link calibration must reproduce the runtime axes."""
    link6_to_camera_link = normalize_homogeneous_matrix(
        DEFAULT_HAND_EYE_MATRIX,
    )
    camera_link_to_optical = np.eye(4)
    camera_link_to_optical[:3, :3] = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    link6_to_optical = link6_to_camera_link @ camera_link_to_optical
    optical_z_in_link6 = link6_to_optical[:3, 2]
    angle = np.arccos(np.clip(optical_z_in_link6[2], -1.0, 1.0))

    assert np.degrees(angle) < 1.0
    assert np.allclose(
        link6_to_optical[:3, :3],
        [
            [-0.012, 1.000, -0.003],
            [-1.000, -0.012, -0.002],
            [-0.002, 0.003, 1.000],
        ],
        atol=0.011,
    )


def test_rotation_quaternion_round_trip():
    """Matrix-to-quaternion conversion must retain the calibrated rotation."""
    matrix = normalize_homogeneous_matrix(DEFAULT_HAND_EYE_MATRIX)
    quaternion = quaternion_from_rotation_matrix(matrix[:3, :3])
    recovered = rotation_matrix_from_quaternion(quaternion)

    assert np.isclose(np.linalg.norm(quaternion), 1.0)
    assert np.allclose(recovered, matrix[:3, :3])


def test_invalid_homogeneous_matrix_is_rejected():
    """Malformed hand-eye input must not produce a TF."""
    invalid = np.eye(4)
    invalid[3, 0] = 1.0

    try:
        normalize_homogeneous_matrix(invalid.reshape(-1))
    except ValueError as error:
        assert 'last row' in str(error)
    else:
        raise AssertionError('invalid homogeneous matrix was accepted')
