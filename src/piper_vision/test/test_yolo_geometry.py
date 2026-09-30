"""Unit tests for RGB-D YOLO geometry helpers."""

import numpy as np
import pytest

from piper_vision.yolo_geometry import (
    box_ring_point_cloud,
    central_box_bounds,
    fit_plane_ransac,
    optical_xyz,
    press_rotation_matrix,
    quaternion_from_rotation_matrix,
    ray_plane_intersection,
    registered_image_shapes_match,
    rotate_vector_by_quaternion,
    robust_box_depth,
    stable_plane_observations,
    valid_box_depths,
)


def test_registered_image_shapes_must_match():
    """Aligned color and depth images must share height and width."""
    color = np.zeros((480, 640, 3), dtype=np.uint8)
    aligned_depth = np.zeros((480, 640), dtype=np.uint16)
    raw_depth = np.zeros((400, 640), dtype=np.uint16)

    assert registered_image_shapes_match(color, aligned_depth)
    assert not registered_image_shapes_match(color, raw_depth)
    assert not registered_image_shapes_match(None, aligned_depth)


def test_central_box_bounds_clamp_and_inset():
    """Detection boxes must be inset and clamped to the image."""
    bounds = central_box_bounds(
        [-10.0, -5.0, 8.0, 10.0],
        (12, 20),
        inset_ratio=0.25,
    )

    assert bounds == (0, 0, 4, 7)


def test_valid_box_depths_ignore_invalid_and_far_pixels():
    """Invalid and over-threshold center-ROI depths must be ignored."""
    depth = np.array([
        [0.0, 1000.0, 1100.0, 0.0],
        [1200.0, np.nan, 1300.0, 2500.0],
        [1400.0, 1500.0, np.inf, 1600.0],
    ])

    depths = valid_box_depths(
        depth,
        [0.0, 0.0, 4.0, 3.0],
        inset_ratio=0.0,
        depth_scale=0.001,
        max_depth=1.5,
    )

    assert np.allclose(
        np.sort(depths),
        [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
    )


def test_robust_box_depth_uses_center_median():
    """Box depth must use the median of its inset center region."""
    depth = np.full((6, 6), 3000, dtype=np.uint16)
    depth[1:5, 1:5] = np.array([
        [900, 1000, 1100, 1200],
        [950, 1000, 1000, 1050],
        [950, 1000, 1000, 1050],
        [900, 1000, 1100, 1200],
    ])

    result = robust_box_depth(
        depth,
        [0.0, 0.0, 6.0, 6.0],
        inset_ratio=0.25,
        depth_scale=0.001,
        max_depth=2.0,
    )

    assert result == pytest.approx(1.0)


@pytest.mark.parametrize(
    'box',
    [
        (1.0, 1.0, 1.0, 3.0),
        (3.0, 1.0, 2.0, 3.0),
        (np.nan, 0.0, 2.0, 2.0),
    ],
)
def test_central_box_bounds_reject_invalid_boxes(box):
    """Invalid YOLO boxes must not be used for depth lookup."""
    with pytest.raises(ValueError):
        central_box_bounds(box, (10, 10))


def test_box_ring_plane_fit_recovers_panel_normal():
    """Noisy ring depths must recover a panel facing the camera."""
    height, width = 120, 160
    fx = fy = 140.0
    cx, cy = 80.0, 60.0
    rows, columns = np.mgrid[0:height, 0:width]
    # Plane z = 0.8 + 0.08*x. Solve using x=(u-cx)z/fx.
    denominator = 1.0 - 0.08 * (columns - cx) / fx
    depth_metres = 0.8 / denominator
    random = np.random.default_rng(3)
    depth_metres += random.normal(0.0, 0.0007, depth_metres.shape)
    depth_mm = depth_metres * 1000.0
    depth_mm[10:12, 10:12] = 0.0
    camera_k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

    points = box_ring_point_cloud(
        depth_mm,
        [60.0, 45.0, 100.0, 75.0],
        camera_k,
        sample_step=2,
    )
    result = fit_plane_ransac(points, min_points=80)

    assert result is not None
    _, normal, rms_error, inlier_ratio = result
    expected = np.array([0.08, 0.0, -1.0])
    expected /= np.linalg.norm(expected)
    assert np.dot(normal, expected) > 0.999
    assert rms_error < 0.002
    assert inlier_ratio > 0.9


def test_stable_plane_observations_average_consistent_samples():
    """Small panel offset and normal noise must produce one locked plane."""
    points = np.array([
        [0.00, 0.00, 0.800],
        [0.03, 0.01, 0.802],
        [-0.02, 0.02, 0.799],
        [0.01, -0.03, 0.801],
        [-0.01, -0.01, 0.800],
    ])
    normals = np.array([
        [0.000, 0.000, 1.000],
        [0.010, 0.000, 1.000],
        [-0.008, 0.004, 1.000],
        [0.000, -0.009, 1.000],
        [0.004, 0.003, 1.000],
    ])

    result = stable_plane_observations(points, normals)

    assert result is not None
    point, normal = result
    assert point[2] == pytest.approx(0.8004)
    assert np.dot(normal, [0.0, 0.0, 1.0]) > 0.999


def test_stable_plane_observations_reject_offset_or_angle_jump():
    """A moving panel or unstable normal must not become a task lock."""
    stable_points = np.array([
        [0.0, 0.0, 0.800],
        [0.0, 0.0, 0.801],
        [0.0, 0.0, 0.799],
    ])
    stable_normals = np.tile([0.0, 0.0, 1.0], (3, 1))
    moved_points = stable_points.copy()
    moved_points[-1, 2] = 0.810
    tilted_normals = stable_normals.copy()
    tilted_normals[-1] = [0.2, 0.0, 0.98]

    assert stable_plane_observations(
        moved_points,
        stable_normals,
    ) is None
    assert stable_plane_observations(
        stable_points,
        tilted_normals,
    ) is None


def test_ray_plane_intersection_returns_forward_hit():
    """A normalized or unnormalized forward ray must hit the panel."""
    result = ray_plane_intersection(
        [0.0, 0.0, 0.0],
        [0.2, 0.0, 2.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        max_distance=2.0,
    )

    assert np.allclose(result, [0.1, 0.0, 1.0])


@pytest.mark.parametrize(
    'direction, plane_point, max_distance',
    [
        ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], 2.0),
        ([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 2.0),
        ([0.0, 0.0, 1.0], [0.0, 0.0, 3.0], 2.0),
    ],
)
def test_ray_plane_intersection_rejects_unsafe_hits(
    direction,
    plane_point,
    max_distance,
):
    """Parallel, rearward, and over-range intersections are invalid."""
    assert ray_plane_intersection(
        [0.0, 0.0, 0.0],
        direction,
        plane_point,
        [0.0, 0.0, 1.0],
        max_distance=max_distance,
    ) is None


def test_press_rotation_uses_positive_z_for_press_axis():
    """The generated orientation must align tool +Z with press direction."""
    press_axis = np.array([0.2, -0.1, 1.0])
    press_axis /= np.linalg.norm(press_axis)
    rotation = press_rotation_matrix(press_axis)
    quaternion = quaternion_from_rotation_matrix(rotation)
    recovered_z = rotate_vector_by_quaternion(
        [0.0, 0.0, 1.0],
        quaternion,
    )

    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
    assert np.isclose(np.linalg.det(rotation), 1.0)
    assert np.allclose(recovered_z, press_axis, atol=1e-8)


def test_optical_point_uses_right_down_forward_order():
    """Optical coordinates must remain X-right, Y-down, Z-forward."""
    assert optical_xyz(0.12, -0.04, 0.75) == (0.12, -0.04, 0.75)


@pytest.mark.parametrize(
    'point',
    [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
        (np.nan, 0.0, 1.0),
    ],
)
def test_optical_point_rejects_invalid_depth(point):
    """Invalid optical points must not be published."""
    with pytest.raises(ValueError):
        optical_xyz(*point)
