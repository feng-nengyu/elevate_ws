"""Geometry helpers for RGB-D YOLO detections."""

import numpy as np


def registered_image_shapes_match(color_image, depth_image):
    """Return whether color and aligned-depth image sizes are identical."""
    if color_image is None or depth_image is None:
        return False
    return color_image.shape[:2] == depth_image.shape[:2]


def central_box_bounds(box, image_shape, inset_ratio=0.25):
    """Clamp a detection box and return its inset center-region bounds."""
    if len(image_shape) < 2:
        raise ValueError('image_shape must contain height and width')
    if not 0.0 <= inset_ratio < 0.5:
        raise ValueError('inset_ratio must be in [0.0, 0.5)')

    height, width = image_shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError('image dimensions must be positive')

    values = np.asarray(box, dtype=np.float64).reshape(-1)
    if values.size != 4 or not np.all(np.isfinite(values)):
        raise ValueError('box must contain four finite coordinates')

    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError('box must have positive width and height')

    inset_x = (x2 - x1) * inset_ratio
    inset_y = (y2 - y1) * inset_ratio
    left = max(0, min(width, int(np.floor(x1 + inset_x))))
    top = max(0, min(height, int(np.floor(y1 + inset_y))))
    right = max(0, min(width, int(np.ceil(x2 - inset_x))))
    bottom = max(0, min(height, int(np.ceil(y2 - inset_y))))

    if right <= left or bottom <= top:
        raise ValueError('box center region is outside the image')
    return left, top, right, bottom


def valid_box_depths(
    depth_image,
    box,
    inset_ratio=0.25,
    depth_scale=0.001,
    max_depth=None,
):
    """Return valid depths in metres from the center of a detection box."""
    if depth_image is None or depth_image.ndim < 2:
        return np.empty(0, dtype=np.float64)
    if not np.isfinite(depth_scale) or depth_scale <= 0.0:
        raise ValueError('depth_scale must be positive')
    if max_depth is not None and (
        not np.isfinite(max_depth) or max_depth <= 0.0
    ):
        raise ValueError('max_depth must be positive')

    left, top, right, bottom = central_box_bounds(
        box,
        depth_image.shape,
        inset_ratio,
    )
    roi_metres = (
        np.asarray(depth_image[top:bottom, left:right], dtype=np.float64)
        * depth_scale
    )
    valid = np.isfinite(roi_metres) & (roi_metres > 0.0)
    if max_depth is not None:
        valid &= roi_metres <= max_depth
    return roi_metres[valid]


def robust_box_depth(
    depth_image,
    box,
    inset_ratio=0.25,
    depth_scale=0.001,
    max_depth=None,
):
    """Estimate object depth as the median of valid center-region pixels."""
    depths = valid_box_depths(
        depth_image,
        box,
        inset_ratio=inset_ratio,
        depth_scale=depth_scale,
        max_depth=max_depth,
    )
    if depths.size == 0:
        return None
    return float(np.median(depths))


def box_ring_point_cloud(
    depth_image,
    box,
    camera_k,
    outer_scale=2.0,
    inner_scale=1.0,
    depth_scale=0.001,
    max_depth=None,
    max_depth_deviation=0.03,
    sample_step=3,
):
    """Deproject valid pixels in a ring around a detection box."""
    if depth_image is None or depth_image.ndim < 2:
        return np.empty((0, 3), dtype=np.float64)
    if outer_scale <= inner_scale or inner_scale < 1.0:
        raise ValueError('outer_scale must be greater than inner_scale >= 1')
    if sample_step < 1:
        raise ValueError('sample_step must be at least one')

    values = np.asarray(box, dtype=np.float64).reshape(-1)
    if values.size != 4 or not np.all(np.isfinite(values)):
        raise ValueError('box must contain four finite coordinates')
    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError('box must have positive width and height')

    camera_k = np.asarray(camera_k, dtype=np.float64).reshape(3, 3)
    fx, fy = camera_k[0, 0], camera_k[1, 1]
    cx, cy = camera_k[0, 2], camera_k[1, 2]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera focal lengths must be positive')

    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    box_width = x2 - x1
    box_height = y2 - y1

    def scaled_bounds(scale):
        half_width = box_width * scale * 0.5
        half_height = box_height * scale * 0.5
        return (
            center_x - half_width,
            center_y - half_height,
            center_x + half_width,
            center_y + half_height,
        )

    outer = scaled_bounds(outer_scale)
    inner = scaled_bounds(inner_scale)
    left, top, right, bottom = central_box_bounds(
        outer,
        depth_image.shape,
        inset_ratio=0.0,
    )
    rows, columns = np.mgrid[
        top:bottom:sample_step,
        left:right:sample_step,
    ]
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    ring = (
        (columns < inner[0])
        | (columns >= inner[2])
        | (rows < inner[1])
        | (rows >= inner[3])
    )
    raw_depths = np.asarray(
        depth_image[rows, columns],
        dtype=np.float64,
    )
    depths = raw_depths * depth_scale
    valid = ring & np.isfinite(depths) & (depths > 0.0)
    if max_depth is not None:
        valid &= depths <= max_depth
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    selected_depths = depths[valid]
    median_depth = np.median(selected_depths)
    selected = valid & (
        np.abs(depths - median_depth) <= max_depth_deviation
    )
    selected_rows = rows[selected].astype(np.float64)
    selected_columns = columns[selected].astype(np.float64)
    selected_depths = depths[selected]

    points_x = (selected_columns - cx) * selected_depths / fx
    points_y = (selected_rows - cy) * selected_depths / fy
    return np.column_stack((points_x, points_y, selected_depths))


def fit_plane_ransac(
    points,
    distance_threshold=0.003,
    iterations=120,
    min_points=100,
    min_inlier_ratio=0.6,
    random_seed=0,
):
    """Fit a plane robustly and orient its normal toward the camera."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    finite_points = points[np.all(np.isfinite(points), axis=1)]
    if finite_points.shape[0] < min_points:
        return None
    if distance_threshold <= 0.0 or iterations < 1:
        raise ValueError('RANSAC threshold and iterations must be positive')

    random = np.random.default_rng(random_seed)
    best_inliers = None
    best_count = 0
    for _ in range(iterations):
        sample_indices = random.choice(
            finite_points.shape[0],
            size=3,
            replace=False,
        )
        first, second, third = finite_points[sample_indices]
        normal = np.cross(second - first, third - first)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        distances = np.abs((finite_points - first) @ normal)
        inliers = distances <= distance_threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_inliers = inliers
            best_count = count

    if best_inliers is None:
        return None
    inlier_ratio = best_count / finite_points.shape[0]
    if best_count < min_points or inlier_ratio < min_inlier_ratio:
        return None

    inlier_points = finite_points[best_inliers]
    center = np.mean(inlier_points, axis=0)
    _, _, vh_matrix = np.linalg.svd(inlier_points - center)
    normal = vh_matrix[-1]
    normal /= np.linalg.norm(normal)
    if np.dot(normal, center) > 0.0:
        normal = -normal

    errors = (inlier_points - center) @ normal
    rms_error = float(np.sqrt(np.mean(errors ** 2)))
    return center, normal, rms_error, float(inlier_ratio)


def stable_plane_observations(
    points,
    normals,
    max_offset_spread=0.005,
    max_angle_spread=np.deg2rad(3.0),
):
    """
    Average mutually consistent plane observations.

    Plane normals are sign-aligned before averaging.  Offset stability is
    evaluated along the averaged normal so lateral changes in the sampled
    patch do not look like panel motion.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0 or points.shape != normals.shape:
        return None
    if max_offset_spread <= 0.0 or max_angle_spread <= 0.0:
        raise ValueError('plane stability limits must be positive')
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(normals)):
        return None

    normal_lengths = np.linalg.norm(normals, axis=1)
    if np.any(normal_lengths < 1e-12):
        return None
    unit_normals = normals / normal_lengths[:, None]
    reference = unit_normals[0]
    aligned = unit_normals.copy()
    aligned[aligned @ reference < 0.0] *= -1.0
    average_normal = np.mean(aligned, axis=0)
    average_length = np.linalg.norm(average_normal)
    if average_length < 1e-12:
        return None
    average_normal /= average_length

    angular_errors = np.arccos(np.clip(
        aligned @ average_normal,
        -1.0,
        1.0,
    ))
    if float(np.max(angular_errors)) > max_angle_spread:
        return None

    offsets = points @ average_normal
    if float(np.max(offsets) - np.min(offsets)) > max_offset_spread:
        return None

    average_point = np.mean(points, axis=0)
    return average_point, average_normal


def ray_plane_intersection(
    ray_origin,
    ray_direction,
    plane_point,
    plane_normal,
    max_distance=None,
    min_abs_cosine=0.05,
):
    """Intersect a forward ray with a plane, returning ``None`` if unsafe."""
    origin = np.asarray(ray_origin, dtype=np.float64).reshape(3)
    direction = np.asarray(ray_direction, dtype=np.float64).reshape(3)
    point = np.asarray(plane_point, dtype=np.float64).reshape(3)
    normal = np.asarray(plane_normal, dtype=np.float64).reshape(3)
    values = np.concatenate((origin, direction, point, normal))
    if not np.all(np.isfinite(values)):
        return None
    direction_length = np.linalg.norm(direction)
    normal_length = np.linalg.norm(normal)
    if direction_length < 1e-12 or normal_length < 1e-12:
        return None
    direction /= direction_length
    normal /= normal_length
    denominator = float(np.dot(normal, direction))
    if abs(denominator) < min_abs_cosine:
        return None
    distance = float(np.dot(normal, point - origin) / denominator)
    if distance <= 0.0:
        return None
    if max_distance is not None:
        if not np.isfinite(max_distance) or max_distance <= 0.0:
            raise ValueError('max_distance must be positive')
        if distance > max_distance:
            return None
    return origin + distance * direction


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a vector by a normalized ``[x, y, z, w]`` quaternion."""
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    quaternion = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError('quaternion norm must be non-zero')
    xyz = quaternion[:3] / norm
    scalar = quaternion[3] / norm
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (scalar * scalar - np.dot(xyz, xyz)) * vector
        + 2.0 * scalar * np.cross(xyz, vector)
    )


def press_rotation_matrix(press_axis, up_axis=(0.0, 0.0, 1.0)):
    """Build a tool rotation whose +Z follows the press direction."""
    tool_z = np.asarray(press_axis, dtype=np.float64).reshape(3)
    tool_z /= np.linalg.norm(tool_z)
    reference_up = np.asarray(up_axis, dtype=np.float64).reshape(3)
    tool_y = reference_up - np.dot(reference_up, tool_z) * tool_z
    if np.linalg.norm(tool_y) < 1e-6:
        fallback = np.array([0.0, 1.0, 0.0])
        tool_y = fallback - np.dot(fallback, tool_z) * tool_z
    tool_y /= np.linalg.norm(tool_y)
    tool_x = np.cross(tool_y, tool_z)
    tool_x /= np.linalg.norm(tool_x)
    tool_y = np.cross(tool_z, tool_x)
    return np.column_stack((tool_x, tool_y, tool_z))


def quaternion_from_rotation_matrix(rotation):
    """Convert a proper rotation matrix to ``[x, y, z, w]``."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(
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
            scale = np.sqrt(
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
            scale = np.sqrt(
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
    return quaternion / np.linalg.norm(quaternion)


def optical_xyz(world_x, world_y, depth):
    """Return a validated REP-103 optical point: right, down, forward."""
    point = np.asarray([world_x, world_y, depth], dtype=np.float64)
    if not np.all(np.isfinite(point)):
        raise ValueError('optical point must contain finite coordinates')
    if point[2] <= 0.0:
        raise ValueError('optical point depth must be positive')
    return tuple(float(value) for value in point)
