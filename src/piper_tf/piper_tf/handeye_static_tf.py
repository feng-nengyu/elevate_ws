"""Publish an eye-in-hand calibration without breaking the camera TF tree."""

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)


# Saved easy_handeye2 result:
# p_link6 = T_link6_camera_link * p_camera_link
DEFAULT_HAND_EYE_MATRIX = [
    -0.003, 0.012, -1.000, -0.068,
    -0.002, 1.000, 0.012, 0.024,
    1.000, 0.002, -0.003, 0.052,
    0.000, 0.000, 0.000, 1.000,
]


def normalize_homogeneous_matrix(values):
    """Validate a row-major transform and project its rotation onto SO(3)."""
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError('handeye_matrix must contain exactly 16 values')
    matrix = matrix.reshape(4, 4).copy()

    if not np.all(np.isfinite(matrix)):
        raise ValueError('handeye_matrix must contain only finite values')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError('handeye_matrix last row must be [0, 0, 0, 1]')

    u_matrix, _, vh_matrix = np.linalg.svd(matrix[:3, :3])
    rotation = u_matrix @ vh_matrix
    if np.linalg.det(rotation) < 0.0:
        u_matrix[:, -1] *= -1.0
        rotation = u_matrix @ vh_matrix
    matrix[:3, :3] = rotation
    return matrix


def rotation_matrix_from_quaternion(quaternion):
    """Return a rotation matrix from an ``[x, y, z, w]`` quaternion."""
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError('quaternion norm must be non-zero')
    x_value, y_value, z_value, w_value = quaternion / norm
    return np.array([
        [
            1.0 - 2.0 * (y_value * y_value + z_value * z_value),
            2.0 * (x_value * y_value - z_value * w_value),
            2.0 * (x_value * z_value + y_value * w_value),
        ],
        [
            2.0 * (x_value * y_value + z_value * w_value),
            1.0 - 2.0 * (x_value * x_value + z_value * z_value),
            2.0 * (y_value * z_value - x_value * w_value),
        ],
        [
            2.0 * (x_value * z_value - y_value * w_value),
            2.0 * (y_value * z_value + x_value * w_value),
            1.0 - 2.0 * (x_value * x_value + y_value * y_value),
        ],
    ])


def quaternion_from_rotation_matrix(rotation):
    """Return a normalized ``[x, y, z, w]`` quaternion."""
    rotation = np.asarray(rotation, dtype=np.float64)
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
        diagonal_index = int(np.argmax(np.diag(rotation)))
        if diagonal_index == 0:
            scale = np.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif diagonal_index == 1:
            scale = np.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = np.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return quaternion / np.linalg.norm(quaternion)


def matrix_from_transform(transform):
    """Convert a geometry_msgs Transform into a homogeneous matrix."""
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation_matrix_from_quaternion([
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    ])
    matrix[:3, 3] = [
        translation.x,
        translation.y,
        translation.z,
    ]
    return matrix


def rigid_inverse(matrix):
    """Invert a homogeneous rigid transform."""
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return inverse


def camera_link_transform(parent_to_calibrated, camera_link_to_calibrated):
    """Compute parent-to-camera-link while preserving the driver's TF tree."""
    return parent_to_calibrated @ rigid_inverse(camera_link_to_calibrated)


def transform_stamped_from_matrix(matrix, parent_frame, child_frame, stamp):
    """Create a TransformStamped from a homogeneous matrix."""
    quaternion = quaternion_from_rotation_matrix(matrix[:3, :3])
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = float(matrix[0, 3])
    message.transform.translation.y = float(matrix[1, 3])
    message.transform.translation.z = float(matrix[2, 3])
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


class HandEyeStaticTF(Node):
    """Bridge the robot flange to the camera driver's root frame."""

    def __init__(self):
        super().__init__('handeye_static_tf')
        self.declare_parameter('parent_frame', 'link6')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter(
            'calibrated_frame',
            'camera_link',
        )
        self.declare_parameter('handeye_matrix', DEFAULT_HAND_EYE_MATRIX)
        self.declare_parameter('lookup_retry_period', 0.5)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.camera_link_frame = self.get_parameter('camera_link_frame').value
        self.calibrated_frame = self.get_parameter('calibrated_frame').value
        retry_period = self.get_parameter('lookup_retry_period').value
        raw_matrix = self.get_parameter('handeye_matrix').value

        if not all([
            self.parent_frame,
            self.camera_link_frame,
            self.calibrated_frame,
        ]):
            raise ValueError('hand-eye frame names must not be empty')
        if self.parent_frame == self.camera_link_frame:
            raise ValueError('parent_frame and camera_link_frame must differ')
        if retry_period <= 0.0:
            raise ValueError('lookup_retry_period must be positive')

        raw_rotation = np.asarray(
            raw_matrix,
            dtype=np.float64,
        ).reshape(4, 4)[:3, :3]
        self.parent_to_calibrated = normalize_homogeneous_matrix(raw_matrix)
        correction = np.linalg.norm(
            self.parent_to_calibrated[:3, :3] - raw_rotation,
            ord='fro',
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.published = False
        self.timer = self.create_timer(retry_period, self.publish_when_ready)

        self.get_logger().info(
            f'Loaded {self.parent_frame} <- {self.calibrated_frame} '
            f'hand-eye calibration; SO(3) correction={correction:.3e}',
        )

    def publish_when_ready(self):
        """Publish once the camera driver's internal transform is available."""
        if self.published:
            return

        if self.camera_link_frame == self.calibrated_frame:
            camera_link_to_calibrated = np.eye(4, dtype=np.float64)
        else:
            try:
                driver_transform = self.tf_buffer.lookup_transform(
                    self.camera_link_frame,
                    self.calibrated_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as error:
                self.get_logger().info(
                    'Waiting for camera driver TF '
                    f'{self.camera_link_frame} <- {self.calibrated_frame}: '
                    f'{error}',
                    throttle_duration_sec=5.0,
                )
                return
            camera_link_to_calibrated = matrix_from_transform(
                driver_transform.transform,
            )

        parent_to_camera_link = camera_link_transform(
            self.parent_to_calibrated,
            camera_link_to_calibrated,
        )
        message = transform_stamped_from_matrix(
            parent_to_camera_link,
            self.parent_frame,
            self.camera_link_frame,
            self.get_clock().now().to_msg(),
        )
        self.tf_broadcaster.sendTransform(message)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f'Published static TF {self.parent_frame} -> '
            f'{self.camera_link_frame}; calibrated frame remains owned by '
            'the camera driver',
        )


def main(args=None):
    """Run the hand-eye static TF bridge."""
    rclpy.init(args=args)
    node = HandEyeStaticTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
