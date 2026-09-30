import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def rotation_matrix_from_quaternion(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    return np.array([
        [1.0 - 2.0 * (y_value * y_value + z_value * z_value),
         2.0 * (x_value * y_value - z_value * w_value),
         2.0 * (x_value * z_value + y_value * w_value)],
        [2.0 * (x_value * y_value + z_value * w_value),
         1.0 - 2.0 * (x_value * x_value + z_value * z_value),
         2.0 * (y_value * z_value - x_value * w_value)],
        [2.0 * (x_value * z_value - y_value * w_value),
         2.0 * (y_value * z_value + x_value * w_value),
         1.0 - 2.0 * (x_value * x_value + y_value * y_value)],
    ])


def quaternion_from_rotation_matrix(rotation):
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        return np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])

    diagonal_index = int(np.argmax(np.diag(rotation)))
    if diagonal_index == 0:
        scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return np.array([
            0.25 * scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
        ])
    if diagonal_index == 1:
        scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return np.array([
            (rotation[0, 1] + rotation[1, 0]) / scale,
            0.25 * scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
        ])

    scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return np.array([
        (rotation[0, 2] + rotation[2, 0]) / scale,
        (rotation[1, 2] + rotation[2, 1]) / scale,
        0.25 * scale,
        (rotation[1, 0] - rotation[0, 1]) / scale,
    ])


class CharucoDetector(Node):
    def __init__(self):
        super().__init__('charuco_detector')

        self.declare_parameter('camera_frame', '')
        self.declare_parameter('reference_frame', '')
        self.declare_parameter('marker_frame', 'camera_marker')
        self.declare_parameter('board_squares_x', 7)
        self.declare_parameter('board_squares_y', 5)
        self.declare_parameter('square_length', 0.035)
        self.declare_parameter('marker_length', 0.026)
        self.declare_parameter('min_charuco_corners', 4)

        self.camera_frame = self.get_parameter('camera_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.min_charuco_corners = self.get_parameter('min_charuco_corners').value
        board_squares_x = self.get_parameter('board_squares_x').value
        board_squares_y = self.get_parameter('board_squares_y').value
        square_length = self.get_parameter('square_length').value
        marker_length = self.get_parameter('marker_length').value

        if board_squares_x < 2 or board_squares_y < 2:
            raise ValueError('board_squares_x and board_squares_y must both be at least 2')
        if marker_length <= 0.0 or square_length <= marker_length:
            raise ValueError('square_length must be greater than marker_length, and both must be positive')
        if self.min_charuco_corners < 4:
            raise ValueError('min_charuco_corners must be at least 4')

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.board = cv2.aruco.CharucoBoard_create(
            board_squares_x,
            board_squares_y,
            square_length,
            marker_length,
            self.dictionary,
        )
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.bridge = CvBridge()

        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_single/pose', 10)
        self.result_publisher = self.create_publisher(Image, '/aruco_single/result', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10,
        )

        self.get_logger().info(
            'ChArUco detector configured for DICT_5X5_100, 7 x 5 squares, '
            'square_length=0.035 m, marker_length=0.026 m')

    def camera_info_callback(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.distortion_coefficients = np.asarray(message.d, dtype=np.float64)
        if not self.camera_frame:
            self.camera_frame = message.header.frame_id

        if not self.camera_frame:
            self.get_logger().error('CameraInfo header.frame_id is empty; set camera_frame explicitly')
        else:
            self.get_logger().info(f'Using camera frame: {self.camera_frame}')
            self.destroy_subscription(self.camera_info_subscription)
            self.camera_info_subscription = None

    def image_callback(self, message):
        if self.camera_matrix is None or not self.camera_frame:
            self.get_logger().info('Waiting for camera calibration information', throttle_duration_sec=5.0)
            return

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'Unable to convert image: {error}')
            return

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )
        result_image = image.copy()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(result_image, marker_corners, marker_ids)

        if marker_ids is None or len(marker_ids) == 0:
            self.publish_result(result_image, message)
            return

        corner_count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray_image,
            self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.distortion_coefficients,
        )
        if charuco_ids is None or corner_count < self.min_charuco_corners:
            self.get_logger().info(
                f'Only {corner_count} ChArUco corners detected; need {self.min_charuco_corners}',
                throttle_duration_sec=2.0)
            self.publish_result(result_image, message)
            return

        cv2.aruco.drawDetectedCornersCharuco(result_image, charuco_corners, charuco_ids)
        success, rotation_vector, translation_vector = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.distortion_coefficients,
            None,
            None,
        )
        if not success:
            self.get_logger().info('Unable to estimate ChArUco board pose', throttle_duration_sec=2.0)
            self.publish_result(result_image, message)
            return

        cv2.drawFrameAxes(
            result_image,
            self.camera_matrix,
            self.distortion_coefficients,
            rotation_vector,
            translation_vector,
            0.05,
        )
        self.publish_pose(rotation_vector, translation_vector, message)
        self.publish_result(result_image, message)

    def publish_pose(self, rotation_vector, translation_vector, image_message):
        camera_rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_translation = np.asarray(translation_vector, dtype=np.float64).reshape(3)
        output_frame = self.reference_frame or self.camera_frame
        output_rotation = camera_rotation
        output_translation = camera_translation

        if self.reference_frame and self.reference_frame != self.camera_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as error:
                self.get_logger().warn(
                    f'Unable to transform board pose into {self.reference_frame}: {error}',
                    throttle_duration_sec=2.0)
                return

            transform_quaternion = transform.transform.rotation
            reference_rotation = rotation_matrix_from_quaternion([
                transform_quaternion.x,
                transform_quaternion.y,
                transform_quaternion.z,
                transform_quaternion.w,
            ])
            transform_translation = transform.transform.translation
            reference_translation = np.array([
                transform_translation.x,
                transform_translation.y,
                transform_translation.z,
            ])
            output_rotation = reference_rotation @ camera_rotation
            output_translation = reference_rotation @ camera_translation + reference_translation

        quaternion = quaternion_from_rotation_matrix(output_rotation)
        pose = PoseStamped()
        pose.header = image_message.header
        pose.header.frame_id = output_frame
        pose.pose.position.x = float(output_translation[0])
        pose.pose.position.y = float(output_translation[1])
        pose.pose.position.z = float(output_translation[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(pose)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.marker_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_result(self, result_image, source_image):
        result = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
        result.header = source_image.header
        self.result_publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = CharucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
