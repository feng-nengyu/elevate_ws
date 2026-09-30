"""YOLO11 RGB-D detector with hand-eye TF output."""

from collections import deque
from pathlib import Path
import queue
import threading
import traceback

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import message_filters
import numpy as np
from piper_msgs.msg import AllObjectPos, ObjectPos
from piper_msgs.srv import SetInterest
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from piper_vision.cv_tool import px2xy
from piper_vision.yolo_geometry import (
    box_ring_point_cloud,
    fit_plane_ransac,
    optical_xyz,
    press_rotation_matrix,
    quaternion_from_rotation_matrix,
    ray_plane_intersection,
    registered_image_shapes_match,
    rotate_vector_by_quaternion,
    robust_box_depth,
    stable_plane_observations,
)


class Yolo11RgbdNode(Node):
    """Detect objects with YOLO11 and publish their 3D positions."""

    def __init__(self):
        """Initialize model, synchronized image inputs, TF, and publishers."""
        super().__init__('yolo_ros2')
        self._declare_parameters()
        self._read_parameters()

        self.yolo = self._load_model(self.model_path)
        self.get_logger().info(
            f"Loaded YOLO detect model '{self.model_path}' with classes: "
            f"{self.yolo.names}"
        )

        self.target_point_pub = self.create_publisher(
            ObjectPos,
            '/piper_vision/target_point',
            10,
        )
        self.pred_image_pub = self.create_publisher(
            Image,
            '/piper_vision/pred_image',
            10,
        )
        self.all_objects_pub = self.create_publisher(
            AllObjectPos,
            '/piper_vision/all_object_points',
            10,
        )
        self.button_pose_pub = self.create_publisher(
            PoseStamped,
            '/piper_vision/button_pose',
            10,
        )

        self.cv_bridge = CvBridge()
        self.camera_info = {}
        self.camera_info_ready = threading.Event()
        self.image_queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.plane_state_lock = threading.Lock()
        self.plane_interest = self.interest
        self.plane_candidates = deque(
            maxlen=self.plane_lock_sample_count,
        )
        self.locked_plane_point = None
        self.locked_press_axis = None

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_image_topic,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_image_topic,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            3,
            0.02,
        )
        self.sync.registerCallback(self.multi_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.interest_srv = self.create_service(
            SetInterest,
            '/set_interest',
            self.interest_callback,
        )

        self.worker = threading.Thread(
            target=self.yolo_main,
            daemon=True,
        )
        self.worker.start()
        self.get_logger().info('YOLO11 RGB-D detector initialized')

    def _declare_parameters(self):
        """Declare ROS parameters used by detection and localization."""
        self.declare_parameter(
            'model_path',
            '',
            ParameterDescriptor(
                description='Absolute path to a YOLO11 detect .pt model',
            ),
        )
        self.declare_parameter(
            'device',
            '',
            ParameterDescriptor(
                description='Ultralytics device; empty selects automatically',
            ),
        )
        self.declare_parameter(
            'interest',
            'all',
            ParameterDescriptor(
                description='Exact model class name, or all',
            ),
        )
        self.declare_parameter(
            'conf_threshold',
            0.7,
            ParameterDescriptor(description='Detection confidence threshold'),
        )
        self.declare_parameter(
            'iou_threshold',
            0.45,
            ParameterDescriptor(description='NMS IoU threshold'),
        )
        self.declare_parameter(
            'depth_threshold',
            2.0,
            ParameterDescriptor(
                description='Maximum accepted depth in metres',
            ),
        )
        self.declare_parameter(
            'depth_scale',
            0.001,
            ParameterDescriptor(
                description='Raw depth-unit to metre conversion',
            ),
        )
        self.declare_parameter(
            'box_roi_inset',
            0.25,
            ParameterDescriptor(
                description='Fraction removed from each detection-box edge',
            ),
        )
        self.declare_parameter('plane_outer_scale', 2.0)
        self.declare_parameter('plane_inner_scale', 1.0)
        self.declare_parameter('plane_ransac_threshold', 0.003)
        self.declare_parameter('plane_min_points', 100)
        self.declare_parameter('plane_min_inlier_ratio', 0.6)
        self.declare_parameter('plane_max_rms', 0.004)
        self.declare_parameter('plane_max_depth_deviation', 0.03)
        self.declare_parameter('plane_sample_step', 3)
        self.declare_parameter('plane_lock_sample_count', 5)
        self.declare_parameter('plane_lock_max_offset_spread', 0.005)
        self.declare_parameter(
            'plane_lock_max_angle_spread',
            float(np.deg2rad(3.0)),
        )
        self.declare_parameter(
            'bg_removal',
            False,
            ParameterDescriptor(
                description='Replace invalid/far RGB pixels before inference',
            ),
        )
        self.declare_parameter(
            'target_frame_id',
            'base_link',
            ParameterDescriptor(description='Frame used for 3D output'),
        )
        self.declare_parameter(
            'camera_frame_id',
            '',
            ParameterDescriptor(
                description=(
                    'Optical input frame; empty uses CameraInfo header'
                ),
            ),
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter(
            'color_image_topic',
            '/camera/color/image_raw',
        )
        self.declare_parameter(
            'depth_image_topic',
            '/camera/depth/image_raw',
        )

    def _read_parameters(self):
        """Read and validate configured ROS parameters."""
        configured_path = str(
            self.get_parameter('model_path').value
        ).strip()
        expanded_path = Path(configured_path).expanduser()
        if not configured_path:
            raise ValueError(
                'model_path is required; pass an absolute path to best.pt'
            )
        if not expanded_path.is_absolute():
            raise ValueError('model_path must be an absolute path')
        if not expanded_path.is_file():
            raise FileNotFoundError(
                f"YOLO model does not exist: '{expanded_path}'"
            )
        if expanded_path.suffix.lower() != '.pt':
            raise ValueError('model_path must point to a .pt model')

        self.model_path = str(expanded_path)
        self.device = str(self.get_parameter('device').value).strip()
        self.interest = str(
            self.get_parameter('interest').value
        ).strip()
        self.conf_threshold = float(
            self.get_parameter('conf_threshold').value
        )
        self.iou_threshold = float(
            self.get_parameter('iou_threshold').value
        )
        self.depth_threshold = float(
            self.get_parameter('depth_threshold').value
        )
        self.depth_scale = float(
            self.get_parameter('depth_scale').value
        )
        self.box_roi_inset = float(
            self.get_parameter('box_roi_inset').value
        )
        self.plane_outer_scale = float(
            self.get_parameter('plane_outer_scale').value
        )
        self.plane_inner_scale = float(
            self.get_parameter('plane_inner_scale').value
        )
        self.plane_ransac_threshold = float(
            self.get_parameter('plane_ransac_threshold').value
        )
        self.plane_min_points = int(
            self.get_parameter('plane_min_points').value
        )
        self.plane_min_inlier_ratio = float(
            self.get_parameter('plane_min_inlier_ratio').value
        )
        self.plane_max_rms = float(
            self.get_parameter('plane_max_rms').value
        )
        self.plane_max_depth_deviation = float(
            self.get_parameter('plane_max_depth_deviation').value
        )
        self.plane_sample_step = int(
            self.get_parameter('plane_sample_step').value
        )
        self.plane_lock_sample_count = int(
            self.get_parameter('plane_lock_sample_count').value
        )
        self.plane_lock_max_offset_spread = float(
            self.get_parameter('plane_lock_max_offset_spread').value
        )
        self.plane_lock_max_angle_spread = float(
            self.get_parameter('plane_lock_max_angle_spread').value
        )
        self.enable_bg_removal = bool(
            self.get_parameter('bg_removal').value
        )
        self.target_frame_id = str(
            self.get_parameter('target_frame_id').value
        ).strip()
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        ).strip()
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.color_image_topic = str(
            self.get_parameter('color_image_topic').value
        )
        self.depth_image_topic = str(
            self.get_parameter('depth_image_topic').value
        )

        if not self.interest:
            raise ValueError('interest must be a class name or all')
        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError('conf_threshold must be in [0.0, 1.0]')
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError('iou_threshold must be in [0.0, 1.0]')
        if self.depth_threshold <= 0.0:
            raise ValueError('depth_threshold must be positive')
        if self.depth_scale <= 0.0:
            raise ValueError('depth_scale must be positive')
        if not 0.0 <= self.box_roi_inset < 0.5:
            raise ValueError('box_roi_inset must be in [0.0, 0.5)')
        if self.plane_outer_scale <= self.plane_inner_scale:
            raise ValueError(
                'plane_outer_scale must exceed plane_inner_scale'
            )
        if self.plane_inner_scale < 1.0:
            raise ValueError('plane_inner_scale must be at least 1.0')
        if self.plane_ransac_threshold <= 0.0:
            raise ValueError('plane_ransac_threshold must be positive')
        if self.plane_min_points < 3:
            raise ValueError('plane_min_points must be at least three')
        if not 0.0 < self.plane_min_inlier_ratio <= 1.0:
            raise ValueError('plane_min_inlier_ratio must be in (0, 1]')
        if self.plane_max_rms <= 0.0:
            raise ValueError('plane_max_rms must be positive')
        if self.plane_max_depth_deviation <= 0.0:
            raise ValueError('plane_max_depth_deviation must be positive')
        if self.plane_sample_step < 1:
            raise ValueError('plane_sample_step must be at least one')
        if self.plane_lock_sample_count < 1:
            raise ValueError('plane_lock_sample_count must be positive')
        if self.plane_lock_max_offset_spread <= 0.0:
            raise ValueError(
                'plane_lock_max_offset_spread must be positive'
            )
        if self.plane_lock_max_angle_spread <= 0.0:
            raise ValueError(
                'plane_lock_max_angle_spread must be positive'
            )
        if not self.target_frame_id:
            raise ValueError('target_frame_id cannot be empty')

    @staticmethod
    def _load_model(model_path):
        """Load an Ultralytics YOLO detect model."""
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                'Ultralytics is unavailable in this Python interpreter. '
                'Use yolo_handeye.launch.py with conda_env:=yolo11.'
            ) from error

        model = YOLO(model_path)
        if model.task != 'detect':
            raise ValueError(
                f"Expected a YOLO detect model, got task '{model.task}'"
            )
        return model

    def interest_callback(self, request, response):
        """Change the selected target class."""
        interest = request.name.strip()
        if not interest:
            response.result = 'interest cannot be empty'
            return response
        available_names = {
            str(name) for name in self.yolo.names.values()
        }
        if interest != 'all' and interest not in available_names:
            response.result = (
                f"unknown model class '{interest}'; "
                f'available classes: {sorted(available_names)}'
            )
            self.get_logger().warn(response.result)
            return response

        self.interest = interest
        self._reset_plane_lock(interest)
        response.result = f"interest changed to '{self.interest}'"
        self.get_logger().info(response.result)
        return response

    def _reset_plane_lock(self, interest):
        """Discard the panel lock whenever a new task selects a target."""
        if not hasattr(self, 'plane_state_lock'):
            return
        with self.plane_state_lock:
            self.plane_interest = interest
            self.plane_candidates.clear()
            self.locked_plane_point = None
            self.locked_press_axis = None

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        """Keep only recent synchronized RGB-D frames."""
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        self.image_queue.put_nowait((ros_rgb_image, ros_depth_image))

    def camera_info_callback(self, msg):
        """Store camera calibration and optical frame information."""
        self.camera_info['k'] = msg.k
        self.camera_info['d'] = msg.d

        if not self.camera_frame_id:
            self.camera_frame_id = msg.header.frame_id
        elif (
            msg.header.frame_id
            and msg.header.frame_id != self.camera_frame_id
        ):
            self.get_logger().warn(
                f"Configured camera frame '{self.camera_frame_id}' differs "
                f"from CameraInfo frame '{msg.header.frame_id}'"
            )

        if not self.camera_frame_id:
            self.get_logger().error(
                'CameraInfo frame_id is empty; set camera_frame_id'
            )
            return

        self.camera_info_ready.set()
        self.get_logger().info(
            f"Using optical frame '{self.camera_frame_id}' for RGB-D points"
        )
        self.destroy_subscription(self.camera_info_sub)

    def yolo_main(self):
        """Process the newest synchronized frame until shutdown."""
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                rgb_msg, depth_msg = self.image_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if not self.camera_info_ready.is_set():
                self.get_logger().info(
                    'Waiting for CameraInfo and its optical frame',
                    throttle_duration_sec=5.0,
                )
                continue

            try:
                self.image_proc(rgb_msg, depth_msg)
            except Exception:
                self.get_logger().error(
                    'YOLO frame processing failed:\n'
                    + traceback.format_exc()
                )

    def prepare_images(self, color_msg, depth_msg):
        """Convert ROS images and optionally remove distant background."""
        color_image = self.cv_bridge.imgmsg_to_cv2(
            color_msg,
            desired_encoding='bgr8',
        )
        depth_image = self.cv_bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding='passthrough',
        )
        color_image = np.asarray(color_image, dtype=np.uint8)
        depth_image = np.asarray(depth_image)

        if not registered_image_shapes_match(color_image, depth_image):
            self.get_logger().error(
                'Color/depth image sizes differ; enable depth registration',
                throttle_duration_sec=5.0,
            )
            return None, None

        if not self.enable_bg_removal:
            return color_image, depth_image

        depth_metres = depth_image.astype(np.float64) * self.depth_scale
        invalid = (
            ~np.isfinite(depth_metres)
            | (depth_metres <= 0.0)
            | (depth_metres > self.depth_threshold)
        )
        inference_image = color_image.copy()
        inference_image[invalid] = 153
        return inference_image, depth_image

    def predict(self, image):
        """Run YOLO inference with configured thresholds."""
        arguments = {
            'source': image,
            'conf': self.conf_threshold,
            'iou': self.iou_threshold,
            'verbose': False,
        }
        if self.device:
            arguments['device'] = self.device
        return self.yolo.predict(**arguments)[0]

    def image_proc(self, ros_rgb_image, ros_depth_image):
        """Detect objects and publish target-frame 3D positions."""
        inference_image, depth_image = self.prepare_images(
            ros_rgb_image,
            ros_depth_image,
        )
        if inference_image is None:
            return

        image_time = rclpy.time.Time.from_msg(
            ros_rgb_image.header.stamp
        )
        try:
            optical_to_target = self.tf_buffer.lookup_transform(
                self.target_frame_id,
                self.camera_frame_id,
                image_time,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Unable to transform {self.camera_frame_id} to "
                f"{self.target_frame_id} at image time: {error}",
                throttle_duration_sec=2.0,
            )
            return

        detection = self.predict(inference_image)
        pred_image_msg = self.cv_bridge.cv2_to_imgmsg(
            detection.plot(),
            encoding='bgr8',
        )
        pred_image_msg.header = ros_rgb_image.header
        self.pred_image_pub.publish(pred_image_msg)

        all_objects = AllObjectPos()
        all_objects.header.frame_id = self.target_frame_id
        all_objects.header.stamp = ros_rgb_image.header.stamp

        if detection.boxes is None or len(detection.boxes) == 0:
            self.all_objects_pub.publish(all_objects)
            return

        boxes = detection.boxes.xyxy.cpu().numpy()
        class_ids = detection.boxes.cls.cpu().numpy().astype(int)
        confidences = detection.boxes.conf.cpu().numpy()
        button_candidates = []

        for box, class_id, confidence in zip(
            boxes,
            class_ids,
            confidences,
        ):
            if confidence < self.conf_threshold:
                continue
            name = str(detection.names[class_id])
            button_pose = self._publish_detection(
                name,
                box,
                depth_image,
                ros_rgb_image,
                optical_to_target,
                all_objects,
            )
            if button_pose is not None:
                button_candidates.append(
                    (float(confidence), button_pose)
                )

        self.all_objects_pub.publish(all_objects)
        if button_candidates:
            _, best_pose = max(
                button_candidates,
                key=lambda candidate: candidate[0],
            )
            self.button_pose_pub.publish(best_pose)

    def _publish_detection(
        self,
        name,
        box,
        depth_image,
        rgb_msg,
        optical_to_target,
        all_objects,
    ):
        """Localize and publish one YOLO bounding-box detection."""
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        depth = robust_box_depth(
            depth_image,
            box,
            inset_ratio=self.box_roi_inset,
            depth_scale=self.depth_scale,
            max_depth=self.depth_threshold,
        )
        if depth is None:
            self.get_logger().warn(
                f"No valid aligned depth for detected object '{name}'; "
                'selected-button localization can continue after the '
                'panel is locked',
                throttle_duration_sec=2.0,
            )
        else:
            world_x, world_y = px2xy(
                [center_x, center_y],
                self.camera_info['k'],
                self.camera_info['d'],
                depth,
            )
            world_x1, world_y1 = px2xy(
                [x1, y1],
                self.camera_info['k'],
                self.camera_info['d'],
                depth,
            )
            world_x2, world_y2 = px2xy(
                [x2, y2],
                self.camera_info['k'],
                self.camera_info['d'],
                depth,
            )

            optical_point = PointStamped()
            optical_point.header.frame_id = self.camera_frame_id
            optical_point.header.stamp = rgb_msg.header.stamp
            (
                optical_point.point.x,
                optical_point.point.y,
                optical_point.point.z,
            ) = optical_xyz(world_x, world_y, depth)
            target_point = do_transform_point(
                optical_point,
                optical_to_target,
            )

            point = Point()
            point.x = target_point.point.x
            point.y = target_point.point.y
            point.z = target_point.point.z
            width = float(abs(world_x2 - world_x1))
            height = float(abs(world_y2 - world_y1))

            all_objects.names.append(name)
            all_objects.points.append(point)
            all_objects.widths.append(width)
            all_objects.heights.append(height)

        if self.interest not in ('all', name):
            return

        if depth is not None:
            selected_object = ObjectPos()
            selected_object.header.frame_id = self.target_frame_id
            selected_object.header.stamp = rgb_msg.header.stamp
            selected_object.point = point
            selected_object.width = width
            selected_object.height = height
            self.target_point_pub.publish(selected_object)
        if self.interest == 'all' or name != self.interest:
            return None
        return self._estimate_button_pose(
            name,
            box,
            depth_image,
            rgb_msg,
            optical_to_target,
            depth,
        )

    @staticmethod
    def _transform_rotation_quaternion(transform):
        """Return a TF rotation as an ``[x, y, z, w]`` array."""
        rotation = transform.transform.rotation
        return np.array([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        ])

    def _observe_panel_plane(
        self,
        name,
        plane_center,
        normal_to_camera,
        rgb_msg,
        optical_to_target,
    ):
        """Transform and accumulate a reliable task-level panel plane."""
        center_message = PointStamped()
        center_message.header.frame_id = self.camera_frame_id
        center_message.header.stamp = rgb_msg.header.stamp
        center_message.point.x = float(plane_center[0])
        center_message.point.y = float(plane_center[1])
        center_message.point.z = float(plane_center[2])
        transformed_center = do_transform_point(
            center_message,
            optical_to_target,
        )
        center = np.array([
            transformed_center.point.x,
            transformed_center.point.y,
            transformed_center.point.z,
        ])
        press_axis = rotate_vector_by_quaternion(
            -np.asarray(normal_to_camera),
            self._transform_rotation_quaternion(optical_to_target),
        )
        press_axis /= np.linalg.norm(press_axis)

        with self.plane_state_lock:
            if name != self.plane_interest:
                return
            if self.locked_plane_point is not None:
                return
            self.plane_candidates.append((center, press_axis))
            if len(self.plane_candidates) < self.plane_lock_sample_count:
                return
            stable = stable_plane_observations(
                [candidate[0] for candidate in self.plane_candidates],
                [candidate[1] for candidate in self.plane_candidates],
                self.plane_lock_max_offset_spread,
                self.plane_lock_max_angle_spread,
            )
            if stable is None:
                return
            self.locked_plane_point, self.locked_press_axis = stable
            point = self.locked_plane_point.copy()
            axis = self.locked_press_axis.copy()

        self.get_logger().info(
            'Locked button panel in base frame: point='
            f'({point[0]:+.4f}, {point[1]:+.4f}, {point[2]:+.4f}) m, '
            'press_axis='
            f'({axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f})'
        )

    def _locked_panel_snapshot(self, name):
        """Return a copy of the selected target's locked panel."""
        with self.plane_state_lock:
            if (
                name != self.plane_interest
                or self.locked_plane_point is None
            ):
                return None
            return (
                self.locked_plane_point.copy(),
                self.locked_press_axis.copy(),
            )

    def _button_ray_intersection(
        self,
        box,
        optical_to_target,
        plane_point,
        plane_normal,
    ):
        """Intersect the current RGB button-center ray with a locked plane."""
        x1, y1, x2, y2 = box
        center_pixel = [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
        ray_x, ray_y = px2xy(
            center_pixel,
            self.camera_info['k'],
            self.camera_info['d'],
            1.0,
        )
        optical_ray = np.array([ray_x, ray_y, 1.0])
        optical_ray /= np.linalg.norm(optical_ray)
        target_ray = rotate_vector_by_quaternion(
            optical_ray,
            self._transform_rotation_quaternion(optical_to_target),
        )
        translation = optical_to_target.transform.translation
        ray_origin = np.array([
            translation.x,
            translation.y,
            translation.z,
        ])
        intersection = ray_plane_intersection(
            ray_origin,
            target_ray,
            plane_point,
            plane_normal,
            max_distance=self.depth_threshold,
        )
        return intersection, ray_origin, optical_ray

    def _estimate_button_pose(
        self,
        name,
        box,
        depth_image,
        rgb_msg,
        optical_to_target,
        raw_depth,
    ):
        """Use a stable panel lock and the current RGB center ray."""
        locked_panel = self._locked_panel_snapshot(name)
        if locked_panel is None:
            plane_points = box_ring_point_cloud(
                depth_image,
                box,
                self.camera_info['k'],
                outer_scale=self.plane_outer_scale,
                inner_scale=self.plane_inner_scale,
                depth_scale=self.depth_scale,
                max_depth=self.depth_threshold,
                max_depth_deviation=self.plane_max_depth_deviation,
                sample_step=self.plane_sample_step,
            )
            plane = fit_plane_ransac(
                plane_points,
                distance_threshold=self.plane_ransac_threshold,
                min_points=self.plane_min_points,
                min_inlier_ratio=self.plane_min_inlier_ratio,
            )
            if plane is None:
                self.get_logger().warn(
                    'Button panel plane fitting failed quality checks: '
                    f'usable_points={plane_points.shape[0]}, '
                    f'min_points={self.plane_min_points}, '
                    f'inlier_ratio>={self.plane_min_inlier_ratio:.2f}, '
                    f'ransac_threshold='
                    f'{self.plane_ransac_threshold:.4f} m',
                    throttle_duration_sec=2.0,
                )
                return None

            plane_center, normal_to_camera, rms_error, inlier_ratio = plane
            if rms_error > self.plane_max_rms:
                self.get_logger().warn(
                    f'Button panel RMS {rms_error:.4f} m exceeds limit',
                    throttle_duration_sec=2.0,
                )
                return None
            self._observe_panel_plane(
                name,
                plane_center,
                normal_to_camera,
                rgb_msg,
                optical_to_target,
            )
            locked_panel = self._locked_panel_snapshot(name)
            if locked_panel is None:
                return None
            self.get_logger().debug(
                f'Button plane inlier ratio={inlier_ratio:.2f}, '
                f'RMS={rms_error:.4f} m'
            )

        plane_point, press_axis = locked_panel
        intersection, ray_origin, optical_ray = (
            self._button_ray_intersection(
                box,
                optical_to_target,
                plane_point,
                press_axis,
            )
        )
        if intersection is None:
            self.get_logger().warn(
                'Button center ray does not safely intersect locked panel',
                throttle_duration_sec=2.0,
            )
            return None

        plane_distance = float(np.linalg.norm(intersection - ray_origin))
        if raw_depth is not None:
            raw_ray_distance = float(raw_depth / optical_ray[2])
            depth_difference = raw_ray_distance - plane_distance
            if abs(depth_difference) > 0.02:
                self.get_logger().warn(
                    'Ignoring inconsistent button center depth: '
                    f'raw_ray={raw_ray_distance:.4f} m, '
                    f'locked_plane={plane_distance:.4f} m, '
                    f'difference={depth_difference:+.4f} m',
                    throttle_duration_sec=2.0,
                )

        rotation = press_rotation_matrix(press_axis)
        quaternion = quaternion_from_rotation_matrix(rotation)

        pose = PoseStamped()
        pose.header.frame_id = self.target_frame_id
        pose.header.stamp = rgb_msg.header.stamp
        pose.pose.position.x = float(intersection[0])
        pose.pose.position.y = float(intersection[1])
        pose.pose.position.z = float(intersection[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose

    def destroy_node(self):
        """Stop the worker before destroying ROS resources."""
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    """Run the YOLO11 RGB-D ROS node."""
    rclpy.init(args=args)
    node = None
    try:
        node = Yolo11RgbdNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
