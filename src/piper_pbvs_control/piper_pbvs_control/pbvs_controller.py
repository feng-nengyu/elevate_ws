"""Vision-guided MoveIt coarse positioning for elevator buttons."""

from collections import deque
import copy
import json
import math
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
from piper_msgs.action import PressButton
from piper_msgs.msg import PiperStatusMsg
from piper_msgs.srv import SetInterest
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String, Float64
from tf2_ros import Buffer, TransformException, TransformListener

from piper_pbvs_control.control_math import (
    align_tool_z_preserve_roll,
    average_stable_poses,
    coarse_pose_is_acceptable,
    coarse_standoff_errors,
    coarse_total_attempts,
    offset_along_panel_horizontal,
    offset_along_panel_vertical,
    offset_along_press_axis,
    pose_error,
    quaternion_to_matrix,
    translated_base_x,
    x_distance_metres,
)


class TaskFailure(RuntimeError):
    """Raised when a guarded coarse-positioning task cannot continue."""


class TaskCanceled(RuntimeError):
    """Raised when the client cancels an active positioning task."""


class PlanningFailure(TaskFailure):
    """A plan-only MoveIt goal returned an ordinary planning failure."""


class PlanningChainFailure(TaskFailure):
    """All pure-planning candidates for a button failed."""


class PlanningBudgetExceeded(TaskFailure):
    """A pure-planning chain exceeded its shared budget."""


class PiperPbvsController(Node):
    """Coordinate perception and guarded MoveIt coarse positioning."""

    ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))
    APPROACH_POSITION_TOLERANCE = 0.008
    X_POSITION_TOLERANCE = 0.006
    RETRACT_POSITION_TOLERANCE = 0.007
    X_ORIENTATION_TOLERANCE = 0.075
    PRESS_PATH_LATERAL_HALF_WIDTH = 0.008
    SNAPSHOT_NORMAL_ANGLE_LIMIT = math.radians(5.0)

    STATE_LABELS = {
        'IDLE': '空闲',
        'WAIT_TARGET': '等待并获取按钮位姿',
        'COARSE_APPROACH': 'MoveIt 粗定位',
        'X_ADVANCE': 'MoveIt 按压移动',
        'DONE': '任务完成',
        'ABORT': '任务中止',
    }

    def __init__(self):
        """Create interfaces and validate safety parameters."""
        super().__init__('piper_pbvs_controller')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._read_parameters()

        self.data_lock = threading.Lock()
        self.active_lock = threading.Lock()
        self.task_active = False
        self.target_samples = deque(maxlen=self.stable_sample_count)
        self.latest_target = None
        self.latest_target_received = 0.0
        self.latest_tcp = None
        self.latest_tcp_received = 0.0
        self.latest_joint_positions = None
        self.latest_joint_received = 0.0
        self.latest_arm_status = None
        self.active_move_goal = None
        self.last_moveit_arm_target = None
        self.current_state = 'IDLE'
        self.sequence_snapshot = {}
        self.sequence_snapshot_created = 0.0
        self.sequence_roll_reference = None
        self.preplanned_buttons = []
        self.preplanned_index = 0
        self.uncertain_motion = False

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/piper_vision/button_pose',
            self._target_callback,
            10,
            callback_group=self.callback_group,
        )
        self.tcp_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose',
            self._tcp_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.status_sub = self.create_subscription(
            PiperStatusMsg,
            '/arm_status',
            self._status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.press_event_pub = self.create_publisher(Float64, '/pbvs/press_event', 10)
        self.press_reached_pub = self.create_publisher(String, '/pbvs/press_reached', 50)
        self.press_command_pub = self.create_publisher(String, '/pbvs/press_command', 50)
        self.segment_timing_pub = self.create_publisher(String, '/pbvs/segment_timing', 50)
        self.transition_command_pub = self.create_publisher(
            String, '/pbvs/transition_command', 10
        )
        self.state_pub = self.create_publisher(String, '/pbvs/state', 10)
        self.state_heartbeat = self.create_timer(
            0.5, lambda: self.state_pub.publish(String(data=self.current_state))
        )
        self.desired_tcp_pub = self.create_publisher(
            PoseStamped,
            '/pbvs/desired_tcp_pose',
            10,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group,
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.callback_group,
        )
        self.interest_client = self.create_client(
            SetInterest,
            '/set_interest',
            callback_group=self.callback_group,
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.action_server = ActionServer(
            self,
            PressButton,
            '/press_button',
            execute_callback=self._execute_press,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._set_state('IDLE')
        self.get_logger().info(
            'Piper MoveIt coarse-positioning controller ready; '
            f'enable_motion={self.enable_motion}, '
            f'orientation_mode={self.orientation_mode}, '
            f'moveit_velocity={self.moveit_velocity_scaling_factor:.3f}, '
            f'moveit_acceleration='
            f'{self.moveit_acceleration_scaling_factor:.3f}, '
            f'distance_mm={self.distance_m * 1000.0:+.3f}'
        )

    def _declare_parameters(self):
        """Declare motion, convergence, timeout, and collision parameters."""
        defaults = {
            'enable_motion': False,
            'close_panel_sequence': False,
            'preplan_sequence': True,
            'preplan_retry_attempts': 2,
            'transition_plan_candidates': 3,
            'planning_interval_target_sec': 0.0,
            'retract_plan_candidates': 2,
            'sequence_search_width': 2,
            'preplan_retry_timeout_sec': 60.0,
            'sequence_retract_distance_mm': 15.0,
            'base_frame': 'base_link',
            'tcp_frame': 'tcp_link',
            'flange_frame': 'link6',
            'camera_link_frame': 'camera_link',
            'move_group_name': 'arm',
            'orientation_mode': 'preserve_current_roll',
            'coarse_standoff': 0.08,
            'coarse_horizontal_offset': 0.026,
            'coarse_vertical_offset': 0.007,
            'coarse_lateral_error_min': 0.021,
            'coarse_lateral_error_max': 0.032,
            'coarse_axial_tolerance': 0.01,
            'coarse_correction_attempts': 0,
            'distance_mm': 67.0,
            'x_advance_axis_mode': 'panel_normal',
            'stable_sample_count': 3,
            'stable_position_spread': 0.003,
            'stable_angle_spread': math.radians(3.0),
            'target_acquire_timeout': 5.0,
            'sequence_snapshot_acquire_timeout': 15.0,
            'target_pause_age': 0.5,
            'tcp_feedback_timeout': 0.5,
            'moveit_timeout': 20.0,
            'moveit_velocity_scaling_factor': 0.09,
            'moveit_acceleration_scaling_factor': 0.09,
            'transition_velocity_scaling_factor': 0.35,
            'transition_acceleration_scaling_factor': 0.35,
            'press_velocity_scaling_factor': 0.17,
            'press_acceleration_scaling_factor': 0.17,
            'retract_velocity_scaling_factor': 0.2,
            'retract_acceleration_scaling_factor': 0.2,
            'moveit_position_tolerance': 0.002,
            'moveit_orientation_tolerance': 0.05,
            'panel_width': 0.6,
            'panel_height': 1.2,
            'panel_thickness': 0.02,
            'camera_collision_enabled': True,
            'camera_size_x': 0.10,
            'camera_size_y': 0.04,
            'camera_size_z': 0.04,
        }
        self.controller_parameter_names = tuple(defaults)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """Read parameters and reject unsafe combinations."""
        values = {
            name: self.get_parameter(name).value
            for name in self.controller_parameter_names
        }
        self.enable_motion = bool(values['enable_motion'])

        string_names = (
            'base_frame',
            'tcp_frame',
            'flange_frame',
            'camera_link_frame',
            'move_group_name',
            'orientation_mode',
            'x_advance_axis_mode',
        )
        for name in string_names:
            value = str(values[name]).strip()
            if not value:
                raise ValueError(f'{name} cannot be empty')
            setattr(self, name, value)
        if self.orientation_mode not in (
            'preserve_current_roll',
            'world_up',
        ):
            raise ValueError(
                'orientation_mode must be preserve_current_roll or world_up'
            )
        if self.x_advance_axis_mode not in ('base_x', 'panel_normal'):
            raise ValueError(
                'x_advance_axis_mode must be base_x or panel_normal'
            )

        integer_names = ('stable_sample_count', 'transition_plan_candidates', 'retract_plan_candidates', 'sequence_search_width')
        for name in integer_names:
            value = int(values[name])
            if value < 1:
                raise ValueError(f'{name} must be positive')
            if name in ('transition_plan_candidates','retract_plan_candidates') and value > 5:
                raise ValueError(f'{name} must be 1..5')
            if name == 'sequence_search_width' and value > 3:
                raise ValueError('sequence_search_width must be 1..3')
            setattr(self, name, value)

        self.coarse_correction_attempts = int(
            values['coarse_correction_attempts']
        )
        if self.coarse_correction_attempts < 0:
            raise ValueError('coarse_correction_attempts cannot be negative')
        self.preplan_retry_attempts = int(values['preplan_retry_attempts'])
        if not 0 <= self.preplan_retry_attempts <= 2:
            raise ValueError('preplan_retry_attempts must be 0, 1, or 2')
        self.preplan_retry_timeout_sec = float(values['preplan_retry_timeout_sec'])
        if not math.isfinite(self.preplan_retry_timeout_sec) or not 0 <= self.preplan_retry_timeout_sec <= 600:
            raise ValueError('preplan_retry_timeout_sec must be within 0..600 seconds')

        self.distance_m = x_distance_metres(values['distance_mm'])

        bool_names = ('camera_collision_enabled', 'close_panel_sequence', 'preplan_sequence')
        for name in bool_names:
            setattr(self, name, bool(values[name]))

        signed_float_names = (
            'coarse_horizontal_offset',
            'coarse_vertical_offset',
        )
        for name in signed_float_names:
            value = float(values[name])
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            setattr(self, name, value)
        if abs(self.coarse_horizontal_offset) > 0.05:
            raise ValueError(
                'coarse_horizontal_offset must be within +/-0.05 m'
            )
        if abs(self.coarse_vertical_offset) > 0.05:
            raise ValueError(
                'coarse_vertical_offset must be within +/-0.05 m'
            )

        excluded = set(string_names + integer_names + bool_names) | {
            'enable_motion',
            'coarse_correction_attempts',
            'preplan_retry_attempts',
            'preplan_retry_timeout_sec',
            'distance_mm',
        } | set(signed_float_names)
        for name, value in values.items():
            if name in excluded:
                continue
            value = float(value)
            if name == 'planning_interval_target_sec':
                if not math.isfinite(value) or not 0.0 <= value <= 3.0:
                    raise ValueError('planning_interval_target_sec must be within 0..3 seconds')
                setattr(self,name,value)
                continue
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
            setattr(self, name, value)

        if not math.isfinite(self.sequence_retract_distance_mm) or not 0 < self.sequence_retract_distance_mm <= 100:
            raise ValueError('sequence_retract_distance_mm must be in (0,100]')
        if (
            self.coarse_lateral_error_min
            >= self.coarse_lateral_error_max
        ):
            raise ValueError(
                'coarse_lateral_error_min must be smaller than '
                'coarse_lateral_error_max'
            )
        for name in (
            'moveit_velocity_scaling_factor',
            'moveit_acceleration_scaling_factor',
            'transition_velocity_scaling_factor',
            'transition_acceleration_scaling_factor',
            'press_velocity_scaling_factor',
            'press_acceleration_scaling_factor',
            'retract_velocity_scaling_factor',
            'retract_acceleration_scaling_factor',
        ):
            if getattr(self, name) > 1.0:
                raise ValueError(f'{name} must not exceed 1.0')

    @staticmethod
    def _pose_arrays(pose):
        """Extract numpy position and quaternion arrays from a ROS Pose."""
        return (
            np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ], dtype=np.float64),
            np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ], dtype=np.float64),
        )

    @staticmethod
    def _format_xyz(position):
        """Format a Cartesian position or displacement for diagnostics."""
        values = np.asarray(position, dtype=np.float64).reshape(3)
        return (
            f'({values[0]:+.4f}, {values[1]:+.4f}, '
            f'{values[2]:+.4f}) m'
        )

    @staticmethod
    def _format_quaternion(quaternion):
        """Format an XYZW quaternion for diagnostics."""
        values = np.asarray(quaternion, dtype=np.float64).reshape(4)
        return (
            f'({values[0]:+.5f}, {values[1]:+.5f}, '
            f'{values[2]:+.5f}, {values[3]:+.5f})'
        )

    @classmethod
    def _format_arm_joints(cls, positions):
        """Format six named arm joint positions in radians."""
        values = np.asarray(positions, dtype=np.float64).reshape(6)
        return ', '.join(
            f'{name}={value:+.6f}'
            for name, value in zip(cls.ARM_JOINT_NAMES, values)
        )

    @classmethod
    def _final_moveit_arm_target(cls, result):
        """Extract the final six-axis target from a MoveGroup result."""
        for field_name in ('executed_trajectory', 'planned_trajectory'):
            robot_trajectory = getattr(result, field_name, None)
            trajectory = getattr(
                robot_trajectory,
                'joint_trajectory',
                None,
            )
            if trajectory is None or not trajectory.points:
                continue
            positions = trajectory.points[-1].positions
            if len(trajectory.joint_names) != len(positions):
                continue
            by_name = dict(zip(trajectory.joint_names, positions))
            if not all(name in by_name for name in cls.ARM_JOINT_NAMES):
                continue
            arm_positions = tuple(
                float(by_name[name]) for name in cls.ARM_JOINT_NAMES
            )
            if all(math.isfinite(value) for value in arm_positions):
                return arm_positions
        return None

    def _log_position_delta(self, label, end_position, start_position):
        """Log a Cartesian displacement vector and its Euclidean norm."""
        delta = np.asarray(end_position) - np.asarray(start_position)
        self.get_logger().info(
            f'【定位诊断】{label}=' + self._format_xyz(delta)
            + f', |{label}|={np.linalg.norm(delta):.4f} m'
        )

    def _pose_message(self, position, quaternion, stamp=None):
        """Construct a base-frame PoseStamped."""
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    def _target_callback(self, message):
        """Cache valid base-frame button poses for stability checks."""
        if message.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Ignoring button pose in '{message.header.frame_id}'"
            )
            return
        position, quaternion = self._pose_arrays(message.pose)
        values = np.concatenate((position, quaternion))
        if not np.all(np.isfinite(values)) or np.linalg.norm(
            quaternion
        ) < 1e-9:
            return
        received = time.monotonic()
        with self.data_lock:
            self.latest_target = copy.deepcopy(message)
            self.latest_target_received = received
            self.target_samples.append(
                (position, quaternion, received)
            )

    def _tcp_callback(self, message):
        """Cache measured Piper TCP feedback."""
        if message.header.frame_id != self.base_frame:
            return
        with self.data_lock:
            self.latest_tcp = copy.deepcopy(message)
            self.latest_tcp_received = time.monotonic()

    def _joint_state_callback(self, message):
        """Cache the latest complete six-axis real joint feedback."""
        if len(message.name) != len(message.position):
            return
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in self.ARM_JOINT_NAMES):
            return
        positions = tuple(
            float(by_name[name]) for name in self.ARM_JOINT_NAMES
        )
        if not all(math.isfinite(value) for value in positions):
            return
        with self.data_lock:
            self.latest_joint_positions = positions
            self.latest_joint_received = time.monotonic()

    def _status_callback(self, message):
        """Cache Piper hardware status."""
        with self.data_lock:
            self.latest_arm_status = copy.deepcopy(message)

    def _goal_callback(self, goal_request):
        """Accept one non-empty target task at a time."""
        if not goal_request.target_name.strip():
            return GoalResponse.REJECT
        with self.active_lock:
            if self.task_active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        """Accept task cancellation; execution performs guarded cleanup."""
        return CancelResponse.ACCEPT

    def _set_state(self, state):
        """Publish the machine-readable state and log its Chinese label."""
        self.current_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)
        label = self.STATE_LABELS.get(state, state)
        if state in ('IDLE', 'DONE', 'ABORT'):
            self.get_logger().info(
                f'【粗定位状态】{label}（{state}）'
            )
        else:
            self.get_logger().info(
                f'【阶段开始】{label}（{state}）'
            )

    def _stage_success(self, state, detail=''):
        """Log a Chinese success message for one task stage."""
        label = self.STATE_LABELS.get(state, state)
        suffix = f'：{detail}' if detail else ''
        self.get_logger().info(
            f'【阶段成功】{label}（{state}）{suffix}'
        )

    def _stage_failure(self, state, error):
        """Log a Chinese failure message for one task stage."""
        label = self.STATE_LABELS.get(state, state)
        self.get_logger().error(
            f'【阶段失败】{label}（{state}）：{error}'
        )

    def _feedback(self, goal_handle, position_error=0.0,
                  angular_error=0.0):
        """Publish Action feedback for the current state and errors."""
        feedback = PressButton.Feedback()
        feedback.state = self.current_state
        feedback.position_error_m = float(position_error)
        feedback.angular_error_rad = float(angular_error)
        with self.data_lock:
            received = self.latest_target_received
        feedback.target_age_s = float(
            max(0.0, time.monotonic() - received)
            if received else math.inf
        )
        goal_handle.publish_feedback(feedback)

    def _arm_fault(self):
        """Return a hardware fault description, or an empty string."""
        with self.data_lock:
            status = copy.deepcopy(self.latest_arm_status)
        if status is None:
            return ''
        if status.err_code != 0:
            return f'Piper error code {status.err_code}'
        limit_fields = [
            status.joint_1_angle_limit,
            status.joint_2_angle_limit,
            status.joint_3_angle_limit,
            status.joint_4_angle_limit,
            status.joint_5_angle_limit,
            status.joint_6_angle_limit,
        ]
        if any(limit_fields):
            return 'Piper joint angle limit fault'
        communication_fields = [
            status.communication_status_joint_1,
            status.communication_status_joint_2,
            status.communication_status_joint_3,
            status.communication_status_joint_4,
            status.communication_status_joint_5,
            status.communication_status_joint_6,
        ]
        if any(communication_fields):
            return 'Piper joint communication fault'
        return ''

    def _guard(self, goal_handle, ignore_cancel=False):
        """Raise when cancellation or a hardware fault prevents motion."""
        if goal_handle.is_cancel_requested and not ignore_cancel:
            raise TaskCanceled('task canceled')
        fault = self._arm_fault()
        if fault:
            raise TaskFailure(fault)

    @staticmethod
    def _wait_future(future, timeout, goal_handle=None):
        """Wait for a ROS future while subscription callbacks continue."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if (
                goal_handle is not None
                and goal_handle.is_cancel_requested
            ):
                return False
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return True

    def _select_interest(self, target_name, goal_handle):
        """Request the unique YOLO class selected by the Action goal."""
        if not self.interest_client.wait_for_service(timeout_sec=3.0):
            raise TaskFailure('/set_interest service unavailable')
        request = SetInterest.Request()
        request.name = target_name
        future = self.interest_client.call_async(request)
        if not self._wait_future(future, 3.0, goal_handle):
            raise TaskFailure('timed out setting YOLO interest')
        response = future.result()
        if response is None or not response.result.startswith(
            'interest changed'
        ):
            detail = response.result if response is not None else 'no response'
            raise TaskFailure(f'YOLO rejected target class: {detail}')

    def _clear_target_tracking(self):
        """Discard target poses captured from an earlier camera viewpoint."""
        with self.data_lock:
            self.target_samples.clear()
            self.latest_target = None
            self.latest_target_received = 0.0

    def _wait_for_stable_target(
        self,
        goal_handle,
        timeout=None,
        timeout_message='stable button pose acquisition timed out',
    ):
        """Wait for enough recent, mutually consistent button poses."""
        timeout = self.target_acquire_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            now = time.monotonic()
            with self.data_lock:
                samples = [
                    sample for sample in self.target_samples
                    if now - sample[2] <= self.target_pause_age
                ]
            if len(samples) >= self.stable_sample_count:
                recent = samples[-self.stable_sample_count:]
                result = average_stable_poses(
                    [sample[0] for sample in recent],
                    [sample[1] for sample in recent],
                    self.stable_position_spread,
                    self.stable_angle_spread,
                )
                if result is not None:
                    return result
            self._feedback(goal_handle)
            time.sleep(0.05)
        now = time.monotonic()
        with self.data_lock:
            recent_count = sum(
                now - sample[2] <= self.target_pause_age
                for sample in self.target_samples
            )
            latest_received = self.latest_target_received
        latest_age = (now - latest_received) if latest_received else math.inf
        raise TaskFailure(
            f'{timeout_message}; recent_samples={recent_count}/'
            f'{self.stable_sample_count}, latest_age_s={latest_age:.2f}'
        )

    def _apply_collision_scene(self, button_position, button_quaternion):
        """Add the elevator panel and eye-in-hand camera collision boxes."""
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise TaskFailure('/apply_planning_scene service unavailable')

        scene = PlanningScene()
        scene.is_diff = True
        panel = CollisionObject()
        panel.header.frame_id = self.base_frame
        panel.id = 'pbvs_elevator_panel'
        panel.operation = CollisionObject.ADD
        panel_shape = SolidPrimitive()
        panel_shape.type = SolidPrimitive.BOX
        panel_shape.dimensions = [
            self.panel_width,
            self.panel_height,
            self.panel_thickness,
        ]
        panel_pose = Pose()
        panel_center = offset_along_press_axis(
            button_position,
            button_quaternion,
            self.panel_thickness * 0.5,
        )
        panel_pose.position.x = float(panel_center[0])
        panel_pose.position.y = float(panel_center[1])
        panel_pose.position.z = float(panel_center[2])
        panel_pose.orientation.x = float(button_quaternion[0])
        panel_pose.orientation.y = float(button_quaternion[1])
        panel_pose.orientation.z = float(button_quaternion[2])
        panel_pose.orientation.w = float(button_quaternion[3])
        panel.primitives.append(panel_shape)
        panel.primitive_poses.append(panel_pose)
        scene.world.collision_objects.append(panel)

        if self.camera_collision_enabled:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.flange_frame,
                    self.camera_link_frame,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as error:
                raise TaskFailure(
                    f'camera collision TF unavailable: {error}'
                ) from error
            attached = AttachedCollisionObject()
            attached.link_name = self.flange_frame
            attached.touch_links = [
                self.flange_frame,
                'gripper_base',
                self.tcp_frame,
            ]
            attached.object.header.frame_id = self.flange_frame
            attached.object.id = 'pbvs_eye_in_hand_camera'
            attached.object.operation = CollisionObject.ADD
            camera_shape = SolidPrimitive()
            camera_shape.type = SolidPrimitive.BOX
            camera_shape.dimensions = [
                self.camera_size_x,
                self.camera_size_y,
                self.camera_size_z,
            ]
            camera_pose = Pose()
            camera_pose.position.x = transform.transform.translation.x
            camera_pose.position.y = transform.transform.translation.y
            camera_pose.position.z = transform.transform.translation.z
            camera_pose.orientation = transform.transform.rotation
            attached.object.primitives.append(camera_shape)
            attached.object.primitive_poses.append(camera_pose)
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(attached)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        if not self._wait_future(future, 3.0):
            raise TaskFailure('planning scene update timed out')
        if future.result() is None or not future.result().success:
            raise TaskFailure('MoveIt rejected coarse collision scene')

    def _moveit_goal(self, target_pose, plan_only):
        """Construct the coarse MoveGroup goal."""
        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.tcp_frame
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        diameter = 2.0 * self.moveit_position_tolerance
        region.dimensions = [diameter, diameter, diameter]
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.tcp_frame
        orientation_constraint.orientation = target_pose.pose.orientation
        tolerance = self.moveit_orientation_tolerance
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.move_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = (
            self.moveit_velocity_scaling_factor
        )
        goal.request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scaling_factor
        )
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = not plan_only
        goal.planning_options.replan_delay = 1.0
        return goal

    def _apply_stage_speed(self, motion_goal, stage):
        prefix = {
            'button transition': 'transition',
            'panel-normal movement': 'press',
            'base-link X movement': 'press',
            'panel retract': 'retract',
        }.get(stage)
        if prefix is not None:
            motion_goal.request.max_velocity_scaling_factor = getattr(self, prefix + '_velocity_scaling_factor')
            motion_goal.request.max_acceleration_scaling_factor = getattr(self, prefix + '_acceleration_scaling_factor')
        self.get_logger().info(
            f'【阶段速度】{stage}: velocity={motion_goal.request.max_velocity_scaling_factor:.3f}, '
            f'acceleration={motion_goal.request.max_acceleration_scaling_factor:.3f}')

    def _run_moveit(
        self,
        target_pose,
        goal_handle,
        stage='coarse approach',
    ):
        """Plan or execute one guarded MoveIt target pose."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise TaskFailure('/move_action unavailable')
        motion_goal = self._moveit_goal(target_pose, not self.enable_motion)
        self._apply_stage_speed(motion_goal, stage)
        if stage == 'panel retract':
            # Retreat must arrive inside the measured 6 mm gate. Use a
            # tighter planning region without relaxing measured acceptance.
            constraints = motion_goal.request.goal_constraints[0]
            for constraint in constraints.position_constraints:
                constraint.constraint_region.primitives[0].dimensions = [0.001] * 3
            for constraint in constraints.orientation_constraints:
                constraint.absolute_x_axis_tolerance = 0.02
                constraint.absolute_y_axis_tolerance = 0.02
                constraint.absolute_z_axis_tolerance = 0.02
            self.get_logger().info(
                '【退回精度】位置规划半宽0.5mm，姿态容差0.02rad；实测门限仍为6mm'
            )
        send_future = self.move_group_client.send_goal_async(motion_goal)
        if not self._wait_future(send_future, 5.0, goal_handle):
            raise TaskFailure('MoveIt goal submission timed out')
        move_goal = send_future.result()
        if move_goal is None or not move_goal.accepted:
            raise TaskFailure(f'MoveIt rejected {stage}')
        self.active_move_goal = move_goal
        result_future = move_goal.get_result_async()
        result_ready = threading.Event()
        result_future.add_done_callback(lambda _: result_ready.set())
        deadline = time.monotonic() + self.moveit_timeout
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                move_goal.cancel_goal_async()
                raise TaskCanceled(f'canceled during MoveIt {stage}')
            if time.monotonic() >= deadline:
                move_goal.cancel_goal_async()
                raise TaskFailure(f'MoveIt {stage} timed out')
            self._feedback(goal_handle)
            result_ready.wait(0.01)
        self.active_move_goal = None
        wrapped_result = result_future.result()
        if (
            wrapped_result is None
            or wrapped_result.result.error_code.val != 1
        ):
            code = (
                wrapped_result.result.error_code.val
                if wrapped_result is not None else 'unknown'
            )
            raise TaskFailure(
                f'MoveIt {stage} failed with error code {code}'
            )
        self.last_moveit_arm_target = self._final_moveit_arm_target(
            wrapped_result.result,
        )

    def _verify_target_pose(
        self,
        goal_handle,
        target_position,
        target_quaternion,
        movement_label,
    ):
        """Require fresh measured TCP feedback at a generic MoveIt target."""
        position_tolerance = (
            self.APPROACH_POSITION_TOLERANCE
            if movement_label in ('coarse approach', 'button transition')
            else self.RETRACT_POSITION_TOLERANCE if movement_label == 'panel retract'
            else self.X_POSITION_TOLERANCE
        )
        deadline = time.monotonic() + 3.0
        last_position_error = math.inf
        last_angular_error = math.inf
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            try:
                current_position, current_quaternion = (
                    self._latest_tcp_arrays()
                )
            except TaskFailure:
                time.sleep(0.01)
                continue
            _, _, position_error, angular_error = pose_error(
                target_position,
                target_quaternion,
                current_position,
                current_quaternion,
            )
            last_position_error = position_error
            last_angular_error = angular_error
            self._feedback(goal_handle, position_error, angular_error)
            if (
                position_error <= position_tolerance
                and angular_error <= self.X_ORIENTATION_TOLERANCE
            ):
                return current_position, current_quaternion
            time.sleep(0.01)
        raise TaskFailure(
            f'{movement_label} did not reach its measured target; '
            f'position_error={last_position_error * 1000.0:.2f} mm, '
            f'position_limit={position_tolerance * 1000.0:.2f} mm, '
            f'angular_error={last_angular_error:.6f} rad, '
            f'angular_limit={self.X_ORIENTATION_TOLERANCE:.6f} rad'
        )

    def _run_x_advance(
        self,
        goal_handle,
        measured_position,
        measured_quaternion,
        press_quaternion,
    ):
        """Move from measured coarse T0 along the configured advance axis."""
        self._set_state('X_ADVANCE')
        if self.x_advance_axis_mode == 'panel_normal':
            target_position = offset_along_press_axis(
                measured_position,
                press_quaternion,
                self.distance_m,
            )
            axis_label = '视觉锁定的面板按压轴'
            movement_label = 'panel-normal movement'
        else:
            target_position = translated_base_x(
                measured_position,
                self.distance_m,
            )
            axis_label = 'base_link X'
            movement_label = 'base-link X movement'
        target_pose = self._pose_message(
            target_position,
            measured_quaternion,
        )
        self.desired_tcp_pub.publish(target_pose)
        self.get_logger().info(
            f'【按压移动】模式={self.x_advance_axis_mode}，'
            f'从粗定位实测 T0 沿 {axis_label} '
            f'移动 {self.distance_m * 1000.0:+.3f} mm，目标='
            + self._format_xyz(target_position)
        )
        self._run_moveit(
            target_pose,
            goal_handle,
            stage=movement_label,
        )
        reached_position, reached_quaternion = self._verify_target_pose(
            goal_handle,
            target_position,
            measured_quaternion,
            movement_label,
        )
        _, _, position_error, angular_error = pose_error(
            target_position,
            measured_quaternion,
            reached_position,
            reached_quaternion,
        )
        self._stage_success(
            'X_ADVANCE',
            f'{axis_label}移动完成，位置误差='
            f'{position_error * 1000.0:.2f} mm，'
            f'姿态误差={angular_error:.6f} rad',
        )

    def _latest_tcp_arrays(self):
        """Return fresh measured TCP pose arrays."""
        with self.data_lock:
            message = copy.deepcopy(self.latest_tcp)
            received = self.latest_tcp_received
        if message is None:
            raise TaskFailure('no /tcp_pose feedback')
        if time.monotonic() - received > self.tcp_feedback_timeout:
            raise TaskFailure('/tcp_pose feedback is stale')
        return self._pose_arrays(message.pose)

    def _log_coarse_verification_failure(
        self,
        attempt_number,
        total_attempts,
        target_position,
        target_quaternion,
        measured_position,
        measured_quaternion,
        position_error,
        angular_error,
        axial_distance,
        axial_error,
        lateral_vector,
        lateral_error,
    ):
        """Log the last coarse-pose sample without changing task behavior."""
        prefix = (
            '【粗定位验收超差诊断 '
            f'{attempt_number}/{total_attempts}】'
        )
        self.get_logger().error(
            prefix + 'C0目标TCP position='
            + self._format_xyz(target_position)
            + ' quaternion='
            + self._format_quaternion(target_quaternion)
        )
        if measured_position is None or measured_quaternion is None:
            self.get_logger().error(prefix + 'T0实测TCP=unavailable')
            self.get_logger().error(prefix + 'T0-C0=unavailable')
            self.get_logger().error(prefix + '位置误差=unavailable')
            self.get_logger().error(prefix + '姿态误差=unavailable')
            self.get_logger().error(prefix + '法向/横向误差=unavailable')
        else:
            delta = np.asarray(measured_position) - np.asarray(
                target_position
            )
            self.get_logger().error(
                prefix + 'T0实测TCP position='
                + self._format_xyz(measured_position)
                + ' quaternion='
                + self._format_quaternion(measured_quaternion)
            )
            self.get_logger().error(
                prefix + 'T0-C0=' + self._format_xyz(delta)
            )
            self.get_logger().error(
                prefix
                + f'位置误差={position_error:.6f} m '
                + f'({position_error * 1000.0:.2f} mm)'
            )
            self.get_logger().error(
                prefix
                + f'姿态误差={angular_error:.6f} rad '
                + f'({math.degrees(angular_error):.2f} deg)'
            )
            self.get_logger().error(
                prefix
                + f'法向距离={axial_distance:.6f} m, '
                + f'法向距离误差={axial_error:+.6f} m, '
                + f'允许±{self.coarse_axial_tolerance:.4f} m'
            )
            self.get_logger().error(
                prefix + '横向误差向量='
                + self._format_xyz(lateral_vector)
                + f', 模长={lateral_error:.6f} m, '
                + '允许范围=['
                + f'{self.coarse_lateral_error_min:.4f}, '
                + f'{self.coarse_lateral_error_max:.4f}] m'
            )

        target_joints = self.last_moveit_arm_target
        if target_joints is None:
            self.get_logger().error(prefix + '目标关节角(rad)=unavailable')
        else:
            self.get_logger().error(
                prefix + '目标关节角(rad): '
                + self._format_arm_joints(target_joints)
            )

        with self.data_lock:
            measured_joints = self.latest_joint_positions
            joint_received = self.latest_joint_received
        if measured_joints is None or joint_received <= 0.0:
            self.get_logger().error(prefix + '实测关节角(rad)=unavailable')
        else:
            joint_age = max(0.0, time.monotonic() - joint_received)
            freshness = (
                'fresh'
                if joint_age <= self.tcp_feedback_timeout else 'stale'
            )
            self.get_logger().error(
                prefix + '实测关节角(rad): '
                + self._format_arm_joints(measured_joints)
                + f', age={joint_age:.3f} s ({freshness})'
            )

    def _verify_coarse_pose(
        self,
        goal_handle,
        button_position,
        position,
        quaternion,
        attempt_number,
        total_attempts,
    ):
        """Verify measured TCP feedback after MoveIt reports success."""
        deadline = time.monotonic() + 3.0
        last_position = None
        last_quaternion = None
        last_position_error = math.inf
        last_angular_error = math.inf
        last_axial_distance = math.nan
        last_axial_error = math.nan
        last_lateral_vector = None
        last_lateral_error = math.inf
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            try:
                current_position, current_quaternion = (
                    self._latest_tcp_arrays()
                )
            except TaskFailure:
                time.sleep(0.05)
                continue
            _, _, position_error, angular_error = pose_error(
                position,
                quaternion,
                current_position,
                current_quaternion,
            )
            last_position = current_position
            last_quaternion = current_quaternion
            last_position_error = position_error
            last_angular_error = angular_error
            (
                axial_distance,
                axial_error,
                lateral_vector,
                lateral_error,
            ) = coarse_standoff_errors(
                button_position,
                current_position,
                quaternion,
                self.coarse_standoff,
            )
            last_axial_distance = axial_distance
            last_axial_error = axial_error
            last_lateral_vector = lateral_vector
            last_lateral_error = lateral_error
            self._feedback(
                goal_handle,
                position_error,
                angular_error,
            )
            if coarse_pose_is_acceptable(
                axial_error,
                lateral_error,
                angular_error,
                self.coarse_axial_tolerance,
                self.coarse_lateral_error_min,
                self.coarse_lateral_error_max,
                1.5 * self.moveit_orientation_tolerance,
            ):
                # MoveIt accepts a region around the requested pose. Return
                # the pose actually reached for final diagnostics.
                return current_position, current_quaternion
            time.sleep(0.05)
        self._log_coarse_verification_failure(
            attempt_number,
            total_attempts,
            position,
            quaternion,
            last_position,
            last_quaternion,
            last_position_error,
            last_angular_error,
            last_axial_distance,
            last_axial_error,
            last_lateral_vector,
            last_lateral_error,
        )
        return None

    def _control_quaternion(
        self,
        detected_quaternion,
        roll_reference_quaternion,
    ):
        """Build a control orientation from the detected press axis."""
        if self.orientation_mode == 'world_up':
            return detected_quaternion
        press_axis = quaternion_to_matrix(detected_quaternion)[:, 2]
        return align_tool_z_preserve_roll(
            roll_reference_quaternion,
            press_axis,
        )

    def _result(self, success, message):
        """Construct a PressButton result."""
        result = PressButton.Result()
        result.success = success
        result.message = message
        result.hard_safety_stop = bool(getattr(self, 'uncertain_motion', False))
        return result

    def _prepare_sequence_snapshot(self, targets, goal_handle):
        """Freeze every target while the arm remains at the observation pose."""
        self.sequence_snapshot = {}
        self.sequence_snapshot_created = 0.0
        targets = tuple(dict.fromkeys(targets))
        if not targets or len(targets) > 3 or any(
            name not in tuple(f'key_{i}' for i in range(10)) + ('key_ok',)
            for name in targets
        ):
            raise TaskFailure('invalid sequence snapshot targets')
        self.sequence_roll_reference = self._latest_tcp_arrays()[1]
        with self.data_lock:
            start_joints = copy.deepcopy(self.latest_joint_positions)
        if start_joints is None:
            raise TaskFailure('no real joint feedback for snapshot')
        collected = {}
        for name in targets:
            self._set_state('SNAPSHOT_TARGET')
            self._select_interest(name, goal_handle)
            # Drain images from the preceding interest before taking samples.
            time.sleep(0.3)
            self._clear_target_tracking()
            collected[name] = self._wait_for_stable_target(
                goal_handle,
                timeout=self.sequence_snapshot_acquire_timeout,
                timeout_message=f'{name} stable button pose acquisition timed out',
            )
            with self.data_lock:
                joints = copy.deepcopy(self.latest_joint_positions)
                age = time.monotonic() - self.latest_joint_received
            if joints is None or age > self.tcp_feedback_timeout or np.max(
                np.abs(np.asarray(joints) - np.asarray(start_joints))
            ) > 0.003:
                raise TaskFailure('arm moved or feedback stale during snapshot')
            self.get_logger().info(f'【序列预采样】{name} 已稳定，机械臂未运动')
        self.sequence_snapshot = collected
        self.sequence_snapshot_created = time.monotonic()
        self.get_logger().info('【序列预采样】全部目标已锁定，允许开始运动')

    def _sequence_target(self, request, goal_handle):
        if request.snapshot_targets:
            if request.sequence_continuation:
                raise TaskFailure('snapshot and continuation cannot both be set')
            self._prepare_sequence_snapshot(request.snapshot_targets, goal_handle)
        if not request.snapshot_targets and not request.sequence_continuation:
            raise TaskFailure('first sequence request requires snapshot targets')
        if not self.sequence_snapshot or (
            time.monotonic() - self.sequence_snapshot_created > 60.0
        ):
            self.sequence_snapshot = {}
            raise TaskFailure('sequence snapshot missing or expired')
        name = request.target_name.strip()
        if name not in self.sequence_snapshot:
            raise TaskFailure(f'{name} absent from sequence snapshot')
        self.get_logger().info(f'【序列锁定目标】使用预采样 {name}，不重新识别')
        return self.sequence_snapshot[name]

    def _wait_planned_action(self, client, goal, goal_handle, timeout,
                             transition_target=None, press_target=None):
        """Record pure-planning requests without changing execution behavior."""
        plan_only = (client is getattr(self,'move_group_client',None)
                     and isinstance(goal,MoveGroup.Goal) and goal.planning_options.plan_only)
        started_ns = time.monotonic_ns()
        result = None
        error = None
        if plan_only:
            self.planning_request_number = getattr(self,'planning_request_number',0)+1
            request_number = self.planning_request_number
            context = dict(getattr(self,'planning_record_context',{}))
        try:
            result = self._wait_planned_action_impl(client,goal,goal_handle,timeout,
                                                   transition_target,press_target)
            return result
        except Exception as failure:
            error = {'type':type(failure).__name__,'message':str(failure)}
            raise
        finally:
            if plan_only and hasattr(self,'segment_timing_pub'):
                ended_ns = time.monotonic_ns()
                self.segment_timing_pub.publish(String(data=json.dumps({
                    **context,'kind':'planning_request','request_number':request_number,
                    'started_monotonic_ns':started_ns,'ended_monotonic_ns':ended_ns,
                    'elapsed_s':(ended_ns-started_ns)/1e9,'success':result is not None,
                    'error':error,
                    'moveit_reported_planning_s':getattr(result,'planning_time',None),
                    'pipeline':goal.request.pipeline_id or 'ompl',
                    'planner':goal.request.planner_id,
                    'start_joint_names':list(goal.request.start_state.joint_state.name),
                    'start_joint_positions':list(goal.request.start_state.joint_state.position),
                })))

    def _wait_planned_action_impl(self, client, goal, goal_handle, timeout,
                             transition_target=None, press_target=None):
        if not client.wait_for_server(timeout_sec=5.0):
            raise TaskFailure('preplanned motion action unavailable')
        stamp = time.monotonic_ns()
        wall_stamp = time.time_ns()
        submission = client.send_goal_async(goal)
        if press_target is not None:
            self.press_command_pub.publish(String(data=json.dumps({
                'target': press_target, 'monotonic_ns': stamp,
                'wall_time_ns': wall_stamp, 'stage': 'panel-normal movement',
                'planned': True, 'reference': 'before_send_goal_async',
            })))
        if transition_target is not None:
            # Timestamp immediately after the action client submitted the
            # trajectory, before waiting for acceptance or execution.
            submitted_ns = time.monotonic_ns()
            self.transition_command_pub.publish(String(data=json.dumps({
                'target': transition_target,
                'monotonic_ns': submitted_ns,
                'wall_time_ns': time.time_ns(),
                'stage': 'button transition',
                'planned': True,
            })))
        if not self._wait_future(submission, 5.0, goal_handle):
            # Cancel an acceptance arriving after timeout/cancellation too.
            def cancel_late(future):
                handle = future.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
            submission.add_done_callback(cancel_late)
            raise TaskFailure('preplanned action submission interrupted')
        handle = submission.result()
        if handle is None or not handle.accepted:
            raise TaskFailure('preplanned motion rejected')
        self.active_move_goal = handle
        result = handle.get_result_async()
        ready = threading.Event()
        result.add_done_callback(lambda _: ready.set())
        deadline = time.monotonic() + timeout
        while not result.done():
            try:
                self._guard(goal_handle)
                if time.monotonic() > deadline:
                    raise TaskFailure('preplanned motion timed out')
            except Exception:
                handle.cancel_goal_async()
                # Do not release the task while termination is uncertain.
                stopped = self._wait_future(result, 5.0)
                if not stopped:
                    self.get_logger().error('【运动终止未确认】停止后续任务，请确认真机已停')
                    self.uncertain_motion = True
                raise
            self._feedback(goal_handle)
            ready.wait(0.01)
        self.active_move_goal = None
        wrapped = result.result()
        if wrapped is None or wrapped.result is None:
            raise TaskFailure('preplanned action returned no result')
        if wrapped.status != 4 or wrapped.result.error_code.val != 1:
            code = wrapped.result.error_code.val
            if (client is self.move_group_client
                    and goal.planning_options.plan_only
                    and wrapped.status in (4, 6)):
                raise PlanningFailure(f'MoveIt plan-only failed with error code {code}')
            raise TaskFailure(
                f'preplanned trajectory planning/execution failed; '
                f'status={wrapped.status}, error_code={code}'
            )
        return wrapped.result

    def _plan_segment_with_retry(self, goal, goal_handle, name, stage):
        """Retry completed plan-only failures; no trajectory has run yet."""
        started = time.monotonic()
        timeout = self.preplan_retry_timeout_sec
        # A positive time budget supersedes the legacy attempt-count limit.
        deadline = started + timeout if timeout > 0 else None
        attempt = 0
        while True:
            self._guard(goal_handle)
            if deadline is not None and attempt > 0 and time.monotonic() >= deadline:
                raise TaskFailure(
                    f'{name} {stage} planning failed after {attempt} attempts '
                    f'and {timeout:g}s retry budget'
                )
            attempt += 1
            try:
                return self._wait_planned_action(
                    self.move_group_client, copy.deepcopy(goal),
                    goal_handle, self.moveit_timeout,
                )
            except PlanningFailure as error:
                self._guard(goal_handle)
                elapsed = time.monotonic() - started
                exhausted = (elapsed >= timeout if deadline is not None
                             else attempt > self.preplan_retry_attempts)
                if exhausted:
                    detail = (f'{timeout:g}s retry budget'
                              if deadline is not None
                              else f'{attempt} attempts')
                    raise TaskFailure(
                        f'{name} {stage} planning failed after {detail}: {error}'
                    ) from error
                self.get_logger().warning(
                    f'【提前规划重试】{name} {stage} 第{attempt}次失败，'
                    f'已耗时{elapsed:.1f}s：{error}；从相同起点重新规划'
                )

    @staticmethod
    def _trajectory_duration(result):
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            raise TaskFailure('empty planned candidate trajectory')
        t = points[-1].time_from_start
        return t.sec + t.nanosec * 1e-9

    def _plan_shortest_transition(self, goal, goal_handle, name, single_attempt=False, check_budget=None):
        best = (self._wait_planned_action(self.move_group_client,copy.deepcopy(goal),goal_handle,self.moveit_timeout)
                if single_attempt else
                self._plan_segment_with_retry(goal,goal_handle,name,'button transition'))
        original_duration = self._trajectory_duration(best)
        durations = [original_duration]
        for _ in range(getattr(self,'transition_plan_candidates',1)-1):
            self._guard(goal_handle)
            if check_budget is not None:
                check_budget()
            try:
                candidate = self._wait_planned_action(self.move_group_client,copy.deepcopy(goal),goal_handle,self.moveit_timeout)
            except PlanningFailure:
                continue
            durations.append(self._trajectory_duration(candidate))
            if self._trajectory_duration(candidate) < self._trajectory_duration(best):
                best = candidate
        if hasattr(self,'segment_timing_pub'):
            self.segment_timing_pub.publish(String(data=json.dumps({
                'target': name, 'stage': 'button transition', 'kind': 'planning_selection',
                'candidate_durations_s': durations,
                'selected_duration_s': self._trajectory_duration(best),
                'saved_planned_time_s': original_duration-self._trajectory_duration(best),
            })))
        return best

    def _plan_shortest_retract(self, goal, goal_handle, name, check_budget):
        """Compare retreats from the same press endpoint; retain a valid plan."""
        best = self._wait_planned_action(self.move_group_client,copy.deepcopy(goal),goal_handle,self.moveit_timeout)
        durations = [self._trajectory_duration(best)]
        for _ in range(getattr(self,'retract_plan_candidates',1)-1):
            self._guard(goal_handle)
            if getattr(self,'planning_interval_target_sec',0.0)>0 and self._trajectory_duration(best)<=0.55:
                break
            try:
                check_budget()
            except PlanningBudgetExceeded:
                self._guard(goal_handle)
                break
            try:
                candidate = self._wait_planned_action(self.move_group_client,copy.deepcopy(goal),goal_handle,self.moveit_timeout)
            except PlanningFailure:
                continue
            durations.append(self._trajectory_duration(candidate))
            if durations[-1] < self._trajectory_duration(best):
                best = candidate
        if hasattr(self,'segment_timing_pub'):
            self.segment_timing_pub.publish(String(data=json.dumps({
                'target':name,'stage':'panel retract','kind':'retract_selection',
                'candidate_durations_s':durations,
                'selected_duration_s':self._trajectory_duration(best),
                'saved_planned_time_s':durations[0]-self._trajectory_duration(best),
            })))
        return best

    def _plan_button_chain_with_retry(self, plan_chain, initial_start, goal_handle, name, candidate_cost=None, candidate_observer=None, shared_deadline=None, candidate_stop=None):
        """Retry the whole approach/press/retract chain before any execution."""
        started = time.monotonic()
        timeout = self.preplan_retry_timeout_sec
        deadline = shared_deadline if shared_deadline is not None else (started + timeout if timeout > 0 else None)
        attempt = 0
        last_error = None
        best = None
        best_cost = math.inf
        successful_costs = []
        desired = getattr(self,'transition_plan_candidates',1) if candidate_cost is not None else 1
        def finish():
            if hasattr(self,'segment_timing_pub'):
                self.segment_timing_pub.publish(String(data=json.dumps({
                    'target': name, 'stage': 'button chain', 'kind': 'planning_selection',
                    'candidate_durations_s': successful_costs, 'selected_duration_s': best_cost,
                    'saved_planned_time_s': successful_costs[0]-best_cost,
                    'reference': 'approach_press_retract_combined',
                })))
            return best
        while True:
            self._guard(goal_handle)
            if deadline is not None and attempt and time.monotonic() >= deadline:
                if best is not None:
                    return finish()
                raise PlanningChainFailure(f'{name} approach/press/retract planning failed after {timeout:g}s retry budget: {last_error}')
            attempt += 1
            try:
                def check_budget():
                    self._guard(goal_handle)
                    if deadline is not None and time.monotonic() >= deadline:
                        raise PlanningBudgetExceeded(f'{name} approach/press/retract planning exceeded {timeout:g}s retry budget')
                candidate = plan_chain(copy.deepcopy(initial_start), check_budget)
                cost = candidate_cost(candidate) if candidate_cost is not None else 0.0
                successful_costs.append(cost)
                if candidate_observer is not None:
                    candidate_observer(candidate)
                if cost < best_cost:
                    best,best_cost = candidate,cost
                if len(successful_costs) >= desired or (candidate_stop is not None and candidate_stop(candidate,len(successful_costs))):
                    return finish()
            except PlanningBudgetExceeded:
                self._guard(goal_handle)
                if best is not None:
                    return finish()
                raise
            except PlanningFailure as error:
                last_error = error
                self._guard(goal_handle)
                exhausted = (time.monotonic() >= deadline if deadline is not None
                             else attempt > self.preplan_retry_attempts)
                if exhausted:
                    if best is not None:
                        return finish()
                    detail = f'{timeout:g}s retry budget' if deadline is not None else f'{attempt} chain attempts'
                    raise PlanningChainFailure(f'{name} approach/press/retract planning failed after {detail}: {error}') from error
                self.get_logger().warning(
                    f'【联合规划重试】{name} 第{attempt}组失败：{error}；'
                    '丢弃本组所有轨迹，从原起点重新规划靠近、按压、回退；尚未运动'
                )

    @staticmethod
    def _segment_seconds(segment):
        points = segment[0].joint_trajectory.points
        if not points:
            raise TaskFailure('empty candidate segment')
        t = points[-1].time_from_start
        return t.sec + t.nanosec * 1e-9

    def _extend_sequence_candidate(self, parent, name, candidate):
        segments, end = candidate
        times = {stage:self._segment_seconds(segment)
                 for segment in segments for stage in [segment[3]]}
        intervals = list(parent['intervals'])
        if parent['buttons']:
            intervals.append(parent['last_retract_s']
                             + times.get('button transition',0.0)
                             + times['panel-normal movement'])
        return {
            'buttons': parent['buttons'] + [(name,segments)],
            'end': copy.deepcopy(end), 'intervals': intervals,
            'last_retract_s': times['panel retract'],
            'planned_total_s': parent['planned_total_s'] + sum(times.values()),
        }

    @staticmethod
    def _sequence_candidate_rank(candidate):
        # Minimize the slowest measured-to-measured trajectory interval first.
        return (max(candidate['intervals'],default=0.0),
                sum(candidate['intervals']),candidate['planned_total_s'])

    def _plan_adjacent_sequence(self, factories, initial, goal_handle):
        states = [{'buttons':[], 'end':copy.deepcopy(initial), 'intervals':[],
                   'last_retract_s':0.0,'planned_total_s':0.0}]
        width = getattr(self,'sequence_search_width',1)
        for layer_index,(name, plan_chain) in enumerate(factories):
            timeout = self.preplan_retry_timeout_sec
            deadline = time.monotonic()+timeout if timeout > 0 else None
            extended = []
            errors = []
            for parent_index,parent in enumerate(states):
                self._guard(goal_handle)
                if deadline is not None and time.monotonic() >= deadline:
                    break
                # Reserve a fair share for remaining endpoint branches.
                branch_deadline = (time.monotonic() + max(0.0,deadline-time.monotonic())/(len(states)-parent_index)
                                   if deadline is not None else None)
                def cost(candidate):
                    return sum(self._segment_seconds(segment) for segment in candidate[0]
                               if segment[3] != 'coarse approach')
                def observe(candidate):
                    extended.append(self._extend_sequence_candidate(parent,name,candidate))
                def fast_enough(candidate,successful_count):
                    target = getattr(self,'planning_interval_target_sec',0.0)
                    if target <= 0 or not parent['buttons']:
                        return False
                    value = self._extend_sequence_candidate(parent,name,candidate)
                    minimum = 1 if layer_index == len(factories)-1 else 2
                    return (successful_count >= minimum and
                            max(value['intervals'],default=math.inf) <= target)
                try:
                    self._plan_button_chain_with_retry(
                        plan_chain,parent['end'],goal_handle,name,cost,
                        candidate_observer=observe,shared_deadline=branch_deadline,candidate_stop=fast_enough,
                    )
                except (PlanningChainFailure,PlanningBudgetExceeded) as error:
                    errors.append(str(error))
                    continue
                target = getattr(self,'planning_interval_target_sec',0.0)
                if (layer_index == len(factories)-1 and target>0 and
                        any(max(c['intervals'],default=math.inf)<=target for c in extended)):
                    break
            if not extended:
                raise TaskFailure(f'{name} adjacent-sequence planning failed; no complete button chain: {errors}')
            extended.sort(key=self._sequence_candidate_rank)
            # Retain different endpoints; duplicate branches cannot help lookahead.
            states = []
            for candidate in extended:
                if any(np.max(np.abs(np.asarray(candidate['end'])-np.asarray(kept['end']))) < 0.001
                       for kept in states):
                    continue
                states.append(candidate)
                if len(states) >= width:
                    break
        selected = min(states,key=self._sequence_candidate_rank)
        if hasattr(self,'segment_timing_pub'):
            self.segment_timing_pub.publish(String(data=json.dumps({
                'target': factories[-1][0], 'stage':'sequence',
                'kind':'sequence_selection', 'planned_press_intervals_s':selected['intervals'],
                'maximum_planned_interval_s':max(selected['intervals'],default=0.0),
                'retained_branches':len(states), 'execution_verified':False,
                'planning_interval_target_sec':getattr(self,'planning_interval_target_sec',0.0),
            })))
        self.get_logger().info(f"【相邻键联合择优】规划到位间隔={selected['intervals']}s；未运动，未包含实机验收/交接")
        return selected['buttons']

    @staticmethod
    def _sample_plan_trace(trajectory, maximum_samples=101):
        """Bound joint-path logging while always keeping both endpoints."""
        jt = trajectory.joint_trajectory
        points = jt.points
        if not points:
            raise TaskFailure('empty selected plan trace')
        indices = sorted(set(int(round(v)) for v in
                             np.linspace(0,len(points)-1,min(maximum_samples,len(points)))))
        samples = []
        for index in indices:
            point = points[index]
            samples.append({
                'point_index':index,
                'time_from_start_s':point.time_from_start.sec+point.time_from_start.nanosec*1e-9,
                'positions_rad':list(point.positions),
            })
        return {'joint_names':list(jt.joint_names),'original_point_count':len(points),
                'sampled_point_count':len(samples),'points':samples,
                'reference':'selected_planned_joint_path_downsampled'}

    @staticmethod
    def _trajectory_joint_diagnostics(trajectory):
        """Profile selected planned joint motion; this is not hardware feedback."""
        jt = trajectory.joint_trajectory
        if not jt.points:
            raise TaskFailure('empty diagnostic trajectory')
        profiles = []
        times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points]
        for index,name in enumerate(jt.joint_names):
            positions = [p.positions[index] for p in jt.points]
            velocities = [abs(p.velocities[index]) for p in jt.points if len(p.velocities)==len(jt.joint_names)]
            accelerations = [abs(p.accelerations[index]) for p in jt.points if len(p.accelerations)==len(jt.joint_names)]
            secants = [abs((positions[k]-positions[k-1])/(times[k]-times[k-1]))
                       for k in range(1,len(times)) if times[k]>times[k-1]]
            profiles.append({
                'joint':name, 'delta_rad':positions[-1]-positions[0],
                'travel_rad':sum(abs(b-a) for a,b in zip(positions,positions[1:])),
                'range_rad':max(positions)-min(positions),
                'peak_planned_velocity_rad_s':max(velocities) if velocities else None,
                'peak_planned_acceleration_rad_s2':max(accelerations) if accelerations else None,
                'peak_secant_velocity_rad_s':max(secants,default=0.0),
            })
        return profiles

    def _preplan_snapshot(self, names, goal_handle):
        """Plan every pose before motion, with each prior endpoint as start."""
        from piper_pbvs_control.control_math import sequence_button_positions
        self.preplanned_buttons = []
        self.preplanned_index = 0
        first_position, first_quaternion = self.sequence_snapshot[names[0]]
        first_axis = quaternion_to_matrix(first_quaternion)[:, 2]
        for name in names:
            position, quaternion = self.sequence_snapshot[name]
            axis = quaternion_to_matrix(quaternion)[:, 2]
            angle = math.acos(float(np.clip(np.dot(axis,first_axis),-1.0,1.0)))
            separation = abs(float(np.dot(np.asarray(position)-np.asarray(first_position),first_axis)))
            if angle > self.SNAPSHOT_NORMAL_ANGLE_LIMIT or separation > 0.01:
                raise TaskFailure(
                    f'snapshot targets disagree on panel plane: {names[0]} vs {name}; '
                    f'normal_angle={math.degrees(angle):.3f} deg (limit=5.000 deg), '
                    f'plane_separation={separation*1000:.2f} mm (limit=10.00 mm)'
                )
        self._apply_collision_scene(first_position, first_quaternion)
        with self.data_lock:
            initial = copy.deepcopy(self.latest_joint_positions)
        if initial is None:
            raise TaskFailure('no initial joints for preplanning')
        factories = []
        sequence_planning_started_ns = time.monotonic_ns()
        for index, name in enumerate(names):
            position, button_quaternion = self.sequence_snapshot[name]
            control = self._control_quaternion(button_quaternion, self.sequence_roll_reference)
            # One frozen panel model for the entire batch. Reject inconsistent
            # plane normals instead of changing collision geometry mid-batch.
            axis = quaternion_to_matrix(button_quaternion)[:, 2]
            first_axis = quaternion_to_matrix(first_quaternion)[:, 2]
            approach, press, retreat = sequence_button_positions(
                position, control, button_quaternion,
                self.coarse_standoff, self.distance_m,
                self.sequence_retract_distance_mm / 1000.0,
                self.coarse_horizontal_offset, self.coarse_vertical_offset,
                first=index == 0,
            )
            stages = ('coarse approach' if index == 0 else 'button transition',
                      'panel-normal movement', 'panel retract')
            def plan_chain(start, check_budget, index=index, name=name,
                           approach=approach, press=press, retreat=retreat,
                           stages=stages, control=control):
                segments = []
                for target, stage in zip((approach, press, retreat), stages):
                    if stage == 'button transition' and names[index] == names[index-1]:
                        self.get_logger().info(f'【重复键优化】{name} 已在同键退回点，跳过零位移转移')
                        continue
                    check_budget()
                    self._set_state('PREPLAN_SEQUENCE')
                    pose = self._pose_message(target, control)
                    goal = self._moveit_goal(pose, True)
                    goal.request.start_state.joint_state.name = list(self.ARM_JOINT_NAMES)
                    goal.request.start_state.joint_state.position = [float(value) for value in start]
                    goal.request.start_state.is_diff = True
                    goal.planning_options.planning_scene_diff.is_diff = True
                    self._apply_stage_speed(goal, stage)
                    if stage in ('panel-normal movement', 'panel retract'):
                        # Keep press/retract inside a narrow normal-axis corridor;
                        # collision-free endpoints alone do not imply straight motion.
                        previous = approach if stage == 'panel-normal movement' else press
                        corridor = PositionConstraint()
                        corridor.header.frame_id = self.base_frame
                        corridor.link_name = self.tcp_frame
                        corridor.weight = 1.0
                        box = SolidPrimitive()
                        box.type = SolidPrimitive.BOX
                        box.dimensions = [
                            2.0 * self.PRESS_PATH_LATERAL_HALF_WIDTH,
                            2.0 * self.PRESS_PATH_LATERAL_HALF_WIDTH,
                            float(np.linalg.norm(target - previous)) + 0.006,
                        ]
                        corridor.constraint_region.primitives.append(box)
                        corridor.constraint_region.primitive_poses.append(
                            self._pose_message((target + previous) / 2.0, control).pose)
                        goal.request.path_constraints.position_constraints.append(corridor)
                    # Tighten every preplanned endpoint to limit chain mismatch.
                    constraints = goal.request.goal_constraints[0]
                    constraints.position_constraints[0].constraint_region.primitives[0].dimensions = [0.001] * 3
                    for constraint in constraints.orientation_constraints:
                        constraint.absolute_x_axis_tolerance = 0.02
                        constraint.absolute_y_axis_tolerance = 0.02
                        constraint.absolute_z_axis_tolerance = 0.02
                    planning_started = time.monotonic()
                    self.planning_record_context = {'target':name,'stage':stage}
                    try:
                        result = (self._plan_shortest_retract(goal,goal_handle,name,check_budget)
                                  if stage == 'panel retract' else
                                  self._wait_planned_action(self.move_group_client,copy.deepcopy(goal),goal_handle,self.moveit_timeout))
                    except PlanningFailure as error:
                        raise PlanningFailure(f'{stage}: {error}') from error
                    trajectory = result.planned_trajectory
                    end = self._final_moveit_arm_target(result)
                    if end is None or not trajectory.joint_trajectory.points:
                        raise TaskFailure('empty preplanned trajectory')
                    duration = trajectory.joint_trajectory.points[-1].time_from_start
                    seconds = duration.sec + duration.nanosec * 1e-9
                    self.get_logger().info(f'【段规划】{name} {stage}: 规划耗时={time.monotonic()-planning_started:.3f}s, 轨迹时长={seconds:.3f}s')
                    segments.append((trajectory, np.asarray(target), control, stage))
                    start = end
                return segments, start
            factories.append((name,plan_chain))
        pending = self._plan_adjacent_sequence(factories,initial,goal_handle)
        with self.data_lock:
            actual = copy.deepcopy(self.latest_joint_positions)
            age = time.monotonic() - self.latest_joint_received
        if actual is None or age > self.tcp_feedback_timeout or np.max(
            np.abs(np.asarray(actual) - np.asarray(initial))
        ) > 0.003:
            raise TaskFailure('arm moved during preplanning')
        if hasattr(self,'segment_timing_pub'):
            for target_name,segments in pending:
                for trajectory,_,_,stage in segments:
                    self.segment_timing_pub.publish(String(data=json.dumps({
                        'target':target_name,'stage':stage,'kind':'joint_plan_profile',
                        'joints':self._trajectory_joint_diagnostics(trajectory),
                        'reference':'selected_plan_not_measured_motor_motion',
                    })))
        if hasattr(self,'segment_timing_pub'):
            planning_ended_ns = time.monotonic_ns()
            self.segment_timing_pub.publish(String(data=json.dumps({
                'target':names[0],'stage':'sequence','kind':'planning_total',
                'started_monotonic_ns':sequence_planning_started_ns,
                'ended_monotonic_ns':planning_ended_ns,
                'elapsed_s':(planning_ended_ns-sequence_planning_started_ns)/1e9,
                'selected_buttons':list(names),'includes_visual_sampling':False,
            })))
            for target_name,segments in pending:
                for trajectory,target,quaternion,stage in segments:
                    self.segment_timing_pub.publish(String(data=json.dumps({
                        'target':target_name,'stage':stage,'kind':'selected_plan_trace',
                        'frame_id':self.base_frame,'tcp_frame':self.tcp_frame,
                        'desired_position_m':list(map(float,target)),
                        'desired_quaternion_xyzw':list(map(float,quaternion)),
                        **self._sample_plan_trace(trajectory),
                    })))
        self.preplanned_buttons = pending
        self.sequence_snapshot_created = time.monotonic()
        self.get_logger().info('【提前规划】全部路径已通过碰撞规划；尚未运动')

    def _execute_preplanned_button(self, name, goal_handle):
        if self.preplanned_index >= len(self.preplanned_buttons):
            raise TaskFailure('preplanned sequence exhausted')
        planned_name, segments = self.preplanned_buttons[self.preplanned_index]
        if name != planned_name:
            raise TaskFailure('preplanned target order mismatch')
        if not self.enable_motion:
            self.preplanned_index += 1
            goal_handle.succeed()
            return self._result(True, 'entire sequence preplanned; dry-run, no motion')
        for trajectory, target, quaternion, stage in segments:
            with self.data_lock:
                actual = copy.deepcopy(self.latest_joint_positions)
                age = time.monotonic() - self.latest_joint_received
            jt = trajectory.joint_trajectory
            by_name = dict(zip(jt.joint_names, jt.points[0].positions))
            if actual is None or age > self.tcp_feedback_timeout or any(
                abs(value - by_name.get(joint, math.inf)) > 0.01
                for joint, value in zip(self.ARM_JOINT_NAMES, actual)
            ):
                raise TaskFailure('preplanned start no longer matches real arm')
            segment_started = time.monotonic()
            self.get_logger().info(f'【段执行开始】{name} {stage}')
            self._set_state('X_ADVANCE' if stage == 'panel-normal movement'
                            else 'RETRACT' if stage == 'panel retract' else 'COARSE_APPROACH')
            self.desired_tcp_pub.publish(self._pose_message(target, quaternion))
            execute = ExecuteTrajectory.Goal()
            execute.trajectory = copy.deepcopy(trajectory)
            # A cached path has no scheduled wall-clock start. Zero means
            # start immediately and cannot become stale during action handoff.
            execute.trajectory.joint_trajectory.header.stamp.sec = 0
            execute.trajectory.joint_trajectory.header.stamp.nanosec = 0
            self._wait_planned_action(
                self.execute_trajectory_client, execute, goal_handle,
                self.moveit_timeout,
                transition_target=name if stage == 'button transition' else None,
                press_target=name if stage == 'panel-normal movement' else None,
            )
            execution_completed = time.monotonic()
            self._verify_target_pose(goal_handle, target, quaternion, stage)
            verification_completed = time.monotonic()
            if stage == 'panel-normal movement':
                reached_ns = time.monotonic_ns()
                self.press_reached_pub.publish(String(data=json.dumps({
                    'target': name, 'monotonic_ns': reached_ns,
                    'wall_time_ns': time.time_ns(),
                    'stage': 'panel-normal movement', 'planned': True,
                    'reference': 'measured_tcp_press_target_verified',
                })))
            elapsed = time.monotonic()-segment_started
            self.get_logger().info(f'【段执行完成】{name} {stage}: {elapsed:.3f}s')
            self.segment_timing_pub.publish(String(data=json.dumps({
                'target': name, 'stage': stage, 'execution_and_verification_s': elapsed,
                'started_monotonic_ns':int(segment_started*1e9),
                'completed_monotonic_ns':int(verification_completed*1e9),
                'trajectory_execution_s': execution_completed-segment_started,
                'tcp_verification_s': verification_completed-execution_completed,
                'planned_duration_s': jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec*1e-9,
            })))
            if stage in ('coarse approach', 'button transition'):
                actual_position, actual_quaternion = self._latest_tcp_arrays()
                button_position, _ = self.sequence_snapshot[name]
                desired_standoff = (self.coarse_standoff if stage == 'coarse approach'
                                    else self.coarse_standoff - self.distance_m
                                    + self.sequence_retract_distance_mm / 1000.0)
                _, axial_error, _, lateral = coarse_standoff_errors(
                    button_position, actual_position, quaternion, desired_standoff)
                _, _, _, angular = pose_error(target, quaternion, actual_position, actual_quaternion)
                if not coarse_pose_is_acceptable(
                    axial_error, lateral, angular, self.coarse_axial_tolerance,
                    self.coarse_lateral_error_min, self.coarse_lateral_error_max,
                    1.5 * self.moveit_orientation_tolerance,
                ):
                    raise TaskFailure('preplanned approach failed measured panel alignment')
            if stage == 'panel-normal movement':
                self.press_event_pub.publish(Float64(data=time.monotonic()))
        self.preplanned_index += 1
        if name == 'key_ok':
            self.sequence_snapshot = {}
            self.preplanned_buttons = []
        self._set_state('DONE')
        goal_handle.succeed()
        return self._result(True, f'preplanned press and {self.sequence_retract_distance_mm:g}mm retreat completed')

    def _execute_press(self, goal_handle):
        """Acquire one button target and complete MoveIt coarse positioning."""
        with self.active_lock:
            if self.task_active:
                goal_handle.abort()
                return self._result(False, 'another press task is active')
            self.task_active = True

        try:
            if self.uncertain_motion:
                raise TaskFailure('previous motion termination uncertain; restart only after arm is stopped')
            self.last_moveit_arm_target = None
            target_name = goal_handle.request.target_name.strip()
            self._set_state('WAIT_TARGET')
            if goal_handle.request.close_panel_sequence:
                if not self.close_panel_sequence:
                    raise TaskFailure('close sequence mode is not enabled')
                if self.x_advance_axis_mode != 'panel_normal' or self.distance_m < 0:
                    raise TaskFailure('close sequence requires nonnegative panel-normal advance')
                button_position, button_quaternion = self._sequence_target(
                    goal_handle.request, goal_handle,
                )
                if self.preplan_sequence:
                    if goal_handle.request.snapshot_targets:
                        self._preplan_snapshot(list(goal_handle.request.snapshot_targets), goal_handle)
                    return self._execute_preplanned_button(target_name, goal_handle)
            else:
                if goal_handle.request.sequence_continuation or goal_handle.request.snapshot_targets:
                    raise TaskFailure('snapshot flags require close sequence mode')
                self.sequence_snapshot = {}
                self._clear_target_tracking()
                self._select_interest(target_name, goal_handle)
                button_position, button_quaternion = self._wait_for_stable_target(goal_handle)
            self._stage_success(
                'WAIT_TARGET',
                f"已取得目标 '{target_name}' 的稳定按钮位姿",
            )
            roll_reference_quaternion = None
            if self.orientation_mode == 'preserve_current_roll':
                if goal_handle.request.close_panel_sequence:
                    roll_reference_quaternion = self.sequence_roll_reference
                else:
                    _, roll_reference_quaternion = self._latest_tcp_arrays()
            control_quaternion = self._control_quaternion(
                button_quaternion,
                roll_reference_quaternion,
            )
            uncompensated_coarse_position = offset_along_press_axis(
                button_position,
                control_quaternion,
                -self.coarse_standoff,
            )
            coarse_position = offset_along_panel_horizontal(
                uncompensated_coarse_position,
                button_quaternion,
                self.coarse_horizontal_offset,
            )
            coarse_position = offset_along_panel_vertical(
                coarse_position,
                button_quaternion,
                self.coarse_vertical_offset,
            )
            coarse_quaternion = control_quaternion
            self.get_logger().info(
                '【粗定位水平补偿】'
                f'{self.coarse_horizontal_offset * 1000.0:+.1f} mm '
                '（正值向左，负值向右）'
            )
            self.get_logger().info(
                '【粗定位竖直补偿】'
                f'{self.coarse_vertical_offset * 1000.0:+.1f} mm '
                '（正值向上，负值向下）'
            )
            coarse_pose = self._pose_message(
                coarse_position,
                coarse_quaternion,
            )
            self.desired_tcp_pub.publish(coarse_pose)
            self._apply_collision_scene(
                button_position,
                button_quaternion,
            )
            self.get_logger().info(
                '【阶段成功】MoveIt 碰撞场景：电梯面板和相机模型已更新'
            )

            self._set_state('COARSE_APPROACH')
            total_coarse_attempts = coarse_total_attempts(
                self.enable_motion,
                self.coarse_correction_attempts,
            )
            verified_pose = None
            successful_attempt = 0
            for attempt_index in range(total_coarse_attempts):
                attempt_number = attempt_index + 1
                if attempt_number > 1:
                    self.get_logger().warning(
                        '【粗定位校正】上一次实测超差，继续使用同一 '
                        f'C0 执行第 {attempt_number}/{total_coarse_attempts} '
                        '次 MoveIt'
                    )
                move_stage = ('button transition' if goal_handle.request.close_panel_sequence
                              and goal_handle.request.sequence_continuation else 'coarse approach')
                self._run_moveit(coarse_pose, goal_handle, stage=move_stage)
                if not self.enable_motion:
                    self._stage_success(
                        'COARSE_APPROACH',
                        'MoveIt 规划成功；当前为 dry-run，'
                        '未发送运动命令',
                    )
                    self._set_state('DONE')
                    if self.distance_m != 0.0:
                        self.get_logger().warning(
                            '已配置 distance_mm='
                            f'{self.distance_m * 1000.0:+.3f}，但当前为 '
                            'dry-run，跳过按压轴实机移动'
                        )
                    goal_handle.succeed()
                    return self._result(
                        True,
                        'MoveIt 粗定位规划成功；'
                        'dry-run 未发送运动命令',
                    )
                verified_pose = self._verify_coarse_pose(
                    goal_handle,
                    button_position,
                    coarse_position,
                    coarse_quaternion,
                    attempt_number,
                    total_coarse_attempts,
                )
                if verified_pose is not None:
                    successful_attempt = attempt_number
                    break

            if verified_pose is None:
                raise TaskFailure(
                    'measured TCP failed button-frame coarse alignment '
                    'after MoveIt correction'
                )

            measured_position, _ = verified_pose
            (
                axial_distance,
                axial_error,
                lateral_vector,
                lateral_error,
            ) = coarse_standoff_errors(
                button_position,
                measured_position,
                coarse_quaternion,
                self.coarse_standoff,
            )
            self._stage_success(
                'COARSE_APPROACH',
                'MoveIt 返回成功，实测 TCP 已进入粗定位验收范围'
                f'（第 {successful_attempt}/{total_coarse_attempts} 次）',
            )
            self.get_logger().info(
                '【定位诊断】首次按钮位置 B0='
                + self._format_xyz(button_position)
            )
            self.get_logger().info(
                '【定位诊断】未补偿粗定位目标 C0_raw='
                + self._format_xyz(uncompensated_coarse_position)
            )
            self.get_logger().info(
                '【定位诊断】补偿后 MoveIt 粗定位目标 C0='
                + self._format_xyz(coarse_position)
            )
            self._log_position_delta(
                '粗定位目标补偿 C0-C0_raw',
                coarse_position,
                uncompensated_coarse_position,
            )
            self.get_logger().info(
                '【定位诊断】真机到达位置 T0='
                + self._format_xyz(measured_position)
            )
            self._log_position_delta(
                'MoveIt执行误差 T0-C0',
                measured_position,
                coarse_position,
            )
            horizontal_axis = quaternion_to_matrix(button_quaternion)[:, 0]
            measured_horizontal_offset = float(np.dot(
                measured_position - uncompensated_coarse_position,
                horizontal_axis,
            ))
            self.get_logger().info(
                '【定位诊断】实测相对未补偿目标水平位移='
                f'{measured_horizontal_offset * 1000.0:+.2f} mm '
                '（正值向左，负值向右）'
            )
            self.get_logger().info(
                '【定位诊断】B0-T0法向距离='
                f'{axial_distance:.6f} m, '
                f'法向误差={axial_error:+.6f} m'
            )
            self.get_logger().info(
                '【定位诊断】B0-T0横向误差向量='
                + self._format_xyz(lateral_vector)
                + f', 模长={lateral_error:.6f} m, '
                + '允许范围=['
                + f'{self.coarse_lateral_error_min:.4f}, '
                + f'{self.coarse_lateral_error_max:.4f}] m'
            )
            if self.distance_m != 0.0:
                measured_position, measured_quaternion = verified_pose
                self._run_x_advance(
                    goal_handle,
                    measured_position,
                    measured_quaternion,
                    coarse_quaternion,
                )
                self.press_event_pub.publish(Float64(data=time.monotonic()))
                if goal_handle.request.close_panel_sequence:
                    if self.x_advance_axis_mode != 'panel_normal':
                        raise TaskFailure('close sequence requires panel_normal')
                    self._set_state('RETRACT')
                    # Return to the measured, verified stand-off pose before
                    # any next target is acquired or lateral motion begins.
                    reached_position, reached_quaternion = self._latest_tcp_arrays()
                    retract_position = offset_along_press_axis(
                        reached_position, coarse_quaternion,
                        -self.sequence_retract_distance_mm / 1000.0,
                    )
                    retract_pose = self._pose_message(retract_position, reached_quaternion)
                    self.desired_tcp_pub.publish(retract_pose)
                    self._run_moveit(retract_pose, goal_handle, stage='panel retract')
                    self._verify_target_pose(
                        goal_handle, retract_position, reached_quaternion,
                        'panel retract',
                    )
                    self.get_logger().info('【连续按键】已从推进终点退回设定距离')
            if goal_handle.request.close_panel_sequence and target_name == 'key_ok':
                self.sequence_snapshot = {}
            self._set_state('DONE')
            goal_handle.succeed()
            if self.distance_m != 0.0:
                return self._result(
                    True,
                    'MoveIt 初定位及按压轴移动完成'
                    f'（{self.x_advance_axis_mode}）',
                )
            return self._result(
                True,
                'MoveIt 初定位完成；机械臂保持在实测 T0，未执行 PBVS',
            )

        except TaskCanceled as error:
            self.sequence_snapshot = {}
            failed_state = self.current_state
            self._stage_failure(failed_state, f'任务被取消：{error}')
            self._set_state('ABORT')
            goal_handle.canceled()
            return self._result(False, str(error))
        except Exception as error:
            self.sequence_snapshot = {}
            failed_state = self.current_state
            self._stage_failure(failed_state, error)
            self._set_state('ABORT')
            goal_handle.abort()
            return self._result(False, str(error))
        finally:
            self.active_move_goal = None
            with self.active_lock:
                self.task_active = False
            if self.current_state in ('DONE', 'ABORT'):
                self._set_state('IDLE')

    def destroy_node(self):
        """Destroy the Action server before the ROS node."""
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    """Run the coarse controller with concurrent action and callbacks."""
    rclpy.init(args=args)
    node = PiperPbvsController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
