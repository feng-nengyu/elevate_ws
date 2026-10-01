"""Real FollowJointTrajectory bridge for the Piper ROS driver."""

from functools import partial
import math
import threading
import time

import can
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from piper.trajectory_execution import (
    ARM_JOINTS,
    COMMAND_JOINTS,
    GRIPPER_JOINTS,
    PiperCanFeedbackState,
    READY_POSITIONS,
    TrajectoryValidationError,
    duration_seconds,
    normalize_trajectory,
    position_tolerances,
    prepare_smooth_trajectory,
    sample_smooth_trajectory,
    startup_state_error,
    updated_settle_count,
    violating_joint,
)


class PiperTrajectoryController(Node):
    """Execute MoveIt trajectories through the real Piper command topic."""

    def __init__(self):
        super().__init__('piper_trajectory_controller')
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('command_rate', 50.0)
        self.declare_parameter('speed_percent', 10)
        self.declare_parameter('gripper_effort', 1.0)
        self.declare_parameter('joint_state_timeout', 0.25)
        self.declare_parameter('can_feedback_timeout', 0.5)
        self.declare_parameter('arm_path_tolerance', 0.5)
        self.declare_parameter('arm_goal_tolerance', 0.01)
        self.declare_parameter('arm_goal_settle_cycles', 1)
        self.declare_parameter('arm_start_tolerance', 0.2)
        self.declare_parameter('gripper_path_tolerance', 0.015)
        self.declare_parameter('gripper_goal_tolerance', 0.003)
        self.declare_parameter('gripper_start_tolerance', 0.01)
        self.declare_parameter('default_goal_time_tolerance', 2.0)
        self.declare_parameter('require_initialization', True)
        self.declare_parameter('boundary_recovery_tolerance', 0.08)
        self.declare_parameter('initialization_timeout', 30.0)
        self.declare_parameter('initialization_reset_gap', 1.0)
        self.declare_parameter('initialization_goal_tolerance', 0.05)
        self.declare_parameter('initialization_duration', 12.0)
        self.declare_parameter('initialization_speed_percent', 12)
        self.declare_parameter('initialization_max_step', 0.002)
        self.declare_parameter('initialization_path_tolerance', 0.25)

        self.can_port = str(self.get_parameter('can_port').value)
        self.command_rate = float(self.get_parameter('command_rate').value)
        self.speed_percent = int(self.get_parameter('speed_percent').value)
        self.arm_goal_settle_cycles = int(
            self.get_parameter('arm_goal_settle_cycles').value
        )
        self.gripper_effort = float(
            self.get_parameter('gripper_effort').value
        )
        self.joint_state_timeout = float(
            self.get_parameter('joint_state_timeout').value
        )
        self.can_feedback_timeout = float(
            self.get_parameter('can_feedback_timeout').value
        )
        self.default_goal_time_tolerance = float(
            self.get_parameter('default_goal_time_tolerance').value
        )
        self.require_initialization = bool(
            self.get_parameter('require_initialization').value
        )
        self.boundary_recovery_tolerance = float(
            self.get_parameter('boundary_recovery_tolerance').value
        )
        self.initialization_timeout = float(
            self.get_parameter('initialization_timeout').value
        )
        self.initialization_reset_gap = float(
            self.get_parameter('initialization_reset_gap').value
        )
        self.initialization_goal_tolerance = float(
            self.get_parameter('initialization_goal_tolerance').value
        )
        self.initialization_duration = float(
            self.get_parameter('initialization_duration').value
        )
        self.initialization_speed_percent = int(
            self.get_parameter('initialization_speed_percent').value
        )
        self.initialization_max_step = float(
            self.get_parameter('initialization_max_step').value
        )
        self.initialization_path_tolerance = float(
            self.get_parameter('initialization_path_tolerance').value
        )
        self.speed_percent = min(max(self.speed_percent, 1), 100)
        if self.command_rate <= 0.0:
            raise ValueError('command_rate must be positive')
        if self.arm_goal_settle_cycles < 1:
            raise ValueError('arm_goal_settle_cycles must be positive')
        for parameter_name, value in (
            (
                'boundary_recovery_tolerance',
                self.boundary_recovery_tolerance,
            ),
            ('initialization_timeout', self.initialization_timeout),
            ('initialization_reset_gap', self.initialization_reset_gap),
            (
                'initialization_goal_tolerance',
                self.initialization_goal_tolerance,
            ),
            ('initialization_duration', self.initialization_duration),
            ('initialization_max_step', self.initialization_max_step),
            (
                'initialization_path_tolerance',
                self.initialization_path_tolerance,
            ),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{parameter_name} must be positive')
        self.initialization_speed_percent = min(
            max(self.initialization_speed_percent, 1),
            20,
        )

        self.group_parameters = {
            'arm': {
                'joints': ARM_JOINTS,
                'path_tolerance': float(
                    self.get_parameter('arm_path_tolerance').value
                ),
                'goal_tolerance': float(
                    self.get_parameter('arm_goal_tolerance').value
                ),
                'start_tolerance': float(
                    self.get_parameter('arm_start_tolerance').value
                ),
                'settle_cycles': self.arm_goal_settle_cycles,
            },
            'gripper': {
                'joints': GRIPPER_JOINTS,
                'path_tolerance': float(
                    self.get_parameter('gripper_path_tolerance').value
                ),
                'goal_tolerance': float(
                    self.get_parameter('gripper_goal_tolerance').value
                ),
                'start_tolerance': float(
                    self.get_parameter('gripper_start_tolerance').value
                ),
                'settle_cycles': 1,
            },
        }

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.execution_lock = threading.Lock()
        self.can_lock = threading.Lock()
        self.current_positions = None
        self.current_velocities = None
        self.raw_positions = None
        self.raw_joint_state_received_at = -math.inf
        self.command_positions = None
        self.joint_state_received_at = -math.inf
        self.driver_command_enabled = False
        self.driver_command_enabled_received_at = -math.inf
        self.active = {'arm': False, 'gripper': False}
        self.initialized = not self.require_initialization
        self.initialization_in_progress = False

        self.command_pub = self.create_publisher(
            JointState,
            '/joint_ctrl_single',
            1,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            '/joint_states_raw',
            self._raw_joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            '/piper_command_enabled',
            self._driver_command_enabled_callback,
            10,
            callback_group=self.callback_group,
        )

        self.state_validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
            callback_group=self.callback_group,
        )
        self.initialize_service = self.create_service(
            Trigger,
            '/initialize_arm',
            self._initialize_arm,
            callback_group=self.callback_group,
        )

        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=partial(self._execute, 'arm'),
            goal_callback=partial(self._goal_callback, 'arm'),
            cancel_callback=partial(self._cancel_callback, 'arm'),
            callback_group=self.callback_group,
        )
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            execute_callback=partial(self._execute, 'gripper'),
            goal_callback=partial(self._goal_callback, 'gripper'),
            cancel_callback=partial(self._cancel_callback, 'gripper'),
            callback_group=self.callback_group,
        )

        self.can_feedback = PiperCanFeedbackState()
        self.can_bus = None
        self.can_stop_event = threading.Event()
        self.can_thread = threading.Thread(
            target=self._receive_can_feedback,
            daemon=True,
        )
        self.can_thread.start()

        self.get_logger().info(
            'Real Piper trajectory controller ready; commands are guarded '
            'by initialization, single-publisher, driver/CAN freshness, '
            'enable, limit, and tracking checks'
        )

    def _joint_state_callback(self, message):
        """Cache one complete real Piper joint-state sample."""
        if len(message.name) != len(message.position):
            return
        positions_by_name = dict(zip(message.name, message.position))
        if not all(name in positions_by_name for name in COMMAND_JOINTS):
            return
        positions = tuple(
            float(positions_by_name[name])
            for name in COMMAND_JOINTS
        )
        if not all(math.isfinite(value) for value in positions):
            return

        velocities_by_name = dict(zip(message.name, message.velocity))
        velocities = tuple(
            float(velocities_by_name.get(name, 0.0))
            for name in COMMAND_JOINTS
        )
        received_at = time.monotonic()
        reset_initialization = False
        with self.state_lock:
            if (
                self.initialized
                and math.isfinite(self.joint_state_received_at)
                and received_at - self.joint_state_received_at
                > self.initialization_reset_gap
            ):
                reset_initialization = True
            self.current_positions = positions
            self.current_velocities = velocities
            self.joint_state_received_at = received_at
            if self.command_positions is None:
                self.command_positions = list(positions)
        if reset_initialization and self.require_initialization:
            with self.execution_lock:
                self.initialized = False
            self.get_logger().warning(
                '/joint_states feedback resumed after a gap; arm '
                'initialization is required again'
            )

    def _raw_joint_state_callback(self, message):
        """Cache untouched driver feedback for startup safety checks."""
        if len(message.name) != len(message.position):
            return
        positions_by_name = dict(zip(message.name, message.position))
        if not all(name in positions_by_name for name in COMMAND_JOINTS):
            return
        positions = tuple(
            float(positions_by_name[name])
            for name in COMMAND_JOINTS
        )
        if not all(math.isfinite(value) for value in positions):
            return
        with self.state_lock:
            self.raw_positions = positions
            self.raw_joint_state_received_at = time.monotonic()

    def _driver_command_enabled_callback(self, message):
        """Track whether this driver instance will forward commands."""
        with self.state_lock:
            self.driver_command_enabled = bool(message.data)
            self.driver_command_enabled_received_at = time.monotonic()

    def _receive_can_feedback(self):
        """Observe real feedback on a second, read-only SocketCAN socket."""
        filters = [
            {
                'can_id': can_id,
                'can_mask': 0x7FF,
                'extended': False,
            }
            for can_id in (
                *PiperCanFeedbackState.ARM_POSITION_CAN_IDS,
                PiperCanFeedbackState.GRIPPER_POSITION_CAN_ID,
                *PiperCanFeedbackState.LOW_SPEED_CAN_IDS,
            )
        ]
        try:
            self.can_bus = can.Bus(
                channel=self.can_port,
                interface='socketcan',
                can_filters=filters,
            )
        except Exception as error:
            self.get_logger().error(
                f'Cannot open read-only CAN feedback monitor: {error}'
            )
            return

        while not self.can_stop_event.is_set():
            try:
                message = self.can_bus.recv(timeout=0.1)
            except Exception as error:
                self.get_logger().error(
                    f'CAN feedback monitor failed: {error}'
                )
                return
            if message is None:
                continue
            with self.can_lock:
                self.can_feedback.observe(
                    message.arbitration_id,
                    message.data,
                    time.monotonic(),
                )

    def _snapshot(self):
        """Return a thread-safe copy of the latest real joint state."""
        with self.state_lock:
            return (
                self.current_positions,
                self.current_velocities,
                self.joint_state_received_at,
            )

    def _raw_snapshot(self):
        """Return the latest untouched driver state."""
        with self.state_lock:
            return (
                self.raw_positions,
                self.raw_joint_state_received_at,
            )

    def _hardware_ready(self, group):
        """Require one real state publisher and fresh enabled CAN feedback."""
        if self.count_publishers('/joint_states') != 1:
            return False, (
                '/joint_states must have exactly one publisher '
                '(piper_ctrl_single_node)'
            )
        positions, _, received_at = self._snapshot()
        if positions is None:
            return False, 'no complete /joint_states feedback'
        if time.monotonic() - received_at > self.joint_state_timeout:
            return False, '/joint_states feedback is stale'

        now = time.monotonic()
        with self.state_lock:
            driver_enabled = self.driver_command_enabled
            driver_enabled_at = self.driver_command_enabled_received_at
        if (
            not driver_enabled
            or now - driver_enabled_at > self.can_feedback_timeout
        ):
            return False, (
                'Piper driver command gate is disabled or stale; call '
                '/enable_srv after every driver restart'
            )
        with self.can_lock:
            if group == 'arm':
                can_ready = self.can_feedback.arm_ready(
                    now,
                    self.can_feedback_timeout,
                )
            else:
                can_ready = self.can_feedback.gripper_ready(
                    now,
                    self.can_feedback_timeout,
                )
        if not can_ready:
            return False, (
                'Piper CAN position/enable feedback is stale or '
                'one of the six joints is not enabled'
            )
        return True, ''

    def _goal_callback(self, group, goal_request):
        """Reject malformed, concurrent, simulated, or disabled goals."""
        if (
            goal_request.multi_dof_trajectory.joint_names
            or goal_request.multi_dof_trajectory.points
        ):
            self.get_logger().error(
                f'Rejecting {group} trajectory: multi-DOF goals unsupported'
            )
            return GoalResponse.REJECT
        try:
            normalize_trajectory(
                goal_request.trajectory,
                self.group_parameters[group]['joints'],
            )
        except TrajectoryValidationError as error:
            self.get_logger().error(f'Rejecting {group} trajectory: {error}')
            return GoalResponse.REJECT

        if group == 'gripper' and any(
            abs(point.positions[0] - READY_POSITIONS[-1]) > 1e-8
            for point in goal_request.trajectory.points
        ):
            self.get_logger().error(
                'Rejecting gripper trajectory: opening fixed at 0.0020'
            )
            return GoalResponse.REJECT

        ready, reason = self._hardware_ready(group)
        if not ready:
            self.get_logger().error(f'Rejecting {group} trajectory: {reason}')
            return GoalResponse.REJECT

        with self.execution_lock:
            if self.require_initialization and not self.initialized:
                self.get_logger().error(
                    f'Rejecting {group} trajectory: call '
                    '/initialize_arm successfully first'
                )
                return GoalResponse.REJECT
            if self.active[group]:
                self.get_logger().warning(
                    f'Rejecting concurrent {group} trajectory'
                )
                return GoalResponse.REJECT
            self.active[group] = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _wait_future(future, timeout):
        """Wait for an asynchronous ROS result without blocking callbacks."""
        deadline = time.monotonic() + float(timeout)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _check_state_validity(self, positions, timeout=5.0):
        """Ask MoveIt whether one complete real state is valid."""
        if not self.state_validity_client.wait_for_service(
            timeout_sec=timeout
        ):
            return False, '/check_state_validity is unavailable'

        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = list(COMMAND_JOINTS)
        request.robot_state.joint_state.position = list(positions)
        future = self.state_validity_client.call_async(request)
        if not self._wait_future(future, timeout):
            return False, '/check_state_validity timed out'
        result = future.result()
        if result is None:
            return False, '/check_state_validity returned no response'
        if not result.valid:
            contacts = ', '.join(
                f'{contact.contact_body_1}-{contact.contact_body_2}'
                for contact in result.contacts
            )
            detail = f' ({contacts})' if contacts else ''
            return False, f'MoveIt reports an invalid robot state{detail}'
        return True, ''

    def _validate_initialization_start(self):
        """Validate real feedback before direct slow initialization."""
        if self.count_publishers('/joint_states_raw') != 1:
            return False, (
                '/joint_states_raw must have exactly one publisher '
                '(piper_ctrl_single_node)'
            )
        raw_positions, raw_received_at = self._raw_snapshot()
        if raw_positions is None:
            return False, 'no complete /joint_states_raw feedback'
        if (
            time.monotonic() - raw_received_at
            > self.joint_state_timeout
        ):
            return False, '/joint_states_raw feedback is stale'

        error = startup_state_error(
            raw_positions[:len(ARM_JOINTS)],
            self.boundary_recovery_tolerance,
        )
        if error is not None:
            return False, error
        return True, ''

    def _run_direct_initialization(self, start_positions):
        """Slowly interpolate all seven real joints to the Ready pose."""
        start = tuple(float(value) for value in start_positions)
        maximum_delta = max(
            abs(target - current)
            for current, target in zip(start, READY_POSITIONS)
        )
        step_limited_duration = (
            1.5 * maximum_delta
            / (self.initialization_max_step * self.command_rate)
        )
        duration = max(
            self.initialization_duration,
            step_limited_duration,
        )
        self.get_logger().warning(
            'Starting direct low-speed JointCtrl initialization to Ready: '
            f'{duration:.2f} s at '
            f'{self.initialization_speed_percent}% speed'
        )

        started_at = time.monotonic()
        period = 1.0 / self.command_rate
        while True:
            elapsed = time.monotonic() - started_at
            if elapsed >= duration:
                break

            ready, reason = self._hardware_ready('gripper')
            if not ready:
                return False, reason
            raw_positions, raw_received_at = self._raw_snapshot()
            if (
                raw_positions is None
                or time.monotonic() - raw_received_at
                > self.joint_state_timeout
            ):
                return False, '/joint_states_raw feedback is stale'

            ratio = min(max(elapsed / duration, 0.0), 1.0)
            blend = 3.0 * ratio ** 2 - 2.0 * ratio ** 3
            desired = tuple(
                current + blend * (target - current)
                for current, target in zip(
                    start,
                    READY_POSITIONS,
                )
            )
            arm_tracking_error = max(
                abs(target - measured)
                for target, measured in zip(
                    desired[:len(ARM_JOINTS)],
                    raw_positions[:len(ARM_JOINTS)],
                )
            )
            if arm_tracking_error > self.initialization_path_tolerance:
                return False, (
                    'initialization tracking error exceeded tolerance '
                    f'({arm_tracking_error:.6f} rad)'
                )
            gripper_tracking_error = abs(
                desired[-1] - raw_positions[-1]
            )
            if (
                gripper_tracking_error
                > self.group_parameters['gripper']['path_tolerance']
            ):
                return False, (
                    'initialization gripper tracking error exceeded '
                    f'tolerance ({gripper_tracking_error:.6f} m)'
                )
            self._publish_complete_command(
                desired,
                speed_percent=self.initialization_speed_percent,
            )
            time.sleep(period)

        deadline = time.monotonic() + self.initialization_timeout
        while time.monotonic() <= deadline:
            ready, reason = self._hardware_ready('gripper')
            if not ready:
                return False, reason
            raw_positions, raw_received_at = self._raw_snapshot()
            if (
                raw_positions is None
                or time.monotonic() - raw_received_at
                > self.joint_state_timeout
            ):
                return False, '/joint_states_raw feedback is stale'
            arm_final_error = max(
                abs(measured - target)
                for measured, target in zip(
                    raw_positions[:len(ARM_JOINTS)],
                    READY_POSITIONS[:len(ARM_JOINTS)],
                )
            )
            gripper_final_error = abs(
                raw_positions[-1] - READY_POSITIONS[-1]
            )
            if (
                arm_final_error <= self.initialization_goal_tolerance
                and gripper_final_error
                <= self.group_parameters['gripper']['goal_tolerance']
            ):
                return True, ''
            self._publish_complete_command(
                READY_POSITIONS,
                speed_percent=self.initialization_speed_percent,
            )
            time.sleep(period)
        return False, 'real arm did not reach Ready before timeout'

    def _initialize_arm(self, _request, response):
        """Move the enabled real arm slowly and directly to Ready."""
        with self.execution_lock:
            if self.initialized:
                response.success = True
                response.message = 'arm is already initialized'
                return response
            if self.initialization_in_progress:
                response.success = False
                response.message = 'arm initialization is already running'
                return response
            if any(self.active.values()):
                response.success = False
                response.message = 'a real trajectory is already active'
                return response
            self.initialization_in_progress = True
            self.active['arm'] = True
            self.active['gripper'] = True

        try:
            ready, reason = self._hardware_ready('gripper')
            if not ready:
                response.success = False
                response.message = reason
                return response

            valid, reason = self._validate_initialization_start()
            if not valid:
                response.success = False
                response.message = reason
                return response

            raw_positions, _ = self._raw_snapshot()
            moved, reason = self._run_direct_initialization(raw_positions)
            if not moved:
                response.success = False
                response.message = reason
                return response

            raw_positions, raw_received_at = self._raw_snapshot()
            if (
                raw_positions is None
                or time.monotonic() - raw_received_at
                > self.joint_state_timeout
            ):
                response.success = False
                response.message = (
                    'raw feedback was lost after initialization'
                )
                return response
            arm_final_error = max(
                abs(measured - target)
                for measured, target in zip(
                    raw_positions[:len(ARM_JOINTS)],
                    READY_POSITIONS[:len(ARM_JOINTS)],
                )
            )
            if arm_final_error > self.initialization_goal_tolerance:
                response.success = False
                response.message = (
                    'real arm did not reach Ready tolerance '
                    f'({arm_final_error:.6f} rad)'
                )
                return response
            gripper_final_error = abs(
                raw_positions[-1] - READY_POSITIONS[-1]
            )
            if (
                gripper_final_error
                > self.group_parameters['gripper']['goal_tolerance']
            ):
                response.success = False
                response.message = (
                    'real gripper did not reach Ready tolerance '
                    f'({gripper_final_error:.6f} m)'
                )
                return response

            valid, reason = self._check_state_validity(raw_positions)
            if not valid:
                response.success = False
                response.message = (
                    f'final Ready state is invalid: {reason}'
                )
                return response

            with self.execution_lock:
                self.initialized = True
            response.success = True
            response.message = (
                'real Piper arm and gripper reached Ready by direct '
                'low-speed JointCtrl'
            )
            self.get_logger().info(response.message)
            return response
        except Exception as error:
            self.get_logger().error(
                f'Unexpected arm initialization failure: {error}'
            )
            response.success = False
            response.message = str(error)
            return response
        finally:
            with self.execution_lock:
                self.initialization_in_progress = False
                self.active['arm'] = False
                self.active['gripper'] = False

    def _cancel_callback(self, group, _goal_handle):
        """Accept cancellation for either real controller."""
        self.get_logger().warning(f'Cancel requested for {group} trajectory')
        return CancelResponse.ACCEPT

    @staticmethod
    def _result(error_code, error_string=''):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    def _publish_command(
        self,
        group,
        group_positions,
        speed_percent=None,
    ):
        """Publish a complete command without resetting another group."""
        actual, _, _ = self._snapshot()
        if actual is None:
            return

        group_joints = self.group_parameters[group]['joints']
        with self.execution_lock:
            if self.command_positions is None:
                self.command_positions = list(actual)
            for joint_name, value in zip(group_joints, group_positions):
                command_index = COMMAND_JOINTS.index(joint_name)
                self.command_positions[command_index] = float(value)
            for other_group, settings in self.group_parameters.items():
                if self.active[other_group]:
                    continue
                for joint_name in settings['joints']:
                    index = COMMAND_JOINTS.index(joint_name)
                    self.command_positions[index] = actual[index]
            self.command_positions[-1] = READY_POSITIONS[-1]
            positions = list(self.command_positions)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(COMMAND_JOINTS)
        command.position = positions
        selected_speed = (
            self.speed_percent
            if speed_percent is None else int(speed_percent)
        )
        selected_speed = min(max(selected_speed, 1), 100)
        # The existing Piper driver uses velocity[6] as global speed percent.
        command.velocity = [0.0] * 6 + [float(selected_speed)]
        command.effort = [0.0] * 6 + [self.gripper_effort]
        self.command_pub.publish(command)

    def _publish_complete_command(self, positions, speed_percent):
        """Publish one complete seven-joint initialization command."""
        complete_positions = tuple(float(value) for value in positions)
        if len(complete_positions) != len(COMMAND_JOINTS):
            raise ValueError('complete command must contain seven joints')
        with self.execution_lock:
            self.command_positions = list(complete_positions)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(COMMAND_JOINTS)
        command.position = list(complete_positions)
        selected_speed = min(max(int(speed_percent), 1), 100)
        command.velocity = [0.0] * 6 + [float(selected_speed)]
        command.effort = [0.0] * 6 + [self.gripper_effort]
        self.command_pub.publish(command)

    def _publish_hold(self, group):
        """Hold one group without interrupting another active controller."""
        actual, _, _ = self._snapshot()
        if actual is None:
            return
        with self.execution_lock:
            if self.command_positions is None:
                self.command_positions = list(actual)
            for joint_name in self.group_parameters[group]['joints']:
                index = COMMAND_JOINTS.index(joint_name)
                self.command_positions[index] = actual[index]
            self.command_positions[-1] = READY_POSITIONS[-1]
            positions = list(self.command_positions)
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(COMMAND_JOINTS)
        command.position = positions
        command.velocity = [0.0] * 6 + [float(self.speed_percent)]
        command.effort = [0.0] * 6 + [self.gripper_effort]
        self.command_pub.publish(command)

    def _publish_feedback(
        self,
        goal_handle,
        joint_names,
        desired_positions,
        desired_velocities,
        elapsed,
    ):
        """Publish desired, measured, and error trajectory feedback."""
        actual, actual_velocities, _ = self._snapshot()
        indices = tuple(COMMAND_JOINTS.index(name) for name in joint_names)
        measured = tuple(actual[index] for index in indices)
        measured_velocities = tuple(
            actual_velocities[index]
            for index in indices
        )
        errors = tuple(
            desired - current
            for desired, current in zip(desired_positions, measured)
        )

        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joint_names)
        feedback.desired.positions = list(desired_positions)
        feedback.desired.velocities = list(desired_velocities)
        feedback.actual.positions = list(measured)
        feedback.actual.velocities = list(measured_velocities)
        feedback.error.positions = list(errors)
        seconds = max(0.0, float(elapsed))
        feedback.desired.time_from_start.sec = int(seconds)
        feedback.desired.time_from_start.nanosec = int(
            (seconds - int(seconds)) * 1e9
        )
        feedback.actual.time_from_start = (
            feedback.desired.time_from_start
        )
        feedback.error.time_from_start = feedback.desired.time_from_start
        goal_handle.publish_feedback(feedback)
        return measured, errors

    def _wait_for_header_stamp(self, trajectory, goal_handle):
        """Honor a future trajectory header and reject an old one."""
        stamp = trajectory.header.stamp
        stamp_nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_nanoseconds == 0:
            return True, None
        delay = (
            stamp_nanoseconds - self.get_clock().now().nanoseconds
        ) * 1e-9
        if delay < -0.1:
            return False, self._result(
                FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP,
                'trajectory header timestamp is in the past',
            )
        deadline = time.monotonic() + max(delay, 0.0)
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, None
            time.sleep(min(0.01, deadline - time.monotonic()))
        return True, None

    def _execute(self, group, goal_handle):
        """Execute one arm or gripper trajectory on the real hardware."""
        settings = self.group_parameters[group]
        joint_names = settings['joints']
        try:
            trajectory = normalize_trajectory(
                goal_handle.request.trajectory,
                joint_names,
            )
            path_tolerances = position_tolerances(
                goal_handle.request.path_tolerance,
                joint_names,
                settings['path_tolerance'],
            )
            goal_tolerances = position_tolerances(
                goal_handle.request.goal_tolerance,
                joint_names,
                settings['goal_tolerance'],
            )
        except TrajectoryValidationError as error:
            goal_handle.abort()
            with self.execution_lock:
                self.active[group] = False
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                str(error),
            )

        try:
            ready, reason = self._hardware_ready(group)
            if not ready:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    reason,
                )

            actual, _, _ = self._snapshot()
            indices = tuple(COMMAND_JOINTS.index(name) for name in joint_names)
            start_positions = tuple(actual[index] for index in indices)
            if trajectory.times[0] == 0.0:
                start_error = max(
                    abs(target - current)
                    for target, current in zip(
                        trajectory.positions[0],
                        start_positions,
                    )
                )
                if start_error > settings['start_tolerance']:
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.INVALID_GOAL,
                        'first trajectory point is too far from the '
                        f'measured state ({start_error:.6f})',
                    )

            should_start, timestamp_result = self._wait_for_header_stamp(
                goal_handle.request.trajectory,
                goal_handle,
            )
            if not should_start:
                self._publish_hold(group)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'trajectory canceled before its start time',
                    )
                goal_handle.abort()
                return timestamp_result

            period = 1.0 / self.command_rate
            started_at = time.monotonic()
            trajectory = prepare_smooth_trajectory(trajectory, start_positions)
            final_time = trajectory.times[-1]
            next_tick = started_at

            while True:
                elapsed = time.monotonic() - started_at
                if elapsed >= final_time:
                    break
                if goal_handle.is_cancel_requested:
                    self._publish_hold(group)
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'trajectory canceled',
                    )

                ready, reason = self._hardware_ready(group)
                if not ready:
                    self._publish_hold(group)
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                        reason,
                    )

                desired, desired_velocities = sample_smooth_trajectory(
                    trajectory,
                    start_positions,
                    elapsed,
                )
                self._publish_command(group, desired)
                _, errors = self._publish_feedback(
                    goal_handle,
                    joint_names,
                    desired,
                    desired_velocities,
                    elapsed,
                )
                violation = violating_joint(
                    joint_names,
                    errors,
                    path_tolerances,
                )
                if violation is not None:
                    self._publish_hold(group)
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                        f'{violation} exceeded path tolerance',
                    )
                next_tick += period
                now = time.monotonic()
                if next_tick <= now:
                    next_tick = started_at + (math.floor((now-started_at)/period)+1)*period
                time.sleep(max(0.0,min(next_tick,started_at+final_time)-now))

            final_positions = trajectory.positions[-1]
            requested_goal_time = duration_seconds(
                goal_handle.request.goal_time_tolerance
            )
            goal_time = (
                requested_goal_time
                if requested_goal_time > 0.0
                else self.default_goal_time_tolerance
            )
            deadline = time.monotonic() + goal_time
            settled_cycles = 0
            while time.monotonic() <= deadline:
                if goal_handle.is_cancel_requested:
                    self._publish_hold(group)
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'trajectory canceled while settling',
                    )
                ready, reason = self._hardware_ready(group)
                if not ready:
                    self._publish_hold(group)
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                        reason,
                    )

                self._publish_command(group, final_positions)
                _, errors = self._publish_feedback(
                    goal_handle,
                    joint_names,
                    final_positions,
                    (0.0,) * len(joint_names),
                    final_time,
                )
                settled_cycles = updated_settle_count(
                    joint_names,
                    errors,
                    goal_tolerances,
                    settled_cycles,
                )
                if settled_cycles >= settings['settle_cycles']:
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'real Piper reached and settled at the requested '
                        'trajectory',
                    )
                time.sleep(period)

            self._publish_hold(group)
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                'real Piper did not reach the final tolerance in time',
            )
        except Exception as error:
            self._publish_hold(group)
            self.get_logger().error(
                f'Unexpected {group} trajectory failure: {error}'
            )
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                str(error),
            )
        finally:
            with self.execution_lock:
                self.active[group] = False

    def destroy_node(self):
        """Stop the read-only CAN monitor before destroying the ROS node."""
        self.can_stop_event.set()
        if self.can_thread.is_alive():
            self.can_thread.join(timeout=1.0)
        if self.can_bus is not None:
            self.can_bus.shutdown()
        self.arm_action_server.destroy()
        self.gripper_action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperTrajectoryController()
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
