"""Orchestrate an elevator floor press, homing, and OK confirmation."""

import copy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from piper_msgs.action import PressButton
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64


ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))
HOME_JOINT_ACCEPTANCE_SLACK = 0.001
DEFAULT_HOME_JOINT_POSITIONS = (
    -1.557051440,
    0.234953236,
    -0.324353736,
    0.013745872,
    0.203885472,
    -0.018106872,
)


class SequenceFailure(RuntimeError):
    """Raised when an elevator sequence cannot continue safely."""


class SequenceCanceled(RuntimeError):
    """Raised when the caller cancels an elevator sequence."""


def normalize_floor_targets(requested_target, default_floor):
    """Return one or two ``key_N`` targets for a numeric floor input."""
    target = str(requested_target).strip()
    if not target:
        target = str(default_floor).strip()
    if target.startswith('key_'):
        target = target[4:]
    if (
        not 1 <= len(target) <= 2
        or not target.isascii()
        or not target.isdigit()
    ):
        raise ValueError(
            'floor target must contain one or two digits from 0 to 9'
        )
    return tuple(f'key_{digit}' for digit in target)


def normalize_floor_target(requested_target, default_floor):
    """Return one ``key_N`` target for legacy single-digit callers."""
    targets = normalize_floor_targets(requested_target, default_floor)
    if len(targets) != 1:
        raise ValueError('single floor target must contain exactly one digit')
    return targets[0]


def validate_home_joint_positions(values):
    """Return six finite home joint positions as a tuple of floats."""
    positions = tuple(float(value) for value in values)
    if len(positions) != len(ARM_JOINT_NAMES):
        raise ValueError('home_joint_positions must contain six values')
    if not all(math.isfinite(value) for value in positions):
        raise ValueError('home_joint_positions must be finite')
    return positions


def home_joint_errors(message, target_positions):
    """Map a JointState by name and return absolute home errors."""
    if len(message.name) != len(message.position):
        return None
    positions_by_name = dict(zip(message.name, message.position))
    if not all(name in positions_by_name for name in ARM_JOINT_NAMES):
        return None
    measured = tuple(
        float(positions_by_name[name]) for name in ARM_JOINT_NAMES
    )
    if not all(math.isfinite(value) for value in measured):
        return None
    return tuple(
        abs(actual - target)
        for actual, target in zip(measured, target_positions)
    )


def home_joint_error_accepted(max_error, tolerance):
    """Accept feedback within tolerance plus a strict 0.001 rad slack."""
    return max_error < tolerance + HOME_JOINT_ACCEPTANCE_SLACK


def make_home_moveit_goal(
    target_positions,
    tolerance,
    group_name,
    plan_only,
    velocity_scaling_factor,
    acceleration_scaling_factor,
):
    """Construct a MoveGroup joint-space goal for the configured home pose."""
    constraints = Constraints()
    for joint_name, position in zip(ARM_JOINT_NAMES, target_positions):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = float(position)
        constraint.tolerance_above = float(tolerance)
        constraint.tolerance_below = float(tolerance)
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)

    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = group_name
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = float(
        velocity_scaling_factor
    )
    goal.request.max_acceleration_scaling_factor = float(
        acceleration_scaling_factor
    )
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = bool(plan_only)
    goal.planning_options.look_around = False
    goal.planning_options.replan = not plan_only
    goal.planning_options.replan_delay = 1.0
    return goal


class ElevatorSequence(Node):
    """Run number press, home, OK press, and home as one Action."""

    def __init__(self):
        """Create the sequence server and clients without starting motion."""
        super().__init__('elevator_sequence')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._read_parameters()

        self.active_lock = threading.Lock()
        self.task_active = False
        self.data_lock = threading.Lock()
        self.latest_joint_state = None
        self.latest_joint_received = 0.0
        self.last_press_motion_state = None
        self.current_state = 'IDLE'
        self.last_press_event = None
        self.create_subscription(Float64, '/pbvs/press_event', self._press_event_callback, 10)
        self.active_press_goal = None
        self.active_move_goal = None

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.pbvs_state_sub = self.create_subscription(
            String,
            '/pbvs/state',
            self._pbvs_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.state_pub = self.create_publisher(
            String,
            '/elevator_sequence/state',
            10,
        )
        self.press_client = ActionClient(
            self,
            PressButton,
            '/press_button',
            callback_group=self.callback_group,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            PressButton,
            '/run_elevator_sequence',
            execute_callback=self._execute_sequence,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._set_state('IDLE')
        self.state_heartbeat = self.create_timer(
            0.5, lambda: self.state_pub.publish(String(data=self.current_state))
        )
        self.get_logger().info(
            'Elevator sequence ready; it will not move until an Action '
            f'goal is received, default_floor={self.floor_number}, '
            f'enable_motion={self.enable_motion}'
        )

    def _declare_parameters(self):
        """Declare sequence, home, and timeout parameters."""
        self.declare_parameter('close_panel_sequence', False)
        self.declare_parameter('floor_number', 1)
        self.declare_parameter(
            'home_joint_positions',
            list(DEFAULT_HOME_JOINT_POSITIONS),
        )
        self.declare_parameter('home_joint_tolerance', 0.012)
        self.declare_parameter('home_velocity_scaling_factor', 0.01)
        self.declare_parameter('home_acceleration_scaling_factor', 0.01)
        self.declare_parameter('home_timeout', 20.0)
        self.declare_parameter('press_timeout', 120.0)
        self.declare_parameter('joint_feedback_timeout', 0.5)
        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('enable_motion', False)

    def _read_parameters(self):
        """Read and validate all parameters before exposing the Action."""
        self.close_panel_sequence = bool(
            self.get_parameter('close_panel_sequence').value
        )
        self.floor_number = int(
            self.get_parameter('floor_number').value
        )
        normalize_floor_targets('', self.floor_number)
        self.home_joint_positions = validate_home_joint_positions(
            self.get_parameter('home_joint_positions').value
        )
        self.home_joint_tolerance = float(
            self.get_parameter('home_joint_tolerance').value
        )
        self.home_velocity_scaling_factor = float(
            self.get_parameter('home_velocity_scaling_factor').value
        )
        self.home_acceleration_scaling_factor = float(
            self.get_parameter('home_acceleration_scaling_factor').value
        )
        self.home_timeout = float(
            self.get_parameter('home_timeout').value
        )
        self.press_timeout = float(
            self.get_parameter('press_timeout').value
        )
        self.joint_feedback_timeout = float(
            self.get_parameter('joint_feedback_timeout').value
        )
        for name in (
            'home_joint_tolerance',
            'home_velocity_scaling_factor',
            'home_acceleration_scaling_factor',
            'home_timeout',
            'press_timeout',
            'joint_feedback_timeout',
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        for name in (
            'home_velocity_scaling_factor',
            'home_acceleration_scaling_factor',
        ):
            if getattr(self, name) > 1.0:
                raise ValueError(f'{name} must not exceed 1.0')
        self.move_group_name = str(
            self.get_parameter('move_group_name').value
        ).strip()
        if not self.move_group_name:
            raise ValueError('move_group_name cannot be empty')
        self.enable_motion = bool(
            self.get_parameter('enable_motion').value
        )

    def _joint_state_callback(self, message):
        """Cache complete finite arm feedback for home verification."""
        if home_joint_errors(message, self.home_joint_positions) is None:
            return
        with self.data_lock:
            self.latest_joint_state = copy.deepcopy(message)
            self.latest_joint_received = time.monotonic()

    def _pbvs_state_callback(self, message):
        """Remember whether the active press entered a motion stage."""
        if message.data == 'WAIT_TARGET':
            with self.data_lock:
                self.last_press_motion_state = None
            return
        if message.data in ('COARSE_APPROACH', 'X_ADVANCE'):
            with self.data_lock:
                self.last_press_motion_state = message.data

    def _goal_callback(self, goal_request):
        """Reject invalid floor targets and concurrent sequences."""
        try:
            normalize_floor_targets(
                goal_request.target_name,
                self.floor_number,
            )
        except ValueError as error:
            self.get_logger().error(f'Rejecting sequence goal: {error}')
            return GoalResponse.REJECT
        with self.active_lock:
            if self.task_active:
                return GoalResponse.REJECT
            self.task_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        """Accept cancellation and stop without starting recovery motion."""
        return CancelResponse.ACCEPT

    def _set_state(self, state):
        """Publish the current sequence state."""
        self.current_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)
        self.get_logger().info(f'【电梯任务状态】{state}')

    def _feedback(self, goal_handle, child_feedback=None):
        """Publish sequence state and optional child positioning errors."""
        feedback = PressButton.Feedback()
        feedback.state = self.current_state
        if child_feedback is not None:
            feedback.position_error_m = child_feedback.position_error_m
            feedback.angular_error_rad = child_feedback.angular_error_rad
            feedback.target_age_s = child_feedback.target_age_s
        goal_handle.publish_feedback(feedback)

    def _child_press_feedback(self, goal_handle, child_feedback):
        """Record the child stage before forwarding its Action feedback."""
        if child_feedback.state == 'WAIT_TARGET':
            with self.data_lock:
                self.last_press_motion_state = None
        elif child_feedback.state in ('COARSE_APPROACH', 'X_ADVANCE'):
            with self.data_lock:
                self.last_press_motion_state = child_feedback.state
        self._feedback(goal_handle, child_feedback)

    @staticmethod
    def _result(success, message):
        """Construct a PressButton result for the sequence Action."""
        result = PressButton.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    @staticmethod
    def _wait_submission(future, timeout, goal_handle, description):
        """Wait for goal acceptance while honoring parent cancellation."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if goal_handle.is_cancel_requested:
                raise SequenceCanceled(
                    f'canceled while submitting {description}'
                )
            if time.monotonic() >= deadline:
                raise SequenceFailure(
                    f'timed out submitting {description}'
                )
            time.sleep(0.02)
        return future.result()

    def _run_press(self, parent_goal, target_name, state):
        """Call the existing single-button Action and forward feedback."""
        self._set_state(state)
        with self.data_lock:
            self.last_press_motion_state = None
        if not self.press_client.wait_for_server(timeout_sec=5.0):
            raise SequenceFailure('/press_button unavailable')
        request = PressButton.Goal()
        request.target_name = target_name
        if getattr(self, 'close_panel_sequence', False):
            request.close_panel_sequence = True
            request.sequence_continuation = self.snapshot_started
            if not self.snapshot_started:
                request.snapshot_targets = list(self.snapshot_targets)
            self.snapshot_started = True
        send_future = self.press_client.send_goal_async(
            request,
            feedback_callback=lambda message: self._child_press_feedback(
                parent_goal,
                message.feedback,
            ),
        )
        child_goal = self._wait_submission(
            send_future,
            5.0,
            parent_goal,
            target_name,
        )
        if child_goal is None or not child_goal.accepted:
            raise SequenceFailure(f'/press_button rejected {target_name}')
        self.active_press_goal = child_goal
        result_future = child_goal.get_result_async()
        result_ready = threading.Event()
        result_future.add_done_callback(lambda _: result_ready.set())
        deadline = time.monotonic() + self.press_timeout
        while not result_future.done():
            if parent_goal.is_cancel_requested:
                child_goal.cancel_goal_async()
                raise SequenceCanceled(f'canceled while pressing {target_name}')
            if time.monotonic() >= deadline:
                child_goal.cancel_goal_async()
                raise SequenceFailure(f'pressing {target_name} timed out')
            self._feedback(parent_goal)
            result_ready.wait(0.05)
        self.active_press_goal = None
        wrapped_result = result_future.result()
        if (
            wrapped_result is None
            or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped_result.result.success
        ):
            detail = (
                wrapped_result.result.message
                if wrapped_result is not None else 'no result'
            )
            self._recover_after_press_failure(
                parent_goal,
                target_name,
                detail,
            )

    def _recover_after_press_failure(
        self,
        parent_goal,
        target_name,
        detail,
    ):
        """Return home after a completed press fails in a motion stage."""
        failure = f'pressing {target_name} failed: {detail}'
        if getattr(self, 'close_panel_sequence', False):
            raise SequenceFailure(failure + '; close sequence stopped without recovery motion')
        with self.data_lock:
            motion_state = self.last_press_motion_state
        if motion_state not in ('COARSE_APPROACH', 'X_ADVANCE'):
            raise SequenceFailure(failure)

        self.get_logger().warning(
            '【按键失败恢复】单键任务在 '
            f'{motion_state} 阶段失败，开始回到 home_joint_positions'
        )
        try:
            self._run_home(parent_goal, 'RECOVERY_HOME')
        except SequenceCanceled:
            raise
        except Exception as recovery_error:
            raise SequenceFailure(
                f'{failure}; recovery home failed: {recovery_error}'
            ) from recovery_error
        raise SequenceFailure(f'{failure}; recovered to home')

    def _home_goal(self):
        """Build the current configured MoveIt home goal."""
        return make_home_moveit_goal(
            self.home_joint_positions,
            self.home_joint_tolerance,
            self.move_group_name,
            not self.enable_motion,
            self.home_velocity_scaling_factor,
            self.home_acceleration_scaling_factor,
        )

    def _run_home(self, parent_goal, state):
        """Plan or execute and verify one return to the home joint pose."""
        self._set_state(state)
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise SequenceFailure('/move_action unavailable for home return')
        send_future = self.move_group_client.send_goal_async(
            self._home_goal()
        )
        move_goal = self._wait_submission(
            send_future,
            5.0,
            parent_goal,
            'home return',
        )
        if move_goal is None or not move_goal.accepted:
            raise SequenceFailure('MoveIt rejected home return')
        self.active_move_goal = move_goal
        result_future = move_goal.get_result_async()
        result_ready = threading.Event()
        result_future.add_done_callback(lambda _: result_ready.set())
        deadline = time.monotonic() + self.home_timeout
        while not result_future.done():
            if parent_goal.is_cancel_requested:
                move_goal.cancel_goal_async()
                raise SequenceCanceled('canceled during home return')
            if time.monotonic() >= deadline:
                move_goal.cancel_goal_async()
                raise SequenceFailure('MoveIt home return timed out')
            self._feedback(parent_goal)
            result_ready.wait(0.05)
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
            raise SequenceFailure(
                f'MoveIt home return failed with error code {code}'
            )
        if self.enable_motion:
            self._verify_home_feedback(parent_goal)
        else:
            self.get_logger().info(
                'Home plan succeeded; enable_motion=false, no motion sent'
            )

    def _verify_home_feedback(self, goal_handle):
        """Require fresh real joint feedback inside the home tolerance."""
        deadline = time.monotonic() + min(3.0, self.home_timeout)
        last_max_error = math.inf
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise SequenceCanceled('canceled during home verification')
            now = time.monotonic()
            with self.data_lock:
                message = copy.deepcopy(self.latest_joint_state)
                received = self.latest_joint_received
            if (
                message is not None
                and now - received <= self.joint_feedback_timeout
            ):
                errors = home_joint_errors(
                    message,
                    self.home_joint_positions,
                )
                if errors is not None:
                    last_max_error = max(errors)
                    if home_joint_error_accepted(
                        last_max_error,
                        self.home_joint_tolerance,
                    ):
                        self.get_logger().info(
                            '【回位验收成功】最大关节误差='
                            f'{last_max_error:.6f} rad，配置容差='
                            f'{self.home_joint_tolerance:.6f} rad，'
                            '允许超差<'
                            f'{HOME_JOINT_ACCEPTANCE_SLACK:.6f} rad'
                        )
                        return
            self._feedback(goal_handle)
            time.sleep(0.05)
        raise SequenceFailure(
            'home joint feedback did not reach tolerance; '
            f'max_error={last_max_error:.6f} rad, '
            f'tolerance={self.home_joint_tolerance:.6f} rad, '
            'allowed_overrun<'
            f'{HOME_JOINT_ACCEPTANCE_SLACK:.6f} rad'
        )

    def _press_event_callback(self, message):
        if not self.task_active:
            return
        if self.last_press_event is not None:
            interval = message.data - self.last_press_event
            self.get_logger().info(
                f'【按键间隔监测】推进完成事件间隔={interval:.3f}s，要求<=3.000s'
            )
            if interval > 3.0:
                self.get_logger().error(
                    '【按键间隔超限】本轮未满足3秒要求；推进完成时间不等于真实按钮触发时间'
                )
        self.last_press_event = message.data

    def _execute_sequence(self, goal_handle):
        """Press each floor digit with a home return, then OK and home."""
        try:
            self.last_press_event = None
            floor_targets = normalize_floor_targets(
                goal_handle.request.target_name,
                self.floor_number,
            )
            floor_label = ', '.join(floor_targets)
            self.snapshot_targets = (*floor_targets, 'key_ok')
            self.snapshot_started = False
            if getattr(self, 'close_panel_sequence', False) and self.enable_motion:
                self._set_state('VERIFY_OBSERVATION_HOME')
                self._verify_home_feedback(goal_handle)
            self.get_logger().info(
                f'【电梯任务开始】数字键={floor_label}, '
                + ('数字间退回安全间距，随后按 key_ok'
                 if getattr(self, 'close_panel_sequence', False) else '每个数字后回位，随后按 key_ok')
            )
            for floor_target in floor_targets:
                self._run_press(
                    goal_handle,
                    floor_target,
                    'PRESS_NUMBER',
                )
                if not getattr(self, 'close_panel_sequence', False):
                    self._run_home(goal_handle, 'RETURN_AFTER_NUMBER')
            self._run_press(goal_handle, 'key_ok', 'PRESS_OK')
            self._run_home(goal_handle, 'RETURN_AFTER_OK')
            self._set_state('DONE')
            goal_handle.succeed()
            return self._result(
                True,
                f'{floor_label}, key_ok, and '
                'final home completed',
            )
        except SequenceCanceled as error:
            self._set_state('ABORT')
            goal_handle.canceled()
            return self._result(False, str(error))
        except Exception as error:
            self.get_logger().error(f'【电梯任务失败】{error}')
            self._set_state('ABORT')
            goal_handle.abort()
            return self._result(False, str(error))
        finally:
            self.active_press_goal = None
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
    """Run the elevator sequence with concurrent Action callbacks."""
    rclpy.init(args=args)
    node = ElevatorSequence()
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
