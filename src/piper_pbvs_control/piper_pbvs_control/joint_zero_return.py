"""Return every commanded Piper joint to zero from its current position."""

import copy
import math
import threading
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))
GRIPPER_JOINT_NAMES = ('joint7',)
COMMAND_JOINT_NAMES = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES
MOVEIT_SUCCESS = 1


class JointZeroFailure(RuntimeError):
    """Raised when a requested all-joint zero return cannot finish."""


def make_zero_moveit_goal(
    joint_names,
    group_name,
    tolerance,
    plan_only,
    velocity_scaling_factor,
    acceleration_scaling_factor,
):
    """Construct a MoveGroup joint-space goal with every target at zero."""
    constraints = Constraints()
    for joint_name in joint_names:
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = 0.0
        constraint.tolerance_above = float(tolerance)
        constraint.tolerance_below = float(tolerance)
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)

    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = str(group_name)
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


def zero_joint_errors(message):
    """Map complete joint feedback and return absolute errors from zero."""
    if len(message.name) != len(message.position):
        return None
    positions_by_name = dict(zip(message.name, message.position))
    if not all(name in positions_by_name for name in COMMAND_JOINT_NAMES):
        return None
    positions = tuple(
        float(positions_by_name[name]) for name in COMMAND_JOINT_NAMES
    )
    if not all(math.isfinite(value) for value in positions):
        return None
    return tuple(abs(value) for value in positions)


class JointZeroReturn(Node):
    """Expose an isolated service that returns all seven joints to zero."""

    def __init__(self):
        super().__init__('joint_zero_return')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._read_parameters()

        self.execution_lock = threading.Lock()
        self.execution_active = False
        self.data_lock = threading.Lock()
        self.latest_joint_state = None
        self.latest_joint_received = -math.inf
        self.current_state = 'IDLE'

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.state_pub = self.create_publisher(
            String,
            '/joint_zero_return/state',
            10,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group,
        )
        self.zero_service = self.create_service(
            Trigger,
            '/return_all_joints_zero',
            self._return_all_joints_zero,
            callback_group=self.callback_group,
        )
        self._set_state('IDLE')
        self.get_logger().info(
            'All-joint zero-return service ready; it only runs after an '
            'explicit /return_all_joints_zero request; '
            f'enable_motion={self.enable_motion}'
        )

    def _declare_parameters(self):
        """Declare parameters owned only by this node."""
        self.declare_parameter('arm_group_name', 'arm')
        self.declare_parameter('gripper_group_name', 'gripper')
        self.declare_parameter('arm_zero_tolerance', 0.01)
        self.declare_parameter('gripper_zero_tolerance', 0.003)
        self.declare_parameter('zero_velocity_scaling_factor', 0.10)
        self.declare_parameter('zero_acceleration_scaling_factor', 0.10)
        self.declare_parameter('zero_timeout', 30.0)
        self.declare_parameter('joint_feedback_timeout', 0.5)
        self.declare_parameter('enable_motion', False)

    def _read_parameters(self):
        """Read parameters and reject unsafe values before serving calls."""
        self.arm_group_name = str(
            self.get_parameter('arm_group_name').value
        ).strip()
        self.gripper_group_name = str(
            self.get_parameter('gripper_group_name').value
        ).strip()
        if not self.arm_group_name or not self.gripper_group_name:
            raise ValueError('MoveIt group names cannot be empty')

        for name in (
            'arm_zero_tolerance',
            'gripper_zero_tolerance',
            'zero_velocity_scaling_factor',
            'zero_acceleration_scaling_factor',
            'zero_timeout',
            'joint_feedback_timeout',
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        for name in (
            'zero_velocity_scaling_factor',
            'zero_acceleration_scaling_factor',
        ):
            if getattr(self, name) > 1.0:
                raise ValueError(f'{name} must not exceed 1.0')
        self.enable_motion = bool(
            self.get_parameter('enable_motion').value
        )

    def _joint_state_callback(self, message):
        """Cache only complete, finite joint1-through-joint7 feedback."""
        if zero_joint_errors(message) is None:
            return
        with self.data_lock:
            self.latest_joint_state = copy.deepcopy(message)
            self.latest_joint_received = time.monotonic()

    def _set_state(self, state):
        """Publish this node's isolated execution state."""
        self.current_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)
        self.get_logger().info(f'【七关节归零】{state}')

    @staticmethod
    def _wait_future(future, timeout, description):
        """Wait for one asynchronous result with a finite timeout."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise JointZeroFailure(f'{description} timed out')
            time.sleep(0.02)
        return future.result()

    def _zero_goal(self, joint_names, group_name, tolerance):
        """Build a zero target for one MoveIt planning group."""
        return make_zero_moveit_goal(
            joint_names,
            group_name,
            tolerance,
            not self.enable_motion,
            self.zero_velocity_scaling_factor,
            self.zero_acceleration_scaling_factor,
        )

    def _run_zero_group(
        self,
        state,
        joint_names,
        group_name,
        tolerance,
    ):
        """Plan or execute one MoveIt group at its all-zero target."""
        self._set_state(state)
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise JointZeroFailure('/move_action unavailable for zero return')
        send_future = self.move_group_client.send_goal_async(
            self._zero_goal(joint_names, group_name, tolerance)
        )
        move_goal = self._wait_future(
            send_future,
            5.0,
            f'submitting {group_name} zero return',
        )
        if move_goal is None or not move_goal.accepted:
            raise JointZeroFailure(
                f'MoveIt rejected {group_name} zero return'
            )

        result_future = move_goal.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future,
                self.zero_timeout,
                f'MoveIt {group_name} zero return',
            )
        except JointZeroFailure:
            move_goal.cancel_goal_async()
            raise
        if (
            wrapped_result is None
            or wrapped_result.result.error_code.val != MOVEIT_SUCCESS
        ):
            code = (
                wrapped_result.result.error_code.val
                if wrapped_result is not None else 'unknown'
            )
            raise JointZeroFailure(
                f'MoveIt {group_name} zero return failed with '
                f'error code {code}'
            )

    def _verify_zero_feedback(self):
        """Require fresh feedback with arm and gripper inside tolerance."""
        self._set_state('VERIFY_ZERO')
        deadline = time.monotonic() + min(3.0, self.zero_timeout)
        last_arm_error = math.inf
        last_gripper_error = math.inf
        while time.monotonic() < deadline:
            now = time.monotonic()
            with self.data_lock:
                message = copy.deepcopy(self.latest_joint_state)
                received_at = self.latest_joint_received
            if (
                message is not None
                and now - received_at <= self.joint_feedback_timeout
            ):
                errors = zero_joint_errors(message)
                if errors is not None:
                    last_arm_error = max(errors[:len(ARM_JOINT_NAMES)])
                    last_gripper_error = errors[-1]
                    if (
                        last_arm_error <= self.arm_zero_tolerance
                        and last_gripper_error
                        <= self.gripper_zero_tolerance
                    ):
                        self.get_logger().info(
                            '七关节归零验收成功：arm_max_error='
                            f'{last_arm_error:.6f} rad, '
                            'joint7_error='
                            f'{last_gripper_error:.6f} m'
                        )
                        return
            time.sleep(0.05)
        raise JointZeroFailure(
            'joint feedback did not reach all-zero tolerance; '
            f'arm_max_error={last_arm_error:.6f} rad, '
            f'joint7_error={last_gripper_error:.6f} m'
        )

    def _return_all_joints_zero(self, _request, response):
        """Handle one explicit request to move all seven joints to zero."""
        with self.execution_lock:
            if self.execution_active:
                response.success = False
                response.message = 'an all-joint zero return is already active'
                return response
            self.execution_active = True

        try:
            self._run_zero_group(
                'ZERO_ARM',
                ARM_JOINT_NAMES,
                self.arm_group_name,
                self.arm_zero_tolerance,
            )
            self._run_zero_group(
                'ZERO_GRIPPER',
                GRIPPER_JOINT_NAMES,
                self.gripper_group_name,
                self.gripper_zero_tolerance,
            )
            if self.enable_motion:
                self._verify_zero_feedback()
                detail = 'joint1-joint7 reached zero'
            else:
                detail = (
                    'dry run complete: joint1-joint7 zero plans '
                    'succeeded; no motion sent'
                )
            self._set_state('DONE')
            response.success = True
            response.message = detail
            return response
        except Exception as error:
            self.get_logger().error(f'【七关节归零失败】{error}')
            self._set_state('ABORT')
            response.success = False
            response.message = str(error)
            return response
        finally:
            with self.execution_lock:
                self.execution_active = False
            if self.current_state in ('DONE', 'ABORT'):
                self._set_state('IDLE')


def main(args=None):
    """Run the isolated all-joint zero-return service node."""
    rclpy.init(args=args)
    node = JointZeroReturn()
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
