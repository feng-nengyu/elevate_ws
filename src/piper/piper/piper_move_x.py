"""Execute one guarded relative movement along the base-frame X axis."""

import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from piper_msgs.msg import PiperStatusMsg, PosCmd
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

from piper.x_motion import (
    distance_metres,
    interpolated_displacements,
    normalize_motion_algorithm,
    translated_x,
)


class PiperMoveX(Node):
    """Move the measured Piper TCP once along ``base_link`` X."""

    FEEDBACK_TIMEOUT = 0.5
    READY_TIMEOUT = 5.0
    SETTLE_TIMEOUT = 5.0
    MOVEIT_GOAL_TIMEOUT = 5.0
    MOVEIT_RESULT_TIMEOUT = 30.0
    CONTROL_PERIOD = 0.1
    MAX_TRACKING_ERROR = 0.010
    MAX_CROSS_AXIS_DRIFT = 0.005
    MAX_ANGULAR_DRIFT = math.radians(3.0)
    FINAL_POSITION_TOLERANCE = 0.002
    REQUIRED_SETTLE_SAMPLES = 3
    MOVEIT_POSITION_TOLERANCE = 0.002
    MOVEIT_ORIENTATION_TOLERANCE = 0.05
    MOVEIT_GROUP = 'arm'
    MOVEIT_LINK = 'tcp_link'

    def __init__(self):
        """Create feedback interfaces and validate the one-shot request."""
        super().__init__('piper_move_x')
        self.declare_parameter('distance_mm', 0.0)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('motion_algorithm', 'cartesian')

        self.finished = False
        self.exit_code = 0
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        try:
            self.motion_algorithm = normalize_motion_algorithm(
                self.get_parameter('motion_algorithm').value
            )
            self.distance_m = distance_metres(
                self.get_parameter('distance_mm').value
            )
        except (TypeError, ValueError) as error:
            self._finish_failure(str(error))
            return

        if self.distance_m == 0.0:
            self.get_logger().info(
                'distance_mm=0: zero displacement requested; no command sent'
            )
            self.finished = True
            return

        self.feedback = {
            'tcp': None,
            'flange': None,
            'joints': None,
            'status': None,
            'enabled': None,
        }
        self.received_at = {name: -math.inf for name in self.feedback}
        self.create_subscription(
            PoseStamped, '/tcp_pose', self._tcp_callback, 10
        )
        self.create_subscription(
            Pose, '/end_pose', self._flange_callback, 10
        )
        self.create_subscription(
            JointState,
            '/joint_states_single',
            self._joint_callback,
            10,
        )
        self.create_subscription(
            PiperStatusMsg, '/arm_status', self._status_callback, 10
        )
        self.create_subscription(
            Bool,
            '/piper_command_enabled',
            self._enabled_callback,
            10,
        )
        self.command_pub = None
        self.move_group_client = None
        if self.motion_algorithm == 'cartesian':
            self.command_pub = self.create_publisher(
                PosCmd, '/pos_cmd', 10
            )
        else:
            self.move_group_client = ActionClient(
                self,
                MoveGroup,
                '/move_action',
            )

        self.state = 'WAITING'
        self.started_at = time.monotonic()
        self.start_tcp = None
        self.start_tcp_quaternion = None
        self.start_flange = None
        self.start_flange_quaternion = None
        self.start_euler = None
        self.gripper = None
        self.target_tcp = None
        self.target_tcp_quaternion = None
        self.offsets = ()
        self.next_offset_index = 0
        self.last_commanded_offset = 0.0
        self.settle_started_at = None
        self.settle_samples = 0
        self.moveit_goal_future = None
        self.moveit_goal_handle = None
        self.moveit_result_future = None
        self.moveit_deadline = None
        self.timer = self.create_timer(self.CONTROL_PERIOD, self._tick)
        self.get_logger().info(
            'X-motion request received: '
            f'{self.distance_m * 1000.0:+.3f} mm in base_link; '
            f'motion_algorithm={self.motion_algorithm}; '
            f'enable_motion={self.enable_motion}'
        )

    def _cache(self, name, message):
        """Store a feedback message with its local receive time."""
        self.feedback[name] = message
        self.received_at[name] = time.monotonic()

    def _tcp_callback(self, message):
        self._cache('tcp', message)

    def _flange_callback(self, message):
        self._cache('flange', message)

    def _joint_callback(self, message):
        self._cache('joints', message)

    def _status_callback(self, message):
        self._cache('status', message)

    def _enabled_callback(self, message):
        self._cache('enabled', message)

    @staticmethod
    def _pose_values(pose):
        """Return finite position and normalized quaternion tuples."""
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        if not all(math.isfinite(value) for value in position + quaternion):
            raise RuntimeError('pose feedback contains a non-finite value')
        try:
            rotation = Rotation.from_quat(quaternion)
        except ValueError as error:
            raise RuntimeError('pose feedback quaternion is invalid') from error
        return position, tuple(rotation.as_quat())

    @staticmethod
    def _gripper_value(message):
        """Read the raw total gripper opening from legacy joint feedback."""
        if len(message.name) != len(message.position):
            raise RuntimeError('/joint_states_single has mismatched fields')
        positions = dict(zip(message.name, message.position))
        if 'joint6' not in positions:
            raise RuntimeError('/joint_states_single has no joint6 gripper')
        value = float(positions['joint6'])
        if not math.isfinite(value) or not 0.0 <= value <= 0.08:
            raise RuntimeError('gripper feedback is outside [0, 0.08] m')
        return value

    @staticmethod
    def _hardware_fault(status):
        """Return a fault description from the Piper status message."""
        if status.err_code != 0:
            return f'Piper error code {status.err_code}'
        if status.arm_status != 0:
            return f'Piper arm status {status.arm_status}'
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

    def _missing_or_stale_feedback(self):
        """Return feedback names that are absent or older than the limit."""
        now = time.monotonic()
        required = ('tcp', 'status', 'enabled')
        if self.motion_algorithm == 'cartesian':
            required += ('flange', 'joints')
        return [
            name
            for name in required
            for message in (self.feedback[name],)
            if message is None
            or now - self.received_at[name] > self.FEEDBACK_TIMEOUT
        ]

    def _check_cartesian_ownership(self):
        """Require exclusive access to the direct Cartesian command path."""
        position_publishers = self.count_publishers('/pos_cmd')
        if position_publishers != 1:
            raise RuntimeError(
                '/pos_cmd must have exactly this node as its only publisher; '
                f'found {position_publishers}'
            )
        joint_publishers = self.count_publishers('/joint_ctrl_single')
        if joint_publishers != 0:
            raise RuntimeError(
                '/joint_ctrl_single has an active command publisher; '
                f'found {joint_publishers}'
            )

    def _check_moveit_ownership(self):
        """Require the real trajectory bridge and no direct pose source."""
        position_publishers = self.count_publishers('/pos_cmd')
        if position_publishers != 0:
            raise RuntimeError(
                '/pos_cmd must have no publishers in moveit mode; '
                f'found {position_publishers}'
            )
        joint_publishers = self.count_publishers('/joint_ctrl_single')
        if joint_publishers != 1:
            raise RuntimeError(
                'moveit mode requires exactly one /joint_ctrl_single '
                'publisher (piper_trajectory_controller); '
                f'found {joint_publishers}'
            )

    def _check_command_ownership(self):
        """Apply command-source rules for the selected algorithm."""
        if self.motion_algorithm == 'cartesian':
            self._check_cartesian_ownership()
        else:
            self._check_moveit_ownership()

    def _initialize_motion(self):
        """Snapshot the current state and enter preview or execution."""
        tcp_message = self.feedback['tcp']
        if tcp_message.header.frame_id != 'base_link':
            raise RuntimeError(
                '/tcp_pose must use base_link, got '
                f'{tcp_message.header.frame_id!r}'
            )
        self.start_tcp, self.start_tcp_quaternion = self._pose_values(
            tcp_message.pose
        )
        self.target_tcp = translated_x(self.start_tcp, self.distance_m)
        self.target_tcp_quaternion = self.start_tcp_quaternion
        target_tcp_x = self.target_tcp[0]
        self.get_logger().info(
            'Current TCP: '
            f'x={self.start_tcp[0]:.6f}, y={self.start_tcp[1]:.6f}, '
            f'z={self.start_tcp[2]:.6f} m; '
            f'target x={target_tcp_x:.6f} m'
        )

        fault = self._hardware_fault(self.feedback['status'])
        if fault:
            raise RuntimeError(fault)
        if self.motion_algorithm == 'cartesian':
            (
                self.start_flange,
                self.start_flange_quaternion,
            ) = self._pose_values(self.feedback['flange'])
            self.start_euler = tuple(
                Rotation.from_quat(
                    self.start_flange_quaternion
                ).as_euler('xyz')
            )
            self.gripper = self._gripper_value(self.feedback['joints'])

        if not self.enable_motion and self.motion_algorithm == 'cartesian':
            self.get_logger().info(
                'Dry run complete; set enable_motion:=true to execute'
            )
            self.finished = True
            return
        if self.enable_motion and not self.feedback['enabled'].data:
            raise RuntimeError(
                'Piper command input is not enabled; call /enable_srv first'
            )
        self._check_command_ownership()
        if self.motion_algorithm == 'cartesian':
            self.offsets = interpolated_displacements(self.distance_m)
            self.state = 'MOVING'
            self.get_logger().info(
                f'Executing {len(self.offsets)} Cartesian steps at 10 Hz'
            )
        else:
            self._start_moveit()

    def _moveit_goal(self):
        """Build a MoveGroup goal for the requested TCP endpoint."""
        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = self.MOVEIT_LINK
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        diameter = 2.0 * self.MOVEIT_POSITION_TOLERANCE
        region.dimensions = [diameter, diameter, diameter]
        target_pose = Pose()
        target_pose.position.x = float(self.target_tcp[0])
        target_pose.position.y = float(self.target_tcp[1])
        target_pose.position.z = float(self.target_tcp[2])
        target_pose.orientation.x = float(self.target_tcp_quaternion[0])
        target_pose.orientation.y = float(self.target_tcp_quaternion[1])
        target_pose.orientation.z = float(self.target_tcp_quaternion[2])
        target_pose.orientation.w = float(self.target_tcp_quaternion[3])
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(
            target_pose
        )
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.link_name = self.MOVEIT_LINK
        orientation_constraint.orientation = target_pose.orientation
        tolerance = self.MOVEIT_ORIENTATION_TOLERANCE
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.MOVEIT_GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = not self.enable_motion
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.enable_motion
        goal.planning_options.replan_delay = 1.0
        return goal

    def _start_moveit(self):
        """Submit the endpoint goal without blocking subscription callbacks."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/move_action unavailable')
        self.moveit_goal_future = self.move_group_client.send_goal_async(
            self._moveit_goal()
        )
        self.moveit_deadline = (
            time.monotonic() + self.MOVEIT_GOAL_TIMEOUT
        )
        self.state = 'MOVEIT_ACCEPTING'
        operation = 'planning and execution' if self.enable_motion else 'plan'
        self.get_logger().info(f'Submitted MoveIt {operation} request')

    @staticmethod
    def _angular_error(first_quaternion, second_quaternion):
        """Return the shortest angular separation between orientations."""
        first = Rotation.from_quat(first_quaternion)
        second = Rotation.from_quat(second_quaternion)
        return float((first.inv() * second).magnitude())

    def _basic_guard(self):
        """Validate live feedback, hardware state, and command ownership."""
        missing = self._missing_or_stale_feedback()
        if missing:
            raise RuntimeError(
                'feedback missing or stale: ' + ', '.join(missing)
            )
        fault = self._hardware_fault(self.feedback['status'])
        if fault:
            raise RuntimeError(fault)
        if self.enable_motion and not self.feedback['enabled'].data:
            raise RuntimeError('Piper command input became disabled')
        self._check_command_ownership()

    def _motion_guard(self, check_tracking=True):
        """Validate the direct Cartesian path and its live tracking."""
        self._basic_guard()

        position, quaternion = self._pose_values(
            self.feedback['tcp'].pose
        )
        cross_axis_drift = math.hypot(
            position[1] - self.start_tcp[1],
            position[2] - self.start_tcp[2],
        )
        if cross_axis_drift > self.MAX_CROSS_AXIS_DRIFT:
            raise RuntimeError(
                'TCP cross-axis drift exceeded 5 mm '
                f'({cross_axis_drift * 1000.0:.2f} mm)'
            )
        angular_drift = self._angular_error(
            self.start_tcp_quaternion,
            quaternion,
        )
        if angular_drift > self.MAX_ANGULAR_DRIFT:
            raise RuntimeError(
                'TCP angular drift exceeded 3 degrees '
                f'({math.degrees(angular_drift):.2f} deg)'
            )
        if check_tracking:
            expected_x = self.start_tcp[0] + self.last_commanded_offset
            tracking_error = abs(position[0] - expected_x)
            if tracking_error > self.MAX_TRACKING_ERROR:
                raise RuntimeError(
                    'TCP X tracking error exceeded 10 mm '
                    f'({tracking_error * 1000.0:.2f} mm)'
                )
        return position, quaternion

    def _make_command(self, position, quaternion, gripper):
        """Build an absolute J6 command while preserving pose and gripper."""
        euler = Rotation.from_quat(quaternion).as_euler('xyz')
        command = PosCmd()
        command.x = float(position[0])
        command.y = float(position[1])
        command.z = float(position[2])
        command.roll = float(euler[0])
        command.pitch = float(euler[1])
        command.yaw = float(euler[2])
        command.gripper = float(gripper)
        command.mode1 = 0
        command.mode2 = 0
        return command

    def _publish_offset(self, offset):
        """Publish a point on the fixed base-frame X path."""
        position = (
            self.start_flange[0] + offset,
            self.start_flange[1],
            self.start_flange[2],
        )
        command = self._make_command(
            position,
            self.start_flange_quaternion,
            self.gripper,
        )
        self.command_pub.publish(command)
        self.last_commanded_offset = offset

    def _attempt_hold(self):
        """Command the latest measured flange pose after a guarded abort."""
        if self.command_pub is None:
            return
        now = time.monotonic()
        required = ('flange', 'joints', 'enabled')
        if any(
            self.feedback[name] is None
            or now - self.received_at[name] > self.FEEDBACK_TIMEOUT
            for name in required
        ):
            return
        if not self.feedback['enabled'].data:
            return
        if self.count_publishers('/pos_cmd') != 1:
            return
        try:
            position, quaternion = self._pose_values(
                self.feedback['flange']
            )
            gripper = self._gripper_value(self.feedback['joints'])
            self.command_pub.publish(
                self._make_command(position, quaternion, gripper)
            )
            self.get_logger().warning(
                'Published the latest measured flange pose as a hold target'
            )
        except (RuntimeError, ValueError):
            return

    def _finish_failure(self, message):
        """Record a terminal failure and prevent further commands."""
        if getattr(self, 'finished', False):
            return
        self.get_logger().error(f'X movement failed: {message}')
        moveit_goal_handle = getattr(self, 'moveit_goal_handle', None)
        if moveit_goal_handle is not None:
            moveit_goal_handle.cancel_goal_async()
        if (
            getattr(self, 'motion_algorithm', None) == 'cartesian'
            and getattr(self, 'state', 'WAITING')
            in ('MOVING', 'SETTLING')
        ):
            self._attempt_hold()
        self.exit_code = 1
        self.finished = True

    def _advance_moveit(self):
        """Advance asynchronous MoveGroup goal and result handling."""
        self._basic_guard()
        now = time.monotonic()
        if self.state == 'MOVEIT_ACCEPTING':
            if self.moveit_goal_future.done():
                try:
                    goal_handle = self.moveit_goal_future.result()
                except Exception as error:
                    raise RuntimeError(
                        f'MoveIt goal submission failed: {error}'
                    ) from error
                if goal_handle is None or not goal_handle.accepted:
                    raise RuntimeError('MoveIt rejected X-motion goal')
                self.moveit_goal_handle = goal_handle
                self.moveit_result_future = goal_handle.get_result_async()
                self.moveit_deadline = now + self.MOVEIT_RESULT_TIMEOUT
                self.state = 'MOVEIT_RUNNING'
                self.get_logger().info('MoveIt accepted X-motion goal')
            elif now > self.moveit_deadline:
                raise RuntimeError('MoveIt goal submission timed out')
            return

        if not self.moveit_result_future.done():
            if now > self.moveit_deadline:
                self.moveit_goal_handle.cancel_goal_async()
                raise RuntimeError('MoveIt X-motion result timed out')
            return
        try:
            wrapped_result = self.moveit_result_future.result()
        except Exception as error:
            raise RuntimeError(f'MoveIt execution failed: {error}') from error
        if wrapped_result is None:
            raise RuntimeError('MoveIt returned no result')
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'MoveIt failed with error code {error_code}'
            )
        if not self.enable_motion:
            self.get_logger().info(
                'MoveIt plan succeeded; dry-run sent no motion command'
            )
            self.finished = True
            return
        self.get_logger().info(
            'MoveIt execution succeeded; verifying measured TCP'
        )
        self.state = 'SETTLING'
        self.settle_started_at = now

    def _settle(self):
        """Require consecutive final TCP samples inside tolerance."""
        if self.motion_algorithm == 'cartesian':
            position, quaternion = self._motion_guard(
                check_tracking=False
            )
        else:
            self._basic_guard()
            position, quaternion = self._pose_values(
                self.feedback['tcp'].pose
            )
        target = self.target_tcp
        position_error = math.sqrt(sum(
            (measured - desired) ** 2
            for measured, desired in zip(position, target)
        ))
        angular_error = self._angular_error(
            self.start_tcp_quaternion,
            quaternion,
        )
        if (
            position_error <= self.FINAL_POSITION_TOLERANCE
            and angular_error <= self.MAX_ANGULAR_DRIFT
        ):
            self.settle_samples += 1
        else:
            self.settle_samples = 0
        if self.settle_samples >= self.REQUIRED_SETTLE_SAMPLES:
            self.get_logger().info(
                'X movement complete: '
                f'target={self.distance_m * 1000.0:+.3f} mm, '
                f'position_error={position_error * 1000.0:.2f} mm'
            )
            self.finished = True
            return
        if time.monotonic() - self.settle_started_at > self.SETTLE_TIMEOUT:
            raise RuntimeError(
                'final TCP did not settle within 2 mm before timeout; '
                f'last error={position_error * 1000.0:.2f} mm'
            )

    def _tick(self):
        """Advance the one-shot state machine at the command rate."""
        if self.finished:
            return
        try:
            if self.state == 'WAITING':
                missing = self._missing_or_stale_feedback()
                if not missing:
                    self._initialize_motion()
                elif time.monotonic() - self.started_at > self.READY_TIMEOUT:
                    raise RuntimeError(
                        'timed out waiting for feedback: ' + ', '.join(missing)
                    )
            elif self.state == 'MOVING':
                self._motion_guard()
                offset = self.offsets[self.next_offset_index]
                self._publish_offset(offset)
                self.next_offset_index += 1
                if self.next_offset_index == len(self.offsets):
                    self.state = 'SETTLING'
                    self.settle_started_at = time.monotonic()
            elif self.state in ('MOVEIT_ACCEPTING', 'MOVEIT_RUNNING'):
                self._advance_moveit()
            elif self.state == 'SETTLING':
                self._settle()
        except (RuntimeError, ValueError) as error:
            self._finish_failure(str(error))


def main(args=None):
    """Run the one-shot node and return a process exit status."""
    rclpy.init(args=args)
    node = PiperMoveX()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code
    except KeyboardInterrupt:
        node._finish_failure('interrupted by operator')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
