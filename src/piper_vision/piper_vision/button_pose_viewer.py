#!/usr/bin/env python3
"""Print the selected button pose expressed in ``base_link``."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from piper_msgs.srv import SetInterest
import rclpy
from rclpy.node import Node


ZERO_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def quaternion_to_rpy(x, y, z, w):
    """Convert a quaternion to roll, pitch, and yaw in radians."""
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


class ButtonPoseViewer(Node):
    """Select one YOLO button class and print its latest base-frame pose."""

    def __init__(self, cli_button_name=None):
        super().__init__('button_pose_viewer')
        self.declare_parameter('button_name', cli_button_name or '')
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('output_period', 0.5)
        self.declare_parameter('expected_frame', 'base_link')

        self.button_name = str(
            self.get_parameter('button_name').value
        ).strip()
        self.pose_timeout = float(
            self.get_parameter('pose_timeout').value
        )
        self.output_period = float(
            self.get_parameter('output_period').value
        )
        self.expected_frame = str(
            self.get_parameter('expected_frame').value
        ).strip()

        if not self.button_name:
            raise ValueError(
                'button_name is required, for example key_3'
            )
        if not math.isfinite(self.pose_timeout) or self.pose_timeout <= 0.0:
            raise ValueError('pose_timeout must be positive')
        if (
            not math.isfinite(self.output_period)
            or self.output_period <= 0.0
        ):
            raise ValueError('output_period must be positive')
        if not self.expected_frame:
            raise ValueError('expected_frame cannot be empty')

        self.latest_pose = None
        self.latest_pose_received_at = -math.inf
        self.interest_ready = False
        self.interest_future = None
        self.interest_rejected = False
        self.frame_warning_emitted = False

        self.create_subscription(
            PoseStamped,
            '/piper_vision/button_pose',
            self._pose_callback,
            10,
        )
        self.interest_client = self.create_client(
            SetInterest,
            '/set_interest',
        )
        self.create_timer(0.2, self._configure_interest)
        self.create_timer(self.output_period, self._print_pose)
        self.get_logger().info(
            f"Waiting for button '{self.button_name}' in "
            f"frame '{self.expected_frame}'"
        )

    def _configure_interest(self):
        """Ask the detector to publish the requested button class."""
        if (
            self.interest_ready
            or self.interest_rejected
            or self.interest_future is not None
        ):
            return
        if not self.interest_client.service_is_ready():
            return

        request = SetInterest.Request()
        request.name = self.button_name
        self.interest_future = self.interest_client.call_async(request)
        self.interest_future.add_done_callback(
            self._interest_response_callback
        )

    def _interest_response_callback(self, future):
        """Validate the detector response and enable pose reception."""
        self.interest_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Calling /set_interest failed: {error}'
            )
            return

        detail = response.result.strip()
        if detail.startswith('interest changed'):
            self.interest_ready = True
            self.latest_pose = None
            self.latest_pose_received_at = -math.inf
            self.get_logger().info(detail)
            return

        self.get_logger().error(
            f"Detector rejected button '{self.button_name}': {detail}"
        )
        self.interest_rejected = True

    def _pose_callback(self, message):
        """Cache only fresh poses in the requested output frame."""
        if not self.interest_ready:
            return
        if message.header.frame_id != self.expected_frame:
            if not self.frame_warning_emitted:
                self.get_logger().warning(
                    '/piper_vision/button_pose uses frame '
                    f"'{message.header.frame_id}', expected "
                    f"'{self.expected_frame}'; displaying zero"
                )
                self.frame_warning_emitted = True
            return

        self.latest_pose = message
        self.latest_pose_received_at = time.monotonic()
        self.frame_warning_emitted = False

    def _current_values(self):
        """Return the current pose, or seven zeros when it is unavailable."""
        age = time.monotonic() - self.latest_pose_received_at
        if (
            not self.interest_ready
            or self.latest_pose is None
            or age > self.pose_timeout
        ):
            return False, ZERO_POSE

        pose = self.latest_pose.pose
        values = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        if not all(math.isfinite(value) for value in values):
            return False, ZERO_POSE
        return True, values

    def _print_pose(self):
        """Print one compact human-readable pose sample."""
        detected, values = self._current_values()
        x, y, z, qx, qy, qz, qw = values
        if detected:
            roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
        else:
            roll = pitch = yaw = 0.0

        print(
            f'button={self.button_name} '
            f'detected={str(detected).lower()} '
            f'frame={self.expected_frame} | '
            f'position[m]: x={x:.6f}, y={y:.6f}, z={z:.6f} | '
            f'quaternion: x={qx:.6f}, y={qy:.6f}, '
            f'z={qz:.6f}, w={qw:.6f} | '
            f'rpy[rad]: roll={roll:.6f}, pitch={pitch:.6f}, '
            f'yaw={yaw:.6f}',
            flush=True,
        )


def _parse_arguments(arguments):
    """Separate the optional positional button name from ROS arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Print a selected button pose in base_link. When the button is '
            'not detected, all displayed pose values are zero.'
        )
    )
    parser.add_argument(
        'button_name',
        nargs='?',
        help='YOLO class name, for example key_3',
    )
    try:
        ros_index = arguments.index('--ros-args')
    except ValueError:
        cli_arguments = arguments
        ros_arguments = []
    else:
        cli_arguments = arguments[:ros_index]
        ros_arguments = arguments[ros_index:]
    return parser.parse_args(cli_arguments), ros_arguments


def main(args=None):
    """Run the button-pose viewer."""
    arguments = sys.argv[1:] if args is None else list(args)
    parsed, ros_arguments = _parse_arguments(arguments)
    rclpy.init(args=ros_arguments)
    node = None
    try:
        node = ButtonPoseViewer(parsed.button_name)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
