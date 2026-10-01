#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip

from piper.joint_state_mapping import (
    MOVEIT_JOINT_NAMES,
    moveit_joint_positions,
)
from piper.enable_feedback import (
    LowSpeedFeedbackCounterUnavailable,
    fresh_low_speed_feedback_joints,
    read_low_speed_feedback_counts,
    requested_state_is_confirmed,
)


def calculate_tcp_position(flange_position, flange_quaternion, tcp_offset):
    """Apply a tool offset expressed in the J6/gripper coordinate frame."""
    rotated_offset = R.from_quat(flange_quaternion).apply(tcp_offset)
    return tuple(
        flange_position[index] + rotated_offset[index]
        for index in range(3)
    )


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('tcp_offset_x', 0.0)
        self.declare_parameter('tcp_offset_y', 0.0)
        self.declare_parameter('tcp_offset_z', 0.1358)#末端夹爪的修改数值
        
        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(
            1,
            min(self.gripper_val_mutiple, 10),
        )
        self.tcp_offset = (
            float(self.get_parameter('tcp_offset_x').value),
            float(self.get_parameter('tcp_offset_y').value),
            float(self.get_parameter('tcp_offset_z').value),
        )
        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(f"tcp_offset is {self.tcp_offset} m in the J6 frame")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.moveit_joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )
        self.raw_moveit_joint_pub = self.create_publisher(
            JointState,
            '/joint_states_raw',
            10,
        )
        self.command_enabled_pub = self.create_publisher(
            Bool,
            '/piper_command_enabled',
            10,
        )
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, 'tcp_pose', 1)
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        self.moveit_joint_states = JointState()
        self.moveit_joint_states.name = list(MOVEIT_JOINT_NAMES)
        self.moveit_joint_states.position = [0.0] * 7
        self.moveit_joint_states.velocity = [0.0] * 7
        self.moveit_joint_states.effort = [0.0] * 7
        self.raw_moveit_joint_states = JointState()
        self.raw_moveit_joint_states.name = list(MOVEIT_JOINT_NAMES)
        self.raw_moveit_joint_states.position = [0.0] * 7
        self.raw_moveit_joint_states.velocity = [0.0] * 7
        self.raw_moveit_joint_states.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)
        self.create_timer(0.1, self.PublishCommandEnabled)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def PublishCommandEnabled(self):
        """Publish whether this driver instance accepts motion commands."""
        message = Bool()
        message.data = self.GetEnableFlag()
        self.command_enabled_pub.publish(message)

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("Enable status:", enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        print("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("Automatic enable timeout, exiting program")
                exit(0)

            self.PublishArmState()
            self.PublishArmJointAndGripper()
            self.PublishArmEndPose()

            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort / 1000
        self.joint_states.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_6]
        self.joint_pub.publish(self.joint_states)

        moveit_positions = moveit_joint_positions(
            self.joint_states.position,
            self.gripper_val_mutiple,
        )

        velocities = [
            vel_0,
            vel_1,
            vel_2,
            vel_3,
            vel_4,
            vel_5,
            0.0,
        ]
        self.moveit_joint_states.header.stamp = self.joint_states.header.stamp
        self.moveit_joint_states.position = list(moveit_positions)
        self.moveit_joint_states.velocity = velocities
        self.moveit_joint_states.effort = [0.0] * 7
        self.moveit_joint_pub.publish(self.moveit_joint_states)

        self.raw_moveit_joint_states.header.stamp = (
            self.joint_states.header.stamp
        )
        self.raw_moveit_joint_states.position = list(moveit_positions)
        self.raw_moveit_joint_states.velocity = list(velocities)
        self.raw_moveit_joint_states.effort = [0.0] * 7
        self.raw_moveit_joint_pub.publish(self.raw_moveit_joint_states)

    def PublishArmEndPose(self):
        # The SDK reports the J6/flange pose. Keep /end_pose backward compatible,
        # and publish the grasp TCP separately as /tcp_pose.
        flange_feedback = self.piper.GetArmEndPoseMsgs().end_pose
        endpos = Pose()
        endpos.position.x = flange_feedback.X_axis / 1000000
        endpos.position.y = flange_feedback.Y_axis / 1000000
        endpos.position.z = flange_feedback.Z_axis / 1000000
        roll = flange_feedback.RX_axis / 1000
        pitch = flange_feedback.RY_axis / 1000
        yaw = flange_feedback.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

        tcp_position = calculate_tcp_position(
            (endpos.position.x, endpos.position.y, endpos.position.z),
            quaternion,
            self.tcp_offset,
        )
        tcp_pose = PoseStamped()
        tcp_pose.header.stamp = self.get_clock().now().to_msg()
        tcp_pose.header.frame_id = 'base_link'
        tcp_pose.pose.position.x = tcp_position[0]
        tcp_pose.pose.position.y = tcp_position[1]
        tcp_pose.pose.position.z = tcp_position[2]
        tcp_pose.pose.orientation.x = quaternion[0]
        tcp_pose.pose.orientation.y = quaternion[1]
        tcp_pose.pose.orientation.z = quaternion[2]
        tcp_pose.pose.orientation.w = quaternion[3]
        self.tcp_pose_pub.publish(tcp_pose)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        factor = 180 / 3.1415926
        self.get_logger().info(f"Received PosCmd:")
        self.get_logger().info(f"x: {pos_data.x}")
        self.get_logger().info(f"y: {pos_data.y}")
        self.get_logger().info(f"z: {pos_data.z}")
        self.get_logger().info(f"roll: {pos_data.roll}")
        self.get_logger().info(f"pitch: {pos_data.pitch}")
        self.get_logger().info(f"yaw: {pos_data.yaw}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            # PBVS streams small Cartesian pose increments.  Select MOVE P
            # before publishing each EndPoseCtrl target and keep that mode;
            # switching modes immediately after the target can cancel it.
            self.piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
            gripper = round(pos_data.gripper * 1000 * 1000)
            if pos_data.gripper > 80000:
                gripper = 80000
            if pos_data.gripper < 0:
                gripper = 0
            if self.gripper_exist:
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        factor = 57324.840764  # 1000*180/3.14
        # self.get_logger().info(f"Received Joint States:")

        # 创建一个字典来存储关节名称与位置的映射
        joint_positions = {}
        joint_6 = 0

        # 遍历joint_data.name来映射位置
        for idx, joint_name in enumerate(joint_data.name):
            # self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)
        
        # 获取第7个关节的位置
        if len(joint_data.position) >= 7:
            # self.get_logger().info(f"joint_7: {joint_data.position[6]}")
            joint_6 = round(joint_data.position[6] * 1000 * 1000)
            joint_6 = joint_6 * self.gripper_val_mutiple

        # 控制电机速度
        if self.GetEnableFlag():
            if joint_data.velocity != []:
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else:
                all_zeros = True
            if not all_zeros:
                lens = len(joint_data.velocity)
                if lens == 7:
                    vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                    self.get_logger().debug(f"vel_all: {vel_all}")
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            # 使用关节名称来动态控制关节
            self.piper.JointCtrl(
                joint_positions.get('joint1', 0),
                joint_positions.get('joint2', 0),
                joint_positions.get('joint3', 0),
                joint_positions.get('joint4', 0),
                joint_positions.get('joint5', 0),
                joint_positions.get('joint6', 0)
            )

            # 夹爪控制
            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 0  # 设置默认值
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        self.get_logger().info(f"Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self.get_logger().info(f"Received request: {req.enable_request}")
        timeout = 5.0
        poll_interval = 0.5
        start_time = time.monotonic()

        try:
            baseline_counts = read_low_speed_feedback_counts(self.piper)
        except LowSpeedFeedbackCounterUnavailable as error:
            self.__enable_flag = False
            resp.enable_response = False
            self.get_logger().error(
                f"Cannot verify fresh low-speed feedback: {error}"
            )
            return resp

        latest_counts = baseline_counts
        enable_states = [False] * 6

        while time.monotonic() - start_time <= timeout:
            if req.enable_request:
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

            try:
                latest_counts = read_low_speed_feedback_counts(self.piper)
            except LowSpeedFeedbackCounterUnavailable as error:
                self.__enable_flag = False
                resp.enable_response = False
                self.get_logger().error(
                    f"Lost low-speed feedback verification: {error}"
                )
                return resp

            low_speed_info = self.piper.GetArmLowSpdInfoMsgs()
            enable_states = [
                getattr(low_speed_info, f"motor_{joint_index}")
                .foc_status.driver_enable_status
                for joint_index in range(1, 7)
            ]

            if requested_state_is_confirmed(
                req.enable_request,
                enable_states,
                baseline_counts,
                latest_counts,
            ):
                self.__enable_flag = req.enable_request
                resp.enable_response = True
                self.get_logger().info(
                    "Requested state confirmed by fresh low-speed "
                    "feedback from all six joints"
                )
                return resp

            time.sleep(poll_interval)

        fresh_joints = fresh_low_speed_feedback_joints(
            baseline_counts,
            latest_counts,
        )
        missing_joints = [
            joint_index
            for joint_index in range(1, 7)
            if joint_index not in fresh_joints
        ]
        self.__enable_flag = False
        resp.enable_response = False
        if missing_joints:
            self.get_logger().warning(
                "Enable service timed out; fresh low-speed feedback missing "
                f"for joints {missing_joints}; enable states: {enable_states}"
            )
        else:
            self.get_logger().warning(
                "Enable service timed out; all feedback was fresh, but the "
                f"requested state was not reached: {enable_states}"
            )
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()
