import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
import math
from piper_msgs.action import GripperControl
import time


class GraspServer(Node):
    def __init__(self):
        super().__init__('grasp_server')
        self.target_point = None
        self.target_pose = None
        self.last_target_point = None
        self.last_update_time = 0.0  # 上一次更新时间戳（秒）
        self.rejected_points = []  # 用于记录连续被拒绝的点

        # 订阅视觉目标点
        self.create_subscription(PointStamped, '/base_target_point', self.target_point_cb, 10)

        # 订阅用户指定的完整目标位姿。Point 和 Pose 中最后收到的有效目标生效。
        self.create_subscription(PoseStamped, '/base_target_pose', self.target_pose_cb, 10)

        # 创建服务用于触发抓取动作
        self.create_service(Trigger, '/grasp_command', self.grasp_callback)

        self.gripper_action_client = ActionClient(self, GripperControl, 'gripper_control')

        # 创建 MoveGroup action 客户端
        self.action_client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("✅ GraspServer 已启动，等待目标并准备抓取!!")


    def target_pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id != 'base_link':
            self.get_logger().error(
                f"❌ 拒绝目标位姿：frame_id 必须是 base_link，"
                f"当前为 '{msg.header.frame_id}'"
            )
            return

        position = msg.pose.position
        orientation = msg.pose.orientation
        values = (
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("❌ 拒绝目标位姿：位置或四元数包含非有限值")
            return

        norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2 +
            orientation.z ** 2 + orientation.w ** 2
        )
        if norm < 1e-6:
            self.get_logger().error("❌ 拒绝目标位姿：四元数的模不能为 0")
            return

        # ROS 用户输入可能不是严格单位四元数，在交给 MoveIt 前统一归一化。
        orientation.x /= norm
        orientation.y /= norm
        orientation.z /= norm
        orientation.w /= norm
        # 采用“最后一次有效目标生效”的策略。收到 Pose 后清除旧 Point；
        # 后续若收到 Point，_accept_new_target() 会反向清除本 Pose。
        self.target_point = None
        self.last_target_point = None
        self.rejected_points.clear()
        self.target_pose = msg

        self.get_logger().info(
            f"🎯 获取完整目标位姿: "
            f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}), "
            f"quaternion=({orientation.x:.4f}, {orientation.y:.4f}, "
            f"{orientation.z:.4f}, {orientation.w:.4f})"
        )


    def target_point_cb(self, msg: PointStamped):
        new_point = msg.point
        now = time.time()

        # 第一次设置，直接接受
        if self.last_target_point is None:
            self._accept_new_target(new_point, now)
            return

        time_diff = now - self.last_update_time
        dist = self._distance(self.last_target_point, new_point)

        if time_diff < 3.0 and dist > 0.1:
            self.get_logger().warn(
                f"🚫 拒绝新目标点：更新间隔 {time_diff:.2f}s，位置跳变 {dist:.3f}m"
            )
            self.rejected_points.append((new_point, now))

            # 若连续被拒绝点达到4个，强行接受第4个
            if len(self.rejected_points) >= 5:
                forced_point, forced_time = self.rejected_points[-1]
                self.get_logger().warn("⚠️ 连续 5 个点被拒绝，强行接受第 5 个点为新目标点")
                self._accept_new_target(forced_point, forced_time)
                self.rejected_points.clear()

        # 否则，接受新目标点
        self._accept_new_target(new_point, now)

    def _accept_new_target(self, point, timestamp):
        # Point 必须能够覆盖之前保存的 Pose，否则 grasp_callback() 会因为
        # Pose 优先级更高而一直执行旧位姿，导致新发布的 Point 实际不生效。
        if self.target_pose is not None:
            self.get_logger().info("🔄 新目标点已覆盖此前的完整目标位姿")
        self.target_pose = None
        self.target_point = point
        self.last_target_point = point
        self.last_update_time = timestamp
        self.get_logger().info(
            f"🎯 获取目标点: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}"
        )

    def _distance(self, p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )
        
    def grasp_callback(self, request, response):
        if self.target_pose is None and self.target_point is None:
            self.get_logger().warn("⚠️ 尚未接收到目标点或目标位姿，无法执行抓取")
            response.success = False
            response.message = "No target received"
            return response

        self.get_logger().info("🚀 开始构造抓取目标请求...")

        # 完整位姿优先；仅收到目标点时使用单位四元数和自由姿态容差。
        has_explicit_orientation = self.target_pose is not None
        if has_explicit_orientation:
            target_pose = self.target_pose
        else:
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position = self.target_point
            target_pose.pose.orientation.w = 1.0

        # 构造约束
        constraints = Constraints()

        # 位置约束（小立方体区域）
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tcp_link"
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        # ori_constraint.orientation.w = 1.0

        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [0.01, 0.01, 0.01]
        pos_constraint.constraint_region.primitives.append(region)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        # 姿态约束（可调整容忍度）
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tcp_link"
        ori_constraint.orientation = target_pose.pose.orientation
        orientation_tolerance = 0.1 if has_explicit_orientation else math.pi
        ori_constraint.absolute_x_axis_tolerance = orientation_tolerance
        ori_constraint.absolute_y_axis_tolerance = orientation_tolerance
        ori_constraint.absolute_z_axis_tolerance = orientation_tolerance
        ori_constraint.weight = 1.0

        # # 保持 tcp_link 姿态平行于地面（允许绕 Z 自由旋转）
        # ori_constraint = OrientationConstraint()
        # ori_constraint.header.frame_id = "base_link"
        # ori_constraint.link_name = "tcp_link"

        # # 设置目标方向（例如 Z 轴朝上，无旋转）
        # ori_constraint.orientation.x = 0.0
        # ori_constraint.orientation.y = 0.0
        # ori_constraint.orientation.z = 0.0
        # ori_constraint.orientation.w = 1.0  # 即无旋转四元数

        # # # 设置容忍度
        # ori_constraint.absolute_x_axis_tolerance = 0.1
        # ori_constraint.absolute_y_axis_tolerance = 0.1
        # ori_constraint.absolute_z_axis_tolerance = 0.1


        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        # 构造目标 Goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"  # ⬅️ 替换成你 moveit 中配置的 group 名称
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_delay = 2.0

        self.get_logger().info("📤 正在发送抓取目标给 MoveIt...")

        # 异步发送目标
        send_goal_future = self.action_client.send_goal_async(goal)

        # 将回调绑定当前 response（必须定义为 self 内成员，否则生命周期太短）
        self._current_response = response

        send_goal_future.add_done_callback(self.goal_response_callback)

        # self.send_gripper_control(0.00005, 0.03)
        # 注意：此处先返回 response（ROS2 Service 是同步接口）
        return response



    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveIt 拒绝了抓取请求")
            self._current_response.success = False
            self._current_response.message = "MoveIt rejected the goal"
            return

        self.get_logger().info("✅ MoveIt 接收成功，等待执行结果...")

        # 获取异步结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()

        if result.result.error_code.val == 1:
            self.get_logger().info("🎉 MoveIt 执行完成，抓取成功")
            self._current_response.success = True
            self._current_response.message = "Grasp executed successfully"
        else:
            self.get_logger().warn(f"⚠️ MoveIt 执行失败，错误码: {result.result.error_code.val}")
            self._current_response.success = False
            self._current_response.message = f"Failed: {result.result.error_code.val}"
        # 最后打印
        self.get_logger().info("✅ 本次服务处理完成")


    def send_gripper_control(self, angle: float, width: float):
        goal = GripperControl.Goal()
        goal.gripper_angle = angle
        goal.gripper_width = width

        self.get_logger().info(f"📤 准备控制夹爪: angle={angle:.6f} rad, width={width:.3f} m")

        # 等待服务端就绪（非阻塞）
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Gripper 控制服务器未响应")
            return

        # 异步发送目标
        send_goal_future = self.gripper_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._gripper_goal_response_callback)

    # ⬇️ 回调：处理服务器接收结果
    def _gripper_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Gripper 控制请求被拒绝")
            return

        self.get_logger().info("✅ Gripper 控制请求已接受，等待执行完成...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._gripper_result_callback)

    # ⬇️ 回调：处理执行结果
    def _gripper_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("✅ 夹爪控制成功")
        else:
            self.get_logger().error(f"❌ 夹爪控制失败: {result.message}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GraspServer())
    rclpy.shutdown()
