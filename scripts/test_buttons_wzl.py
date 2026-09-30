#!/usr/bin/env python3
"""Sequential zero-distance button tests with monitored MoveIt home returns."""
import argparse
import json
import math
from datetime import datetime
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from piper_msgs.action import PressButton
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from piper_pbvs_control.elevator_sequence import make_home_moveit_goal, home_joint_errors

ALIGNMENT_FAILURE = 'measured TCP failed button-frame coarse alignment after MoveIt correction'
DEFAULT = ['key_5', 'key_1', 'key_3', 'key_7', 'key_9', 'key_0', 'key_2', 'key_4', 'key_6', 'key_8', 'key_ok']

class Tester(Node):
    def __init__(self, stream, click=False):
        super().__init__('wzl_button_test')
        self.stream = stream
        self.click = click
        self.active = None
        self.target = None
        self.joints = None
        self.joint_time = 0
        self.state = None
        self.sequence_state = None
        self.action_status = {}
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        for action in ['/press_button', '/run_elevator_sequence']:
            self.create_subscription(GoalStatusArray, action + '/_action/status', lambda m, a=action: self.action_status.__setitem__(a, [entry.status for entry in m.status_list]), status_qos)
        self.press = ActionClient(self, PressButton, '/press_button')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/pbvs/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(String, '/elevator_sequence/state', lambda m: setattr(self, 'sequence_state', m.data), 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        for topic in ['/tcp_pose', '/piper_vision/button_pose', '/pbvs/desired_tcp_pose']:
            self.create_subscription(PoseStamped, topic, lambda m, t=topic: self.on_pose(t, m), 10)
        self.last_pose_log = {}

    def record(self, kind, **data):
        self.stream.write(json.dumps(dict(time=datetime.now().astimezone().isoformat(), target=self.target, kind=kind, **data), ensure_ascii=False) + '\n')
        self.stream.flush()

    def on_joints(self, m):
        self.joints, self.joint_time = m, time.monotonic()

    def on_log(self, m):
        if self.target and 'piper_pbvs_controller' in m.name:
            self.record('rosout', level=m.level, message=m.msg)
            if '模长=' in m.msg or '阶段失败' in m.msg:
                print(m.msg, flush=True)

    def on_pose(self, topic, m):
        now = time.monotonic()
        if not self.target or now - self.last_pose_log.get(topic, 0) < 0.5:
            return
        self.last_pose_log[topic] = now
        p, q = m.pose.position, m.pose.orientation
        self.record('pose', topic=topic, frame=m.header.frame_id,
                    position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w])

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > end:
                raise RuntimeError('等待动作或服务超时；停止测试')
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result()

    def params(self, node, names):
        client = self.create_client(GetParameters, node + '/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'{node} 参数服务不可用')
        request = GetParameters.Request()
        request.names = names
        values = self.wait(client.call_async(request), 8).values
        self.destroy_client(client)
        result = {}
        for name, v in zip(names, values):
            fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value', 8:'double_array_value'}
            if v.type not in fields:
                raise RuntimeError(f'{node}/{name} 未设置或类型不支持')
            value = getattr(v, fields[v.type])
            result[name] = list(value) if v.type == 8 else value
        return result

    def check(self):
        pbvs = self.params('/piper_pbvs_controller', ['distance_mm','enable_motion','coarse_correction_attempts','coarse_horizontal_offset','coarse_vertical_offset','coarse_lateral_error_min','coarse_lateral_error_max'])
        home = self.params('/elevator_sequence', ['enable_motion','home_joint_positions','home_joint_tolerance','home_velocity_scaling_factor','home_acceleration_scaling_factor','move_group_name'])
        if self.click:
            expected = dict(distance_mm=65.0, coarse_horizontal_offset=0.026,
                            coarse_vertical_offset=0.007, coarse_lateral_error_min=0.021,
                            coarse_lateral_error_max=0.032)
            for name, value in expected.items():
                if not math.isfinite(pbvs[name]) or abs(pbvs[name] - value) > 1e-8:
                    raise RuntimeError(f'实际点击参数不匹配：{name}={pbvs[name]}，预期 {value}')
            driver = self.params('/piper_ctrl_single_node', ['tcp_offset_z'])
            if abs(driver['tcp_offset_z'] - 0.1358) > 1e-8:
                raise RuntimeError('实际点击需要 tcp_offset_z=0.1358')
            self.record('driver_configuration', **driver)
        elif not math.isfinite(pbvs['distance_mm']) or abs(pbvs['distance_mm']) > 1e-9:
            raise RuntimeError('distance_mm 必须是 0；实际点击请使用 test_click_buttons_wzl.bash')
        if not pbvs['enable_motion'] or not home['enable_motion']:
            raise RuntimeError('粗定位和回位节点必须 enable_motion=true')
        if pbvs['coarse_correction_attempts'] != 0:
            raise RuntimeError('测试需要 coarse_correction_attempts=0')
        self.record('configuration', pbvs=pbvs, home=home)
        return home

    def verify_idle(self, timeout=5.0):
        # State topics are volatile and only published on transitions. A late
        # subscriber may never receive IDLE. Action status retains its last
        # sample, including an empty list when the server has no goals.
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state == 'IDLE' and self.sequence_state == 'IDLE':
                if not any(status in (1, 2, 3) for states in self.action_status.values() for status in states):
                    return
            if len(self.action_status) == 2:
                busy = {name: states for name, states in self.action_status.items()
                        if any(status in (1, 2, 3) for status in states)}
                if not busy:
                    return
        raise RuntimeError(f'无法确认动作空闲：{self.action_status}；请确认没有其他任务')

    def verify_home(self, home):
        last_errors = None
        end = time.monotonic() + 5
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joints is not None and time.monotonic() - self.joint_time < 0.5:
                errors = home_joint_errors(self.joints, home['home_joint_positions'])
                last_errors = errors
                if errors and max(errors) < home['home_joint_tolerance'] + 0.001:
                    self.record('home_verified', max_joint_error_rad=max(errors))
                    return
        self.record('home_verification_failed', joint_errors_rad=last_errors, tolerance_rad=home['home_joint_tolerance'])
        raise RuntimeError(f'未通过 Ready 回位验收；关节误差={last_errors}；请在 RViz 规划回位后再运行')

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Action 服务不可用')
        # A late acceptance is tracked so cleanup can cancel it too.
        submission = client.send_goal_async(goal, feedback_callback=self.feedback if client is self.press else None)
        def accepted(future):
            handle = future.result()
            if handle and handle.accepted:
                self.active = handle
        submission.add_done_callback(accepted)
        handle = self.wait(submission, 8)
        if not handle or not handle.accepted:
            raise RuntimeError('Action 目标被拒绝')
        self.active = handle
        result = self.wait(handle.get_result_async(), timeout)
        self.active = None
        return result

    def feedback(self, message):
        f = message.feedback
        self.record('feedback', state=f.state, position_error_m=f.position_error_m, angular_error_rad=f.angular_error_rad, target_age_s=f.target_age_s)

    def cancel_active(self):
        if self.active:
            try:
                self.wait(self.active.cancel_goal_async(), 5)
                print('已请求取消当前动作；请确认机械臂已停止。', flush=True)
            except Exception as e:
                print(f'取消未确认：{e}；请使用物理急停停止异常运动。', flush=True)


def main():
    parser = argparse.ArgumentParser(description='逐键测试与监测；默认只允许零按压距离，--click 使用已确认的65毫米行程')
    parser.add_argument('targets', nargs='*', default=DEFAULT)
    parser.add_argument('--click', action='store_true', help='实际点击模式；强制核对65mm和26/7mm补偿')
    parser.add_argument('--pause', type=float, default=None, help='到位后观察秒数，粗定位默认5，点击默认1')
    args = parser.parse_args()
    if args.pause is None:
        args.pause = 1.0 if args.click else 5.0
    if any(t not in DEFAULT for t in args.targets) or not math.isfinite(args.pause) or args.pause < 0:
        parser.error('按钮必须是 key_0..key_9 或 key_ok；pause 必须为非负有限数')
    out = Path(__file__).resolve().parents[1] / ('button_click_logs' if args.click else 'button_test_logs') / datetime.now().strftime('%Y%m%d_%H%M%S_%f.jsonl')
    out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    with out.open('w', encoding='utf-8') as stream:
        try:
            node = Tester(stream, click=args.click)
            node.record('test_mode', actual_click=args.click, targets=args.targets)
            if args.click:
                print('实际点击模式：每键推进65mm，成功后回Ready；任何失败立即停止。', flush=True)
            print(f'日志：{out}', flush=True)
            home = node.check()
            node.verify_home(home)
            node.verify_idle()
            for target in args.targets:
                home = node.check()
                node.verify_home(home)
                node.verify_idle()
                node.target = target
                print(f'\n测试 {target}', flush=True)
                goal = PressButton.Goal()
                goal.target_name = target
                result = node.action(node.press, goal, 90)
                message = result.result.message
                node.record('result', status=result.status, success=result.result.success, message=message)
                print(f'{target}: {message}', flush=True)
                if getattr(result.result, 'hard_safety_stop', False):
                    raise RuntimeError('硬安全停止；不自动回位')
                if not (result.status == 4 and result.result.success):
                    if args.click or result.status != 6 or message != ALIGNMENT_FAILURE:
                        raise RuntimeError('非预期动作失败；不自动继续或回位')
                end = time.monotonic() + args.pause
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                # Recheck zero distance and idle status before the next motion.
                home = node.check()
                node.verify_idle()
                home_goal = make_home_moveit_goal(home['home_joint_positions'], min(0.003, home['home_joint_tolerance']), home['move_group_name'], False, home['home_velocity_scaling_factor'], home['home_acceleration_scaling_factor'])
                returned = node.action(node.move, home_goal, 45)
                node.record('home_result', status=returned.status, error_code=returned.result.error_code.val)
                if returned.status != 4 or returned.result.error_code.val != 1:
                    raise RuntimeError('MoveIt 回位失败，停止后续按钮')
                node.verify_home(home)
                print('已回 Ready', flush=True)
                node.target = None
            print(f'测试完成：{out}', flush=True)
        except (Exception, KeyboardInterrupt) as e:
            code = 2
            print(f'测试停止：{e}', flush=True)
            if node:
                node.record('stopped', reason=str(e))
                node.cancel_active()
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return code

if __name__ == '__main__':
    raise SystemExit(main())
