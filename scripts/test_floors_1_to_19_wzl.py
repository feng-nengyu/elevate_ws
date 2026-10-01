#!/usr/bin/env python3
"""Execute floor goals 1..19 and time each forward-press submission."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from piper_pbvs_control.elevator_sequence import make_home_moveit_goal, home_joint_errors
from piper_msgs.action import PressButton
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FloorIntervalTest(Node):
    def __init__(self, events, intervals):
        super().__init__('wzl_floor_interval_test')
        self.events = events
        self.intervals = intervals
        self.floor = None
        self.active_goal = None
        self.last_command = None
        self.floor_commands = []
        self.joints = None
        self.joint_received = 0.0
        self.action_status = {}
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        for name in ('/press_button', '/run_elevator_sequence', '/move_action', '/execute_trajectory', '/arm_controller/follow_joint_trajectory'):
            self.create_subscription(GoalStatusArray, name+'/_action/status', lambda msg, action=name: self.action_status.__setitem__(action,[entry.status for entry in msg.status_list]), qos)
        self.client = ActionClient(self, PressButton, '/run_elevator_sequence')
        self.create_subscription(
            String, '/pbvs/press_reached', self.on_transition, 20
        )
        self.create_subscription(String, '/pbvs/segment_timing', self.on_segment_timing, 50)

    def on_joints(self, message):
        self.joints = message
        self.joint_received = time.monotonic()

    def initialize_before_test(self):
        """Call /initialize_arm and require success before any test goal."""
        client = self.create_client(Trigger, '/initialize_arm')
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('/initialize_arm服务不可用，停止测试')
            print('测试前：调用 /initialize_arm…', flush=True)
            self.record('initialization_requested')
            response = self.wait(client.call_async(Trigger.Request()), 40.0)
            if response is None:
                raise RuntimeError('初始化结果为空，停止测试')
            self.record('initialization_result', success=response.success, message=response.message)
            print(f'初始化：success={response.success}，{response.message}', flush=True)
            if not response.success:
                raise RuntimeError(f'初始化失败，停止测试：{response.message}')
        finally:
            self.destroy_client(client)

    def return_ready_before_test(self):
        home = self.params('/elevator_sequence', [
            'home_joint_positions', 'home_joint_tolerance',
            'home_velocity_scaling_factor', 'home_acceleration_scaling_factor',
            'move_group_name',
        ])
        # A fresh measured Ready pose needs no additional MoveIt motion.
        deadline = time.monotonic()+2.0
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.1)
            if self.joints is not None and time.monotonic()-self.joint_received<=.5:
                errors = home_joint_errors(self.joints,home['home_joint_positions'])
                if errors and max(errors)<home['home_joint_tolerance']+.001:
                    if any(status in (1,2,3) for states in self.action_status.values() for status in states):
                        raise RuntimeError('已有动作运行，停止测试')
                    self.record('initial_home_verified',max_joint_error_rad=max(errors),motion_skipped=True)
                    print('测试前：实测已在 Ready，跳过重复回位',flush=True)
                    return
                break
        expected = {'/press_button','/run_elevator_sequence','/move_action','/execute_trajectory','/arm_controller/follow_joint_trajectory'}
        end = time.monotonic()+5.0
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
            if expected.issubset(self.action_status):
                if any(status in (1,2,3) for states in self.action_status.values() for status in states):
                    raise RuntimeError('已有动作运行，停止测试前回Ready')
                break
        else:
            raise RuntimeError(f'机械臂未通过Ready验收且无法确认运动空闲；未收到状态：{sorted(expected-set(self.action_status))}；先调用initialize_arm或在RViz回Ready后重试')
        if not self.move_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt不可用，无法规划回Ready')
        print('测试前：规划回 Ready…',flush=True)
        goal = make_home_moveit_goal(home['home_joint_positions'],min(.003,home['home_joint_tolerance']),home['move_group_name'],False,home['home_velocity_scaling_factor'],home['home_acceleration_scaling_factor'])
        submission = self.move_client.send_goal_async(goal)
        try:
            handle = self.wait(submission,8)
        except BaseException:
            submission.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result() is not None and f.result().accepted else None)
            raise
        if handle is None or not handle.accepted:
            raise RuntimeError('测试前回Ready被拒绝；确认initialize_arm已成功')
        self.active_goal = handle
        response = self.wait(handle.get_result_async(),45)
        self.active_goal = None
        self.record('initial_home_result',status=response.status,error_code=response.result.error_code.val)
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != 1:
            raise RuntimeError('测试前回Ready失败，停止后续测试')
        end = time.monotonic()+3
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.joints is not None and time.monotonic()-self.joint_received<=.5:
                errors = home_joint_errors(self.joints,home['home_joint_positions'])
                if errors and max(errors)<home['home_joint_tolerance']+.001:
                    self.record('initial_home_verified',max_joint_error_rad=max(errors))
                    print('测试前：已回 Ready，实测验收通过',flush=True)
                    return
        raise RuntimeError('测试前Ready实测验收失败，停止后续测试')

    def on_segment_timing(self, message):
        self.record('segment_timing', **json.loads(message.data))

    def record(self, kind, **fields):
        entry = dict(time=datetime.now().astimezone().isoformat(),
                     floor=self.floor, kind=kind)
        entry.update(fields)
        self.events.write(json.dumps(entry, ensure_ascii=False) + '\n')
        self.events.flush()

    def on_transition(self, message):
        try:
            event = json.loads(message.data)
            stamp = int(event['monotonic_ns'])
            if (event.get('stage') != 'panel-normal movement' or not event.get('planned')
                    or event.get('reference') != 'measured_tcp_press_target_verified'):
                raise ValueError('unexpected transition event')
            if self.floor is None:
                raise ValueError('transition received outside active floor goal')
            if self.last_command and stamp <= self.last_command['monotonic_ns']:
                raise ValueError('transition timestamp did not advance')
        except (ValueError, KeyError, TypeError) as error:
            self.record('invalid_transition', message=message.data, error=str(error))
            return
        event['floor'] = self.floor
        event['received_monotonic_ns'] = time.monotonic_ns()
        self.floor_commands.append(event)
        self.record('press_reached', **event)
        if self.last_command:
            previous = self.last_command
            interval = (stamp - previous['monotonic_ns']) / 1e9
            row = {
                'previous_floor': previous['floor'],
                'previous_target': previous['target'],
                'floor': self.floor,
                'target': event['target'],
                'previous_command_monotonic_ns': previous['monotonic_ns'],
                'command_monotonic_ns': stamp,
                'interval_s': f'{interval:.6f}',
                'over_3s': interval > 3.0,
            }
            self.intervals.writerow(row)
            self.record('interval', **row)
            print(f"按键间隔 {previous['floor']}/{previous['target']} → "
                  f"{self.floor}/{event['target']}: {interval:.3f} s", flush=True)
        else:
            print(f"首个按压到位事件：{self.floor}/{event['target']}", flush=True)
        self.last_command = event

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'等待Action超时（{timeout:g}秒）')
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result()

    def params(self, node, names):
        client = self.create_client(GetParameters, node + '/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{node} 参数服务不可用')
        request = GetParameters.Request()
        request.names = names
        response = self.wait(client.call_async(request), 8.0)
        self.destroy_client(client)
        if response is None or len(response.values) != len(names):
            raise RuntimeError(f'{node} 参数读取失败')
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value',
                  4: 'string_value', 8: 'double_array_value'}
        values = {}
        for name, value in zip(names, response.values):
            if value.type not in fields:
                raise RuntimeError(f'{node}/{name} 未设置')
            item = getattr(value, fields[value.type])
            values[name] = list(item) if value.type == 8 else item
        return values

    def preflight(self, floors):
        pbvs = self.params('/piper_pbvs_controller', [
            'enable_motion', 'close_panel_sequence', 'preplan_sequence',
            'preplan_retry_attempts', 'preplan_retry_timeout_sec',
            'sequence_snapshot_acquire_timeout',
            'distance_mm', 'sequence_retract_distance_mm',
            'moveit_velocity_scaling_factor', 'moveit_acceleration_scaling_factor',
            'transition_acceleration_scaling_factor', 'transition_velocity_scaling_factor',
            'press_velocity_scaling_factor', 'retract_velocity_scaling_factor',
        ])
        sequence = self.params('/elevator_sequence', [
            'enable_motion', 'close_panel_sequence', 'home_joint_positions',
        ])
        if not all((pbvs['enable_motion'], pbvs['close_panel_sequence'],
                    pbvs['preplan_sequence'], sequence['enable_motion'],
                    sequence['close_panel_sequence'])):
            raise RuntimeError('测试要求两个控制节点都开启真机运动和连续模式，并开启整轮提前规划')
        if pbvs['distance_mm'] <= 0 or pbvs['sequence_retract_distance_mm'] <= 0:
            raise RuntimeError('测试要求正数按压行程和退回距离')
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/run_elevator_sequence Action不可用')
        # A successful floor goal does not prove that the running controller
        # contains the new timing publisher. Check before any robot movement.
        deadline = time.monotonic() + 3.0
        publishers = []
        while time.monotonic() < deadline:
            publishers = self.get_publishers_info_by_topic(
                '/pbvs/press_reached'
            )
            if any(info.node_name == 'piper_pbvs_controller'
                   for info in publishers):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise RuntimeError(
                '未发现 /pbvs/press_reached 发布者；请停止旧 launch，'
                '用已构建的 WZL 工作区重新启动并调用 /initialize_arm，'
                '再运行测试脚本'
            )
        self.record('configuration', pbvs=pbvs, sequence=sequence,
                    floors=list(floors))
        print(f"实测行程 {pbvs['distance_mm']:g} mm，退回 "
              f"{pbvs['sequence_retract_distance_mm']:g} mm；测试楼层 "
              f"{', '.join(map(str, floors))}。", flush=True)

    def execute_floor(self, floor):
        if self.last_command is not None:
            self.record('interval_discontinuity',
                        previous_floor=self.last_command['floor'],
                        next_floor=floor,
                        reason='new floor; only within-floor press intervals are measured')
            self.last_command = None
        self.floor = floor
        self.floor_commands = []
        goal = PressButton.Goal()
        goal.target_name = str(floor)
        print(f'\n测试楼层 {floor}', flush=True)
        self.record('goal_submitted', target_name=goal.target_name)
        handle = self.wait(self.client.send_goal_async(goal), 8.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(f'楼层 {floor} 任务未被接受')
        self.active_goal = handle
        response = self.wait(handle.get_result_async(), 240.0)
        self.active_goal = None
        # Drain command events queued just ahead of the Action result.
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)
        if response is None:
            raise RuntimeError(f'楼层 {floor} 结果为空')
        result = response.result
        self.record('goal_result', status=response.status,
                    success=result.success, message=result.message,
                    hard_safety_stop=result.hard_safety_stop,
                    press_reached_events=len(self.floor_commands))
        print(f'楼层 {floor}: {result.message}', flush=True)
        if response.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise RuntimeError(f'楼层 {floor} 失败；已停止后续测试，不自动回位')
        expected_targets = [f'key_{digit}' for digit in str(floor)] + ['key_ok']
        expected = len(expected_targets)
        if [event['target'] for event in self.floor_commands] != expected_targets:
            raise RuntimeError(f'楼层 {floor} 预期{expected}条按压到位事件，'
                               f'实际记录{len(self.floor_commands)}条；停止测试')
        self.floor = None

    def cancel_active(self):
        if self.active_goal is None:
            return
        try:
            self.wait(self.active_goal.cancel_goal_async(), 5.0)
            print('已请求取消当前楼层任务；请确认机械臂停止。', flush=True)
        except Exception as error:
            print(f'取消请求未确认：{error}', flush=True)


def selected_floors(start=None, floors=None):
    """Choose exactly the requested floors before creating ROS interfaces."""
    if floors is not None:
        if not floors or any(not 1 <= floor <= 19 for floor in floors):
            raise ValueError('--floors 中的楼层必须在1～19之间')
        if floors != sorted(set(floors)):
            raise ValueError('--floors 必须按升序给出，且不能重复')
        return list(floors)
    first = 1 if start is None else start
    if not 1 <= first <= 19:
        raise ValueError('--start 必须在1～19之间')
    return list(range(first, 20))


def main():
    parser = argparse.ArgumentParser(description='依次测试楼层并记录实测按压到位间隔（包含OK）')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--start', type=int,
                       help='起始楼层，1～19；例如失败在13楼时用 --start 13')
    group.add_argument('--floors', type=int, nargs='+', metavar='N',
                       help='仅测试指定楼层，如 --floors 16 19')
    args = parser.parse_args()
    try:
        floors = selected_floors(args.start, args.floors)
    except ValueError as error:
        parser.error(str(error))
    directory = Path(__file__).resolve().parents[1] / 'floor_interval_logs'
    directory.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    json_path = directory / f'{stem}.jsonl'
    csv_path = directory / f'{stem}.csv'
    print(f'事件日志：{json_path}\n间隔表：{csv_path}', flush=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    columns = ['previous_floor', 'previous_target', 'floor', 'target',
               'previous_command_monotonic_ns', 'command_monotonic_ns',
               'interval_s', 'over_3s']
    with json_path.open('w', encoding='utf-8') as events, \
            csv_path.open('w', encoding='utf-8', newline='') as table:
        writer = csv.DictWriter(table, fieldnames=columns)
        writer.writeheader()
        try:
            node = FloorIntervalTest(events, writer)
            node.preflight(floors)
            node.initialize_before_test()
            node.return_ready_before_test()
            for floor in floors:
                node.execute_floor(floor)
                table.flush()
            print(f'楼层 {", ".join(map(str, floors))} 测试完成；间隔见 {csv_path}', flush=True)
        except (Exception, KeyboardInterrupt) as error:
            code = 2
            print(f'测试停止：{error}', flush=True)
            if node is not None:
                node.record('stopped', reason=str(error))
                node.cancel_active()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
