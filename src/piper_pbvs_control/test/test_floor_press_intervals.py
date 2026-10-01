"""Validate command timestamp intervals include the first digit and OK."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock
from std_msgs.msg import String
spec=importlib.util.spec_from_file_location('floor_script',Path(__file__).resolve().parents[3]/'scripts/test_floors_1_to_19_wzl.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_digit_to_ok_is_measured_from_source_timestamps():
    n=object.__new__(m.FloorIntervalTest)
    n.floor=8;n.floor_commands=[];n.last_command=None;n.record=Mock();n.intervals=Mock()
    for target,stamp in [('key_8',1_000_000_000),('key_ok',3_500_000_000)]:
        n.on_transition(String(data=json.dumps(dict(target=target,monotonic_ns=stamp,stage='panel-normal movement',planned=True,reference='measured_tcp_press_target_verified'))))
    row=n.intervals.writerow.call_args.args[0]
    assert row['previous_target']=='key_8' and row['target']=='key_ok'
    assert row['interval_s']=='2.500000'
    assert not row['over_3s']
    assert len(n.floor_commands)==2


def test_already_ready_skips_moveit_even_without_initial_status(monkeypatch):
    import time
    from sensor_msgs.msg import JointState
    n=object.__new__(m.FloorIntervalTest)
    target=[-.5,.2,-.3,.1,.2,.0]
    n.params=Mock(return_value=dict(home_joint_positions=target,home_joint_tolerance=.012))
    n.joints=JointState(name=[f'joint{i}' for i in range(1,7)],position=target)
    n.joint_received=time.monotonic();n.action_status={};n.record=Mock();n.move_client=Mock()
    monkeypatch.setattr(m.rclpy,'spin_once',lambda *args,**kwargs:None)
    n.return_ready_before_test()
    n.move_client.send_goal_async.assert_not_called()
    assert n.record.call_args.kwargs['motion_skipped']


def test_initialization_failure_stops_test():
    from types import SimpleNamespace
    import pytest
    n=object.__new__(m.FloorIntervalTest)
    client=Mock();client.wait_for_service.return_value=True
    n.create_client=Mock(return_value=client);n.destroy_client=Mock();n.record=Mock()
    n.wait=Mock(return_value=SimpleNamespace(success=False,message='arm not enabled'))
    with pytest.raises(RuntimeError,match='初始化失败'):
        n.initialize_before_test()
    client.call_async.assert_called_once()
    n.destroy_client.assert_called_once_with(client)


def test_initialization_success_is_recorded():
    from types import SimpleNamespace
    n=object.__new__(m.FloorIntervalTest)
    client=Mock();client.wait_for_service.return_value=True
    n.create_client=Mock(return_value=client);n.destroy_client=Mock();n.record=Mock()
    n.wait=Mock(return_value=SimpleNamespace(success=True,message='ready'))
    n.initialize_before_test()
    assert n.record.call_args.args==('initialization_result',)
    assert n.record.call_args.kwargs['success'] is True


def test_planning_selection_does_not_conflict_with_log_kind():
    n=object.__new__(m.FloorIntervalTest);n.record=Mock()
    n.on_segment_timing(String(data=json.dumps(dict(kind='planning_selection',target='key_3',candidate_durations_s=[2.7,1.6],selected_duration_s=1.6))))
    assert n.record.call_args.args==('segment_timing',)
    assert n.record.call_args.kwargs['timing_kind']=='planning_selection'
    assert 'kind' not in n.record.call_args.kwargs


def test_measured_tcp_trace_is_recorded_at_most_ten_hz(monkeypatch):
    from geometry_msgs.msg import PoseStamped
    n=object.__new__(m.FloorIntervalTest)
    n.floor=13;n.last_tcp_record_ns=0;n.record=Mock()
    clock=iter([1_000_000_000,1_050_000_000,1_110_000_000])
    monkeypatch.setattr(m.time,'monotonic_ns',lambda:next(clock))
    msg=PoseStamped();msg.header.frame_id='base_link';msg.pose.position.x=.25;msg.pose.orientation.w=1.
    n.on_tcp_pose(msg);n.on_tcp_pose(msg);n.on_tcp_pose(msg)
    assert n.record.call_count==2
    assert n.record.call_args.args==('measured_tcp_trace',)
    assert n.record.call_args.kwargs['position_m']==[.25,0.,0.]
    assert n.record.call_args.kwargs['frame_id']=='base_link'


def test_tcp_trace_is_not_recorded_outside_floor_test(monkeypatch):
    from geometry_msgs.msg import PoseStamped
    n=object.__new__(m.FloorIntervalTest)
    n.floor=None;n.last_tcp_record_ns=0;n.record=Mock()
    monkeypatch.setattr(m.time,'monotonic_ns',lambda:1_000_000_000)
    n.on_tcp_pose(PoseStamped())
    n.record.assert_not_called()


def test_wait_timeout_identifies_parameter_read_instead_of_action(monkeypatch):
    import pytest
    n=object.__new__(m.FloorIntervalTest)
    future=Mock();future.done.return_value=False
    clock=iter([0.,21.])
    monkeypatch.setattr(m.time,'monotonic',lambda:next(clock))
    with pytest.raises(TimeoutError,match='读取 /move_group 参数超时（20秒）'):
        n.wait(future,20,description='读取 /move_group 参数')


def test_parameter_timeout_cancels_read_and_destroys_client():
    import pytest
    n=object.__new__(m.FloorIntervalTest)
    client=Mock();client.wait_for_service.return_value=True
    future=Mock();client.call_async.return_value=future
    n.create_client=Mock(return_value=client);n.destroy_client=Mock()
    n.wait=Mock(side_effect=TimeoutError('parameter timeout'))
    with pytest.raises(TimeoutError):n.params('/move_group',['robot_description'])
    assert n.wait.call_args.args[1]==20.0
    assert n.wait.call_args.kwargs['description']=='读取 /move_group 参数'
    future.cancel.assert_called_once()
    n.destroy_client.assert_called_once_with(client)
