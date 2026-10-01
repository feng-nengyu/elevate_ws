"""Tests for near-panel geometry and no-motion batch planning failures."""
import threading
import time
from unittest.mock import Mock
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive

import numpy as np
import pytest

from piper_pbvs_control.control_math import sequence_button_positions
from piper_pbvs_control.pbvs_controller import PiperPbvsController, TaskFailure, PlanningFailure


def test_twenty_mm_retreat_and_near_next_approach():
    args=([0,0,0],[0,0,0,1],[0,0,0,1],.08,.067,.02,.026,.007)
    initial,press,retreat=sequence_button_positions(*args,first=True)
    next_approach,_,_=sequence_button_positions(*args,first=False)
    assert np.allclose(initial,[.026,.007,-.08])
    assert np.allclose(press,[.026,.007,-.013])
    assert np.allclose(retreat,[.026,.007,-.033])
    assert np.allclose(next_approach,retreat)
    assert np.isclose(np.linalg.norm(retreat-press),.02)


@pytest.mark.parametrize('advance,retract', [(.067,0),(.067,.101),(-.01,.02)])
def test_invalid_short_retreat_geometry_rejected(advance,retract):
    with pytest.raises(ValueError):
        sequence_button_positions([0,0,0],[0,0,0,1],[0,0,0,1],.08,advance,retract,0,0,True)


def test_entire_batch_failure_commits_no_trajectory():
    n=object.__new__(PiperPbvsController)
    n.sequence_snapshot={'key_1':(np.zeros(3),np.array([0,0,0,1]))}
    n.data_lock=threading.Lock()
    n.latest_joint_positions=[0]*6
    n.latest_joint_received=time.monotonic()
    n.sequence_roll_reference=np.array([0,0,0,1])
    n.coarse_standoff=.08;n.distance_m=.067;n.sequence_retract_distance_mm=20
    n.coarse_horizontal_offset=.026;n.coarse_vertical_offset=.007
    n.moveit_timeout=20
    n.preplan_retry_attempts=2
    n.preplan_retry_timeout_sec=0
    n._guard=Mock()
    n.move_group_client=Mock()
    n._apply_collision_scene=Mock();n._set_state=Mock()
    n._control_quaternion=Mock(return_value=np.array([0,0,0,1]))
    n._pose_message=Mock()
    goal=MoveGroup.Goal()
    c=Constraints();pc=PositionConstraint()
    pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.001]*3)]
    c.position_constraints=[pc];goal.request.goal_constraints=[c]
    n._moveit_goal=Mock(return_value=goal)
    n._apply_stage_speed=Mock()
    n._wait_planned_action=Mock(side_effect=TaskFailure('collision'))
    with pytest.raises(TaskFailure):
        n._preplan_snapshot(['key_1','key_1'],None)
    assert n.preplanned_buttons==[]
    assert n.preplanned_index==0


def test_out_of_order_press_cannot_execute():
    n=object.__new__(PiperPbvsController)
    n.preplanned_index=0;n.preplanned_buttons=[('key_1',[])]
    n.execute_trajectory_client=Mock()
    with pytest.raises(TaskFailure):
        n._execute_preplanned_button('key_0',None)
    n.execute_trajectory_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('first_press_fails', [False, True])
def test_repeated_digits_are_planned_in_order_before_any_execution(first_press_fails):
    from types import SimpleNamespace
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    n=object.__new__(PiperPbvsController)
    n.sequence_snapshot={name:(np.zeros(3),np.array([0,0,0,1])) for name in ('key_1','key_ok')}
    n.data_lock=threading.Lock();n.latest_joint_positions=[0.0]*6
    n.latest_joint_received=time.monotonic();n.tcp_feedback_timeout=.5
    n.sequence_roll_reference=np.array([0,0,0,1])
    n.coarse_standoff=.08;n.distance_m=.067;n.sequence_retract_distance_mm=20
    n.coarse_horizontal_offset=.026;n.coarse_vertical_offset=.007;n.moveit_timeout=20
    n.preplan_retry_attempts=2
    n.preplan_retry_timeout_sec=0
    n._guard=Mock()
    n.move_group_client=Mock();n.execute_trajectory_client=Mock()
    n.base_frame='base_link';n.tcp_frame='tcp_link'
    n._apply_collision_scene=Mock();n._set_state=Mock();n.get_logger=Mock(return_value=Mock())
    n._control_quaternion=Mock(return_value=np.array([0,0,0,1]))
    n._pose_message=Mock(return_value=PoseStamped())
    def make_goal(*_):
        g=MoveGroup.Goal();c=Constraints();pc=PositionConstraint()
        pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.001]*3)]
        c.position_constraints=[pc];g.request.goal_constraints=[c]
        return g
    n._moveit_goal=Mock(side_effect=make_goal);n._apply_stage_speed=Mock()
    trajectory=RobotTrajectory();trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.0]*6)]
    planned=SimpleNamespace(planned_trajectory=trajectory)
    call_count=[0]
    def plan(*args):
        call_count[0]+=1
        if first_press_fails and call_count[0]==2:
            raise PlanningFailure('MoveIt plan-only failed with error code -2')
        return planned
    n._wait_planned_action=Mock(side_effect=plan)
    n._final_moveit_arm_target=Mock(return_value=[0.0]*6)
    n._preplan_snapshot(['key_1','key_1','key_ok'],None)
    assert [name for name,_ in n.preplanned_buttons]==['key_1','key_1','key_ok']
    assert n._wait_planned_action.call_count==(10 if first_press_fails else 8)
    assert [len(segments) for _,segments in n.preplanned_buttons]==[3,2,3]
    for index, call in enumerate(n._wait_planned_action.call_args_list):
        request=call.args[1].request
        assert request.pipeline_id == ''
        assert request.planner_id == ''

    for call in n._wait_planned_action.call_args_list:
        for constraint in call.args[1].request.path_constraints.position_constraints:
            dimensions=constraint.constraint_region.primitives[0].dimensions
            assert list(dimensions[:2]) == pytest.approx([.016,.016])
            assert dimensions[2] <= .073 + 1e-9

    n.execute_trajectory_client.send_goal_async.assert_not_called()


def test_preplan_failure_retries_twice_from_same_goal_without_execution():
    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
    n.preplan_retry_timeout_sec = 0
    n.move_group_client = Mock()
    n.moveit_timeout = 20
    n._guard = Mock()
    n.get_logger = Mock(return_value=Mock())
    completed = object()
    n._wait_planned_action = Mock(side_effect=[
        PlanningFailure('planner failed'),
        PlanningFailure('planner failed'),
        completed,
    ])
    goal = MoveGroup.Goal()
    result = n._plan_segment_with_retry(goal, None, 'key_0', 'button transition')
    assert result is completed
    assert n._wait_planned_action.call_count == 3
    for call in n._wait_planned_action.call_args_list:
        assert call.args[0] is n.move_group_client
        assert call.args[1] is not goal
        assert call.args[3] == 20
    assert n.get_logger.return_value.warning.call_count == 2


def test_preplan_failure_exhausts_retries_before_any_execution():
    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
    n.preplan_retry_timeout_sec = 0
    n.move_group_client = Mock()
    n.moveit_timeout = 20
    n._guard = Mock()
    n.get_logger = Mock(return_value=Mock())
    n._wait_planned_action = Mock(side_effect=PlanningFailure('planner failed'))
    with pytest.raises(TaskFailure, match='after 3 attempts'):
        n._plan_segment_with_retry(MoveGroup.Goal(), None, 'key_0', 'button transition')
    assert n._wait_planned_action.call_count == 3


def test_transport_or_execution_failure_is_not_retried():
    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
    n.preplan_retry_timeout_sec = 0
    n.move_group_client = Mock()
    n.moveit_timeout = 20
    n._guard = Mock()
    n._wait_planned_action = Mock(side_effect=TaskFailure('timeout or controller failure'))
    with pytest.raises(TaskFailure, match='timeout or controller failure'):
        n._plan_segment_with_retry(MoveGroup.Goal(), None, 'key_0', 'button transition')
    assert n._wait_planned_action.call_count == 1


def test_only_plan_only_result_is_retryable():
    from types import SimpleNamespace
    from moveit_msgs.action import ExecuteTrajectory

    n = object.__new__(PiperPbvsController)
    n.move_group_client = Mock()
    n.execute_trajectory_client = Mock()
    n._wait_future = Mock(return_value=True)
    n._feedback = Mock()
    result_future = Mock()
    result_future.done.return_value = True
    result_future.result.return_value = SimpleNamespace(
        status=6, result=SimpleNamespace(
            error_code=SimpleNamespace(val=-1)
        )
    )
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    submission = Mock()
    submission.result.return_value = handle
    for client in (n.move_group_client, n.execute_trajectory_client):
        client.wait_for_server.return_value = True
        client.send_goal_async.return_value = submission

    plan = MoveGroup.Goal()
    plan.planning_options.plan_only = True
    with pytest.raises(PlanningFailure, match='error code -1'):
        n._wait_planned_action(n.move_group_client, plan, None, 20)

    with pytest.raises(TaskFailure, match='planning/execution failed') as error:
        n._wait_planned_action(
            n.execute_trajectory_client, ExecuteTrajectory.Goal(), None, 20
        )
    assert not isinstance(error.value, PlanningFailure)


def test_time_budget_allows_more_than_two_replans(monkeypatch):
    from piper_pbvs_control import pbvs_controller as module

    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
    n.preplan_retry_timeout_sec = 60.0
    n.move_group_client = Mock()
    n.moveit_timeout = 20
    n._guard = Mock()
    n.get_logger = Mock(return_value=Mock())
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    completed = object()
    attempts = [0]

    def plan(*_):
        attempts[0] += 1
        clock[0] += 10.0
        if attempts[0] < 5:
            raise PlanningFailure('MoveIt plan-only failed with error code -2')
        return completed

    n._wait_planned_action = Mock(side_effect=plan)
    assert n._plan_segment_with_retry(
        MoveGroup.Goal(), None, 'key_1', 'panel-normal movement'
    ) is completed
    assert attempts[0] == 5


def test_time_budget_stops_after_sixty_seconds(monkeypatch):
    from piper_pbvs_control import pbvs_controller as module

    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
    n.preplan_retry_timeout_sec = 60.0
    n.move_group_client = Mock()
    n.moveit_timeout = 20
    n._guard = Mock()
    n.get_logger = Mock(return_value=Mock())
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    attempts = [0]

    def fail(*_):
        attempts[0] += 1
        clock[0] += 10.0
        raise PlanningFailure('MoveIt plan-only failed with error code -2')

    n._wait_planned_action = Mock(side_effect=fail)
    with pytest.raises(TaskFailure, match='60s retry budget'):
        n._plan_segment_with_retry(
            MoveGroup.Goal(), None, 'key_1', 'panel-normal movement'
        )
    assert attempts[0] == 6


def test_shortest_transition_chooses_fastest_valid_plan():
    from types import SimpleNamespace
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    def plan(seconds):
        t=RobotTrajectory();p=JointTrajectoryPoint();p.time_from_start.sec=seconds
        t.joint_trajectory.points=[p]
        return SimpleNamespace(planned_trajectory=t)
    n=object.__new__(PiperPbvsController);n.transition_plan_candidates=3
    n._guard=Mock();n.move_group_client=Mock();n.moveit_timeout=20
    n._plan_segment_with_retry=Mock(return_value=plan(4))
    short=plan(2)
    n._wait_planned_action=Mock(side_effect=[short,plan(3)])
    assert n._plan_shortest_transition(MoveGroup.Goal(),None,'key_3') is short


def test_normal_limit_accepts_4_42_degrees_within_five_degrees():
    import math
    assert math.radians(4.42)<PiperPbvsController.SNAPSHOT_NORMAL_ANGLE_LIMIT
    assert math.radians(5.1)>PiperPbvsController.SNAPSHOT_NORMAL_ANGLE_LIMIT


def test_chain_retry_discards_partial_plan_and_restores_original_start():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    initial=[0.0]*6;starts=[];attempt=[0];partial=object();completed=object()
    def chain(start,guard):
        starts.append(list(start));guard();attempt[0]+=1
        # First approach selects a bad endpoint; next attempt selects another.
        start[0]=float(attempt[0])
        if attempt[0]==1:
            raise PlanningFailure('panel-normal movement: invalid path')
        return [completed],start
    segments,end=n._plan_button_chain_with_retry(chain,initial,None,'key_1')
    assert starts==[[0.0]*6,[0.0]*6]
    assert initial==[0.0]*6
    assert segments==[completed] and partial not in segments
    assert end[0]==2.0


def test_chain_budget_is_shared_between_approach_press_and_retract(monkeypatch):
    from piper_pbvs_control import pbvs_controller as module
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=60;n.preplan_retry_attempts=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    clock=[0.0];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    calls=[]
    def chain(start,guard):
        calls.append('approach');clock[0]+=30;guard()
        calls.append('press');clock[0]+=31;guard()
        calls.append('retract')
    with pytest.raises(TaskFailure,match='exceeded 60s'):
        n._plan_button_chain_with_retry(chain,[0.0]*6,None,'key_1')
    assert calls==['approach','press']


def test_chain_transport_failure_is_not_retried():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    chain=Mock(side_effect=TaskFailure('submission timed out'))
    with pytest.raises(TaskFailure,match='submission timed out'):
        n._plan_button_chain_with_retry(chain,[0.0]*6,None,'key_1')
    chain.assert_called_once()


def test_complete_chain_selection_keeps_matching_endpoint():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=2;n.transition_plan_candidates=3
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    candidates=[(['slow'],[1.0]*6),(['fast'],[2.0]*6),(['medium'],[3.0]*6)]
    chain=Mock(side_effect=candidates)
    cost=lambda result: {'slow':4,'fast':2,'medium':3}[result[0][0]]
    assert n._plan_button_chain_with_retry(chain,[0.0]*6,None,'key_1',cost)==candidates[1]
    assert all(call.args[0]==[0.0]*6 for call in chain.call_args_list)


def test_optional_chain_failure_preserves_complete_success():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=0;n.transition_plan_candidates=3
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    complete=(['valid'],[1.0]*6)
    chain=Mock(side_effect=[complete,PlanningFailure('bad alternate press')])
    assert n._plan_button_chain_with_retry(chain,[0.0]*6,None,'key_1',lambda _:2.0)==complete


def _search_chain(stages, end):
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    segments=[]
    for stage,seconds in stages:
        t=RobotTrajectory();p=JointTrajectoryPoint();p.time_from_start.sec=int(seconds);p.time_from_start.nanosec=int((seconds-int(seconds))*1e9)
        t.joint_trajectory.points=[p]
        segments.append((t,np.zeros(3),np.array([0,0,0,1]),stage))
    return segments,[end]*6


def test_adjacent_search_keeps_slower_parent_that_has_faster_next_interval():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=0
    n.transition_plan_candidates=2;n.sequence_search_width=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    first=Mock(side_effect=[
        _search_chain([('coarse approach',1),('panel-normal movement',1),('panel retract',.4)],1),
        _search_chain([('coarse approach',2),('panel-normal movement',1),('panel retract',.5)],2),
    ])
    starts=[]
    def second(start,budget):
        starts.append(list(start));budget()
        travel=3 if start[0]==1 else 1
        return _search_chain([('button transition',travel),('panel-normal movement',.4),('panel retract',.5)],3)
    selected=n._plan_adjacent_sequence([('key_1',first),('key_3',second)],[0]*6,None)
    assert n._segment_seconds(selected[0][1][0])==2
    assert n._segment_seconds(selected[1][1][0])==1
    assert starts==[[1]*6,[1]*6,[2]*6,[2]*6]


def test_adjacent_search_can_drop_dead_end_without_losing_other_branch():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=0
    n.transition_plan_candidates=2;n.sequence_search_width=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    first=Mock(side_effect=[
        _search_chain([('coarse approach',1),('panel-normal movement',1),('panel retract',.4)],1),
        _search_chain([('coarse approach',2),('panel-normal movement',1),('panel retract',.5)],2),
    ])
    def second(start,budget):
        if start[0]==1:raise PlanningFailure('bad press branch')
        return _search_chain([('button transition',1),('panel-normal movement',.4),('panel retract',.5)],3)
    selected=n._plan_adjacent_sequence([('key_1',first),('key_3',second)],[0]*6,None)
    assert n._segment_seconds(selected[0][1][0])==2


def test_joint_profile_records_large_rotation_and_not_missing_velocity_as_zero():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    t=RobotTrajectory();t.joint_trajectory.joint_names=['joint4','joint6']
    t.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.,0.]),JointTrajectoryPoint(positions=[1.5,.1])]
    t.joint_trajectory.points[1].time_from_start.sec=2
    profiles=PiperPbvsController._trajectory_joint_diagnostics(t)
    assert profiles[0]['travel_rad']==1.5
    assert profiles[0]['peak_secant_velocity_rad_s']==.75
    assert profiles[0]['peak_planned_velocity_rad_s'] is None


def test_adjacent_search_does_not_swallow_communication_failure():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=2
    n.transition_plan_candidates=2;n.sequence_search_width=2
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    chain=Mock(side_effect=TaskFailure('transport lost'))
    with pytest.raises(TaskFailure,match='transport lost'):
        n._plan_adjacent_sequence([('key_1',chain)],[0]*6,None)
    chain.assert_called_once()


def test_adjacent_search_interval_uses_previous_retract_not_current_retract():
    n=object.__new__(PiperPbvsController)
    parent={'buttons':[('key_1',[])],'end':[1]*6,'intervals':[], 'last_retract_s':.6,'planned_total_s':4}
    candidate=_search_chain([('button transition',1.5),('panel-normal movement',.4),('panel retract',2)],2)
    result=n._extend_sequence_candidate(parent,'key_ok',candidate)
    assert result['intervals']==pytest.approx([2.5])
    assert result['last_retract_s']==2


def test_retract_selection_uses_same_press_start_and_shortest_endpoint():
    from types import SimpleNamespace
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    def result(seconds,endpoint):
        t=RobotTrajectory();p=JointTrajectoryPoint(positions=[endpoint]*6)
        p.time_from_start.sec=seconds;t.joint_trajectory.points=[p]
        return SimpleNamespace(planned_trajectory=t)
    n=object.__new__(PiperPbvsController)
    n.move_group_client=Mock();n.moveit_timeout=20;n.retract_plan_candidates=3;n._guard=Mock()
    short=result(1,2);n._wait_planned_action=Mock(side_effect=[result(3,1),short,result(2,3)])
    goal=MoveGroup.Goal();goal.request.start_state.joint_state.position=[.1]*6
    assert n._plan_shortest_retract(goal,None,'key_1',Mock()) is short
    assert all(list(call.args[1].request.start_state.joint_state.position)==[.1]*6 for call in n._wait_planned_action.call_args_list)


def test_retract_optional_failure_retains_valid_original_plan():
    from types import SimpleNamespace
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    t=RobotTrajectory();p=JointTrajectoryPoint();p.time_from_start.sec=1;t.joint_trajectory.points=[p]
    valid=SimpleNamespace(planned_trajectory=t)
    n=object.__new__(PiperPbvsController);n.move_group_client=Mock();n.moveit_timeout=20;n.retract_plan_candidates=2;n._guard=Mock()
    n._wait_planned_action=Mock(side_effect=[valid,PlanningFailure('invalid alternate')])
    assert n._plan_shortest_retract(MoveGroup.Goal(),None,'key_1',Mock()) is valid


def test_retract_budget_expiry_does_not_discard_valid_original_plan():
    from types import SimpleNamespace
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from piper_pbvs_control.pbvs_controller import PlanningBudgetExceeded
    t=RobotTrajectory();p=JointTrajectoryPoint();p.time_from_start.sec=1;t.joint_trajectory.points=[p]
    valid=SimpleNamespace(planned_trajectory=t)
    n=object.__new__(PiperPbvsController);n.move_group_client=Mock();n.moveit_timeout=20;n.retract_plan_candidates=3;n._guard=Mock()
    n._wait_planned_action=Mock(return_value=valid)
    assert n._plan_shortest_retract(MoveGroup.Goal(),None,'key_1',Mock(side_effect=PlanningBudgetExceeded('budget'))) is valid
    n._wait_planned_action.assert_called_once()


def test_selected_trace_is_bounded_and_preserves_both_endpoints():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    t=RobotTrajectory();t.joint_trajectory.joint_names=['joint4']
    for i in range(250):
        p=JointTrajectoryPoint(positions=[i/1000.0]);p.time_from_start.nanosec=i*1_000_000
        t.joint_trajectory.points.append(p)
    trace=PiperPbvsController._sample_plan_trace(t)
    assert trace['sampled_point_count']==101 and trace['original_point_count']==250
    assert trace['points'][0]['point_index']==0
    assert trace['points'][-1]['point_index']==249
    assert trace['points'][-1]['positions_rad']==[.249]
    assert trace['joint_names']==['joint4']


def test_planning_request_log_records_failure_without_swallowing_it():
    import json
    n=object.__new__(PiperPbvsController)
    n.move_group_client=Mock();n.segment_timing_pub=Mock()
    n.planning_record_context={'target':'key_3','stage':'panel-normal movement'}
    n._wait_planned_action_impl=Mock(side_effect=PlanningFailure('error code -2'))
    goal=MoveGroup.Goal();goal.planning_options.plan_only=True
    with pytest.raises(PlanningFailure,match='error code -2'):
        n._wait_planned_action(n.move_group_client,goal,None,20)
    event=json.loads(n.segment_timing_pub.publish.call_args.args[0].data)
    assert event['kind']=='planning_request' and not event['success']
    assert event['error']['type']=='PlanningFailure' and event['target']=='key_3'
    assert event['elapsed_s']>=0


def test_execution_does_not_generate_planning_request_log():
    from moveit_msgs.action import ExecuteTrajectory
    n=object.__new__(PiperPbvsController)
    n.move_group_client=Mock();n.segment_timing_pub=Mock()
    expected=object();n._wait_planned_action_impl=Mock(return_value=expected)
    assert n._wait_planned_action(Mock(),ExecuteTrajectory.Goal(),None,20) is expected
    n.segment_timing_pub.publish.assert_not_called()


def test_cached_execution_uses_immediate_stamp_and_preserves_cache():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from geometry_msgs.msg import PoseStamped
    n=object.__new__(PiperPbvsController)
    n.preplanned_index=0;n.enable_motion=True;n.data_lock=threading.Lock()
    n.latest_joint_positions=[0.0]*6;n.latest_joint_received=time.monotonic();n.tcp_feedback_timeout=.5
    t=RobotTrajectory();t.joint_trajectory.joint_names=list(n.ARM_JOINT_NAMES)
    p=JointTrajectoryPoint(positions=[0.0]*6);p.time_from_start.sec=1;t.joint_trajectory.points=[p]
    t.joint_trajectory.header.stamp.sec=123
    n.preplanned_buttons=[('key_1',[(t,np.zeros(3),np.array([0,0,0,1]),'panel retract')])]
    n._set_state=Mock();n.get_logger=Mock(return_value=Mock())
    n.desired_tcp_pub=Mock();n._pose_message=Mock(return_value=PoseStamped())
    n.execute_trajectory_client=Mock();n._wait_planned_action=Mock();n.moveit_timeout=20
    n._verify_target_pose=Mock();n.segment_timing_pub=Mock();n.sequence_retract_distance_mm=15
    n._result=Mock(return_value=object());handle=Mock()
    n._execute_preplanned_button('key_1',handle)
    sent=n._wait_planned_action.call_args.args[1].trajectory.joint_trajectory
    assert sent.header.stamp.sec==0 and sent.header.stamp.nanosec==0
    assert sent.points[0].time_from_start.sec==1
    assert t.joint_trajectory.header.stamp.sec==123
    handle.succeed.assert_called_once()


def test_retract_fast_enough_skips_optional_comparisons():
    from types import SimpleNamespace
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    t=RobotTrajectory();p=JointTrajectoryPoint();p.time_from_start.nanosec=500_000_000;t.joint_trajectory.points=[p]
    valid=SimpleNamespace(planned_trajectory=t)
    n=object.__new__(PiperPbvsController)
    n.move_group_client=Mock();n.moveit_timeout=20;n.retract_plan_candidates=3;n.planning_interval_target_sec=2.5;n._guard=Mock()
    n._wait_planned_action=Mock(return_value=valid)
    assert n._plan_shortest_retract(MoveGroup.Goal(),None,'key_1',Mock()) is valid
    n._wait_planned_action.assert_called_once()


def test_complete_fast_sequence_stops_remaining_optional_branches():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=0
    n.transition_plan_candidates=3;n.sequence_search_width=2;n.planning_interval_target_sec=2.5
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    first=Mock(side_effect=[_search_chain([('coarse approach',i),('panel-normal movement',1),('panel retract',.5)],i) for i in (1,2,3)])
    second=Mock(return_value=_search_chain([('button transition',1),('panel-normal movement',.5),('panel retract',.5)],4))
    result=n._plan_adjacent_sequence([('key_1',first),('key_ok',second)],[0]*6,None)
    assert first.call_count==3
    assert second.call_count==1
    assert len(result)==2 and [s[3] for s in result[-1][1]]==['button transition','panel-normal movement','panel retract']


def test_early_stop_does_not_accept_over_target_interval():
    n=object.__new__(PiperPbvsController)
    n.preplan_retry_timeout_sec=0;n.preplan_retry_attempts=0
    n.transition_plan_candidates=2;n.sequence_search_width=1;n.planning_interval_target_sec=2.5
    n._guard=Mock();n.get_logger=Mock(return_value=Mock())
    first=Mock(return_value=_search_chain([('coarse approach',1),('panel-normal movement',1),('panel retract',.5)],1))
    second=Mock(return_value=_search_chain([('button transition',2),('panel-normal movement',.6),('panel retract',.5)],2))
    n._plan_adjacent_sequence([('key_1',first),('key_ok',second)],[0]*6,None)
    assert second.call_count==2
