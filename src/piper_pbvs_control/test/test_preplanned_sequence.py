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


def test_repeated_digits_are_planned_in_order_before_any_execution():
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
    n._wait_planned_action=Mock(return_value=SimpleNamespace(planned_trajectory=trajectory))
    n._final_moveit_arm_target=Mock(return_value=[0.0]*6)
    n._preplan_snapshot(['key_1','key_1','key_ok'],None)
    assert [name for name,_ in n.preplanned_buttons]==['key_1','key_1','key_ok']
    assert n._wait_planned_action.call_count==9
    n.execute_trajectory_client.send_goal_async.assert_not_called()


def test_preplan_failure_retries_twice_from_same_goal_without_execution():
    n = object.__new__(PiperPbvsController)
    n.preplan_retry_attempts = 2
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
    n.move_group_client = Mock()
    n.moveit_timeout = 20
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
