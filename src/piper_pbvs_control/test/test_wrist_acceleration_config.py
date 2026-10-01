"""Validate opt-in per-joint planning acceleration without moving hardware."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest

path=Path(__file__).resolve().parents[2]/'piper_moveit/piper_with_gripper_moveit/launch/move_group.launch.py'
spec=importlib.util.spec_from_file_location('wrist_move_group_launch',path)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def config():
    return SimpleNamespace(joint_limits={'robot_description_planning':{'joint_limits':{
        f'joint{i}':dict(has_acceleration_limits=False,max_acceleration=0,max_velocity=3 if i==6 else 5)
        for i in range(1,8)
    }}})


def test_override_only_changes_wrist_acceleration():
    c=config();original=copy.deepcopy(c.joint_limits)
    m.apply_wrist_acceleration_limit(c,2.5)
    limits=c.joint_limits['robot_description_planning']['joint_limits']
    for i in range(1,8):
        name=f'joint{i}'
        if i in (4,6):
            assert limits[name]['has_acceleration_limits']
            assert limits[name]['max_acceleration']==2.5
            assert limits[name]['max_velocity']==original['robot_description_planning']['joint_limits'][name]['max_velocity']
        else:
            assert limits[name]==original['robot_description_planning']['joint_limits'][name]


def test_zero_restores_legacy_configuration():
    c=config();original=copy.deepcopy(c.joint_limits)
    m.apply_wrist_acceleration_limit(c,0)
    assert c.joint_limits==original


@pytest.mark.parametrize('limit',[-1,3.1,float('nan'),float('inf')])
def test_invalid_limits_rejected(limit):
    with pytest.raises(ValueError):m.apply_wrist_acceleration_limit(config(),limit)
