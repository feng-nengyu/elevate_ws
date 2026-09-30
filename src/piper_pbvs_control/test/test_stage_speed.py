"""Verify fast transitions do not accelerate retreat commands."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from piper_pbvs_control.pbvs_controller import PiperPbvsController

@pytest.mark.parametrize('stage,expected', [('button transition',.20),('panel-normal movement',.17),('panel retract',.07),('coarse approach',.15)])
def test_independent_stage_speeds(stage, expected):
    node=object.__new__(PiperPbvsController)
    node.get_logger=Mock(return_value=Mock())
    for prefix,value in [('transition',.20),('press',.17),('retract',.07)]:
        setattr(node,prefix+'_velocity_scaling_factor',value)
        setattr(node,prefix+'_acceleration_scaling_factor',value)
    goal=SimpleNamespace(request=SimpleNamespace(max_velocity_scaling_factor=.15,max_acceleration_scaling_factor=.15))
    node._apply_stage_speed(goal,stage)
    assert goal.request.max_velocity_scaling_factor == expected
    assert goal.request.max_acceleration_scaling_factor == expected
