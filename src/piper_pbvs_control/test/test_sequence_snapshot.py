"""Tests for frozen targets: no visual reacquisition or stale-cache motion."""
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import time

import numpy as np
import pytest

from piper_pbvs_control.pbvs_controller import PiperPbvsController, TaskFailure


def node():
    n = object.__new__(PiperPbvsController)
    n.sequence_snapshot = {'key_0': ([0, 0, 0], [0, 0, 0, 1])}
    n.sequence_snapshot_created = time.monotonic()
    n.get_logger = Mock(return_value=Mock())
    return n


def test_continuation_uses_snapshot_without_reacquiring():
    n = node()
    n._prepare_sequence_snapshot = Mock()
    request = SimpleNamespace(snapshot_targets=[], sequence_continuation=True, target_name='key_0')
    assert n._sequence_target(request, None) is n.sequence_snapshot['key_0']
    n._prepare_sequence_snapshot.assert_not_called()


@pytest.mark.parametrize('name,age,continuation', [('key_1',0,True),('key_0',61,True),('key_0',0,False)])
def test_missing_expired_or_uninitialized_snapshot_is_rejected(name, age, continuation):
    n = node()
    n.sequence_snapshot_created -= age
    request = SimpleNamespace(snapshot_targets=[], sequence_continuation=continuation, target_name=name)
    with pytest.raises(TaskFailure):
        n._sequence_target(request, None)


def test_failed_preparation_never_commits_partial_snapshot():
    n = node()
    n.data_lock = threading.Lock()
    n.latest_joint_positions = [0.0] * 6
    n.latest_joint_received = time.monotonic()
    n.tcp_feedback_timeout = 0.5
    n._latest_tcp_arrays = Mock(return_value=(np.zeros(3),np.array([0,0,0,1])))
    n._set_state = Mock()
    n._select_interest = Mock()
    n._clear_target_tracking = Mock()
    n._wait_for_stable_target = Mock(side_effect=[(np.zeros(3),np.array([0,0,0,1])),TaskFailure('unseen target')])
    with pytest.raises(TaskFailure):
        n._prepare_sequence_snapshot(['key_1','key_0'],None)
    assert n.sequence_snapshot == {}
