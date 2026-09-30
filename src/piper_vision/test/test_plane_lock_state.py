"""Tests for task-scoped panel lock state management."""

from collections import deque
import threading

import numpy as np

from piper_vision.yolo_detect_3d import Yolo11RgbdNode


def _bare_node():
    node = object.__new__(Yolo11RgbdNode)
    node.plane_state_lock = threading.Lock()
    node.plane_interest = 'key_3'
    node.plane_candidates = deque(maxlen=5)
    node.plane_candidates.append((np.ones(3), np.ones(3)))
    node.locked_plane_point = np.array([0.4, 0.0, 0.2])
    node.locked_press_axis = np.array([1.0, 0.0, 0.0])
    return node


def test_interest_reset_discards_previous_task_plane():
    """Selecting a target, including the same name, must reset its lock."""
    node = _bare_node()

    node._reset_plane_lock('key_3')

    assert node.plane_interest == 'key_3'
    assert list(node.plane_candidates) == []
    assert node.locked_plane_point is None
    assert node.locked_press_axis is None


def test_locked_panel_snapshot_is_target_scoped_and_copied():
    """A worker must not mutate lock state through a returned snapshot."""
    node = _bare_node()

    assert node._locked_panel_snapshot('key_8') is None
    point, axis = node._locked_panel_snapshot('key_3')
    point[0] = 9.0
    axis[0] = 0.0

    assert node.locked_plane_point[0] == 0.4
    assert node.locked_press_axis[0] == 1.0
