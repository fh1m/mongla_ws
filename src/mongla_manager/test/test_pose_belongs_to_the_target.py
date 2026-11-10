"""A pose of one prop never clears a shot at another (#76).

`TargetPose` had no field saying which target it was solved for, so a valid,
fresh pose of the gate passed `square_within` for a shot at the torpedo board
just after the mission switched, and supplied that board's tool-range
correction. The label now rides correspondences -> PnP -> fuse -> VisionState,
and every geometric gate asks for the class it is acting on.
"""
from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip('mongla_interfaces')
from mongla_interfaces.msg import TargetCorrespondences, TargetPose   # noqa: E402

from mongla_manager.vision_state import VisionState                  # noqa: E402


def _pose(label, off_axis=2.0, range_m=2.0):
    m = TargetPose()
    m.ok, m.target_label = True, label
    m.off_axis_deg, m.range_m = off_axis, range_m
    return m


def _vs(pose):
    vs = VisionState(MagicMock(), camera='forward')
    with vs._lock:
        vs._pose, vs._pose_stamp = pose, time.monotonic()
    return vs


def test_a_square_pose_of_another_class_does_not_clear_the_shot():
    vs = _vs(_pose('gate'))
    assert vs.square_within(5.0, class_name='gate') is True
    assert vs.square_within(5.0, class_name='torpedo_board') is False


def test_an_unlabelled_pose_cannot_clear_a_named_shot():
    assert _vs(_pose('')).square_within(5.0, class_name='gate') is False


def test_obliquity_and_range_are_refused_for_another_class():
    vs = _vs(_pose('gate'))
    assert vs.obliquity_deg(class_name='torpedo_board') is None
    assert vs.obliquity_deg(class_name='gate') == pytest.approx(2.0)
    assert vs.target_pose(class_name='torpedo_board') is None


def test_no_class_asked_keeps_the_old_behaviour():
    assert _vs(_pose('')).square_within(5.0) is True


def test_pnp_passes_the_label_through_on_success_and_refusal():
    from mongla_localization import pnp_node
    sent = []
    fake = SimpleNamespace(_pub=SimpleNamespace(publish=sent.append), _cam='forward',
                           _K_full=None)
    msg = TargetCorrespondences()
    msg.target_label, msg.camera = 'torpedo_board', 'forward'
    fake._refuse = lambda m, why: pnp_node.PnPNode._refuse(fake, m, why)
    pnp_node.PnPNode._on_corr(fake, msg)          # no camera_info: a refusal
    assert sent and sent[-1].ok is False and sent[-1].target_label == 'torpedo_board'


def test_the_fuse_node_never_votes_across_targets():
    from mongla_localization import pose_fuse_node
    from mongla_localization.pose_cluster import PoseCluster
    sent = []
    fake = SimpleNamespace(
        _pub=SimpleNamespace(publish=sent.append),
        _cluster=PoseCluster(min_poses=2, window_s=10.0, tol_deg=5.0),
        yaw_at=lambda t: 0.0)

    def feed(label, yaw):
        m = _pose(label)
        m.yaw_deg, m.ambiguity, m.reproj_px, m.n_points = yaw, 0.1, 1.0, 30
        pose_fuse_node.PoseFuseNode._on_pose(fake, m)
        return sent[-1]

    feed('gate', 10.0)
    out = feed('torpedo_board', 10.0)            # would agree with the gate's
    assert out.target_label == 'torpedo_board'
    assert out.ok is False, 'a board pose was fused with a gate pose'
    assert feed('torpedo_board', 10.0).ok is True


def test_the_align_binds_every_gate_to_its_own_target():
    """Behaviourally: the closures the verb hands the loop ask for the class."""
    import pathlib
    import sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / 'tools'))
    from code_text import code_of_file
    src = code_of_file(pathlib.Path(__file__).resolve().parents[2]
                       / 'mongla_control' / 'mongla_control' / 'vision_verbs.py')
    for fn in ('square_within(', 'obliquity_deg(', 'tool_offset_px('):
        for i in [k for k in range(len(src)) if src.startswith('vstate.' + fn, k)]:
            assert 'class_name=target_class' in src[i:i + 120], (fn, src[i:i + 120])


def _wait_for(msg, target_class):
    import mongla_planner.dsl_heading as D
    from mongla_planner.mongla_dsl import MonglaMission
    m = MonglaMission.__new__(MonglaMission)
    m._fused_subs = {'forward': object()}
    m._fused_pose = {'forward': (msg, time.monotonic())}
    m.client = SimpleNamespace(node=None)
    real = D.rclpy.spin_once
    D.rclpy.spin_once = lambda *a, **k: None
    try:
        return MonglaMission._wait_fused_pose(m, 'forward', timeout=0.05,
                                              target_class=target_class)
    finally:
        D.rclpy.spin_once = real


def test_the_heading_anchor_takes_only_its_own_props_pose():
    """anchor_on('gate') writes the gate's surveyed bearing; a board's pose
    would put the world heading off by the angle between their faces."""
    pytest.importorskip('mongla_planner')
    board = _pose('torpedo')
    board.yaw_deg, board.n_points = 12.0, 5
    refused = _wait_for(board, 'gate')
    assert not refused.decided and "not 'gate'" in refused.reason, refused.reason
    got = _wait_for(board, 'torpedo')
    assert got is not None and got.decided and got.yaw_deg == pytest.approx(12.0)
