"""The three opt-in consumers: run budget, model provenance, outlines.

Each is EXECUTED: the budget through the real `arm()` and `RunBudget`; the two
topic consumers through real rclpy publishers, the way the detector sends them.
Each also pins its OPT-OUT -- not calling it must leave behaviour unchanged.
"""
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from vision_msgs.msg import VisionInfo
from mongla_interfaces.msg import TargetContours

from mongla_planner.mongla_dsl import MonglaMission, _pick_outline
from mongla_planner.run_budget import RunBudget


# --------------------------------------------------------------------------- #
#  run budget                                                                  #
# --------------------------------------------------------------------------- #

def _mission(arm_ok=True):
    m = MagicMock()
    m._budget = None
    m._scoreboard = []
    m._send.return_value = SimpleNamespace(success=arm_ok, message='' if arm_ok else 'no')
    return m


def test_no_budget_means_every_task_is_attempted():
    m = _mission()
    v = MonglaMission.worth_attempting(m, 'torpedo', points=300, worst_case_s=9999)
    assert v.attempt and v.mode == 'full'


def test_the_clock_starts_on_a_successful_arm_and_rations():
    m = _mission()
    clock = {'t': 100.0}
    MonglaMission.use_budget(m, 200.0, reserve_s=40.0)
    m._budget = RunBudget(200.0, reserve_s=40.0, now=lambda: clock['t'])
    assert not m._budget.started
    MonglaMission.arm(m)
    assert m._budget.started
    clock['t'] += 100.0                                  # 60 s usable left
    full = MonglaMission.worth_attempting(m, 'a', points=10, worst_case_s=50)
    fb = MonglaMission.worth_attempting(m, 'b', points=300, worst_case_s=120,
                                        fallback_s=30, fallback_points=100)
    skip = MonglaMission.worth_attempting(m, 'c', points=300, worst_case_s=120)
    assert (full.mode, fb.mode, skip.mode) == ('full', 'fallback', 'skip')
    assert [e['cmd'] for e in m._scoreboard] == ['budget:a', 'budget:b', 'budget:c']


def test_a_failed_arm_does_not_start_the_clock():
    m = _mission(arm_ok=False)
    MonglaMission.use_budget(m, 900.0)
    with pytest.raises(Exception):
        MonglaMission.arm(m)
    assert not m._budget.started


# --------------------------------------------------------------------------- #
#  topic consumers                                                             #
# --------------------------------------------------------------------------- #

@pytest.fixture
def node():
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = Node('opt_in_probe')
    yield n
    n.destroy_node()
    if started and rclpy.ok():
        rclpy.shutdown()


def _dsl(node):
    m = MagicMock()
    m.__dict__['camera'] = 'forward'
    m.client = SimpleNamespace(node=node)
    m._det_cache = {'forward': (time.monotonic(), [('gate', 1, 1, 1, 1, 0.9)])}
    m._det_seen = {'forward': {'gate': time.monotonic()}}
    m.log = MagicMock()
    m.active_models = lambda cam=None, timeout=1.0: MonglaMission.active_models(m, cam, timeout=timeout)
    return m


def _contours(*items):
    msg = TargetContours()
    msg.camera, msg.image_width, msg.image_height = 'forward', 640, 480
    off, pts = [0], []
    for name, score, angle, area, poly in items:
        msg.class_name.append(name)
        msg.score.append(score)
        msg.angle_deg.append(angle)
        msg.area_px.append(area)
        for x, y in poly:
            pts += [x, y]
        off.append(len(pts) // 2)
    msg.offset, msg.points = off, pts
    return msg


def test_outline_picks_the_largest_of_the_class_and_decodes_its_points():
    msg = _contours(('pipe', 0.9, 10, 400, [(0, 0), (20, 0), (20, 20), (0, 20)]),
                    ('gate', 0.8, 0, 900, [(1, 1), (31, 1), (31, 31)]),
                    ('pipe', 0.7, -35, 2500, [(5, 5), (55, 5), (55, 55), (5, 55), (0, 30)]))
    o = _pick_outline(msg, 'PIPE')
    assert (o.area_px, o.angle_deg, len(o.points)) == (2500, -35, 5)
    assert o.points[0] == (5, 5)
    assert _pick_outline(msg, 'drum') is None


def test_outline_reads_the_live_topic_and_goes_stale(node):
    pub = node.create_publisher(TargetContours, '/mongla/vision/forward/contours',
                                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    m = _dsl(node)
    assert MonglaMission.outline(m, 'gate', timeout=0.05) is None     # nothing sent
    msg = _contours(('gate', 0.8, 5, 900, [(1, 1), (31, 1), (31, 31)]))
    for _ in range(40):
        pub.publish(msg)
        got = MonglaMission.outline(m, 'gate', timeout=0.05)
        if got:
            break
    assert got and got.angle_deg == 5
    ts, frame = m._contours['forward']
    m._contours['forward'] = (ts - 10.0, frame)
    assert MonglaMission.outline(m, 'gate', stale_after=1.0, timeout=0.0) is None


def _latched(node):
    return node.create_publisher(VisionInfo, '/mongla/vision/forward/vision_info',
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))


def _info(stems, epoch):
    v = VisionInfo()
    v.database_location, v.database_version = stems, epoch
    return v


def test_active_models_reads_the_latched_announcement(node):
    pub = _latched(node)
    pub.publish(_info('gate_rescue_repair,gate_seg', 3))
    m = _dsl(node)
    assert MonglaMission.active_models(m, 'forward', timeout=2.0) == \
        (('gate_rescue_repair', 'gate_seg'), 3)


def test_confirm_waits_for_the_new_model_then_drops_pre_switch_detections(node):
    pub = _latched(node)
    pub.publish(_info('old_model', 4))
    m = _dsl(node)
    before = MonglaMission.active_models(m, 'forward', timeout=2.0)
    with pytest.raises(RuntimeError):                    # never announced
        MonglaMission._confirm_model(m, 'forward', 'new_model', before, 0.3)
    assert 'forward' in m._det_cache                     # nothing flushed on failure
    pub.publish(_info('new_model', 1))                   # a RESTARTED detector: epoch reset
    MonglaMission._confirm_model(m, 'forward', 'new_model', before, 2.0)
    assert 'forward' not in m._det_cache and 'forward' not in m._det_seen


def test_confirming_the_model_already_live_returns_at_once_and_keeps_detections(node):
    pub = _latched(node)
    pub.publish(_info('gate_rescue_repair', 9))
    m = _dsl(node)
    before = MonglaMission.active_models(m, 'forward', timeout=2.0)
    t0 = time.monotonic()
    MonglaMission._confirm_model(m, 'forward', 'gate_rescue_repair', before, 5.0)
    assert time.monotonic() - t0 < 1.0
    assert 'forward' in m._det_cache


def test_set_model_without_confirm_never_waits_or_flushes():
    m = MagicMock()
    m.camera = 'forward'
    m._detector_node.return_value = '/mongla_detector_forward'
    MonglaMission.set_model(m, 'gate')
    m.active_models.assert_not_called()
    m._confirm_model.assert_not_called()


def test_side_on_with_an_outline_uses_the_symbols_visible_pixels():
    """A placard half occluded: its BOX straddles the gate midline ('centre'),
    its visible outline sits right of it."""
    from mongla_planner.mongla_dsl import _outline_view
    from mongla_vision.identity import identify
    import mongla_planner.mongla_dsl as d
    Outline = SimpleNamespace
    o = Outline(class_name='rescue', score=0.9, angle_deg=0, area_px=100,
                points=[(55, 40), (70, 40), (70, 60), (55, 60)])
    view = _outline_view(o)
    gate = d._DetView(('gate', 50.0, 50.0, 100.0, 100.0, 0.9))
    box = d._DetView(('rescue', 50.0, 50.0, 40.0, 20.0, 0.9))     # centred box
    assert identify(gate, [box]).side == 'centre'
    assert identify(gate, [view]).side == 'right'


def test_side_on_without_the_flag_never_reads_outlines():
    m = MagicMock()
    m._records.return_value = [('gate', 50.0, 50.0, 100.0, 100.0, 0.9),
                               ('rescue', 70.0, 50.0, 20.0, 20.0, 0.9)]
    m.camera = 'forward'
    del m._resolve_camera
    assert MonglaMission.side_on(m, 'rescue') == 'right'
    m.outline.assert_not_called()
