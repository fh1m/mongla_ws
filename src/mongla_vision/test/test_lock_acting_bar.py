"""The ladder must not act on a detection it would not bet the hull on.

⛔ B-59. `lock_node._on_det` accepted the highest-scoring box with `score > 0`
as THE target, at full authority. Measured on real Mirpur footage,
`gate_rescue_repair` claims a `gate` on **90.8 % of gate-free frames** at conf
0.15 -- half-frame boxes over empty turquoise at 0.44-0.56, confirmed by
rendering them. So on open water the ladder locked onto nothing: the follower
seeded on it, the anchor enrolled against it, and the vision verbs drove the
hull at it.

⚠ WHY THE BAR IS HERE AND NOT ONLY IN THE DETECTOR. The detector's `conf` is
deliberately permissive because the tracker needs low-scoring boxes to keep
association across a gap. What may be ASSOCIATED and what may be ACTED ON are
different questions; conflating them turned a 0.15 detector bar into a 0.15
control bar.

These tests drive `_on_det` directly rather than standing up a node, so they
exercise the shipped acceptance logic without a ROS graph.
"""
from __future__ import annotations

import types

import pytest

pytest.importorskip('vision_msgs')
from vision_msgs.msg import (Detection2D, Detection2DArray,        # noqa: E402
                             ObjectHypothesisWithPose)


def _det(score: float, name: str = 'gate', cx: float = 320.0,
         cy: float = 240.0, size: float = 80.0) -> Detection2D:
    d = Detection2D()
    h = ObjectHypothesisWithPose()
    h.hypothesis.class_id = name
    h.hypothesis.score = float(score)
    d.results = [h]
    d.bbox.center.position.x = cx
    d.bbox.center.position.y = cy
    d.bbox.size_x = size
    d.bbox.size_y = size
    return d


class _Stub:
    """The minimum `_on_det` touches. Built by hand so the test cannot pass
    because some unrelated node wiring changed."""

    def __init__(self, act_conf: float):
        import threading
        self._act_conf = act_conf
        self._cls = ''
        self._lock = threading.Lock()
        self._stamp_warned = True          # silence the capture-time warning
        self._det_box = None
        self._det_conf = 0.0
        self._det_cls = ''
        self._det_t = 0.0
        self._det_header = None

    def get_logger(self):
        return types.SimpleNamespace(warn=lambda *a, **k: None,
                                     info=lambda *a, **k: None)


def _on_det(stub, msg):
    from mongla_vision.lock_node import LockNode
    LockNode._on_det(stub, msg)


def _msg(*dets) -> Detection2DArray:
    m = Detection2DArray()
    m.detections = list(dets)
    return m


def test_a_box_below_the_acting_bar_is_not_acted_on():
    """⭐ THE HALLUCINATION CASE, at its measured score. An open-water `gate`
    at 0.44-0.56 must not become the target when the bar is 0.60."""
    s = _Stub(act_conf=0.60)
    _on_det(s, _msg(_det(0.52)))
    assert s._det_box is None
    assert s._det_conf == 0.0


def test_a_confident_box_is_accepted():
    s = _Stub(act_conf=0.60)
    _on_det(s, _msg(_det(0.81)))
    assert s._det_box is not None
    assert s._det_conf == pytest.approx(0.81)
    assert s._det_cls == 'gate'


def test_the_shipped_default_still_clears_the_real_gate():
    """0.45 is the shipped bar. On gate.mkv the real gate scores 0.52-0.81 and
    98.9 % of frames clear 0.45, so the bar must not reject the low end of the
    true distribution."""
    s = _Stub(act_conf=0.45)
    _on_det(s, _msg(_det(0.52)))
    assert s._det_box is not None


def test_the_bar_bites_only_because_of_the_bar():
    """Injection-verify. The SAME detection, only the bar changes: it must be
    accepted below and rejected above, or these tests prove nothing."""
    low, high = _Stub(act_conf=0.30), _Stub(act_conf=0.60)
    _on_det(low, _msg(_det(0.52)))
    _on_det(high, _msg(_det(0.52)))
    assert low._det_box is not None, 'a 0.52 box must pass a 0.30 bar'
    assert high._det_box is None, 'the same box must fail a 0.60 bar'


def test_a_coasted_track_is_still_refused():
    """score == 0 marks a Kalman-coasted track, not an observation. The old
    `score > 0` test existed for this and the new bar must keep doing it."""
    s = _Stub(act_conf=0.45)
    _on_det(s, _msg(_det(0.0)))
    assert s._det_box is None


def test_the_best_box_above_the_bar_wins_not_the_best_overall():
    """A box below the bar must not become the target merely by being present,
    and among acceptable boxes the highest scorer wins."""
    s = _Stub(act_conf=0.45)
    _on_det(s, _msg(_det(0.44, cx=100.0), _det(0.70, cx=500.0)))
    assert s._det_conf == pytest.approx(0.70)
    assert s._det_box[0] > 400.0, 'took the 0.44 box that should be refused'


def test_an_empty_frame_clears_the_target():
    s = _Stub(act_conf=0.45)
    _on_det(s, _msg(_det(0.90)))
    assert s._det_box is not None
    _on_det(s, _msg())
    assert s._det_box is None, 'a frame with nothing must not keep the old box'


def test_a_frame_with_only_sub_bar_boxes_also_clears_the_target():
    """⛔ The regression that would reintroduce B-59 quietly: if a sub-bar
    frame left the previous box standing, the ladder would keep acting on a
    target the detector has stopped confirming."""
    s = _Stub(act_conf=0.60)
    _on_det(s, _msg(_det(0.90)))
    _on_det(s, _msg(_det(0.50)))
    assert s._det_box is None


def test_the_class_filter_still_applies():
    s = _Stub(act_conf=0.45)
    s._cls = 'gate'
    _on_det(s, _msg(_det(0.99, name='repair')))
    assert s._det_box is None
