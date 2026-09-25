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

import math
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


def _state(yaw=12.5, depth=-1.2, armed=True):
    """A MonglaState-shaped object. Built by hand so the test does not need
    mongla_interfaces built -- the same reason lock_node imports it
    optionally."""
    return types.SimpleNamespace(yaw_deg=yaw, depth_m=depth, armed=armed)


def _yawnode(**kw):
    from mongla_vision.lock_node import LockNode
    s = _Stub(act_conf=0.45)
    LockNode._on_state(s, _state(**kw))
    return LockNode, s


def test_vision_now_knows_the_hull_heading():
    """⭐ THE HARMONY GAP. The ladder had odometry (where) and floor height
    (how high) but never yaw, so a target leaving frame could not be turned
    into a world bearing -- WorldTarget was unreachable for want of one number
    the board already publishes."""
    LockNode, s = _yawnode(yaw=12.5)
    assert s._yaw_deg == pytest.approx(12.5)
    assert s._depth_m == pytest.approx(-1.2)
    assert s._armed is True
    assert LockNode._yaw_fresh(s) == pytest.approx(12.5)


def test_an_absent_yaw_stays_NaN_and_is_never_coerced_to_zero():
    """⛔ THE RECURRING DEFECT. The board suppresses a yaw it cannot stand
    behind. Coercing NaN to 0.0 would hand every rung a confident heading of
    due north -- a plausible number standing in for an absent measurement."""
    LockNode, s = _yawnode(yaw=float('nan'))
    assert math.isnan(s._yaw_deg)
    assert math.isnan(LockNode._yaw_fresh(s))


def test_a_stale_yaw_is_refused():
    """A remembered world position is built from yaw AT THE MOMENT OF THE
    SIGHTING. One second old on a turning hull is a different direction."""
    LockNode, s = _yawnode(yaw=90.0)
    s._state_t -= 5.0
    assert math.isnan(LockNode._yaw_fresh(s))
    assert s._yaw_deg == pytest.approx(90.0), 'the value is stale, not gone'


def test_yaw_is_NaN_before_any_state_arrives():
    """The window between construction and the first message must not read as
    'pointing at 0 degrees'."""
    from mongla_vision.lock_node import LockNode
    s = _Stub(act_conf=0.45)
    s._yaw_deg, s._state_t = float('nan'), 0.0
    assert math.isnan(LockNode._yaw_fresh(s))


def test_the_class_filter_still_applies():
    s = _Stub(act_conf=0.45)
    s._cls = 'gate'
    _on_det(s, _msg(_det(0.99, name='repair')))
    assert s._det_box is None


# ── hull roll, and the -1.0 "no data" convention ───────────────────────────


def _imu(roll_rad=0.0, has_orientation=True):
    """A sensor_msgs/Imu-shaped object. `orientation_covariance[0] = -1.0` is
    the ROS marker for "this backend cannot supply attitude", which
    auv_manager_node sets while still publishing a structurally valid
    message."""
    import math as _m
    cov = [0.0] * 9
    if not has_orientation:
        cov[0] = -1.0
    q = types.SimpleNamespace(x=_m.sin(roll_rad / 2), y=0.0, z=0.0,
                              w=_m.cos(roll_rad / 2))
    return types.SimpleNamespace(orientation=q, orientation_covariance=cov)


def _imunode(**kw):
    from mongla_vision.lock_node import LockNode
    s = _Stub(act_conf=0.45)
    LockNode._on_imu(s, _imu(**kw))
    return LockNode, s


def test_roll_is_read_from_the_imu_quaternion():
    """⭐ The de-rotation the operator's R&D fusion does and the ladder did
    not. On a five-thruster hull roll is UNACTUATED, so it is the one attitude
    the controller cannot null and a visual estimate must compensate for."""
    LockNode, s = _imunode(roll_rad=0.35)
    assert LockNode._roll_fresh(s) == pytest.approx(0.35, abs=1e-6)


def test_the_minus_one_covariance_convention_is_obeyed():
    """⛔ THE LOAD-BEARING CHECK. auv_manager_node publishes a valid Imu with
    orientation_covariance[0] = -1.0 when the board cannot supply attitude.
    Reading the quaternion anyway would take (0,0,0,1) as LEVEL -- a confident
    wrong answer on a rolled hull, and exactly what that convention exists to
    prevent."""
    LockNode, s = _imunode(roll_rad=0.0, has_orientation=False)
    assert math.isnan(LockNode._roll_fresh(s))


def test_a_stale_roll_is_refused():
    """A second-old roll on a rolling hull is a different attitude."""
    LockNode, s = _imunode(roll_rad=0.2)
    s._imu_t -= 5.0
    assert math.isnan(LockNode._roll_fresh(s))


def test_roll_is_NaN_before_any_imu_arrives():
    from mongla_vision.lock_node import LockNode
    s = _Stub(act_conf=0.45)
    s._roll_rad, s._imu_t = float('nan'), 0.0
    assert math.isnan(LockNode._roll_fresh(s))


def test_the_derotation_is_actually_applied_to_the_bearing():
    """⛔ §9 INLINE: reading roll and never using it would be the defect this
    repo keeps finding. The world-frame bearing must be rotated by it."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    assert 'roll = self._roll_fresh()' in src
    assert 'bx * cr - by * sr' in src, 'roll is read but never applied'
