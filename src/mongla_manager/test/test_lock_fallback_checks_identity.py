"""The ladder fallback steers only on the class the loop asked for (#74).

`/lock` names the rung in its hypothesis (detection, follow, anchor). Before
this, `bbox_error('A', lock_s=1)` with no live A box returned whatever the
ladder held, whatever its class, so a ladder pinned or stale on prop B could
steer an align meant for prop A. The class now rides in `Detection2D.id`, and
the consumer refuses a mismatch and an unverifiable (empty) identity.

These run the real publisher (`LockPublishMixin._publish`), the real
subscriber callback (`VisionState._on_lock`) and the real `bbox_error`.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip('vision_msgs')
from std_msgs.msg import Header                                    # noqa: E402
from vision_msgs.msg import Detection2DArray                       # noqa: E402

from mongla_manager.vision_state import VisionState                # noqa: E402
from mongla_vision.lock_publish import LockPublishMixin            # noqa: E402
from mongla_vision.tracking.lock_state import Rung                 # noqa: E402

W, H = 640, 480


class _Lock(LockPublishMixin):
    """The publisher with only what `_publish` reads."""

    def __init__(self, cls='', det_cls='', anchor_label=''):
        self._cls, self._det_cls, self._anchor_label = cls, det_cls, anchor_label
        self._pub_min_dt, self._last_pub_rung, self._last_pub_t = 0.0, None, 0.0
        self.sent = []
        self._pub = SimpleNamespace(publish=self.sent.append)

    def _note_visibility(self, x1, y1, x2, y2):
        pass

    def _remember_world(self, x1, y1, x2, y2):
        pass


def _ladder(rung, **ids):
    lk = _Lock(**ids)
    st = SimpleNamespace(have_target=True, rung=rung, confidence=0.8, authority=0.8,
                         xyxy=(300.0, 200.0, 380.0, 280.0))
    lk._publish(st, Header())
    return lk.sent[-1]


def _consumer(lock_msg):
    vs = VisionState(MagicMock(), camera='forward')
    vs._image_size = (W, H)
    vs._latest_array = Detection2DArray()          # the detector sees nothing live
    vs._on_lock(lock_msg)
    return vs


def test_a_wrong_class_lock_is_refused():
    vs = _consumer(_ladder(Rung.FOLLOW, det_cls='gate'))
    assert vs.bbox_error('torpedo_board', lock_s=1.0) is None


def test_the_right_class_lock_is_accepted_and_names_the_class():
    vs = _consumer(_ladder(Rung.FOLLOW, det_cls='gate'))
    s = vs.bbox_error('gate', lock_s=1.0)
    assert s is not None and s.coasted and s.class_id == 'gate'
    assert s.ex == pytest.approx((340 - W / 2) / (W / 2))   # box centred at x=340


def test_the_anchor_answers_for_what_it_enrolled_on():
    msg = _ladder(Rung.ANCHOR, det_cls='gate', anchor_label='torpedo_board')
    assert _consumer(msg).bbox_error('torpedo_board', lock_s=1.0) is not None
    assert _consumer(msg).bbox_error('gate', lock_s=1.0) is None


def test_an_unverifiable_identity_is_refused():
    """An older lock_node, or a ladder that never saw a real detection."""
    msg = _ladder(Rung.FOLLOW)
    assert msg.detections[0].id == ''
    assert _consumer(msg).bbox_error('gate', lock_s=1.0) is None


def test_no_class_asked_keeps_the_single_target_fallback():
    vs = _consumer(_ladder(Rung.FOLLOW, det_cls='gate'))
    assert vs.bbox_error('', lock_s=1.0) is not None


def test_the_ladder_stays_opt_in():
    vs = _consumer(_ladder(Rung.FOLLOW, det_cls='gate'))
    assert vs.bbox_error('gate', lock_s=0.0) is None


def test_the_rung_still_rides_in_the_hypothesis():
    """The HUD and health reporters read the rung; identity is beside it."""
    d = _ladder(Rung.ANCHOR, anchor_label='bin').detections[0]
    assert d.results[0].hypothesis.class_id == 'anchor' and d.id == 'bin'
