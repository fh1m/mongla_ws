"""The anchor's bank work runs beside the ladder, not inside its tick.

Bead a2c, measured on Kutum 2026-11-23: one locate costs ~110 ms and ran
inside `_ladder_tick`, so the /lock behind it waited (lock age p50 142 ms;
with the anchor at 0.5 Hz, 88-97 ms). The falsifiers written before the
change: a 150 ms locate must not delay the tick; a result for a target that
changed mid-locate is dropped; the full-rate locate returns the moment the
detection stops holding the target; a busy bank skips a place look and says so.
"""
import collections
import threading
import time
import types

import numpy as np

from mongla_vision.lock_anchor import ANCHOR_KEEPWARM_S, AnchorMixin, detection_holds
from mongla_vision.lock_inputs import LockInputsMixin


class _Bank:
    """A bank whose locate takes as long as the Pi's."""

    def __init__(self, delay_s=0.15):
        self.delay_s, self.calls = delay_s, []
        self.has_reference, self.size, self._cap, self._refresh = True, 3, 64, 100

    def covers_scale(self, gray, roi):
        return True

    def locate(self, gray, label=None):
        self.calls.append(('locate', time.monotonic()))
        time.sleep(self.delay_s)
        return types.SimpleNamespace(inliers=150, pose=types.SimpleNamespace(ok=False, corners=None))

    def last_lookup_scores(self):
        return [], []


class _Node(AnchorMixin):
    def __init__(self, bank):
        self._anchor = bank
        self._anchor_inliers, self._anchor_enrolled, self._anchor_label = 150, 0, 'gate'
        self._anchor_enrol_next = float('inf')        # no enrol in these tests
        self._anchor_pose = self._anchor_header = None
        self._anchor_next, self._anchor_period = 0.0, 1.0 / 4.5
        self._anchor_exec = self._anchor_job = None
        self._bank_lock = threading.Lock()
        self._anchor_gray, self._anchor_locate_t = None, float('-inf')
        self._anchor_trend = collections.deque(maxlen=8)
        self._prepared = 0

    def _note_anchor_health(self):
        pass

    def get_logger(self):
        return types.SimpleNamespace(info=print, debug=print, warn=print)


def _tick(now, gen=0, det=False):
    return types.SimpleNamespace(
        gray=np.zeros((240, 320), np.uint8), header=('frame', now), now=now, gen=gen,
        det_header=None, det_box=(1, 1, 10, 10) if det else None, det_conf=0.9 if det else 0.0,
        det_cls='gate', det_t=now, live_s=0.2)


def _drain(node, gen=0):
    node._anchor_job.future.result()
    return node._enrol_or_locate(_tick(time.monotonic(), gen))


def test_a_slow_locate_does_not_hold_up_the_tick():
    node = _Node(_Bank(0.15))
    t0 = time.monotonic()
    node._enrol_or_locate(_tick(t0))
    assert time.monotonic() - t0 < 0.03, 'the tick waited for the locate'
    assert node._anchor_job is not None
    assert _drain(node) is True, 'the finished locate is applied on a later tick'
    assert node._anchor_header == ('frame', t0), 'the pose carries the frame it was fitted on'
    node._anchor_exec.shutdown()


def test_a_result_for_an_old_target_is_dropped():
    node = _Node(_Bank(0.05))
    node._enrol_or_locate(_tick(time.monotonic(), gen=1))
    assert _drain(node, gen=2) is False
    assert node._anchor_header is None
    node._anchor_exec.shutdown()


def test_the_locate_idles_while_a_detection_holds_and_returns_when_it_fades():
    bank = _Bank(0.0)
    node = _Node(bank)
    t = 100.0
    node._enrol_or_locate(_tick(t, det=True))          # first look: keep-warm due
    node._anchor_job.future.result()
    n = len(bank.calls)
    for k in range(1, 8):                               # 0.7 s of held target
        node._enrol_or_locate(_tick(t + 0.1 * k, det=True))
        if node._anchor_job:
            node._anchor_job.future.result()
    assert len(bank.calls) == n, 'no locate while a fresh detection holds'
    node._enrol_or_locate(_tick(t + 0.8, det=False))   # the detection is gone
    assert node._anchor_job is not None, 'full rate the moment it fades'
    node._anchor_job.future.result()
    node._enrol_or_locate(_tick(t + 0.8 + ANCHOR_KEEPWARM_S, det=True))
    if node._anchor_job:
        node._anchor_job.future.result()
    assert detection_holds(_tick(1.0, det=True)) and not detection_holds(_tick(1.0))
    node._anchor_exec.shutdown()


def test_a_place_look_waits_for_no_one():
    calls = []
    node = types.SimpleNamespace(_closer_on=True, _anchor=object(), _bank_lock=threading.Lock(),
                                 _place_look=lambda *a: calls.append('look'))
    node._count_refusal = lambda k: calls.append(k)
    node._bank_lock.acquire()
    LockInputsMixin._place_tick(node, np.zeros((4, 4), np.uint8), None, 0.0)
    assert calls == ['bank busy']
    node._bank_lock.release()
    LockInputsMixin._place_tick(node, np.zeros((4, 4), np.uint8), None, 0.0)
    assert calls[-1] == 'look'
