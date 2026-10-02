"""align verifies against a RE-MEASURED pose, not the controller's own error.

Plan Block 1C. BumblebeeAS's `cluster_goto` re-clusters after the move and
gates the result; ours declared ALIGNED on the last in-band tick BEFORE the
brake and reported that pre-brake frame as where it ended. A hull strafing
through centre, or one the brake kicked, was reported aligned.

Each scene below is scripted against the CLOCK: centred while the controller
drives, and whatever the scene says once align has stopped. The stop instant is
detected from the writers going neutral, which is what align does on exit.
"""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from test_motion_vision import (                     # noqa: E402
    _FakePixhawk, _FakeWriters, _Log, _sample)
from mongla_control.motion_vision import (           # noqa: E402
    ALIGNED, DRIFTED, LOST, VERIFY_TIMEOUT_S, align_loop)
import sys as _sys_ct
import pathlib as _pl_ct
_sys_ct.path.insert(0, str(_pl_ct.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_of, code_of_file  # noqa: E402  (issue #22)


class _Writers(_FakeWriters):
    """Marks the instant align goes neutral at exit -- the stop."""
    stopped_at = None

    def neutral(self, *a, **k):
        r = super().neutral(*a, **k) if hasattr(super(), 'neutral') else None
        if self.stopped_at is None:
            self.stopped_at = time.monotonic()
        return r


class _Scene:
    """bbox_error() by phase: `before` until the stop, then `after`."""

    def __init__(self, writers, before, after):
        self.w, self.before, self.after = writers, before, after

    def image_size(self):
        return (640, 480)

    def info_seen(self):
        return True

    def list_classes(self):
        return ['gate']

    def bbox_error(self, _cls, **_kw):
        stopped = self.w.stopped_at is not None
        return self.after if stopped else self.before


def _align(after, before=None):
    w = _Writers()
    vs = _Scene(w, before or _sample(ex=0.0, w_frac=0.3, h_frac=0.3), after)
    t0 = time.monotonic()
    out = align_loop(pixhawk=_FakePixhawk(), vision_state=vs, target_class='gate',
                     axes={'lat'}, offsets={}, err_px=40.0, duration=5.0,
                     gain=30.0, align_stable_frames=3, lost_grace_s=0.5,
                     brake=False, writers=w, log=_Log(), abort_fn=None)
    return out, time.monotonic() - t0


def test_a_target_still_centred_after_the_stop_is_ALIGNED_and_says_verified():
    out, _ = _align(after=_sample(ex=0.02, w_frac=0.3, h_frac=0.3))
    assert out.code == ALIGNED and 'verified' in out.reason
    assert out.last_err_px == pytest.approx(0.02 * 320, abs=1.0)


def test_a_hull_that_drifts_once_stopped_is_DRIFTED_not_aligned():
    """Falsified by ALIGNED: before this, the pre-brake frame was the answer.
    0.3 x 320 px = 96 px off against a 40 px band."""
    out, _ = _align(after=_sample(ex=0.30, w_frac=0.3, h_frac=0.3))
    assert out.code == DRIFTED, out.reason
    assert not out.ok
    assert out.end_x_px == pytest.approx(96.0, abs=1.0), 'report where it ENDED'
    assert '96px off once stopped' in out.reason


def test_a_target_not_re_seen_once_stopped_is_LOST_and_unverified():
    out, t = _align(after=None)
    assert out.code == LOST and 'unverified' in out.reason
    assert t < VERIFY_TIMEOUT_S + 1.0, 'it must give up at the verify timeout'


def test_a_coasted_box_cannot_verify_anything():
    """A tracker prediction is not a re-measurement."""
    out, _ = _align(after=_sample(ex=0.0, w_frac=0.3, h_frac=0.3, coasted=True))
    assert out.code == LOST


def test_verify_off_is_the_old_behaviour_exactly():
    w = _Writers()
    vs = _Scene(w, _sample(ex=0.0, w_frac=0.3, h_frac=0.3),
                _sample(ex=0.30, w_frac=0.3, h_frac=0.3))
    out = align_loop(pixhawk=_FakePixhawk(), vision_state=vs, target_class='gate',
                     axes={'lat'}, offsets={}, err_px=40.0, duration=5.0,
                     gain=30.0, align_stable_frames=3, lost_grace_s=0.5,
                     brake=False, verify=False, writers=w, log=_Log(),
                     abort_fn=None)
    assert out.code == ALIGNED and 'verified' not in out.reason


def test_the_verification_and_the_controller_ask_the_SAME_question():
    """One copy of the centring error: the helper the loop uses is the one the
    re-measurement uses. Falsified if either inlines its own formula again."""
    import inspect
    from mongla_control import motion_vision as mv
    src = code_of(mv.align_loop)
    assert src.count('_axis_ctrl(') >= 4, 'the control law must use the helper'
    assert '_centring_error_px(' in src
    assert "ex_now - (offsets.get(" not in src
