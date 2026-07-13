"""Heading-lock x vision-align Ch4 arbitration (verification of existing design).

The facade already arbitrates Ch4 between the background HeadingLock and
``vision_align``:
  * yaw IS an align axis  -> suspend the lock for the loop, drive Ch4 from
    vision, then retarget the lock to the FINAL heading and resume.
  * yaw is NOT an axis     -> leave the lock running (it owns Ch4); the loop
    drives lateral via ``send_rc_translation`` so it never clobbers Ch4.

These tests pin that behaviour, plus the one untested seam with the deferred
heading lock: a yaw-align as the FIRST armed command must activate the
deferred lock (resume), then suspend it for the loop, then retarget+resume --
landing the lock active at the final heading with the heartbeat balanced.
"""

import logging
import time
from types import SimpleNamespace

import pytest

from duburi_control.duburi import Duburi


class ThrottleLogger:
    def __init__(self, inner):
        self._inner = inner

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakeHeartbeat:
    def __init__(self):
        self.hold_count = 0

    def pause(self):
        self.hold_count += 1

    def resume(self):
        if self.hold_count > 0:
            self.hold_count -= 1


class FakePixhawk:
    """Records Ch4 (yaw) and translation writes; togglable arm state."""

    def __init__(self, *, yaw=10.0, armed=True):
        self._attitude = {'yaw': yaw, 'pitch': 0.0, 'roll': 0.0, 'depth': -0.5}
        self._mode  = 'ALT_HOLD'
        self._armed = armed
        self.yaw_only_calls = 0
        self.rc = []            # send_rc_override kwargs
        self.translations = []  # send_rc_translation kwargs

    def get_attitude(self):
        return dict(self._attitude)

    def get_mode(self):
        return self._mode

    def is_armed(self):
        return self._armed

    def send_neutral(self):
        self.rc.append({'neutral': True})

    def send_rc_override(self, **kw):
        self.rc.append(kw)

    def send_rc_translation(self, **kw):
        self.translations.append(kw)

    def send_rc_yaw_only(self, pwm):
        self.yaw_only_calls += 1

    def set_target_depth(self, d):
        self._attitude['depth'] = d

    def arm(self, timeout=15.0, abort=None):
        if abort is not None:
            abort()                 # mirror real contract; guards facade wiring
        self._armed = True
        return True, 'ACCEPTED'

    def disarm(self, timeout=20.0):
        self._armed = False
        return True, 'ACCEPTED'

    def set_mode(self, name, timeout=8.0):
        self._mode = name
        return True, 'ACCEPTED'

    @staticmethod
    def percent_to_pwm(pct):
        return max(1100, min(1900, int(1500 + (pct / 100.0) * 400)))

    @staticmethod
    def heading_error(target, current):
        return (target - current + 540) % 360 - 180


class FakeVision:
    """vision_state stand-in: a fixed off-centre sample so the loop runs."""

    def __init__(self, ex=0.6, size=(640, 480)):
        self._sample = SimpleNamespace(ex=ex, ey=0.0, w_frac=0.3, h_frac=0.3,
                                       age_s=0.0, track_id=-1, coasted=False)
        self._size = size

    def image_size(self):
        return self._size

    def info_seen(self):
        return True

    def list_classes(self):
        return ['gate']

    def bbox_error(self, _cls, **_kw):
        return self._sample


def _make(armed=True, ex=0.6):
    pix = FakePixhawk(armed=armed)
    hb  = FakeHeartbeat()
    log = ThrottleLogger(logging.getLogger('test.vis.lock'))
    vis = FakeVision(ex=ex)
    duburi = Duburi(pix, log, heartbeat=hb, vision_state_provider=lambda _cam: vis)
    return duburi, pix, hb


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)


def test_yaw_align_suspends_lock_and_retargets_to_final_heading():
    """Lock active + vision_align(yaw) -> suspend during, retarget to final."""
    duburi, pix, hb = _make(armed=True)
    duburi.lock_heading(target=90.0, timeout=30.0)
    assert duburi._heading_lock is not None
    assert duburi._lock_active() is True

    # Drive a short yaw-align; FakePixhawk reports yaw=10.0 throughout, so the
    # final heading the lock should retarget to is ~10.0 (not the original 90).
    duburi.vision_align(camera='forward', target_class='gate', axes='yaw',
                        err_px=40.0, duration=0.2, gain=30.0, kp_yaw=60.0)

    # Lock survived (resumed, not stopped) and now targets the final heading.
    assert duburi._heading_lock is not None
    assert duburi._heading_lock.is_suspended is False
    assert abs(duburi._heading_lock.target_deg - 10.0) < 0.5, (
        'lock must retarget to the FINAL heading after a yaw-align')

    duburi.unlock_heading()
    assert hb.hold_count == 0


def test_lat_only_align_keeps_lock_and_releases_ch4():
    """Lock active + vision_align(lat) only -> lock keeps Ch4 (release_yaw)."""
    duburi, pix, hb = _make(armed=True)
    duburi.lock_heading(target=90.0, timeout=30.0)
    pix.rc.clear(); pix.translations.clear()

    duburi.vision_align(camera='forward', target_class='gate', axes='lat',
                        err_px=40.0, duration=0.2, gain=30.0, kp_lat=60.0)

    # The align loop must drive lateral via translation and never write Ch4.
    assert pix.translations, 'lat-only align under a lock must use send_rc_translation'
    yaw_overrides = [c for c in pix.rc
                     if 'yaw' in c and c.get('yaw') not in (1500, None)]
    assert not yaw_overrides, 'lat-only align must not author Ch4 (lock owns it)'
    # Lock stayed live and on its original target (never retargeted).
    assert duburi._heading_lock is not None
    assert abs(duburi._heading_lock.target_deg - 90.0) < 0.5

    duburi.unlock_heading()


def test_lat_only_align_no_lock_still_releases_ch4():
    """NO lock + vision_align(lat) only -> the verb must STILL leave Ch4 alone.

    The 2026-06 fix: Ch4 release is gated on the YAW AXIS, not on lock-state.
    Before the fix a no-lock lat-only align spammed Ch4=1500 every tick (which
    fights ArduSub's heading hold / a later lock); now it never authors Ch4
    unless yaw is a requested axis.
    """
    duburi, pix, hb = _make(armed=True)         # NO lock_heading
    assert duburi._lock_active() is False
    pix.rc.clear(); pix.translations.clear()

    duburi.vision_align(camera='forward', target_class='gate', axes='lat',
                        err_px=40.0, duration=0.2, gain=30.0, kp_lat=60.0)

    assert pix.translations, 'lat-only align must drive lateral via translation'
    yaw_writes = [c for c in pix.rc if 'yaw' in c and c.get('yaw') is not None]
    assert not yaw_writes, (
        'lat-only align with no lock must NOT author Ch4 (the 1500-spam bug)')


def test_vision_move_always_releases_ch4():
    """vision_move never computes a yaw command, so it must ALWAYS leave Ch4
    alone (translation), with or without a lock."""
    duburi, pix, hb = _make(armed=True)         # NO lock
    pix.rc.clear(); pix.translations.clear()

    duburi.vision_move(camera='forward', target_class='gate', fwd_fill=95.0,
                       duration=0.2, gain=30.0)

    assert pix.translations, 'move must drive forward via send_rc_translation'
    yaw_writes = [c for c in pix.rc if 'yaw' in c and c.get('yaw') is not None]
    assert not yaw_writes, 'move must never author Ch4 (no yaw command)'


def test_deferred_lock_then_yaw_align_first_command():
    """Deferred lock + vision_align(yaw) as first armed command.

    Sequence inside one command scope: activation hook resumes the deferred
    lock, then vision_align suspends it for the loop, then retargets + resumes.
    End state: lock active at the final heading, heartbeat balanced.
    """
    duburi, pix, hb = _make(armed=False)        # start DISARMED
    duburi.lock_heading(target=0.0, timeout=30.0)
    assert duburi._lock_deferred is True
    assert duburi._heading_lock.is_suspended is True

    duburi.arm()
    assert duburi._lock_deferred is True          # arm alone doesn't activate

    # First armed actuating command is a yaw-align.
    duburi.vision_align(camera='forward', target_class='gate', axes='yaw',
                        err_px=40.0, duration=0.2, gain=30.0, kp_yaw=60.0)

    assert duburi._lock_deferred is False         # activated
    assert duburi._heading_lock is not None       # not stranded
    assert duburi._heading_lock.is_suspended is False  # resumed after the loop
    assert abs(duburi._heading_lock.target_deg - 10.0) < 0.5  # final heading
    assert hb.hold_count == 1                      # exactly one hold while locked

    duburi.unlock_heading()
    assert hb.hold_count == 0                      # balanced release
