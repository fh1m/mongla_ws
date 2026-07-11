"""wait_for_depth freshness gate (CTRL-7).

A frozen AHRS2 must never let a stale reading declare "depth reached" -- that
false positive would advance the mission while the hull isn't actually at depth.
`_fresh_depth` returns None on a stale sample so the reached-check can't fire.
No ROS/MAVLink: a fake pixhawk with configurable attitude + age.
"""

import pytest

from duburi_control.motion_depth import (
    _fresh_depth, wait_for_depth, _ATTITUDE_STALE_S, TOL_M,
)
from duburi_control.errors import MovementTimeout


class _Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


class _FakePixhawk:
    def __init__(self, depth, age=0.0):
        self._depth = depth
        self._age = age
        self.setpoints = []

    def get_attitude(self):
        if self._depth is None:
            return None
        return {'depth': self._depth, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}

    def get_attitude_age(self):
        return self._age

    def set_target_depth(self, d):
        self.setpoints.append(d)


# --------------------------------------------------------------------------- #
#  _fresh_depth                                                               #
# --------------------------------------------------------------------------- #
def test_fresh_depth_returns_value_when_fresh():
    assert _fresh_depth(_FakePixhawk(-1.5, age=0.02)) == pytest.approx(-1.5)


def test_fresh_depth_returns_none_when_stale():
    assert _fresh_depth(_FakePixhawk(-1.5, age=_ATTITUDE_STALE_S + 1.0)) is None


def test_fresh_depth_none_when_no_attitude():
    assert _fresh_depth(_FakePixhawk(None)) is None


def test_fresh_depth_treats_missing_age_method_as_fresh():
    class _NoAge:
        def get_attitude(self):
            return {'depth': -1.0}
    assert _fresh_depth(_NoAge()) == pytest.approx(-1.0)


# --------------------------------------------------------------------------- #
#  wait_for_depth: no false "reached" on a frozen reading                      #
# --------------------------------------------------------------------------- #
def test_wait_for_depth_reaches_when_fresh_at_target(monkeypatch):
    monkeypatch.setattr('duburi_control.motion_depth.time.sleep', lambda *_: None)
    pix = _FakePixhawk(-1.5, age=0.02)              # fresh, exactly at target
    # returns (None) without raising -> reached
    assert wait_for_depth(pix, -1.5, timeout=2.0, log=_Log()) is None


def test_wait_for_depth_does_not_false_reach_on_stale(monkeypatch):
    # advance a fake clock so the bounded loop actually times out under patched sleep
    t = {'now': 0.0}
    monkeypatch.setattr('duburi_control.motion_depth.time.monotonic',
                        lambda: t.__setitem__('now', t['now'] + 0.05) or t['now'])
    monkeypatch.setattr('duburi_control.motion_depth.time.sleep', lambda *_: None)
    pix = _FakePixhawk(-1.5, age=_ATTITUDE_STALE_S + 1.0)  # STALE, sitting at target
    # A frozen reading at the target must NOT be accepted as reached -> timeout.
    with pytest.raises(MovementTimeout):
        wait_for_depth(pix, -1.5, timeout=1.0, log=_Log())


# --------------------------------------------------------------------------- #
#  F1 (CTRL-C2): the failsafe keepalive is streamed every tick of the hold     #
# --------------------------------------------------------------------------- #
# A long depth hold with no heading-lock used to go RC-silent -> FS_PILOT_INPUT
# disarmed mid-command. wait_for_depth now streams `keepalive` each tick to keep
# RC_CHANNELS_OVERRIDE warm. The keepalive releases Ch3 so depth is still reached.

def test_keepalive_streamed_each_tick_until_reached(monkeypatch):
    monkeypatch.setattr('duburi_control.motion_depth.time.sleep', lambda *_: None)
    # depth arrives at target on the 3rd read: keepalive must have fired per tick.
    reads = iter([-1.0, -1.3, -1.5, -1.5])
    pix = _FakePixhawk(-1.5, age=0.02)
    monkeypatch.setattr(pix, 'get_attitude',
                        lambda: {'depth': next(reads, -1.5), 'yaw': 0.0})
    calls = {'n': 0}
    wait_for_depth(pix, -1.5, timeout=5.0, log=_Log(),
                   keepalive=lambda: calls.__setitem__('n', calls['n'] + 1))
    # one keepalive per set_target_depth tick (>=1; loop ran until reached).
    assert calls['n'] >= 1
    assert calls['n'] == len(pix.setpoints)   # exactly one per tick, never skipped


def test_keepalive_none_is_legacy_noop(monkeypatch):
    monkeypatch.setattr('duburi_control.motion_depth.time.sleep', lambda *_: None)
    pix = _FakePixhawk(-1.5, age=0.02)
    # No keepalive passed -> must behave exactly as before (reaches, no crash).
    assert wait_for_depth(pix, -1.5, timeout=2.0, log=_Log()) is None


def test_make_writers_depth_keepalive_releases_ch3_both_lockstates():
    from duburi_control.motion_writers import make_writers
    from duburi_control.pixhawk import NO_OVERRIDE, CH_THROTTLE

    class _Pix:
        def __init__(self): self.frames = []
        def send_rc_override(self, pitch=1500, roll=1500, throttle=1500,
                             yaw=1500, forward=1500, lateral=1500):
            self.frames.append(('override', throttle, yaw))
        def send_rc_translation(self, throttle=1500, forward=1500, lateral=1500):
            self.frames.append(('translation', throttle, None))
        def send_neutral(self):
            self.frames.append(('neutral', 1500, 1500))

    # No lock: send_rc_override with Ch3 RELEASED, Ch4 (yaw) neutral -> 5 channels
    # feed FS_PILOT exactly like the heartbeat, only Ch3 handed to ALT_HOLD.
    p = _Pix(); make_writers(p, release_yaw=False).depth_keepalive()
    assert p.frames == [('override', NO_OVERRIDE, 1500)]
    # Lock active: send_rc_translation with Ch3 released (Ch4 left to the lock).
    p = _Pix(); make_writers(p, release_yaw=True).depth_keepalive()
    assert p.frames == [('translation', NO_OVERRIDE, None)]
