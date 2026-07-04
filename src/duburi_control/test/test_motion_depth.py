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
