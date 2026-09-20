"""Tests for the DVL distance verbs.

These verbs had NO tests at all, and shipped three silent-success paths:

  1. No DVL at all -> open-loop timed guess at a hardcoded 0.3 m/s, returning
     None exactly like the success path, so the facade hardcoded success=True.
     Measured against Gazebo ground truth: 2.361 m travelled for a 1.0 m
     command, reported as "completed".
  2. Connected-but-frozen DVL -> spun to the deadline, logged at info, and
     still reported completed.
  3. Because the deadline bounds TIME and not distance, a DVL reading zero
     meant full thrust for the whole budget: 11.3 m of real travel on a 1.0 m
     command, which is worse than the fallback it replaced.

All three now raise.
"""
import logging
from pathlib import Path

import pytest

from mongla_control.errors import MovementError, MovementTimeout
from mongla_control.motion_writers import make_writers
from mongla_control.motion_forward import drive_forward_dist
from mongla_control.motion_lateral import drive_lateral_dist


class ThrottleLogger:
    def __init__(self):
        self._inner = logging.getLogger('test.motion_dist')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self):
        self.overrides = []

    def get_attitude(self):
        return {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'depth': -0.5}

    def send_rc_override(self, *a, **k):
        self.overrides.append((a, k))

    def send_rc_translation(self, *a, **k):
        self.overrides.append((a, k))

    def send_rc_yaw_only(self, *a, **k):
        self.overrides.append((a, k))


class NoDvlSource:
    """A heading-only source, e.g. mavlink_ahrs. No position methods."""
    name = 'mavlink_ahrs'

    def read_yaw(self):
        return 0.0


class FakeDvl:
    """Position source. `speed` metres per get_position() call; 0 = frozen."""
    name = 'fake_dvl'

    def __init__(self, speed=0.25):
        self.speed = speed
        self.x = 0.0
        self.y = 0.0
        self.resets = 0

    def read_yaw(self):
        return 0.0

    def reset_position(self):
        self.resets += 1
        self.x = self.y = 0.0

    def get_position(self):
        self.x += self.speed
        self.y += self.speed
        return self.x, self.y


def _args(pixhawk, log, src, distance=1.0):
    return dict(pixhawk=pixhawk, signed_dir=1, distance_m=distance, gain=50,
                tolerance=0.1, log=log, writers=make_writers(pixhawk, log),
                yaw_source=src)


@pytest.mark.parametrize('fn', [drive_forward_dist, drive_lateral_dist])
def test_no_dvl_raises_instead_of_guessing(fn):
    """The whole point: a distance verb with no way to measure distance must
    not run an unmeasured move and call it a success."""
    px, log = FakePixhawk(), ThrottleLogger()
    with pytest.raises(MovementError) as exc:
        fn(**_args(px, log, NoDvlSource()))
    msg = str(exc.value)
    assert 'no DVL position source' in msg
    assert 'mavlink_ahrs' in msg, 'the error must name the actual yaw_source'
    assert not px.overrides, 'must refuse BEFORE commanding any thrust'


@pytest.mark.parametrize('fn', [drive_forward_dist, drive_lateral_dist])
def test_no_dvl_names_a_real_alternative(fn):
    px, log = FakePixhawk(), ThrottleLogger()
    with pytest.raises(MovementError) as exc:
        fn(**_args(px, log, NoDvlSource()))
    assert 'sim_dvl' in str(exc.value)


@pytest.mark.parametrize('fn', [drive_forward_dist, drive_lateral_dist])
def test_frozen_dvl_fails_rather_than_reporting_completed(fn):
    """A connected DVL stuck at (0,0) used to burn the deadline and report
    success. It must fail, and it must not keep driving for the full budget."""
    px, log = FakePixhawk(), ThrottleLogger()
    with pytest.raises((MovementError, MovementTimeout)) as exc:
        fn(**_args(px, log, FakeDvl(speed=0.0)))
    assert 'no progress' in str(exc.value) or 'timeout' in str(exc.value)


@pytest.mark.parametrize('fn', [drive_forward_dist, drive_lateral_dist])
def test_healthy_dvl_returns_the_measured_distance(fn):
    """The success path must hand back a real number, not None -- that None is
    why the facade could only hardcode success=True."""
    px, log = FakePixhawk(), ThrottleLogger()
    src = FakeDvl(speed=0.25)
    travelled = fn(**_args(px, log, src, distance=1.0))
    assert travelled is not None
    assert travelled == pytest.approx(1.0, abs=0.35)
    assert src.resets == 1, 'the integrator must be zeroed at the start'


@pytest.mark.parametrize('fn', [drive_forward_dist, drive_lateral_dist])
def test_overshoot_guard_stops_a_runaway(fn):
    """DVL that reports motion but never reaches the target: the loop must give
    up on distance, not run the full time budget."""
    px, log = FakePixhawk(), ThrottleLogger()
    # 1.5 m per poll steps straight over the 0.1 m tolerance band, so the loop
    # never gets to declare "reached" -- it lands at 1.5 then 3.0, and 3.0 is
    # past the 1.5*target+0.5 = 2.0 m overshoot limit. That is the case the
    # guard exists for: real motion the controller cannot stop at the target.
    with pytest.raises(MovementError) as exc:
        fn(**_args(px, log, FakeDvl(speed=1.5), distance=1.0))
    assert 'overshoot' in str(exc.value)


# --------------------------------------------------------------------------
# arc: error_value must be the real heading residual
# --------------------------------------------------------------------------
#
# `command-reference.md` advertises arc's error_value as "heading drift vs
# expected". It was a hardcoded 0.0. Measured against Gazebo ground truth, a 6 s
# arc commanded to target_yaw=200 finished at 5.1 deg -- a 165 deg miss -- and
# reported err=0.000, "arc: completed". The one motion verb that promises a real
# residual was the one inventing it.
#
# Not a failure case: `duration` bounds the manoeuvre, so ending short is
# legitimate. Claiming to have ended on target is not.

def _arc_residual(target_yaw, final_heading):
    """The expression mongla.arc uses to fill error_value."""
    return ((float(target_yaw) - final_heading + 180.0) % 360.0) - 180.0


@pytest.mark.parametrize('target,final,expect', [
    (200.0, 5.1, -165.1),      # the measured case
    (60.0, 80.5, -20.5),
    (0.0, 0.0, 0.0),           # on target -> genuinely zero
    (10.0, 350.0, 20.0),       # wraps the short way, not 340
    (350.0, 10.0, -20.0),
])
def test_arc_error_value_is_the_real_residual(target, final, expect):
    assert _arc_residual(target, final) == pytest.approx(expect, abs=0.05)


def test_arc_residual_is_always_shortest_way_round():
    """A residual outside +/-180 means the sign convention wrapped the long way,
    which would make a small miss look enormous and vice versa."""
    for target in range(0, 360, 15):
        for final in range(0, 360, 15):
            assert -180.0 <= _arc_residual(target, final) <= 180.0


def test_arc_no_longer_hardcodes_zero():
    import mongla_control.mongla as mongla_mod
    body = Path(mongla_mod.__file__).read_text()
    arc = body[body.index("with self._command_scope('arc')"):]
    arc = arc[:arc.index('def style_roll')]
    assert 'error_value=0.0' not in arc, 'arc is hardcoding a zero residual again'
    assert 'error_value=err' in arc
