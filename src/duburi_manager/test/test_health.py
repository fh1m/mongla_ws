"""The health surface, and the four times "absence" was read as "fine".

Every test here is one of those, or the machinery that keeps them from
recurring. None of it is about plumbing.
"""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_manager.health import (                     # noqa: E402
    Health, HealthBoard, State, degraded, failed, ok, unknown)
from duburi_manager import health_reporters as R        # noqa: E402


# --------------------------------------------------------------------------- #
#  UNKNOWN is not OK -- the whole point
# --------------------------------------------------------------------------- #
def test_a_subsystem_that_never_reported_is_UNKNOWN_not_OK():
    assert Health('x').state is State.UNKNOWN
    assert Health('x').ok is False


def test_UNKNOWN_is_WORSE_than_DEGRADED():
    """A degraded subsystem is telling us something. An unknown one is not
    being watched at all, and the vehicle has repeatedly acted as though
    silence were good news."""
    assert State.UNKNOWN > State.DEGRADED
    b = HealthBoard()
    b.register('a', lambda: degraded('a', 'noisy'))
    b.register('b', lambda: unknown('b'))
    b.poll()
    assert b.worst() is State.UNKNOWN


def test_a_STALE_ok_is_demoted_to_UNKNOWN():
    """Good news does not keep. A reporter that stops being called must not
    leave an OK standing -- that is exactly how a dead subsystem passes."""
    b = HealthBoard(stale_s=0.05)
    b.register('a', lambda: ok('a', 'fine'))
    b.poll()
    assert b.worst() is State.OK
    time.sleep(0.08)
    assert b.worst() is State.UNKNOWN
    assert 'stale' in b.report()[0]


def test_a_RAISING_reporter_is_UNKNOWN_not_SKIPPED():
    """Skipping would leave the previous OK standing, so a subsystem whose
    health check is itself broken would keep reporting the last good news."""
    b = HealthBoard()
    calls = {'n': 0}

    def flaky():
        calls['n'] += 1
        if calls['n'] == 1:
            return ok('flaky', 'fine')
        raise RuntimeError('sensor died')

    b.register('flaky', flaky)
    assert b.poll()['flaky'].state is State.OK
    assert b.poll()['flaky'].state is State.UNKNOWN
    assert 'RuntimeError' in b.get('flaky').evidence


def test_a_reporter_returning_a_BOOL_is_UNKNOWN():
    """There are no booleans in this vocabulary. `True` would otherwise be
    truthy and slide through as good news."""
    b = HealthBoard()
    b.register('x', lambda: True)
    assert b.poll()['x'].state is State.UNKNOWN


def test_the_report_puts_the_WORST_first():
    b = HealthBoard()
    b.register('a', lambda: ok('a', 'fine'))
    b.register('z', lambda: failed('z', 'broken'))
    b.poll()
    assert b.report()[0].startswith('z: FAILED')


# --------------------------------------------------------------------------- #
#  The dialects, including the ones that read backwards
# --------------------------------------------------------------------------- #
def test_BARO_HEALTH_is_a_FAULT_CODE_not_a_score():
    """3 is the WORST value, not the best. It was once read as a health score
    and taken for 'mostly fine'; it means the barometer never initialised."""
    assert R.barometer(lambda _n: 0).state is State.OK
    assert R.barometer(lambda _n: 3).state is State.FAILED
    assert 'NOT INITIALISED' in R.barometer(lambda _n: 3).evidence
    assert R.barometer(lambda _n: None).state is State.UNKNOWN


def test_only_YAW_REF_2_means_the_heading_is_ABSOLUTE():
    assert R.heading_reference(lambda _n: 2).state is State.OK
    for v in (0, 1, 3, 4, 5):
        assert R.heading_reference(lambda _n: v).state is not State.OK, v


def test_a_DISABLED_leak_sensor_is_a_FAILURE_not_a_dry_hull():
    """LEAK_EN=0 reads DRY either way, so the hull looks safe precisely when
    nothing is watching. Measured on this board: LEAK_EN = 0."""
    assert R.leak_sensor(False, False).state is State.FAILED
    assert 'NOTHING IS WATCHING' in R.leak_sensor(False, False).evidence
    assert R.leak_sensor(True, False).state is State.OK
    assert R.leak_sensor(True, True).state is State.FAILED
    assert R.leak_sensor(None, None).state is State.UNKNOWN


def test_NO_ESC_TELEMETRY_is_not_EIGHT_HAPPY_THRUSTERS():
    """An empty RPM list is the same shape whether the bus is silent or the
    hull is still. Gated on the message count, or the pre-fire thruster check
    passes on a bus that is not talking."""
    assert R.thrusters([], esc_msgs=0).state is State.UNKNOWN
    assert R.thrusters([0] * 8, esc_msgs=40).state is State.OK
    assert R.thrusters([0] * 5, esc_msgs=40).state is State.DEGRADED


def test_a_lower_LADDER_RUNG_is_DEGRADED_not_FAILED():
    """Buying time on the follower is the ladder working. Authority at zero is
    the loss, and only that is a failure."""
    assert R.target_lock(1.0, 'detection').state is State.OK
    assert R.target_lock(0.4, 'follow').state is State.DEGRADED
    assert R.target_lock(0.0, 'lost').state is State.FAILED
    assert R.target_lock(None).state is State.UNKNOWN


def test_NO_POSE_is_not_SQUARE():
    assert R.target_pose(None).state is State.UNKNOWN


def test_the_pose_is_judged_on_the_WORSE_FLIP_BRANCH():
    """A 3 deg estimate with a 20 deg interval is not a 3 deg answer."""
    m = SimpleNamespace(ok=True, reason='', off_axis_deg=3.0,
                        yaw_spread_deg=20.0, pitch_spread_deg=0.0, range_m=1.5)
    assert R.target_pose(m, max_tilt_deg=8.0).state is State.DEGRADED
    m.yaw_spread_deg = 1.0
    assert R.target_pose(m, max_tilt_deg=8.0).state is State.OK


def test_a_dead_board_link_is_FAILED_and_a_broken_check_is_UNKNOWN():
    assert R.board_link(SimpleNamespace(link_alive=lambda: True)).state is State.OK
    assert R.board_link(SimpleNamespace(link_alive=lambda: False)).state is State.FAILED

    def boom():
        raise IOError('port gone')
    assert R.board_link(SimpleNamespace(link_alive=boom)).state is State.UNKNOWN
