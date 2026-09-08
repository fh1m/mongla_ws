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


def test_A_TALKING_BOARD_WITH_NO_ESCS_IS_NOT_EIGHT_HAPPY_THRUSTERS():
    """The regression this reporter exists to prevent, in one line.

    MEASURED: 958 CRC-valid ESC_STATUS frames across two recorded sessions
    with NOTHING plugged in -- both index blocks, every rpm exactly 0. So the
    board fills all eight slots regardless, and any gate built on "frames are
    arriving" or "len(rpm) == 8" reports OK for a vehicle with no thrusters.

    The previous version of this test asserted exactly that false OK
    (`thrusters([0] * 8, esc_msgs=40) is OK`) and passed. It was testing the
    bug. The reporter now grades the driver's presence verdict instead, and a
    count of ESC messages is not an input to it at all.
    """
    assert R.thrusters((None, 'not announced')).state is State.UNKNOWN
    assert R.thrusters((False, 'thruster 3 LOST')).state is State.FAILED
    assert R.thrusters((True, 'all 8 thrusters reporting')).state is State.OK
    assert R.thrusters(None).state is State.UNKNOWN, 'no backend method = UNKNOWN'


def test_a_kill_switch_NOBODY_CAN_SEE_is_not_CLEAR():
    """⛔ UNKNOWN is not OK, and here it is not FAILED either.

    The switch lives on the 2nd board and reaches us over ESP-NOW; the firmware
    reports kill=false on link loss on purpose, so `KILL = 0` on the wire means
    "live" OR "nobody is telling us". A bench vehicle with no second board is a
    legitimate configuration, not a fault -- but it is emphatically not OK,
    because nothing is watching the switch that can stop the hull dead.
    """
    assert R.thruster_power(None).state is State.UNKNOWN
    assert R.thruster_power(True).state is State.FAILED
    assert R.thruster_power(False).state is State.OK


def test_UNKNOWN_thrusters_never_decays_into_OK():
    """UNKNOWN is the CORRECT answer on this hull today, not a defect to fix.

    The tempting repair -- count ESC messages and call it OK -- is the one the
    measurement above forbids. Pinned so nobody re-derives it.
    """
    assert R.thrusters((None, 'anything at all')).state is not State.OK


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


# --------------------------------------------------------------------------- #
#  The wiring: something must actually POLL it
# --------------------------------------------------------------------------- #
def _manager_stub(fc=None, det_hz=None):
    """The manager's health methods on a bare object.

    The methods under test are the SHIPPING ones, taken off the class, so they
    cannot drift from what runs -- the same reason `test_detector_recovery`
    stopped testing the handler in isolation.
    """
    from duburi_manager.auv_manager_node import AUVManagerNode as N
    m = SimpleNamespace()
    m.fc = fc or SimpleNamespace()
    # ⛔ THIS USED TO SET `m.vision`, AND PRODUCTION NEVER HAS SUCH AN ATTRIBUTE.
    # The stub manufactured it, so the D16 detector tests below were green against
    # a lookup the real node could not perform: `_detection_rate_hz` read
    # `getattr(self, 'vision', None)`, always missed, and the `detector` reporter
    # could only ever say UNKNOWN on a live vehicle. Mirror the real structure --
    # VisionStates live in the `_vision_states` pool, keyed by camera.
    m._vision_states = ({'forward': SimpleNamespace(stats=lambda: {'det_hz': det_hz})}
                        if det_hz is not None else {})
    m._health = HealthBoard()
    m._health_last = None
    lines = []
    m.get_logger = lambda: SimpleNamespace(
        info=lambda s: lines.append(('info', s)),
        warn=lambda s: lines.append(('warn', s)),
        error=lambda s: lines.append(('error', s)))
    m.lines = lines
    m._register_health = N._register_health.__get__(m)
    m._live_vision_state = N._live_vision_state.__get__(m)
    m._detection_rate_hz = N._detection_rate_hz.__get__(m)
    m._health_tick = N._health_tick.__get__(m)
    return m


def test_a_backend_MISSING_a_method_is_UNKNOWN_not_a_crash():
    """The manager runs against two backends and a sim. A health board that
    takes down the node it is watching is worse than no board."""
    m = _manager_stub(fc=SimpleNamespace())      # no link_alive, no ESCs
    m._register_health()
    states = {k: v.state for k, v in m._health.poll().items()}
    assert set(states) >= {'board_link', 'barometer', 'thrusters', 'detector'}
    assert all(s is not State.OK for s in states.values())


def test_the_tick_speaks_only_when_the_STATE_CHANGES():
    """A board that logs every second is a board nobody reads."""
    m = _manager_stub(fc=SimpleNamespace(link_alive=lambda: True))
    m._register_health()
    m._health_tick()
    n1 = len(m.lines)
    m._health_tick(); m._health_tick()
    assert len(m.lines) == n1, 'logged an unchanged state'


def test_a_DEAD_DETECTOR_reaches_the_board():
    """THE D16 SIGNAL. A detector that aborts keeps every topic and every
    process; only the RATE changes, and nothing was watching it."""
    m = _manager_stub(fc=SimpleNamespace(link_alive=lambda: True), det_hz=0.0)
    m._register_health()
    # NO explicit poll: the TICK must do it. Calling poll() here first is what
    # let an injected "the tick never polls" defect pass -- the test was doing
    # the work it was meant to be checking.
    m._health_tick()
    assert m._health.get('detector').state is State.FAILED
    assert any('detector' in s for _k, s in m.lines), m.lines


def test_a_HEALTHY_detector_does_not_trip_it():
    m = _manager_stub(fc=SimpleNamespace(link_alive=lambda: True), det_hz=42.0)
    m._register_health()
    m._health.poll()
    assert m._health.get('detector').state is State.OK


def test_the_gyro_is_actually_REQUESTED_from_the_board():
    """Flow de-rotation needs raw gyro, and SCALED_IMU2 was missing from the
    srot rate table entirely -- so it ran at the board's 10 Hz default while
    the camera ran at 30+, sharing one omega across three frames. Rotational
    flow does not depend on range, so a wrong omega is subtracted at full
    strength before the translation is ever scaled."""
    from pymavlink import mavutil
    from duburi_manager.auv_manager_node import SROT_MESSAGE_RATES
    mid = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2
    assert mid in SROT_MESSAGE_RATES, 'the estimator has no gyro'
    assert SROT_MESSAGE_RATES[mid] >= 50, 'gyro below the camera rate'


# --------------------------------------------------------------------------- #
#  B01 -- the reporters that were DEFINED and never REGISTERED
#
#  Nine reporters existed; six were registered. The three that were not
#  included `leak_sensor` -- the one written for a flooding hull, which returns
#  FAILED for the exact `LEAK_EN = 0` state this board was measured in. It had
#  never executed. These pin the wiring, not the reporters' own logic (which
#  was always correct and is covered above).
# --------------------------------------------------------------------------- #
def _registered_names():
    """Reporter names `_register_health` actually wires, without booting ROS.

    Reads the source rather than constructing the node: the node needs rclpy,
    a link and a HAL. The registration list is a literal, so the source IS the
    fact under test.
    """
    import ast
    src = (Path(__file__).resolve().parents[1]
           / 'duburi_manager' / 'auv_manager_node.py').read_text()
    names = set()
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'attr', None) == 'register'
                and node.args and isinstance(node.args[0], ast.Constant)):
            names.add(node.args[0].value)
    return names


def test_the_leak_reporter_is_registered():
    """B01. It grades a flooding hull and it was never wired in."""
    assert 'leak' in _registered_names(), (
        'leak_sensor is defined in health_reporters.py but not registered -- '
        'it can never run, and it is the reporter for a flooding hull')


def test_every_defined_reporter_is_registered_or_explained():
    """A reporter that exists but is not wired is dead code that LOOKS live.

    `target_lock` is the one deliberate exception: it needs the ladder's decayed
    authority, which nothing currently exposes, and inventing it from the /lock
    confidence would publish a made-up number in a health line.
    """
    import inspect
    from duburi_manager import health_reporters as _R
    # Health CONSTRUCTORS (ok/failed/degraded/unknown) live in health.py and are
    # re-exported here; they are not reporters. Reporters are the module's own.
    constructors = {'ok', 'failed', 'degraded', 'unknown'}
    defined = {n for n, f in inspect.getmembers(_R, inspect.isfunction)
               if not n.startswith('_')
               and n not in constructors
               and getattr(f, '__module__', '') == _R.__name__}
    registered = _registered_names()
    # name-in-source mapping: reporters are registered under short keys
    aliases = {'heading_reference': 'heading_ref', 'leak_sensor': 'leak'}
    missing = {d for d in defined
               if aliases.get(d, d) not in registered} - {'target_lock'}
    assert not missing, f'defined but never registered (and unexplained): {sorted(missing)}'


def test_leak_sensor_separates_NOT_WATCHING_from_DRY():
    """The distinction the whole finding is about.

    `LEAK_EN = 0` reads dry either way, so "no leak" and "nobody is looking"
    must not collapse into the same verdict.
    """
    assert R.leak_sensor(False, None).state is State.FAILED   # nothing watching
    assert R.leak_sensor(True, False).state is State.OK        # watching, dry
    assert R.leak_sensor(True, True).state is State.FAILED     # watching, wet
    assert R.leak_sensor(None, None).state is State.UNKNOWN    # no reading
    # and the not-watching case must SAY so, not just fail
    assert 'NOTHING IS WATCHING' in R.leak_sensor(False, None).evidence
