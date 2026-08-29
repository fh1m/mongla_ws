"""Gamepad teleop.

The full chain (pad -> kernel js -> reader -> TeleopStreamer -> MAVLink ->
hull moves) is measured against a running sim with a virtual gamepad created
through /dev/uinput; the numbers are in JOYSTICK.md. These pin the parts that
break silently.
"""

import struct
import threading
import time

import pytest

from duburi_sim_web.joystick import (
    DEFAULT_AXIS_MAP,
    DEFAULT_BUTTON_MAP,
    GAIN_MAX,
    GAIN_MIN,
    JS_EVENT_AXIS,
    JS_EVENT_BUTTON,
    JS_EVENT_INIT,
    JoystickReader,
    shape,
)

_JS_EVENT = struct.Struct('IhBB')


class FakeTeleop:
    def __init__(self):
        self.calls = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)
        self.gain = None
        self.lock = threading.Lock()

    def set_axes(self, fwd, lat, up, yaw, gain=None):
        with self.lock:
            self.calls += 1
            self.axes = (fwd, lat, up, yaw)
            self.gain = gain


def _reader(**kw):
    return JoystickReader(FakeTeleop(), device='/dev/null', **kw)


def test_deadzone_kills_stick_slop():
    """Sticks do not return to exactly zero; without this the vehicle creeps."""
    assert shape(0.0, 0.08, 0.35) == 0.0
    assert shape(0.05, 0.08, 0.35) == 0.0
    assert shape(-0.05, 0.08, 0.35) == 0.0
    assert shape(0.10, 0.08, 0.35) > 0.0


def test_deadzone_does_not_leave_a_step():
    """Just past the deadzone the output must start near zero, not jump.

    A naive `if |v| > dz: return v` starts the axis at 0.08 of full authority
    the instant the stick leaves centre, which reads as a lurch.
    """
    assert shape(0.081, 0.08, 0.35) < 0.01


def test_full_deflection_is_full_authority():
    """Expo must shape the middle without stealing the top."""
    assert shape(1.0, 0.08, 0.35) == pytest.approx(1.0, abs=1e-6)
    assert shape(-1.0, 0.08, 0.35) == pytest.approx(-1.0, abs=1e-6)


def test_expo_softens_the_middle():
    """Half stick gives well under half authority -- that is the point."""
    half = shape(0.5, 0.08, 0.35)
    assert 0.25 < half < 0.40
    # Monotonic, or the stick would feel non-deterministic.
    assert shape(0.3, 0.08, 0.35) < half < shape(0.7, 0.08, 0.35)


def test_axis_map_matches_a_standard_pad():
    """Verified against a real Logitech F310 in XInput mode.

    It reports axes 0-7 (LS x/y = 0/1, triggers = 2/5, RS x/y = 3/4) and
    buttons 0-10 (A/B/X/Y = 0-3, LB/RB = 4/5). A pad that does NOT declare
    triggers shifts every later index, which is why these are parameters and
    why QGC ships a joystick calibration page.
    """
    assert DEFAULT_AXIS_MAP == {'lat': 0, 'fwd': 1, 'yaw': 3, 'up': 4}
    assert DEFAULT_BUTTON_MAP[0] == 'arm'
    assert DEFAULT_BUTTON_MAP[1] == 'disarm'
    assert DEFAULT_BUTTON_MAP[4] == 'gain_down'
    assert DEFAULT_BUTTON_MAP[5] == 'gain_up'


def test_the_kernel_init_burst_cannot_arm_the_vehicle():
    """At open the kernel replays every control's CURRENT state.

    Those events carry JS_EVENT_INIT. Their axis values are real and must be
    applied, but a button reported as held in that burst is a state report,
    not an operator action -- firing arm() from it would arm the vehicle the
    moment the pad is plugged in.
    """
    fired = []
    r = JoystickReader(FakeTeleop(), device='/dev/null', on_button=fired.append)

    r._handle(0, 1, JS_EVENT_BUTTON | JS_EVENT_INIT, 0)
    assert fired == [], 'the init burst armed the vehicle'

    r._handle(0, 1, JS_EVENT_BUTTON, 0)
    assert fired == ['arm']

    # An init AXIS value is real state and must be applied.
    r._handle(0, -32767, JS_EVENT_AXIS | JS_EVENT_INIT, 1)
    assert r.snapshot()['axes']['fwd'] == pytest.approx(1.0, abs=1e-3)


def test_forward_and_vertical_are_inverted():
    """The kernel reports stick-up as NEGATIVE.

    Unflipped, pushing the stick forward drives the vehicle backwards -- and
    nothing errors.
    """
    r = _reader()
    r._handle(0, -32767, JS_EVENT_AXIS, DEFAULT_AXIS_MAP['fwd'])
    assert r.snapshot()['axes']['fwd'] > 0.9
    r._handle(0, -32767, JS_EVENT_AXIS, DEFAULT_AXIS_MAP['up'])
    assert r.snapshot()['axes']['up'] > 0.9
    # Yaw and lateral are NOT inverted: right is positive on both.
    r._handle(0, 32767, JS_EVENT_AXIS, DEFAULT_AXIS_MAP['yaw'])
    assert r.snapshot()['axes']['yaw'] > 0.9


def test_gain_buttons_step_and_clamp():
    r = _reader(gain=0.55)
    for _ in range(20):
        r._press(5)                      # RB
    assert r.snapshot()['gain'] == pytest.approx(GAIN_MAX)
    for _ in range(40):
        r._press(4)                      # LB
    assert r.snapshot()['gain'] == pytest.approx(GAIN_MIN)


def test_a_held_stick_keeps_being_pushed():
    """THE bug that made teleop twitchy.

    A held stick produces NO further events -- the kernel reports changes. If
    the reader only pushes on an event, TeleopStreamer's 0.35 s watchdog
    centres the vehicle while the operator is still holding full forward.
    Measured before the fix: 6 s of full forward moved the hull 0.188 m; after,
    1.220 m.
    """
    teleop = FakeTeleop()
    r = JoystickReader(teleop, device='/dev/null')
    r._handle(0, -32767, JS_EVENT_AXIS, DEFAULT_AXIS_MAP['fwd'])

    before = teleop.calls
    for _ in range(5):
        r._push()                        # no new events in between
    assert teleop.calls == before + 5
    assert teleop.axes[0] > 0.9, 'the held value must persist across pushes'


def test_snapshot_reports_enough_to_map_an_unknown_pad():
    """Operators need raw state to map a pad that is not an F310."""
    r = _reader()
    snap = r.snapshot()
    for key in ('connected', 'device', 'name', 'axes', 'buttons', 'gain',
                'deadzone', 'expo', 'events'):
        assert key in snap
