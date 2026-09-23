"""`check_yaw_authority` -- refuse a yaw axis that cannot produce thrust.

⛔ THE COUPLING. Two firmware literals stand between a host yaw demand and a
turning propeller: the `0.02f` stick gate in `attitude_control.cpp:87` and the
`0.005f` centre gap in `mixer.cpp:102`. Crossing the first is only useful if it
also crosses the second, and whether it does depends on two parameters an
operator can change:

    0.02 * PILOT_YAW_RATE(rad/s) * ATC_RAT_YAW_P  >  0.005

This board reads 160 deg/s and 0.18 -> 0.0101, comfortable. `config.h` defaults
to 45 deg/s -> 0.0028, where yaw demands are ACCEPTED and produce NO thrust.
"""
import math

import pytest

from mongla_control.fc import srot_protocol as sp
from mongla_control.fc.srot_fc import SrotFC


class _Params:
    """Only what `check_yaw_authority` touches."""

    def __init__(self, rate, gain):
        self._vals = {'PILOT_YAW_RATE': rate, 'ATC_RAT_YAW_P': gain}

    def get_param(self, name, timeout=3.0):
        return self._vals.get(name)


def _check(rate, gain):
    return SrotFC.check_yaw_authority(_Params(rate, gain))


def test_this_board_passes():
    """160 deg/s and 0.18 -- read off the vehicle 2026-09-23."""
    ok, msg = _check(160.0, 0.18)
    assert ok, msg
    assert '160' in msg


def test_the_config_h_default_is_refused():
    """⛔ 45 deg/s is what a params reset restores, silently. At that rate a
    demand past the stick gate produces exactly zero thrust."""
    ok, msg = _check(45.0, 0.18)
    assert ok is False
    assert 'DEAD BAND' in msg
    assert '80' in msg, 'the message must say what to raise it to'


def test_the_break_even_is_where_the_arithmetic_says():
    """Not a guessed threshold: it is where the stick gate's torque equals the
    mixer's centre gap."""
    gain = 0.18
    break_even = math.degrees(sp.MIXER_CENTRE_EPS
                              / (sp.STABILIZE_YAW_STICK_GATE * gain))
    assert 75.0 < break_even < 85.0
    assert _check(break_even * 1.02, gain)[0] is True
    assert _check(break_even * 0.98, gain)[0] is False


def test_a_low_rate_gain_opens_the_same_band():
    """⚠ It is not only PILOT_YAW_RATE. Detuning ATC_RAT_YAW_P does it too, and
    that is a thing anyone tuning the rate loop will try."""
    assert _check(160.0, 0.18)[0] is True
    assert _check(160.0, 0.05)[0] is False


def test_an_unread_parameter_degrades_the_report_and_does_not_block():
    """Best-effort, like `check_move_cruise_max`. Refusing to fly because a
    parameter would not read is its own failure mode."""
    ok, msg = _check(None, 0.18)
    assert ok is True
    assert 'UNKNOWN' in msg


def test_the_copied_literals_match_the_actuation_model():
    """One truth, two copies is the bug. `srot_protocol` and `actuation_model`
    both carry the stick gate; a test comparing them is the cheap fix until the
    firmware exposes it as a parameter (upstream ask N section 3)."""
    from mongla_control import actuation_model as am
    assert sp.STABILIZE_YAW_STICK_GATE == am.STABILIZE_YAW_STICK_GATE
    assert sp.MIXER_CENTRE_EPS == am.CENTRE_EPS
