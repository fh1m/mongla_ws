"""An unwired sense pin must not be reported as a battery.

Measured on the vehicle, 2026-09-22, exclusive port, 41 consecutive frames of
BATTERY_STATUS instance 0 with nothing else reading the link:

    span 1.39 .. 25.05 V      sd 8.72 V
    MEDIAN step between consecutive samples 6.48 V   (max 20.42 V)
    57.5 % of steps exceed 2 V;  67.5 % exceed 0.20 V

`get_batteries()` already documented the cause -- "On this vehicle PM1 reads
~1.3 V because nothing is wired to GPIO36" -- and the stack promoted that
floating ADC to `battery_voltage` anyway. The manager printed `BAT main 1.39V`
on a perfectly healthy bench, which to an operator or a failsafe reads as a
vehicle about to die.

The cost was not only cosmetic. `/mongla/state` publishes ON CHANGE, with a
0.20 V battery threshold. In one manager run on that bench, **2939 of 2939
state-change publications were this pin** -- every single one. Real state
changes had nowhere to appear.

Sibling coverage: `test_srot_fc.py` already tests WHICH instance is reported and
how it ages out. This file tests whether the value is physically possible, which
is a different question and was not asked anywhere.

The statistic is the MEDIAN step, not the max: one glitchy sample from a real
pack must not blank it, and a pin that is noise on every sample must not hide
behind a few quiet ones.
"""
import math

import pytest

from mongla_control.fc import srot_protocol as sp


class _Msg:
    """The parts of BATTERY_STATUS that `note_battery` reads.

    voltages: 10 x uint16 millivolts, 0xFFFF = unpopulated cell.
    current_battery: int centiamps, -1 = unknown.
    """

    def __init__(self, bid, volts_v, current_a=None):
        self.id = bid
        self.voltages = [int(volts_v * 1000)] + [0xFFFF] * 9
        self.current_battery = -1 if current_a is None else int(current_a * 100)


@pytest.fixture()
def fc():
    """A bare SrotFC with only the battery bookkeeping initialised -- the real
    constructor opens a serial port."""
    from mongla_control.fc.srot_fc import SrotFC

    obj = SrotFC.__new__(SrotFC)
    obj._battery_cache = {}
    obj._battery_steps = {}
    obj._battery_last = {}
    obj._drain_battery = lambda: None
    return obj


# The first twelve voltages of the real capture, in order.
FLOATING_PIN_V = [22.074, 23.466, 24.504, 17.331, 12.487, 16.042,
                  24.651, 6.583, 24.693, 23.154, 14.391, 24.860]
# A 4S pack idling, then sagging under a thruster burst.
REAL_PACK_V = [16.71, 16.70, 16.68, 16.69, 16.65, 16.20, 15.80, 15.92,
               16.31, 16.58, 16.66, 16.70]


def _feed(fc, volts, bid=0):
    for v in volts:
        fc.note_battery(_Msg(bid, v))


def test_the_floating_pin_is_not_trusted(fc):
    """THE REGRESSION, with the vehicle's own numbers."""
    _feed(fc, FLOATING_PIN_V)

    assert fc.battery_trusted(0) is False


def test_a_real_pack_including_a_load_sag_stays_trusted(fc):
    """A 1 V sag under a thruster burst is a battery doing its job. Blanking it
    would lose the reading exactly when it matters."""
    _feed(fc, REAL_PACK_V)

    assert fc.battery_trusted(0) is True


def test_one_glitch_does_not_blank_a_real_pack(fc):
    """A single corrupt sample must not cost the operator the battery reading --
    which is why the median is the statistic and not the max."""
    volts = list(REAL_PACK_V)
    volts[6] = 0.4                      # one bad frame

    _feed(fc, volts)

    assert fc.battery_trusted(0) is True


def test_an_untrusted_instance_reports_nan_not_a_number(fc):
    """Absence renders `--`, never a value. NaN is how this tree spells that,
    and the manager's formatter turns it into `--`."""
    _feed(fc, FLOATING_PIN_V)

    volts = fc.get_batteries()[0]['voltage']

    assert math.isnan(volts)


def test_a_trusted_instance_still_reports_its_voltage(fc):
    _feed(fc, REAL_PACK_V)

    assert fc.get_batteries()[0]['voltage'] == pytest.approx(REAL_PACK_V[-1])


def test_it_refuses_to_accuse_an_instance_on_too_little_evidence(fc):
    """Below the sample floor there is not enough to judge, and guessing wrong
    early would blank a good battery at the start of every connection -- the
    same reason `_baro_healthy` is tri-state."""
    _feed(fc, FLOATING_PIN_V[:3])

    assert fc.battery_trusted(0) is True
    assert not math.isnan(fc.get_batteries()[0]['voltage'])


def test_instances_are_judged_independently(fc):
    """A floating PM1 must not condemn a real PM2 arriving over ESP-NOW."""
    _feed(fc, FLOATING_PIN_V, bid=0)
    _feed(fc, REAL_PACK_V, bid=1)

    assert fc.battery_trusted(0) is False
    assert fc.battery_trusted(1) is True
    assert math.isnan(fc.get_batteries()[0]['voltage'])
    assert not math.isnan(fc.get_batteries()[1]['voltage'])


def test_the_bar_sits_between_the_two_measured_regimes():
    """Guard the constant itself. The floating pin steps a median 6.48 V; a pack
    sags about 1 V. A bar outside that gap silently stops separating them."""
    assert 1.0 < sp.BATTERY_MAX_STEP_V < 6.48


def test_the_battery_on_the_state_topic_is_guarded_too(fc):
    """Kutum on the real board, 2026-11-22: /mongla/state read 1.39, 2.47,
    13.77, 1.39 V within a minute from the unwired PM1 pin. The guard sat in
    get_batteries(); the manager reads get_battery(), which skipped it."""
    _feed(fc, FLOATING_PIN_V, bid=0)
    assert math.isnan(fc.get_battery()['voltage'])
    _feed(fc, REAL_PACK_V * 2, bid=0)     # a whole window, so the pin's jumps have left it
    assert not math.isnan(fc.get_battery()['voltage'])


# The same pin on Kutum, 2026-11-22, one board sample a second: the value each
# second, a held value repeated for as long as the board held it.
FLOATING_PIN_HELD_V = ([10.19] + [1.39] * 5 + [13.83, 11.99, 13.79, 3.25, 3.03]
                       + [1.39] * 3 + [5.86, 13.51, 13.85, 14.03, 14.11, 13.02]
                       + [1.39] * 6 + [12.61, 14.22, 13.89, 14.0, 12.69])


def test_a_pin_that_holds_a_value_between_jumps_is_not_trusted(fc):
    """The median step is 0 V on this pin (it holds a value for seconds) and
    it sits at 13-14 V like a real pack, so the median passed it to
    /mongla/state. A real pack never jumps 2 V; one glitch jumps twice."""
    _feed(fc, FLOATING_PIN_HELD_V)
    assert fc.battery_trusted(0) is False


def test_a_condemned_pin_stays_condemned_through_its_quiet_spells(fc):
    """Kutum, 2026-11-22: condemned, the pin then held 1.39 V long enough that
    fewer than three jumps were left in the window, and /mongla/state read
    1.4 V again. A line is released only by a whole window with no jump."""
    flips = []
    for v in FLOATING_PIN_HELD_V + [1.39] * 14 + [13.0] + FLOATING_PIN_HELD_V:
        fc.note_battery(_Msg(0, v))
        flips.append(fc.battery_trusted(0))
    first = flips.index(False)
    assert not any(flips[first:]), f'trusted again at sample {flips.index(True, first)}'
    _feed(fc, REAL_PACK_V * 2)
    assert fc.battery_trusted(0) is True


# Kutum, 2026-11-22 10:23, PM1 unwired, /mongla/state recorded for 80 s: (s, V)
# at each change, one BATTERY_STATUS a second. The pin climbs from its 1.391 V
# rest in steps under the 2 V bar and drops back once, so a 16-sample window
# never holds three jumps and the jump rule alone passed it.
_RAMP_CHANGES = [(0, 1.908), (1, 1.391), (8, 1.423), (9, 2.484), (10, 2.305),
                 (11, 1.391), (19, 2.525), (20, 1.391), (21, 2.964), (22, 3.618),
                 (23, 1.391), (30, 3.934), (31, 2.506), (32, 3.912), (33, 1.391),
                 (40, 2.748), (41, 3.937), (42, 3.993), (43, 1.391), (44, 1.413),
                 (45, 1.391), (51, 2.744), (52, 3.799), (53, 3.871), (54, 1.685),
                 (55, 1.391), (62, 1.545), (63, 1.83), (64, 2.3), (65, 1.391),
                 (70, 1.519), (71, 1.391), (72, 3.309), (73, 1.423), (74, 1.391),
                 (79, 3.667), (80, 3.902)]
FLOATING_PIN_RAMP_V = [next(v for t0, v in reversed(_RAMP_CHANGES) if t0 <= t)
                       for t in range(81)]


def test_a_pin_that_ramps_under_the_jump_bar_is_not_trusted(fc):
    """No lithium pack in service reads under 3.0 V (an empty cell; 1S is the
    smallest pack), and this pin rests at 1.391 V for seconds at a time. Two
    samples under the floor in a window condemn it; one is a glitch."""
    flips = []
    for v in FLOATING_PIN_RAMP_V:
        fc.note_battery(_Msg(0, v))
        flips.append(fc.battery_trusted(0))
    first = flips.index(False)
    assert first <= sp.BATTERY_STEP_MIN_SAMPLES + 1
    assert not any(flips[first:]), f'trusted again at sample {flips.index(True, first)}'
