"""An unwired sense pin must not be reported as a battery.

⛔ MEASURED ON THE VEHICLE, 2026-09-22, exclusive port, 41 consecutive frames of
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
