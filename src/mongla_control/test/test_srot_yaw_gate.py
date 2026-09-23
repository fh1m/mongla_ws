"""The SROT yaw gate: measured on the board's own code, not assumed.

⛔ WHAT THIS REPLACES. `VISION_YAW_MIN_PCT = 5.0` labels itself "a hardware
spin-up assumption, NOT a measured value". On the srot backend it is now
measured, and it is wrong in both directions at once: too high to be the
smallest usable command, and attached to a taper that cannot taper.

The firmware fact underneath, `attitude_control.cpp:87`:

    if (fabsf(yaw_stick) > 0.02f)  -> yaw is a RATE COMMAND
    else                           -> the board HOLDS THE LAST HEADING

compared AFTER PILOT_EXPO. Measured through `tools/control_bench` -- the
board's own attitude_control.cpp and mixer.cpp compiled natively -- under the
live vehicle's parameters, 2026-09-23:

    stick 2.80 %  ->  torque 0.000000  ->   0.00 % output
    stick 2.86 %  ->  torque 0.010369  ->  17.42 % output
    stick 5.00 %  ->  torque 0.018140  ->  19.12 % output
"""
import os
import re
from pathlib import Path

import pytest

from mongla_control import actuation_model as am
from mongla_control import motion_vision as mv

HERE = Path(__file__).resolve().parent
FIRMWARE = Path(os.environ.get(
    'SROT_FIRMWARE',
    HERE.parents[4] / 'Mongla_others' / 'srot-control-board'))


# --------------------------------------------------------------------------- #
#  The constant is the firmware's, and a test says so
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not FIRMWARE.exists(), reason='no firmware checkout')
def test_the_yaw_gate_constant_is_the_one_in_the_firmware():
    """`STABILIZE_YAW_STICK_GATE` is a SECOND COPY of a firmware literal,
    because the firmware does not expose it as a parameter (upstream ask N).
    One truth, two copies is the bug -- so this reads theirs."""
    src = (FIRMWARE / 'src/control/attitude_control.cpp').read_text()
    m = re.search(r'fabsf\(yaw_stick\)\s*>\s*([0-9.]+)f', src)
    assert m, 'the yaw stick gate is no longer a literal comparison -- re-read it'
    assert float(m.group(1)) == pytest.approx(am.STABILIZE_YAW_STICK_GATE), (
        f'firmware gates yaw at {m.group(1)}, actuation_model says '
        f'{am.STABILIZE_YAW_STICK_GATE}')


def test_the_floor_is_derived_from_the_gate_and_the_expo():
    """Not a magic number: invert PILOT_EXPO at the gate."""
    floor = am.yaw_rate_command_floor()
    assert am.pilot_expo(floor) == pytest.approx(am.STABILIZE_YAW_STICK_GATE, abs=1e-6)
    assert floor == pytest.approx(0.028561, abs=1e-5)


def test_a_different_expo_moves_the_floor():
    """The floor is not a constant of the vehicle, it is a function of a
    parameter an operator can change. Zero expo makes stick and gate equal."""
    assert am.yaw_rate_command_floor(0.0) == pytest.approx(
        am.STABILIZE_YAW_STICK_GATE, abs=1e-6)
    # More expo means finer resolution near centre, so MORE stick is needed to
    # reach the same post-expo value: the floor RISES with expo. At expo 1.0 the
    # shaping is pure cubic, so the gate sits at 0.02 ** (1/3) = 0.271.
    assert am.yaw_rate_command_floor(1.0) > am.yaw_rate_command_floor(0.0)
    assert am.yaw_rate_command_floor(1.0) == pytest.approx(0.02 ** (1 / 3), abs=1e-5)


# --------------------------------------------------------------------------- #
#  The snap
# --------------------------------------------------------------------------- #

def test_srot_floor_is_below_the_guessed_constant():
    assert mv.SROT_YAW_MIN_PCT == pytest.approx(2.8561, abs=1e-3)
    assert mv.SROT_YAW_MIN_PCT < mv.VISION_YAW_MIN_PCT
    # and the guess is 1.75x the measured floor -- recorded so the ratio is
    # visible if either number is ever edited
    assert mv.VISION_YAW_MIN_PCT / mv.SROT_YAW_MIN_PCT == pytest.approx(1.751, rel=1e-2)


@pytest.mark.parametrize('mag,want', [
    (0.0, 0.0),
    (0.5, 0.0),
    (2.0, 0.0),
    (2.8, 0.0),         # measured: exactly zero output on the board
    (2.86, 2.86),       # measured: 17.42 % output
    (5.0, 5.0),
    (30.0, 30.0),
])
def test_sub_gate_yaw_collapses_to_a_true_zero(mag, want):
    assert mv._srot_yaw_expressible(mag) == pytest.approx(want)


def test_the_snap_never_invents_authority():
    """It may only zero a value or pass it through -- never raise one. Flooring
    a demand UP to the gate is the behaviour this replaces: it buys a 17 %
    lurch and takes the board's heading hold away."""
    for mag in (0.0, 1.0, 2.0, 2.8, 2.9, 5.0, 50.0):
        out = mv._srot_yaw_expressible(mag)
        assert out in (0.0, mag)
        assert out <= mag


def test_the_taper_is_left_alone_for_the_ardusub_path():
    """⛔ DO NOT 'SIMPLIFY' THIS AWAY. `_vision_yaw_floor` fixed a real observed
    close-in wobble, and on ArduSub -- where PWM is continuous and there is no
    MOT_SPIN_MIN relay -- the taper does what it says. Only srot snaps."""
    tapered = mv._vision_yaw_floor(15.0, 10.0, 5.0)
    assert 0.0 < tapered < 5.0, 'the taper must still produce intermediate values'
