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
    (0.0, 0.0),                 # inside the deadband -- stays an exact zero
    (0.5, mv.SROT_YAW_MIN_PCT),
    (2.0, mv.SROT_YAW_MIN_PCT),
    (2.8, mv.SROT_YAW_MIN_PCT),
    (2.86, 2.86),
    (5.0, 5.0),
    (30.0, 30.0),
])
def test_a_sub_gate_correction_is_lifted_to_something_that_actuates(mag, want):
    assert mv._srot_yaw_expressible(mag, 100.0) == pytest.approx(want)


def test_an_exact_zero_is_never_lifted():
    """⛔ The deadband must stay a deadband. Lifting 0 would make the loop
    command 17 % of full scale while it is already on target."""
    assert mv._srot_yaw_expressible(0.0, 100.0) == 0.0


def test_the_callers_speed_cap_is_respected():
    """A mission that asked for a gentle yaw does not get a lurch because of
    this. If the cap is below the floor the axis simply cannot actuate, and the
    cap wins -- we do not silently exceed what was asked for."""
    assert mv._srot_yaw_expressible(0.5, 2.0) == pytest.approx(2.0)
    assert mv._srot_yaw_expressible(50.0, 10.0) == pytest.approx(10.0)


def test_it_never_rounds_a_correction_DOWN():
    """⛔ THE BUG THIS REPLACES, and it was shipped for one commit. Rounding a
    sub-gate correction down to zero stalls the hole-lock just outside err_px --
    the exact failure the yaw floor was written for. Worked example: with
    kp_yaw scaled to 10 by range_gain_floor, the old taper actuated across the
    upper 43 % of the approach band and the round-down law actuated nowhere
    until the band edge."""
    for mag in (0.1, 1.0, 2.0, 2.8):
        assert mv._srot_yaw_expressible(mag, 100.0) >= mag
        assert mv._srot_yaw_expressible(mag, 100.0) > 0.0


def test_a_sub_gate_demand_does_not_disturb_the_boards_heading_hold():
    """⚠ A RETRACTION, kept as a test so it is not re-argued. The first version
    of this change claimed a true zero leaves heading hold engaged while a
    sub-gate demand takes it away. That is false: both land in the same branch
    of `attitude::stabilize`, and the bench measured identical hold torque
    (-0.097075) for sticks of 0.0, 0.5, 1.0, 2.0 and 2.8 % against a 5 deg
    heading error. The reason to act on a sub-gate demand is that it produces no
    thrust -- nothing to do with the hold."""
    import math
    import sys
    from pathlib import Path
    bench = HERE.parents[2] / 'tools' / 'control_bench'
    if not (bench / 'libcontrolbench.so').exists():
        pytest.skip('control bench not built')
    sys.path.insert(0, str(bench))
    from control_bench import Board

    b = Board().from_board()

    def hold_torque(pct):
        b.reset()
        b.hold_yaw(0.0)
        for _ in range(50):
            _, _, y = b.stabilize(stick_yaw=pct / 100.0, yaw=math.radians(5.0))
        return y

    baseline = hold_torque(0.0)
    for pct in (0.5, 1.0, 2.0, 2.8):
        assert hold_torque(pct) == pytest.approx(baseline), (
            f'{pct} % disturbed the hold -- the retracted claim would be true '
            f'after all, and the docstrings need changing back')


def test_the_taper_is_left_alone_for_the_ardusub_path():
    """⛔ DO NOT 'SIMPLIFY' THIS AWAY. `_vision_yaw_floor` fixed a real observed
    close-in wobble, and on ArduSub -- where PWM is continuous and there is no
    MOT_SPIN_MIN relay -- the taper does what it says. Only srot snaps."""
    tapered = mv._vision_yaw_floor(15.0, 10.0, 5.0)
    assert 0.0 < tapered < 5.0, 'the taper must still produce intermediate values'
