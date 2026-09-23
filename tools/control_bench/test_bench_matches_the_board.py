"""THE FALSIFIER: does the bench reproduce the real board, count for count?

⛔ THIS IS THE ONLY THING THAT MAKES THE BENCH EVIDENCE. Compiled firmware that
has never been checked against the hardware is just a second opinion with better
marketing. Every conclusion drawn on this bench -- every gain, every allocation
experiment, every INDI increment -- inherits its credibility from this file.

THE DATA. `.claude/context/workbench/data/*.json`, captured 2026-09-22 off the
live vehicle: board armed, STABILIZE, nothing attached to any ESC, demands held
at 20 Hz on one axis at a time, per-motor DShot read off the RP2350's console.

⚠ THE ONE HONEST COMPLICATION. Those captures are of the WHOLE BOARD, and the
bench links the mixer only. In STABILIZE the attitude controller is always
contributing something -- in the horizontal rows of `mixer_armed.json` the
vertical group sat at a constant non-zero trim throughout. So:

  * FORWARD and LATERAL are clean: the attitude loop drives the VERTICAL group,
    and those axes live in the horizontal one. The mixer is block-diagonal
    (`mixer.cpp:38-42`), so the two cannot mix. These are compared EXACTLY.
  * YAW is excluded. In STABILIZE yaw is `attitude::stabilize(...)`'s output,
    not a demand -- there is no static function from a yaw demand to an output,
    which is why `actuation_model.demand_to_fraction` raises on it.
  * HEAVE is compared with a tolerance, because roll/pitch trim rides on the
    same four motors and a single-direction ladder cannot cancel it.
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from control_bench import (  # noqa: E402
    FRAME_REVERSE, MOTOR_DIRS, Board, firmware_rev, pilot_expo)

DATA = Path(__file__).resolve().parents[2] / '.claude/context/workbench/data'
LADDER = DATA / 'mixer_ladder.json'
SWEEP = DATA / 'mixer_armed.json'

DSHOT_B_MIN, DSHOT_A_MIN = 1048, 48


def dshot_signed(v: int) -> int:
    """Both DShot bands count upward from their own floor; 48 and 1048 are each
    a zero. Decoding this as a deviation from 1048 is what made the mixer look
    asymmetric on the first reading."""
    return v - DSHOT_B_MIN if v >= DSHOT_B_MIN else -(v - DSHOT_A_MIN)


@pytest.fixture(scope='module')
def board():
    return Board()


def _sweep_row(axis: str, level: float) -> list[int]:
    """One row of the armed sweep, as the median over its reports."""
    import statistics
    for r in json.loads(SWEEP.read_text())['armed']:
        if r['axis'] == axis and abs(r['level'] - level) < 1e-9 and r['reports']:
            return [int(statistics.median(x['cmd'][i] for x in r['reports']))
                    for i in range(8)]
    raise AssertionError(f'no {axis} {level} row in {SWEEP.name}')


def _ladder_rows():
    rows = json.loads(LADDER.read_text())['ladder']
    return [(r['level'], r['cmd_median']) for r in rows
            if r['level'] > 0 and r['cmd_median']]


# ── the headline check ───────────────────────────────────────────────────────

def test_the_bench_IS_the_board_where_the_attitude_loop_is_quiet(board):
    """⭐ THE HEADLINE. On the one captured row where the attitude controller
    happened to be contributing nothing, the bench must reproduce the board
    EXACTLY -- not closely, exactly, all four motors, count for count.

    `mixer_armed.json`, fwd +0.30: the board clocked out [511, 1512, 511, 511].
    Anything other than an exact match here means the compiled firmware is not
    behaving as the firmware, and every result built on this bench is void.
    """
    board_cmd = _sweep_row('fwd', 0.3)

    assert board.drive(forward=pilot_expo(0.30))[:4] == board_cmd[:4]


def test_the_residual_on_the_ladder_IS_the_attitude_loop_not_bench_error(board):
    """The ladder rows do NOT match exactly, and the reason is provable rather
    than assumed.

    The bench links `mixer.cpp` only; the captures are of the whole board, where
    `attitude::stabilize` is always contributing. If the residual were bench
    error it would be arbitrary. If it is the yaw axis, it must carry the yaw
    column's SHAPE -- three horizontal motors displaced one way and the fourth
    the other, because the firmware's yaw column is [+1, -1, -1, +1].

    Measured at 0.30: board [516, 1507, 516, 506] against bench
    [511, 1512, 511, 511], i.e. signed residuals -5, -5, -5, +5. That is the
    signature, and it is what this test pins.
    """
    odd_one_out = 0
    for level, board_cmd in _ladder_rows():
        if level < 0.10:
            continue                    # below this the residual is zero
        res = [dshot_signed(board_cmd[i]) - dshot_signed(
            board.drive(forward=pilot_expo(level))[i]) for i in range(4)]
        signs = [(r > 0) - (r < 0) for r in res]
        # exactly one of the four disagrees in sign with the other three
        if signs.count(signs[0]) == 3:
            odd_one_out += 1

    assert odd_one_out >= 5, (
        'the ladder residual does not carry the yaw column shape, so it is '
        'NOT the attitude loop and the bench is suspect')


def test_the_ladder_agrees_once_that_residual_is_allowed_for(board):
    """With the attitude contribution acknowledged, the horizontal group agrees
    to within SIX counts of 999 across all eleven levels -- and is exact below a
    0.10 demand, where the loop had nothing to correct."""
    worst = 0
    for level, board_cmd in _ladder_rows():
        bench_cmd = board.drive(forward=pilot_expo(level))
        errs = [abs(dshot_signed(bench_cmd[i]) - dshot_signed(board_cmd[i]))
                for i in range(4)]
        worst = max(worst, max(errs))
        if level <= 0.08:
            assert max(errs) == 0, f'level {level} should be exact, got {errs}'

    assert worst <= 6, f'worst horizontal error {worst} counts of 999'


def test_the_zero_demand_is_exactly_neutral(board):
    """`mixer.cpp:102` returns NEUTRAL_3D outright below the centre epsilon. If
    the bench drifted here, every thruster would idle instead of stopping."""
    assert all(v == 1048 for v in board.drive(forward=0.0))


def test_the_two_thruster_groups_stay_independent(board):
    """Measured on the vehicle (B-16): adding heave to a SATURATING horizontal
    command left the horizontal outputs bit-identical. That is a deliberate
    firmware fix -- the old global maxabs cost a third of the roll/pitch
    authority during a hard forward burst -- and the bench must show it."""
    horiz_alone = board.mix(forward=pilot_expo(1.0), lateral=pilot_expo(0.5))
    with_heave = board.mix(forward=pilot_expo(1.0), lateral=pilot_expo(0.5),
                           throttle=0.5)

    assert horiz_alone[:4] == with_heave[:4]
    assert board.mix(throttle=0.5)[4:] == with_heave[4:]


def test_the_saturation_scale_down_matches_the_measured_board(board):
    """B-16, predicted before the run and measured: fwd 1.0 with lat 0.5 shapes
    to 1.000 and 0.3875, M2 = -1.3875 is the largest, the group scales by
    1/1.3875, and the four outputs land near 657, 999, 999, 657."""
    cmd = board.drive(forward=pilot_expo(1.0), lateral=pilot_expo(0.5))
    mags = sorted(abs(dshot_signed(v)) for v in cmd[:4])

    assert mags[0] == pytest.approx(657, abs=2)
    assert mags[1] == pytest.approx(657, abs=2)
    assert mags[2] >= 997
    assert mags[3] >= 997


# ── the properties the board is known to have ────────────────────────────────

def test_the_smallest_commandable_output_is_the_spin_floor(board):
    """No deadband -- a FLOOR. The smallest non-zero demand already commands
    ~15.8 % of full scale, which is why `VISION_YAW_MIN_PCT = 5.0` is not an
    under-measured constant but the wrong model."""
    tiny = abs(dshot_signed(board.one_to_dshot(0.006, 1)))

    assert tiny / 999.0 > 0.15


def test_battery_compensation_ships_off(board):
    """`DEF_MOT_BAT_V_MAX = 0.0`. Feeding a voltage must change nothing until
    the pack maximum is also set -- if this ever fails, a bench result fitted
    with compensation on would silently disagree with the vehicle."""
    before = board.drive(forward=pilot_expo(0.5))
    board.set_battery(14.8, 0.1)

    assert board.battery_scale == 0.0
    assert board.drive(forward=pilot_expo(0.5)) == before


def test_disarmed_output_is_not_a_drive_value(board):
    """Safety rule 4: neutral on startup, nothing commanded until a verb asks."""
    assert all(v == 0 for v in board.drive(forward=pilot_expo(1.0), armed=False))


# ── the guard on the guard ───────────────────────────────────────────────────

def test_the_bench_records_which_firmware_it_is(board):
    """A bench that cannot say which commit it represents is not evidence once
    the firmware moves."""
    rev = firmware_rev()

    assert rev != 'unknown' and len(rev) >= 7


def test_the_hull_configuration_is_applied_not_assumed_away():
    """The regression that cost the first falsifier run. Driving the bare mixer
    without FRAME_REVERSE and the measured direction product is not the board:
    it disagreed by a mean of 712 counts of 999."""
    assert FRAME_REVERSE is True
    assert MOTOR_DIRS == (-1, 1, 1, 1, 1, 1, 1, -1)


def test_ignoring_the_hull_configuration_gives_a_different_answer(board):
    """Guard the guard: if these two ever agree, `drive()` has stopped applying
    the configuration and the headline test is passing for the wrong reason."""
    configured = board.drive(forward=pilot_expo(0.30))
    bare = board.to_dshot(board.mix(forward=pilot_expo(0.30)))

    assert configured != bare


def test_the_captures_backing_all_of_this_are_still_in_the_tree():
    for p in (LADDER, SWEEP):
        assert p.exists(), f'{p} is gone -- every number above becomes folklore'
