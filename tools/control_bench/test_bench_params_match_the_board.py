"""The bench must be able to BE the board, and must say when it is not.

⛔ WHY THIS FILE EXISTS. On 2026-09-23 an entire session of yaw measurements was
produced against `Board().defaults()`. The board does not run the defaults:
`config.h` ships `DEF_PILOT_YAW_RATE = 45.0` and the vehicle reads back
**160.0**, so every yaw torque was low by 160/45 = 3.556x. The mixer-ladder
falsifier could not have caught it, because `mixer.cpp` reads no gain that has
been retuned -- the divergence lived entirely in the attitude cascade.

These tests are the guard that replaces the assumption.
"""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from control_bench import BOARD_PARAMS, Board, firmware_rev  # noqa: E402

HERE = Path(__file__).resolve().parent
FIRMWARE = Path(os.environ.get(
    'SROT_FIRMWARE',
    Path(__file__).resolve().parents[4] / 'Mongla_others' / 'srot-control-board'))


def _capture() -> dict:
    return json.loads(BOARD_PARAMS.read_text())


# --------------------------------------------------------------------------- #
#  The capture is real, and it is the vehicle's
# --------------------------------------------------------------------------- #

def test_the_capture_exists_and_names_its_firmware():
    data = _capture()
    assert data['params'], 'an empty capture is not evidence'
    assert data['_firmware_rev'], 'a capture that cannot say which firmware it came from'
    assert '2026' in data['_source']


def test_from_board_applies_every_captured_value():
    b = Board().from_board()
    for name, want in _capture()['params'].items():
        assert b.get_param(name) == pytest.approx(want, abs=1e-6), name


def test_defaults_does_not_claim_to_be_the_board():
    assert Board().defaults().is_board_parameterised is False
    assert Board().from_board().is_board_parameterised is True


# --------------------------------------------------------------------------- #
#  ⭐ The divergence itself, pinned
# --------------------------------------------------------------------------- #

def test_the_defaults_and_the_board_disagree_on_pilot_yaw_rate():
    """The specific lie that cost a session. If this ever stops failing to
    match, the firmware default was changed or the board was retuned -- either
    way somebody must look, because every yaw number in the repo was computed
    under one of these two values."""
    default = Board().defaults().get_param('PILOT_YAW_RATE')
    board = Board().from_board().get_param('PILOT_YAW_RATE')
    assert default == pytest.approx(45.0)
    assert board == pytest.approx(160.0)
    assert board / default == pytest.approx(3.5556, rel=1e-3)


def test_the_divergence_is_invisible_to_the_mixer_and_large_in_the_cascade():
    """Why the existing falsifier could not catch it: the mixer reads no
    retuned parameter, so it agrees under either set. The cascade does not."""
    dflt = Board().defaults()
    mix_d, dsh_d = dflt.mix(yaw=0.30), dflt.to_dshot(dflt.mix(yaw=0.30))
    board = Board().from_board()
    assert board.mix(yaw=0.30) == mix_d
    assert board.to_dshot(board.mix(yaw=0.30)) == dsh_d

    # ⚠ `g_params` is ONE global inside the shared library, so every Board is a
    # handle on the same parameter set -- `defaults()` and `from_board()` are
    # switches, not independent instances. Re-select before each measurement.
    def tau(select, pct):
        b = select()
        b.reset()
        for _ in range(50):
            _, _, y = b.stabilize(stick_yaw=pct / 100.0)
        return y

    hi = tau(lambda: Board().from_board(), 5.0)
    lo = tau(lambda: Board().defaults(), 5.0)
    assert hi / lo == pytest.approx(3.5556, rel=0.02)


def test_the_integrator_limits_are_written(  ):
    """`attitude::loadGains` falls back to 0.5 when `rat_*_imax` is 0, so a
    memset left the bench integrating to 0.5 while the board integrates to
    0.222 on yaw. A zeroed imax is not the firmware's behaviour."""
    assert Board().defaults().get_param('ATC_RAT_YAW_IMAX') == pytest.approx(0.222)


# --------------------------------------------------------------------------- #
#  The table is a second copy of theirs -- this is what makes it honest
# --------------------------------------------------------------------------- #

ROW = re.compile(r'\{\s*"([A-Z0-9_]+)"\s*,\s*"[^"]*"\s*,\s*&g_params\.([a-z0-9_]+)')


@pytest.mark.skipif(not FIRMWARE.exists(), reason='no firmware checkout')
def test_every_bench_param_name_maps_to_the_same_member_as_the_firmware():
    """`s_bench_params` in bench_api.cpp duplicates `comms/params.cpp`. The
    duplication is unavoidable (their table drags in MAVLink and NVS); a test
    that reads their file is what stops it drifting."""
    theirs = dict(ROW.findall((FIRMWARE / 'src/comms/params.cpp').read_text()))
    assert theirs, 'could not parse the firmware parameter table'

    ours = dict(re.findall(r'\{\s*"([A-Z0-9_]+)"\s*,\s*&g_params\.([a-z0-9_]+)',
                           (HERE / 'bench_api.cpp').read_text()))
    assert ours, 'could not parse the bench parameter table'

    for name, member in ours.items():
        assert name in theirs, f'{name} is not a parameter the firmware has'
        assert theirs[name] == member, (
            f'{name}: firmware writes g_params.{theirs[name]}, '
            f'the bench writes g_params.{member}')


def test_an_unknown_parameter_is_refused_loudly():
    with pytest.raises(KeyError):
        Board().set_param('NOT_A_PARAM', 1.0)


# --------------------------------------------------------------------------- #
#  The yaw thresholds, under the BOARD's parameters
# --------------------------------------------------------------------------- #

def test_the_only_yaw_gate_is_the_stabilize_stick_literal():
    """On the board's own gains there is no band between the `0.02f` stick gate
    in `attitude_control.cpp` and the `0.005f` centre gap in `mixer.cpp`: the
    first stick that is a rate command at all is already well past the mixer.
    Under the config.h defaults that is NOT true, which is exactly how the
    wrong-parameter run produced a dead band that does not exist."""
    b = Board().from_board()

    def out(pct):
        b.reset()
        for _ in range(50):
            _, _, y = b.stabilize(stick_yaw=pct / 100.0)
        d = b.drive(yaw=y)
        return max((v - 1048 if v >= 1049 else v - 47) for v in d if v != 1048) if any(
            v != 1048 for v in d) else 0

    assert out(2.80) == 0
    assert out(2.90) > 0
    # and the step is a cliff, not a ramp: nothing small is expressible
    assert out(2.90) / 999 > 0.15


def test_a_board_is_a_handle_on_one_global_parameter_set():
    """⚠ Stated as a test because it is a trap. `g_params` is a single global in
    the shared library; constructing a second `Board` does not give a second
    parameter set, it re-selects the same one. A test that holds two Boards and
    compares them measures nothing."""
    a = Board().from_board()
    b = Board().defaults()
    assert a.get_param('PILOT_YAW_RATE') == b.get_param('PILOT_YAW_RATE') == 45.0


# --------------------------------------------------------------------------- #
#  ACRO vs STABILIZE -- what the mode choice actually buys
# --------------------------------------------------------------------------- #

def _yaw_out(fn, pct, ticks=50):
    b = Board().from_board()
    b.reset()
    for _ in range(ticks):
        _, _, y = fn(b, pct / 100.0)
    d = b.drive(yaw=y)
    live = [v for v in d if v != 1048]
    return y, (max((v - 1048 if v >= 1049 else v - 47) for v in live) if live else 0)


_STAB = lambda b, s: b.stabilize(stick_yaw=s)      # noqa: E731
_ACRO = lambda b, s: b.acro(stick_yaw=s)           # noqa: E731


def test_acro_has_no_yaw_stick_gate():
    """`stabilize()` gates the yaw stick at 0.02 and hands anything smaller to
    heading hold; `acro()` does not gate at all. So ACRO commands yaw where
    STABILIZE commands nothing -- 1.00 % of stick against 2.86 %."""
    assert _yaw_out(_STAB, 1.00)[1] == 0
    assert _yaw_out(_ACRO, 1.00)[1] > 0
    assert _yaw_out(_ACRO, 0.50)[1] == 0     # the mixer's centre gap still binds


def test_acro_does_not_shrink_the_output_quantum():
    """⛔ THE THING ACRO DOES NOT FIX, and the reason it is not the answer to
    precision alignment. `MOT_SPIN_MIN` is in the mixer, downstream of both
    modes: the smallest non-zero output is ~16 % of full scale either way. ACRO
    buys finer COMMAND resolution, not finer THRUST."""
    smallest_acro = _yaw_out(_ACRO, 1.00)[1] / 999
    smallest_stab = _yaw_out(_STAB, 2.86)[1] / 999
    assert 0.15 < smallest_acro < 0.18
    assert 0.15 < smallest_stab < 0.18


def test_acro_has_more_yaw_authority_at_full_stick():
    """Full stick: ACRO 84.8 % of full scale against STABILIZE's 70.7 %,
    because MAX_ACRO_RATE (4.0 rad/s) exceeds PILOT_YAW_RATE (160 deg/s =
    2.793 rad/s). Worth knowing before a hard turn is tuned in the wrong mode."""
    assert _yaw_out(_ACRO, 100.0)[1] > _yaw_out(_STAB, 100.0)[1]
    assert _yaw_out(_ACRO, 100.0)[1] / 999 == pytest.approx(0.848, abs=0.01)
    assert _yaw_out(_STAB, 100.0)[1] / 999 == pytest.approx(0.707, abs=0.01)


def test_the_capture_covers_every_bench_parameter():
    """⛔ PARTIAL COVERAGE IS THE SAME BUG IN SMALLER LETTERS. The first version
    of the capture carried 20 of these 37 names, so roll/pitch I, D, FF and
    IMAX, both ANG_*_P, the DEPTH_* gains and the TRIM group all stayed at
    config.h defaults while `is_board_parameterised` reported True. That is
    exactly the PILOT_YAW_RATE failure, just quieter."""
    b = Board()
    names = {b._lib.bench_param_name(i).decode()
             for i in range(b._lib.bench_param_count())}
    captured = set(_capture()['params'])
    assert names - captured == set(), (
        f'not read off the board: {sorted(names - captured)}')
    assert captured - names == set(), (
        f'captured but not a bench parameter: {sorted(captured - names)}')


def test_battery_compensation_does_not_move_a_threshold():
    """⚠ The board has MOT_BAT_V_MAX = 16.8, so compensation is configured on
    the vehicle, while `batteryScale()` returns 0 -- uncompensated -- until the
    board has seen a pack voltage, which is the state the mixer_ladder capture
    was taken in. Measured: across the whole 13.2-16.8 V range the output moves
    by at most ONE percentage point, so every threshold quoted upstream stands.
    This test is what lets those numbers be quoted without a pack voltage."""
    def out(volts, pct):
        b = Board().from_board()
        b.reset()
        if volts is not None:
            for _ in range(40):
                b.set_battery(volts, 0.1)
        for _ in range(50):
            _, _, y = b.stabilize(stick_yaw=pct / 100.0)
        d = b.drive(yaw=y)
        live = [v for v in d if v != 1048]
        return (max((v - 1048 if v >= 1049 else v - 47) for v in live) if live else 0) / 999

    for pct in (2.86, 5.0, 100.0):
        base = out(None, pct)
        for volts in (16.8, 14.7, 13.2):
            assert abs(out(volts, pct) - base) <= 0.011, (
                f'{pct} % at {volts} V moved more than one point from the '
                f'uncompensated figure -- the upstream numbers need a pack voltage')
