"""The bench's fake board rises in SURFACE, so `surface()` can be seen to work.

Before, its depth was fixed at -0.5 m: every bench mission ended with
`surface` failing after 60 s. Only the bench (`bench_motion`) moves; the
manager's own tests keep a board that holds still. Its mode numbers are the
wire's, read from `srot_protocol.py`, not a second copy that can drift.
"""
import pytest

import fake_srot_board as fake
from mongla_control.fc import srot_protocol as wire


def test_the_fake_speaks_the_wires_mode_numbers():
    assert fake.MODE_SURFACE == wire.MODE_SURFACE
    assert fake.MODE_AUTO == wire.MODE_AUTO
    assert fake.MODE_STABILIZE == wire.MODE_STABILIZE


def _after(board, mode, secs):
    board.mode = mode
    board._ascend()                          # starts the clock
    board._t_depth -= secs                   # as if `secs` passed
    board._ascend()
    return board.depth_m


def test_a_bench_board_in_surface_rises_and_stops_at_the_surface():
    b = fake.FakeSrotBoard.__new__(fake.FakeSrotBoard)
    b.bench_motion, b.depth_m = True, -0.5
    assert _after(b, fake.MODE_SURFACE, 1.0) == pytest.approx(-0.5 + fake.BENCH_ASCENT_MPS, abs=1e-3)
    assert _after(b, fake.MODE_SURFACE, 60.0) == 0.0


def test_other_modes_and_a_still_board_hold_their_depth():
    b = fake.FakeSrotBoard.__new__(fake.FakeSrotBoard)
    b.bench_motion, b.depth_m = True, -0.5
    assert _after(b, fake.MODE_AUTO, 10.0) == -0.5
    b.bench_motion = False
    assert _after(b, fake.MODE_SURFACE, 10.0) == -0.5
