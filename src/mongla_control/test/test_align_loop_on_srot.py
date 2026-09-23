"""The align loop's yaw axis, driven on the SROT backend.

⛔ WHY THIS FILE EXISTS. Every loop test in `test_motion_vision.py` runs on
`_FakePixhawk`, whose `name` is not `'srot'`, so the whole `_is_srot(pixhawk)`
branch of `align_loop` had NO loop-level coverage at all. The srot yaw law was
shipped for one commit rounding corrections DOWN to zero, which stalls the
hole-lock just outside `err_px` -- the exact failure the yaw floor was written
for -- and no test here would have caught it.
"""
import sys
from pathlib import Path

import pytest

from mongla_control.motion_vision import SROT_YAW_MIN_PCT, align_loop

# The test doubles live next door; the test directory is not a package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_motion_vision import (  # noqa: E402
    _FakeVision, _FakeWriters, _Log, _sample)


class _FakeSrot:
    """A srot backend that records what `_srot_drive` sent."""
    name = 'srot'

    def __init__(self):
        self.frames = []          # (fwd, lat, up, yaw) percentages

    def get_attitude(self):
        return {'depth': -0.5}

    def manual(self, fwd, lat, up, yaw):
        # ⚠ MANUAL_CONTROL carries -1..1 UNITS, not percent. `_srot_drive`
        # divides by 100 on the way out, so everything recorded here is a unit
        # and every comparison against a *_PCT constant must scale by 100.
        self.frames.append((fwd, lat, up, yaw))

    def is_armed(self):
        return True

    def mode(self):
        return 'STABILIZE'


def _align_srot(vision, fc=None, **kw):
    fc = fc or _FakeSrot()
    writers = _FakeWriters()
    defaults = dict(
        pixhawk=fc, vision_state=vision, target_class='gate',
        axes={'yaw'}, offsets={}, err_px=40.0, duration=0.4, gain=30.0,
        align_stable_frames=3, lost_grace_s=0.1,
        writers=writers, log=_Log(), abort_fn=None)
    defaults.update(kw)
    return align_loop(**defaults), fc


def _yaws_pct(fc):
    """Non-zero yaw magnitudes, converted back to percent."""
    return [abs(f[3]) * 100.0 for f in fc.frames if f[3] != 0.0]


# --------------------------------------------------------------------------- #
#  The regression that shipped
# --------------------------------------------------------------------------- #

def test_a_close_small_residual_still_actuates():
    """⭐ THE FALSIFIER FOR THE ROUND-DOWN BUG. Big bbox (close, so the floor is
    armed), a residual just outside the deadband, and a kp low enough that pure
    proportional lands under the gate -- which is what `range_gain_floor` does
    close in. The old taper actuated here; rounding down does not."""
    # ex 0.15 -> 48 px, 8 px past a 40 px deadband. kp 10 -> P = 1.5 %.
    _align_srot(_FakeVision(_sample(ex=0.15, w_frac=0.6, h_frac=0.6)),
                kp_yaw=10.0, err_px=40.0, duration=0.3)
    # run again capturing the backend
    _, fc = _align_srot(_FakeVision(_sample(ex=0.15, w_frac=0.6, h_frac=0.6)),
                        kp_yaw=10.0, err_px=40.0, duration=0.3)
    sent = _yaws_pct(fc)
    assert sent, 'a close residual outside the deadband commanded NO yaw at all'
    assert min(sent) >= SROT_YAW_MIN_PCT - 1e-6, (
        'yaw was commanded below the board gate -- it produces no thrust')


def test_nothing_is_commanded_inside_the_deadband():
    """The deadband must stay a deadband: the snap lifts corrections, never
    zeros. A target already centred gets an exact zero."""
    _, fc = _align_srot(_FakeVision(_sample(ex=0.0, w_frac=0.6, h_frac=0.6)),
                        kp_yaw=10.0, err_px=40.0, duration=0.3)
    assert _yaws_pct(fc) == []


def test_the_far_field_is_left_pure_proportional():
    """⛔ THE OPPOSITE RULE, and it must survive the snap. A small bbox means the
    target is distant; a minimum on a rate channel out there is a relay that
    limit-cycles the hull (the far-field wobble). The fill gate suppresses the
    floor, and the snap lives INSIDE that gate, so a far-field residual is never
    lifted."""
    _, fc = _align_srot(_FakeVision(_sample(ex=0.15, w_frac=0.05, h_frac=0.05)),
                        kp_yaw=10.0, err_px=40.0, duration=0.3)
    sent = _yaws_pct(fc)
    if sent:
        assert min(sent) < SROT_YAW_MIN_PCT, (
            'the far field was lifted to the floor -- that is the relay the '
            'fill gate exists to prevent')


def test_the_speed_cap_is_never_exceeded():
    """A mission that asked for a gentle yaw does not get a lurch."""
    _, fc = _align_srot(_FakeVision(_sample(ex=0.3, w_frac=0.6, h_frac=0.6)),
                        kp_yaw=60.0, gain=2.0, err_px=40.0, duration=0.3)
    for fwd, lat, up, yaw in fc.frames:
        assert abs(yaw) * 100.0 <= 2.0 + 1e-6, f'yaw {yaw*100:.3f} % exceeded the 2 % cap'


@pytest.mark.parametrize('kp', [5.0, 10.0, 20.0, 60.0])
def test_every_commanded_yaw_is_either_zero_or_expressible(kp):
    """The invariant the whole change exists for: on srot the loop never emits a
    yaw that the board silently declines."""
    _, fc = _align_srot(_FakeVision(_sample(ex=0.15, w_frac=0.6, h_frac=0.6)),
                        kp_yaw=kp, err_px=40.0, duration=0.3)
    for fwd, lat, up, yaw in fc.frames:
        assert yaw == 0.0 or abs(yaw) * 100.0 >= SROT_YAW_MIN_PCT - 1e-6
