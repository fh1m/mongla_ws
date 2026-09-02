"""The vision loops must actuate the SROT board, and must not touch ArduSub.

`vision_align` / `vision_move` came out of `UNSUPPORTED_VERBS` on 2026-09-03.
That set was the ONLY thing keeping the host motion stack unreachable on this
backend, so the port needs a check that runs the real loop rather than one that
greps for a function name.

A grep was written first and is recorded here as insufficient: renaming the
DEFINITION of `_srot_drive` left every call site containing the string, so the
assertion stayed green through exactly the change it existed to catch. Same
shape as the drift suite's own caveat -- grep for structure, exercise for
behaviour.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Reuse the existing align/move harness rather than re-building fakes that
# would drift from it. The test DIRECTORY has to be importable for that.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from duburi_control.motion_vision import (       # noqa: E402
    _is_srot, _srot_drive, align_loop, move_loop,
)

from test_motion_vision import (                 # noqa: E402
    _FakeVision, _FakeWriters, _Log, _sample,
)


class _FakeSrot:
    """A SrotFC-shaped double: it has `manual()` and NONE of the RC primitives.

    Deliberately missing `send_rc_override` / `send_rc_translation` /
    `set_target_depth`, exactly as the real class is. A loop that reaches for
    one raises AttributeError, which is the honest outcome -- a stub that
    silently accepted them would let the test pass on a broken port.
    """
    name = 'srot'

    def __init__(self):
        self.manual_calls = []

    def manual(self, fwd, lat, up, yaw):
        self.manual_calls.append((fwd, lat, up, yaw))

    # Read surface the loops touch.
    def get_attitude(self):
        return {'yaw': 0.0, 'depth': -0.5}

    def get_mode(self):
        return 'STABILIZE'


def test_the_backend_predicate_is_not_true_for_pixhawk():
    from test_motion_vision import _FakePixhawk
    assert _is_srot(_FakeSrot()) is True
    assert _is_srot(_FakePixhawk()) is False


def test_srot_drive_scales_percent_to_unit():
    """The loops speak percent (-100..100); MANUAL_CONTROL speaks -1..1. A
    missing /100 is a 100x command, which on a real vehicle is full deflection
    for every non-zero error."""
    fc = _FakeSrot()
    _srot_drive(fc, fwd_pct=50.0, lat_pct=-25.0, yaw_pct=10.0)
    fwd, lat, up, yaw = fc.manual_calls[-1]
    assert fwd == pytest.approx(0.5)
    assert lat == pytest.approx(-0.25)
    assert yaw == pytest.approx(0.1)
    assert up == 0.0, 'the depth axis is not ported; up must stay neutral'


# --------------------------------------------------------------------------- #
#  The real loops, driven end to end
# --------------------------------------------------------------------------- #
def _run_align(**kw):
    fc = _FakeSrot()
    writers = _FakeWriters()
    defaults = dict(
        pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.6, ey=0.0)),
        target_class='gate', axes={'yaw', 'lat'}, offsets={}, err_px=40.0,
        duration=0.3, gain=30.0, align_stable_frames=3, lost_grace_s=0.1,
        writers=writers, log=_Log(), abort_fn=None)
    defaults.update(kw)
    align_loop(**defaults)
    return fc


def test_align_actuates_through_manual_control_on_srot():
    """The headline: a real align_loop against a srot backend must produce
    MANUAL_CONTROL frames. If the port regressed, the FakeSrot has no
    send_rc_override and this raises instead of silently doing nothing."""
    fc = _run_align()
    assert fc.manual_calls, 'align_loop drove nothing on the srot backend'
    lat_cmds = [c[1] for c in fc.manual_calls]
    assert any(abs(v) > 0.01 for v in lat_cmds), (
        'every lateral command was ~0 for a target at ex=+0.6 -- the loop ran '
        'but commanded nothing')


def test_a_target_to_the_right_drives_lateral_positive():
    """Sign, end to end through the real loop. Backwards here means the hull
    accelerates away from the target at exactly the rate it should close."""
    right = _run_align(vision_state=_FakeVision(_sample(ex=0.6, ey=0.0)))
    left = _run_align(vision_state=_FakeVision(_sample(ex=-0.6, ey=0.0)))
    r = max((c[1] for c in right.manual_calls), key=abs)
    l = max((c[1] for c in left.manual_calls), key=abs)
    assert r > 0 and l < 0, f'lateral sign wrong: right={r}, left={l}'


def test_every_command_is_inside_the_unit_range():
    """MANUAL_CONTROL axes are -1..1. The board clamps, but a host that relies
    on the clamp has lost its own gain limit: `gain` is a SPEED CAP, and a
    value that only survives because the far end truncates it is not capped."""
    fc = _run_align(gain=100.0)
    for fwd, lat, up, yaw in fc.manual_calls:
        for v in (fwd, lat, up, yaw):
            assert -1.0 <= v <= 1.0, f'axis out of range: {(fwd, lat, up, yaw)}'


def test_gain_actually_caps_the_command():
    fc = _run_align(gain=20.0)
    assert max(abs(c[1]) for c in fc.manual_calls) <= 0.20 + 1e-6


def test_move_actuates_through_manual_control_on_srot():
    fc = _FakeSrot()
    move_loop(pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.0, h_frac=0.2)),
              target_class='gate', fwd_fill=80.0, mode='area',
              maintain_px=None, duration=0.3, gain=30.0, lost_grace_s=0.1,
              writers=_FakeWriters(), log=_Log(), abort_fn=None)
    assert fc.manual_calls, 'move_loop drove nothing on the srot backend'
    assert all(c[3] == 0.0 for c in fc.manual_calls), (
        'move must never command yaw -- it is true on the ArduSub path and must '
        'stay true here')


def test_no_frame_ever_commands_the_depth_axis():
    """`up` is not ported: SrotFC has no set_target_depth, and vision_verbs
    refuses a depth-axis align. A non-zero `up` slipping through would be
    uncommanded vertical thrust."""
    fc = _run_align()
    assert all(c[2] == 0.0 for c in fc.manual_calls)
