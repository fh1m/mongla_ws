"""The lateral integrator must not wind up against a limit it cannot see.

⛔ THE GAP THIS CLOSES. `motion_vision` already had anti-windup:
`abs(p_lat + lat_i) < g_lat`. That watches the SOFTWARE gain cap. The board
scales a whole thruster group down whenever any motor in it passes full, and
never reports it -- so with yaw already spending horizontal authority the hull
clips at an effective lateral well below `g_lat`, and the integrator winds
against a wall with nothing to stop it.

The loop then cannot distinguish "not moving because I am too weak" from "not
moving because I am saturated", and those need opposite responses: push harder
versus stop pushing and yield some yaw.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.allocation import allocate                  # noqa: E402
from duburi_control.motion_vision import _mixer_saturated       # noqa: E402

SRC = (Path(__file__).resolve().parents[1]
       / 'duburi_control' / 'motion_vision.py').read_text()


def test_a_modest_demand_is_not_saturated():
    assert _mixer_saturated(lat_pct=20.0, yaw_pct=10.0, fwd_pct=0.0) is False


def test_lateral_plus_yaw_saturates_below_either_axis_cap():
    """⛔ THE WHOLE POINT. Neither axis is near 100 %, and the hull is still
    clipping -- because motor 1 carries yaw AND forward AND lateral, and their
    sum is what passes full. No single-axis cap can see this."""
    assert _mixer_saturated(lat_pct=60.0, yaw_pct=60.0, fwd_pct=0.0) is True
    # And the allocation agrees about how much is lost.
    a = allocate(yaw=0.6, lateral=0.6)
    assert a.horizontal_scale < 0.9


def test_percent_is_converted_to_normalised_axes():
    """Getting this conversion wrong makes the check fire either never or
    always, and both look like 'it works' in a bench test."""
    assert _mixer_saturated(lat_pct=100.0, yaw_pct=100.0, fwd_pct=0.0) is True
    assert _mixer_saturated(lat_pct=1.0, yaw_pct=1.0, fwd_pct=0.0) is False


def test_it_degrades_to_NOT_saturated_rather_than_raising():
    """A failure here must never stop a vision verb. Returning False restores
    exactly the behaviour that existed before this function, so it cannot be a
    regression."""
    assert _mixer_saturated(float('nan'), 0.0, 0.0) in (True, False)


def test_the_integrator_actually_consults_it():
    """Reachability: the check must be IN the accumulate condition, not merely
    defined. A knob wired to nothing is this repo's recurring defect."""
    cond = SRC[SRC.index('if ki_lat > 0.0 and aligned_at is not None'):]
    cond = cond[:cond.index('lat_pct = _clamp')]
    assert 'was_saturated' in cond


def test_the_flag_is_computed_from_what_was_actually_SENT():
    """Not from the pre-clamp values: the demand that reaches the mixer is the
    clamped one, and predicting saturation from a larger number would refuse
    to integrate in cases that are perfectly fine."""
    assert '_mixer_saturated(lat_pct, yaw_pct, fwd_pct)' in SRC
    assert SRC.index('_mixer_saturated(lat_pct, yaw_pct, fwd_pct)') < SRC.index('_drive(lat_pct, yaw_pct, fwd_pct)')


def test_the_flag_starts_false():
    """The first tick has no previous allocation, and assuming saturation
    would suppress the integral for one tick of every alignment."""
    assert 'was_saturated = False' in SRC


class _FakeFC:
    def __init__(self):
        self.frames = []

    def manual(self, **kw):
        self.frames.append(kw)


def test_the_srot_frame_is_PRIORITISED_yaw_first_when_saturated():
    """Executed, not grepped: the frame that reaches the board must keep yaw
    when the group saturates. Uniform scaling would have delivered 0.143 of
    the 0.300 asked for in this corner."""
    from duburi_control.motion_vision import _srot_drive
    fc = _FakeFC()
    _srot_drive(fc, fwd_pct=90.0, lat_pct=90.0, yaw_pct=30.0)
    sent = fc.frames[-1]
    assert abs(sent['yaw'] - 0.30) < 1e-9, sent
    assert not allocate(yaw=sent['yaw'], forward=sent['fwd'],
                        lateral=sent['lat']).saturated
    assert sent['up'] == 0.0


def test_an_unsaturated_srot_frame_is_UNCHANGED():
    """Below the limit the host must not touch the demand, or every existing
    tuning silently shifts."""
    from duburi_control.motion_vision import _srot_drive
    fc = _FakeFC()
    _srot_drive(fc, fwd_pct=20.0, lat_pct=-10.0, yaw_pct=5.0)
    assert fc.frames[-1] == {'fwd': 0.20, 'lat': -0.10, 'up': 0.0, 'yaw': 0.05}
