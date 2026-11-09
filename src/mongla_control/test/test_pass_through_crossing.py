"""Losing a gate is not seeing the hull cross it (#79).

A pass-through `move` used to report ALIGNED ("passed through") as soon as a
gate it had seen left the frame, whatever made it leave: a far gate blinking
out in turbidity, one sliding out sideways, a camera outage. Each then drove
an open-loop commit and scored the mission as passed. The verdict now comes
from the last live box: a gate the hull is entering surrounds the camera axis
and runs off the frame (ESTIMATED_CROSSING); anything else is a commit the
loop cannot stand behind (COMMIT_UNVERIFIED). Neither is ALIGNED.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from mongla_control.motion_vision import (
    ALIGNED, COMMIT_UNVERIFIED, ESTIMATED_CROSSING, LOST)
from mongla_control.motion_vision.control import _crossing_evidence

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_motion_vision import _FakeVision, _move  # noqa: E402


def _box(ex, w_frac, age_s=0.0, coasted=False):
    return SimpleNamespace(ex=ex, ey=0.0, w_frac=w_frac, h_frac=w_frac,
                           age_s=age_s, track_id=-1, coasted=coasted,
                           source='detector' if not coasted else 'ladder',
                           score=1.0)


def _approach(*last):
    """A gate seen far and centred, growing, then whatever `last` is, then gone."""
    return [_box(0.0, 0.2), _box(0.0, 0.4), *last, None]


SEQUENCES = {
    # The hull goes in: the gate grows around the axis and runs off both sides.
    'genuine crossing': (_approach(_box(0.02, 0.98), _box(0.0, 1.0)),
                         ESTIMATED_CROSSING),
    # Lost in front of the gate: still small, still inside the frame.
    'pre-gate loss': (_approach(_box(0.05, 0.35)), COMMIT_UNVERIFIED),
    # Slid out of the side: runs off one edge, but no longer around the axis.
    'sideways exit': (_approach(_box(0.7, 0.3), _box(0.85, 0.2)), COMMIT_UNVERIFIED),
    # The camera dies mid-approach: the last box is a far one.
    'camera outage': (_approach(), COMMIT_UNVERIFIED),
    # One spurious far box, then nothing.
    'false initial gate': ([_box(-0.4, 0.08), None], COMMIT_UNVERIFIED),
}


@pytest.mark.parametrize('name', list(SEQUENCES))
def test_each_way_of_losing_the_gate_gets_its_own_verdict(name):
    frames, want = SEQUENCES[name]
    out, _, writers = _move(_FakeVision(frames), passthrough=True, hold_s=0.05,
                            lost_grace_s=5.0, duration=2.0)
    assert out.code == want, (name, out.reason)
    assert out.code != ALIGNED
    assert writers.neutralised >= 1, 'the commit did not end in neutral'


def test_a_gate_never_seen_is_still_a_loss_and_never_commits():
    out, board, _ = _move(_FakeVision(None), passthrough=True,
                          lost_grace_s=0.1, duration=1.0)
    assert out.code == LOST
    assert all(c.get('forward', 0) == 0 for c in board.rc), 'drove blind'


def test_the_verdict_explains_itself():
    out, _, _ = _move(_FakeVision(SEQUENCES['pre-gate loss'][0]), passthrough=True,
                      hold_s=0.05, lost_grace_s=5.0, duration=2.0)
    assert 'not run off the frame' in out.reason and 'open-loop' in out.reason
    out, _, _ = _move(_FakeVision(SEQUENCES['genuine crossing'][0]),
                      passthrough=True, hold_s=0.05, lost_grace_s=5.0, duration=2.0)
    assert 'estimate' in out.reason


def test_duration_bounds_the_commit_and_still_gives_a_verdict():
    """The commit is capped by the verb's budget; a cut-short commit is still a
    crossing verdict, not a TIMEOUT that hides which way the gate went."""
    frames, _ = SEQUENCES['genuine crossing']
    out, _, _ = _move(_FakeVision(frames), passthrough=True, hold_s=30.0,
                      lost_grace_s=5.0, duration=0.4)
    assert out.code == ESTIMATED_CROSSING and 'cut short' in out.reason
    assert out.elapsed_s < 1.0


def test_a_ladder_box_does_not_hold_the_gate_present():
    """With lock_s on, the follower would keep the gate "seen" for up to 2.5 s
    after the detector lost it, and the commit would start inside the posts."""
    frames = _approach(_box(0.0, 1.0)) + [_box(0.0, 1.0, coasted=True)] * 50
    seen = []

    class _Recording(_FakeVision):
        def bbox_error(self, *a, **kw):
            seen.append(kw.get('lock_s'))
            return super().bbox_error(*a, **kw)
    _move(_Recording(frames), passthrough=True, hold_s=0.05, lost_grace_s=5.0,
          duration=0.5, lock_s=1.0)
    assert seen and all(v == 0.0 for v in seen), seen


@pytest.mark.parametrize('ex,w,crossing', [
    (0.0, 1.0, True),       # fills the frame
    (0.3, 0.75, True),      # off-centre, but around the axis and off the right edge
    (0.0, 0.5, False),      # big, but inside the frame
    (0.6, 0.45, False),     # off the edge, axis outside the box
    (-0.6, 0.45, False),
])
def test_the_geometry(ex, w, crossing):
    assert _crossing_evidence(ex, w) is crossing


# ---------------------------------------------------------------- the DSL

def _dsl_result(code):
    return SimpleNamespace(final_value=float(code), error_value=0.0, fill_frac=0.0,
                           end_x_px=0.0, end_y_px=0.0, elapsed_s=1.0,
                           success=True, message='why')


@pytest.mark.parametrize('code,ok', [(ESTIMATED_CROSSING, True),
                                     (COMMIT_UNVERIFIED, False)])
def test_the_dsl_names_the_verdict_and_never_re_enters(code, ok):
    pytest.importorskip('mongla_planner')
    from unittest.mock import MagicMock
    from mongla_planner.vision_dsl import _VisionDSL
    m = MagicMock()
    m.camera, m.target, m.detected.return_value = 'forward', 'gate', False
    m._send = send = MagicMock(return_value=_dsl_result(code))
    res = _VisionDSL(m).move('gate', duration=5.0, fallback=lambda _m: None)
    assert res.ok is ok and res.committed is True
    assert res.status in ('ESTIMATED_CROSSING', 'COMMIT_UNVERIFIED')
    assert send.call_count == 1, 'a commit leg was driven twice'


# ---------------------------------------------------------------- the mission

@pytest.mark.parametrize('mission,fn', [('sauvc_navigation', 'navigate'),
                                        ('sauvc_qualification', '_pass')])
@pytest.mark.parametrize('code,blind', [(COMMIT_UNVERIFIED, False),
                                        (ESTIMATED_CROSSING, False),
                                        (LOST, True)])
def test_no_blind_leg_is_stacked_on_a_commit(mission, fn, code, blind):
    """A blind transit after a commit drives the hull twice past the gate."""
    pytest.importorskip('mongla_planner')
    import importlib
    from unittest.mock import MagicMock
    from mongla_planner.vision_dsl import VisionResult
    mod = importlib.import_module(f'mongla_planner.missions.{mission}')
    mongla = MagicMock()
    mongla.vision.align.return_value = VisionResult(True, 'ALIGNED', ALIGNED)
    mongla.vision.move.return_value = VisionResult(
        code == ESTIMATED_CROSSING, 'x', code, saw_target=code != LOST)
    args = (mongla,) if fn == 'navigate' else (mongla, 5.0, 'out', None)
    getattr(mod, fn)(*args)
    assert mongla.move_forward.called is blind
