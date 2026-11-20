"""localization_check: each stage a verdict, the truth stage independent of
the filter, and the hull surfaced whatever fails."""
from __future__ import annotations

from types import SimpleNamespace as NS
from unittest.mock import MagicMock

from mongla_localization.course_map import Course, Prop
from mongla_planner.missions import localization_check as lc


def _state(x=0.0, y=0.0, frame='pool', aiding='aided'):
    return NS(x_m=x, y_m=y, yaw_deg=0.0, frame=frame, aiding=aiding,
              sigma_xy_m=0.1, flow_fused=5, flow_refused=0, usable=True, reason='')


def _mission(monkeypatch, *, states, fix=(0.1, 0.0), anchor_ok=True):
    monkeypatch.setattr(lc, 'COURSE', 't')
    monkeypatch.setattr(lc, 'ANCHOR_PROP', 'gate')
    monkeypatch.setattr(lc, 'MODEL', 'm')
    monkeypatch.setattr(lc, 'AIDED_WAIT_S', 0.0)
    m = MagicMock()
    m.use_course.return_value = Course('t', {'gate': Prop(
        'gate', x_m=5.0, y_m=0.0, bearing_deg=0.0, detect_class='gate', measured=True)})
    seq = list(states)
    m.localization.side_effect = lambda **_k: seq.pop(0) if len(seq) > 1 else seq[0]
    m.anchor_on.return_value = NS(ok=anchor_ok, reason='' if anchor_ok else 'no pose')
    m.go_to.return_value = NS(success=True, message='go_to ARRIVED')
    m.fix_from_prop.return_value = NS(ok=fix is not None, x_m=(fix or (0, 0))[0],
                                      y_m=(fix or (0, 0))[1], reason='no sighting')
    lines = []
    lc.run(m, lines.append)
    return m, lines


def _verdicts(lines):
    out = {}
    for ln in lines:
        parts = ln.split()
        if len(parts) >= 3 and parts[2] in ('PASS', 'FAIL'):
            out[parts[1]] = parts[2]
    return out


def test_every_stage_passes_when_the_fix_agrees(monkeypatch):
    m, lines = _mission(monkeypatch, states=[_state()])
    assert _verdicts(lines) == {k: 'PASS' for k in
                                ('live', 'aided', 'anchored', 'out', 'back', 'truth')}
    gx, gy = m.go_to.call_args_list[0].args
    assert (gx, gy) == (lc.LEG_M, 0.0)
    names = [c[0] for c in m.method_calls]
    assert names[-2:] == ['surface', 'disarm']


def test_a_fix_far_from_the_filter_fails_the_truth_stage(monkeypatch):
    _, lines = _mission(monkeypatch, states=[_state()], fix=(1.2, 0.4))
    assert _verdicts(lines)['truth'] == 'FAIL'


def test_the_estimate_compared_is_read_before_the_fix_reaches_the_filter(monkeypatch):
    m, _ = _mission(monkeypatch, states=[_state()])
    names = [c[0] for c in m.method_calls]
    last_read = max(i for i, n in enumerate(names) if n == 'localization')
    assert last_read < names.index('fix_from_prop')


def test_no_localiser_stops_before_arming(monkeypatch):
    m, lines = _mission(monkeypatch, states=[None])
    m.arm.assert_not_called()
    assert _verdicts(lines) == {'live': 'FAIL'}


def test_unaided_or_unanchored_stops_and_surfaces(monkeypatch):
    m, lines = _mission(monkeypatch, states=[_state(aiding='unaided')])
    assert _verdicts(lines)['aided'] == 'FAIL'
    m.go_to.assert_not_called()
    assert [c[0] for c in m.method_calls][-2:] == ['surface', 'disarm']
    m, lines = _mission(monkeypatch, states=[_state()], anchor_ok=False)
    assert _verdicts(lines)['anchored'] == 'FAIL'
    m.go_to.assert_not_called()
