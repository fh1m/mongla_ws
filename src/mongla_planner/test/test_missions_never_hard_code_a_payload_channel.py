"""One place assigns payload channels, and today it assigns none (B53).

The missions carried an invented map -- 1,2 torpedoes, 3,4 droppers -- and read
live off the board (2026-09-22, re-read 2026-10-01) every one of those is the
on-board arm or unroled: SWITCH 9,10,12-16; SERVO 1,2,3,5-8; unroled 4,11.
`fire()` refused them all, so every torpedo and drop would have scored zero.

The map is now 0 = NOT ASSIGNED in `competition_config.py`, until the payload is
wired to a SWITCH channel and confirmed. These tests keep it one place, keep the
missions reading it, and prove 0 is a loud refusal rather than a silent no-op.
"""
import ast
import pathlib

import pytest

MISSIONS = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_planner'
            / 'missions')


def _calls(tree):
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            yield n


def test_no_mission_passes_a_literal_channel_to_fire():
    """`fire(3)` or `align(..., fire=1)` is a second copy of the map -- the one
    the swap from 1-4 would have to find by hand. Falsified by any literal."""
    offenders = []
    for py in sorted(MISSIONS.glob('*.py')):
        tree = ast.parse(py.read_text(encoding='utf-8'))
        for c in _calls(tree):
            name = getattr(c.func, 'attr', getattr(c.func, 'id', ''))
            args = []
            if name == 'fire':
                args += c.args[:1]
            args += [k.value for k in c.keywords if k.arg in ('fire', 'fire_channel')]
            for a in args:
                consts = [a] if isinstance(a, ast.Constant) else (
                    list(a.elts) if isinstance(a, (ast.List, ast.Tuple)) else [])
                for k in consts:
                    if isinstance(k, ast.Constant) and isinstance(k.value, int):
                        offenders.append(f'{py.name}:{c.lineno} {name}({k.value})')
    assert not offenders, offenders


def test_no_mission_module_assigns_a_channel_number_itself():
    """`FOO_CHANNEL = 3` outside competition_config is the same second copy."""
    offenders = []
    for py in sorted(MISSIONS.glob('*.py')):
        if py.name == 'competition_config.py':
            continue
        for n in ast.walk(ast.parse(py.read_text(encoding='utf-8'))):
            if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant) \
                    and isinstance(n.value.value, int):
                for t in n.targets:
                    if isinstance(t, ast.Name) and t.id.endswith('CHANNEL'):
                        offenders.append(f'{py.name}:{n.lineno} {t.id}={n.value.value}')
    assert not offenders, offenders


def test_the_channels_are_unassigned_until_someone_confirms_them():
    """Not a permanent rule -- the record of today's state. When the payload is
    wired, this test is where the person setting the channels says so."""
    from mongla_planner.missions import competition_config as cc
    for name in ('TORPEDO_1_CHANNEL', 'TORPEDO_2_CHANNEL',
                 'DROPPER_1_CHANNEL', 'DROPPER_2_CHANNEL'):
        assert getattr(cc, name) == 0, f'{name} assigned -- update B53 and this test'
    assert cc.BIN_DROPPER_CHANNEL is cc.DROPPER_1_CHANNEL
    assert cc.SAUVC_DROPPER_CHANNEL is cc.DROPPER_1_CHANNEL


def test_an_unassigned_channel_is_REFUSED_with_the_way_out():
    """Falsified by FIRED, or by a reason that does not say where to set it."""
    from mongla_control.fc.srot_fc import SrotPayload
    from mongla_control.fc.base import FIRE_DENIED
    p = SrotPayload.__new__(SrotPayload)
    p._roles = {9: 2, 10: 2, 1: 1}
    res = p.fire(0)
    assert res.code == FIRE_DENIED
    assert 'NOT ASSIGNED' in res.reason and 'competition_config' in res.reason
    assert 'SWITCH [9, 10]' in res.reason


def test_an_unassigned_channel_survives_parsing_into_the_align_fire():
    """The silent path: `_parse_channels` used to drop 0, so align fired
    nothing and nobody was told."""
    from mongla_control.vision_verbs import _parse_channels
    assert _parse_channels('0') == [0]
