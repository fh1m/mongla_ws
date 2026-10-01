"""Degradation is CONFIGURED, not coded (plan Block 1D).

BumblebeeAS: `claw_required: false` -- "degrades to front-only if absent" -- is a
config value. Ours was a branch in one mission (task_bin); three others steered
on the downward camera with no check at all, so one dead USB camera made
`_ensure_detector` abort the WHOLE run. Now every task asks
`mongla.camera_usable(name, task=...)`, and the skip-or-abort answer lives in
`competition_config.CAMERA_REQUIRED`.
"""
import ast
import pathlib
from unittest.mock import MagicMock

import pytest

from mongla_planner.mongla_dsl import MonglaMission
from mongla_planner.missions import competition_config as cc

MISSIONS = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_planner'
            / 'missions')


def _m(available):
    m = MagicMock()
    m.camera_available.return_value = available
    return m


def test_a_present_camera_is_usable():
    assert MonglaMission.camera_usable(_m(True), 'downward', task='bin') is True


def test_an_absent_OPTIONAL_camera_skips_the_task_loudly(monkeypatch):
    monkeypatch.setitem(cc.CAMERA_REQUIRED, 'downward', False)
    m = _m(False)
    assert MonglaMission.camera_usable(m, 'downward', task='bin drop') is False
    msg = m.log.warning.call_args.args[0]
    assert 'SKIPPING bin drop' in msg and 'still scores' in msg


def test_an_absent_REQUIRED_camera_aborts_before_any_motion(monkeypatch):
    monkeypatch.setitem(cc.CAMERA_REQUIRED, 'downward', True)
    with pytest.raises(RuntimeError, match='ABSENT'):
        MonglaMission.camera_usable(_m(False), 'downward', task='bin drop')


def test_an_unknown_camera_name_is_treated_as_required():
    """A typo must not read as 'optional and absent' and quietly skip a task."""
    with pytest.raises(RuntimeError):
        MonglaMission.camera_usable(_m(False), 'downwrad', task='bin')


def test_the_policy_is_one_table_with_both_cameras():
    assert set(cc.CAMERA_REQUIRED) == {'forward', 'downward'}
    assert cc.CAMERA_REQUIRED['forward'] is True


def test_no_mission_enters_the_downward_camera_without_asking():
    """Falsified by a function that switches to, or steers on, the downward
    camera with no `camera_usable('downward', ...)` in the same function --
    the shape that let one dead camera abort a whole run."""
    offenders = []
    for py in sorted(MISSIONS.glob('*.py')):
        tree = ast.parse(py.read_text(encoding='utf-8'))
        for fn in ast.walk(tree):
            if not isinstance(fn, ast.FunctionDef):
                continue
            uses, asks = False, False
            for c in ast.walk(fn):
                if not isinstance(c, ast.Call):
                    continue
                attr = getattr(c.func, 'attr', '')
                lit = [a.value for a in c.args if isinstance(a, ast.Constant)]
                kw = {k.arg: getattr(k.value, 'value', None) for k in c.keywords}
                if attr == 'use_camera' and 'downward' in lit:
                    uses = True
                if attr == 'camera_usable' and 'downward' in lit:
                    asks = True
            if uses and not asks:
                offenders.append(f'{py.name}:{fn.name}')
    assert not offenders, offenders
