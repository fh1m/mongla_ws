"""Scorecard tests -- dedicated run folder + traceable metadata.

`log_scoreboard(json_path='auto', mission=...)` must land a JSON in the run
folder (MONGLA_RUN_DIR, default ~/mongla_runs), named per mission+timestamp,
carrying mission/timestamp/git_sha/counts/phases. Built without rclpy: the
method only touches self._scoreboard/_mission_start/log, so we construct via
object.__new__ and set those three.
"""

import json
import time as _time
from types import SimpleNamespace

from mongla_planner import mongla_dsl
from mongla_planner.mongla_dsl import MonglaMission, _run_dir


def _board(entries):
    m = object.__new__(MonglaMission)
    m._scoreboard = entries
    m._mission_start = _time.monotonic()
    m.log = SimpleNamespace(info=lambda *a, **k: None,
                            warning=lambda *a, **k: None)
    return m


def _entry(cmd, ok):
    return {'cmd': cmd, 'success': ok, 'elapsed': 1.0, 'msg': 'ok' if ok else 'x'}


def test_auto_writes_into_run_dir_with_mission_name(tmp_path, monkeypatch):
    monkeypatch.setenv('MONGLA_RUN_DIR', str(tmp_path))
    m = _board([_entry('arm', True), _entry('vision_align', False)])

    m.log_scoreboard(json_path='auto', mission='task_gate')

    cards = list(tmp_path.glob('task_gate_*.json'))
    assert len(cards) == 1, cards
    payload = json.loads(cards[0].read_text())
    assert payload['mission'] == 'task_gate'
    assert payload['success_count'] == 1
    assert payload['fail_count'] == 1
    assert len(payload['phases']) == 2
    assert 'timestamp' in payload and 'git_sha' in payload


def test_explicit_path_still_honored(tmp_path):
    m = _board([_entry('arm', True)])
    out = tmp_path / 'custom.json'

    m.log_scoreboard(json_path=str(out), mission='demo')

    assert out.exists()
    assert json.loads(out.read_text())['mission'] == 'demo'


def test_missionless_auto_uses_generic_prefix(tmp_path, monkeypatch):
    monkeypatch.setenv('MONGLA_RUN_DIR', str(tmp_path))
    _board([_entry('arm', True)]).log_scoreboard(json_path='auto')
    assert list(tmp_path.glob('mission_*.json'))


def test_run_dir_env_override_and_creation(tmp_path, monkeypatch):
    target = tmp_path / 'nested' / 'runs'
    monkeypatch.setenv('MONGLA_RUN_DIR', str(target))
    assert _run_dir() == str(target)
    assert target.is_dir()


def test_git_sha_never_raises(monkeypatch):
    # Even if git is missing, the helper returns '' rather than raising.
    def _boom(*a, **k):
        raise FileNotFoundError('git')
    monkeypatch.setattr(mongla_dsl.subprocess, 'run', _boom)
    assert mongla_dsl._git_sha() == ''
