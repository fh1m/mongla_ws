"""A vehicle release is cut only from a gate that passed, on clean code from main (#96).

`kutum/cut_release.sh` runs for real against a scratch repository with its
own origin, and a fake `gh` that records what it was asked to do. Each
refusal is a case where a release would name a configuration nobody could
fly again: a gate that refused, uncommitted code, a commit not on main.
"""
import json
import os
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'kutum/cut_release.sh'


def _git(cwd, *args):
    return subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True,
                          text=True).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    origin = tmp_path / 'origin.git'
    work = tmp_path / 'work'
    _git(tmp_path, 'init', '-q', '--bare', '-b', 'main', str(origin))
    _git(tmp_path, 'init', '-q', '-b', 'main', str(work))
    for k, v in (('user.name', 't'), ('user.email', 't@t'), ('commit.gpgsign', 'false')):
        _git(work, 'config', k, v)
    (work / 'a').write_text('a')
    _git(work, 'add', 'a')
    _git(work, 'commit', '-q', '-m', 'a')
    _git(work, 'remote', 'add', 'origin', str(origin))
    _git(work, 'push', '-q', 'origin', 'main')
    bin_ = tmp_path / 'bin'
    bin_.mkdir()
    log = tmp_path / 'gh.log'
    (bin_ / 'gh').write_text(f'#!/bin/sh\nprintf "%s\\n" "$@" > {log}\n')
    (bin_ / 'gh').chmod(0o755)
    manifest = tmp_path / 'release.json'
    manifest.write_text(json.dumps({
        'commit': 'abc from devbox', 'host': 'mongla', 'vehicle_gate': 'pass',
        'vehicle_gate_utc': '2026-11-10T00:00:00Z', 'board_firmware_rev_required': 14,
        'models': {'gate.hef': 'x'}, 'calibration': {'pi_downward_1280x720.json': 'y'},
        'ros_distro': 'jazzy', 'hailortcli': 'HailoRT-CLI version 4.24.0'}))
    return work, log, manifest, bin_


def _cut(repo, gate='0', dirty=''):
    work, log, manifest, bin_ = repo
    env = dict(os.environ, PATH=f'{bin_}:{os.environ["PATH"]}')
    out = subprocess.run(['bash', str(SCRIPT), gate, dirty, str(manifest)], cwd=work,
                         env=env, capture_output=True, text=True, check=True).stdout
    return out, (log.read_text().splitlines() if log.exists() else None)


def test_a_passed_gate_on_main_is_released_with_its_manifest(repo):
    out, gh = _cut(repo)
    full = _git(repo[0], 'rev-parse', 'HEAD')
    assert gh is not None, out
    assert gh[:3] == ['release', 'create', f'vehicle-{gh[2].split("-")[1]}-{full[:8]}']
    assert gh[gh.index('--target') + 1] == full
    notes = gh[gh.index('--notes') + 1:]
    assert any('firmware behaviour revision required: 14' in l for l in notes)
    assert gh[-1] == f'{repo[2]}#release.json'
    assert 'released vehicle-' in out


@pytest.mark.parametrize('gate, dirty, why', [
    ('1', '', 'gate refused'),
    ('0', '-dirty', 'uncommitted'),
])
def test_a_refused_gate_or_a_dirty_tree_is_not_released(repo, gate, dirty, why):
    out, gh = _cut(repo, gate, dirty)
    assert gh is None and why in out


def test_a_commit_that_is_not_on_main_is_not_released(repo):
    work = repo[0]
    (work / 'b').write_text('b')
    _git(work, 'add', 'b')
    _git(work, 'commit', '-q', '-m', 'local only')
    out, gh = _cut(repo)
    assert gh is None and 'is not on main' in out


def test_a_missing_manifest_is_not_released(repo):
    repo[2].write_text('')
    out, gh = _cut(repo)
    assert gh is None and 'no release.json' in out


def test_deploy_runs_it_after_the_gate_and_never_on_its_exit_code():
    d = (ROOT / 'kutum/deploy.sh').read_text()
    gate = d.index('release_manifest.py gate')
    call = d.index('kutum/cut_release.sh "$GATE" "$dirty"')
    assert gate < call < d.index('exit $GATE')
    assert '|| warn "the release step failed; the deploy stands"' in d
