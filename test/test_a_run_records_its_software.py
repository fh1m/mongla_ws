"""A pool session writes down which software produced it (#87).

The RoboSub 2026 runs can be tied to a commit only by date, because nothing
recorded one (docs/evidence/water.json, W-RS26). Sourcing
`scripts/pool_session.sh` now appends the workspace and its commit, or the
Kutum release's own `release.json`, to `$MONGLA_RUN_DIR/software.txt`.
Sourced for real, in each shell the operators use.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/pool_session.sh'


def _head():
    return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True,
                          capture_output=True, text=True).stdout.strip()


@pytest.mark.parametrize('shell', ['bash', 'zsh'])
def test_sourcing_it_records_the_commit(tmp_path, shell):
    if shutil.which(shell) is None:
        pytest.skip(f'{shell} is not installed here')
    env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin',
           'MONGLA_RUNS_ROOT': str(tmp_path / 'runs')}
    subprocess.run([shell, '-c', f'source {SCRIPT} t1 >/dev/null'], cwd='/',
                   env=env, check=True)
    text = (tmp_path / 'runs/t1/software.txt').read_text()
    assert f'workspace: {ROOT.resolve()}' in text
    assert f'commit: {_head()}' in text


def test_a_kutum_release_records_its_own_manifest(tmp_path):
    ws = tmp_path / 'release'
    (ws / 'scripts').mkdir(parents=True)
    shutil.copy(SCRIPT, ws / 'scripts/pool_session.sh')
    (ws / 'release.json').write_text('{"id": "abc1234", "vehicle_gate": "pass"}\n')
    env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin',
           'MONGLA_RUNS_ROOT': str(tmp_path / 'runs')}
    subprocess.run(['bash', '-c', f'source {ws}/scripts/pool_session.sh r1 >/dev/null'],
                   env=env, check=True)
    text = (tmp_path / 'runs/r1/software.txt').read_text()
    assert '"id": "abc1234"' in text and 'commit:' not in text
