"""A pool session writes down which software produced it (#87).

The RoboSub 2026 runs can be tied to a commit only by date, because nothing
recorded one (docs/evidence/water.json, W-RS26). Sourcing
`tools/pool/pool_session.sh` now appends the workspace and its commit, or the
Kutum release's own `release.json`, to `$MONGLA_RUN_DIR/software.txt`.
Sourced for real, in each shell the operators use.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/pool/pool_session.sh'


def _head():
    return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True,
                          capture_output=True, text=True).stdout.strip()


@pytest.mark.parametrize('shell', ['bash', 'zsh'])
def test_sourcing_it_records_the_commit(tmp_path, shell):
    if shutil.which(shell) is None:
        pytest.skip(f'{shell} is not installed here')
    # A scratch HOME hides the caller's git config, and with it the
    # safe.directory that CI's root container needs for a checkout it does not
    # own; without it git refuses the tree and the commit reads `unknown`.
    env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin',
           'MONGLA_RUNS_ROOT': str(tmp_path / 'runs'),
           'GIT_CONFIG_COUNT': '1', 'GIT_CONFIG_KEY_0': 'safe.directory',
           'GIT_CONFIG_VALUE_0': str(ROOT.resolve())}
    subprocess.run([shell, '-c', f'source {SCRIPT} t1 >/dev/null'], cwd='/',
                   env=env, check=True)
    text = (tmp_path / 'runs/t1/software.txt').read_text()
    assert f'workspace: {ROOT.resolve()}' in text
    assert f'commit: {_head()}' in text


def test_a_kutum_release_records_its_own_manifest(tmp_path):
    ws = tmp_path / 'release'
    (ws / 'tools/pool').mkdir(parents=True)
    shutil.copy(SCRIPT, ws / 'tools/pool/pool_session.sh')
    (ws / 'release.json').write_text('{"id": "abc1234", "vehicle_gate": "pass"}\n')
    env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin',
           'MONGLA_RUNS_ROOT': str(tmp_path / 'runs')}
    subprocess.run(['bash', '-c', f'source {ws}/tools/pool/pool_session.sh r1 >/dev/null'],
                   env=env, check=True)
    text = (tmp_path / 'runs/r1/software.txt').read_text()
    assert '"id": "abc1234"' in text and 'commit:' not in text
