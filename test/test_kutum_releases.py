"""Releases on Kutum: staged beside the live tree, switched atomically (#64).

These run the real kutum/release.sh in a throwaway HOME with a stand-in
colcon. Each case is one of #64's acceptance cases. The same script was also
run against the vehicle itself (measured-bars and the PR say when); this file
is what keeps it true between runs.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KUTUM = ROOT / 'kutum'

pytestmark = pytest.mark.skipif(shutil.which('bash') is None or shutil.which('pgrep') is None,
                                reason='needs bash and pgrep')


@pytest.fixture()
def pi(tmp_path):
    """A fake vehicle: HOME, a colcon that obeys a flag file, an empty ROS."""
    home = tmp_path / 'home'
    home.mkdir()
    bin_ = tmp_path / 'bin'
    bin_.mkdir()
    (bin_ / 'colcon').write_text(
        '#!/bin/sh\n'
        '[ -e "$HOME/colcon-fails" ] && { echo "Summary: 1 package failed"; exit 1; }\n'
        'mkdir -p install && touch install/setup.bash\n'
        'echo "Summary: 7 packages finished"\n')
    (bin_ / 'colcon').chmod(0o755)
    ros = tmp_path / 'ros_setup.bash'
    ros.write_text('')
    env = dict(os.environ, HOME=str(home), KUTUM_ROS_SETUP=str(ros),
               PATH=f'{bin_}:{os.environ["PATH"]}')
    return home, env


def stage(home, rel_id):
    """What deploy.sh's rsync leaves: a full tree in releases/ID."""
    d = home / 'mongla/releases' / rel_id
    shutil.copytree(KUTUM, d / 'kutum')
    proto = d / 'src/mongla_control/mongla_control/fc'
    proto.mkdir(parents=True)
    (proto / 'srot_protocol.py').write_text('FW_BEHAVIOUR_REV_REQUIRED = 14\n')
    return d


def rel(env, *args):
    return subprocess.run(['bash', str(KUTUM / 'release.sh'), *args], env=env,
                          capture_output=True, text=True)


def live(home):
    return os.path.basename(os.path.realpath(home / 'mongla_ws'))


def deploy(home, env, rel_id):
    stage(home, rel_id)
    assert rel(env, 'build', rel_id).returncode == 0
    r = rel(env, 'activate', rel_id, f'{rel_id} from test')
    assert r.returncode == 0, r.stderr
    return r


def test_a_release_builds_beside_the_live_tree_and_switches_atomically(pi):
    home, env = pi
    deploy(home, env, 'aaa-1')
    assert (home / 'mongla_ws').is_symlink() and live(home) == 'aaa-1'
    doc = json.loads((home / 'mongla_ws/release.json').read_text())
    assert doc['release'] == 'aaa-1' and doc['board_firmware_rev_required'] == 14
    assert doc['software'] == 'built and active' and doc['vehicle_gate'] == 'not run'
    assert (home / 'mongla_ws/.kutum-deployed').read_text().startswith('aaa-1 from test')
    listed = rel(env, 'list').stdout
    assert '* aaa-1' in listed and 'gate: not run' in listed, listed


def test_the_first_deploy_keeps_the_old_workspace_whole(pi):
    home, env = pi
    old = home / 'mongla_ws'
    (old / 'install').mkdir(parents=True)
    (old / 'notes.txt').write_text('only here')
    deploy(home, env, 'aaa-1')
    legacy = [p for p in (home / 'mongla/releases').iterdir() if p.name.startswith('legacy-')]
    assert len(legacy) == 1 and (legacy[0] / 'notes.txt').read_text() == 'only here'
    assert (home / 'mongla/previous').exists() is False   # never a rollback target


def test_a_failed_build_leaves_the_active_release_untouched(pi):
    home, env = pi
    deploy(home, env, 'good-1')
    (home / 'colcon-fails').touch()
    stage(home, 'bad-2')
    assert rel(env, 'build', 'bad-2').returncode != 0
    r = rel(env, 'activate', 'bad-2', 'bad')
    assert r.returncode != 0 and 'never built' in r.stderr, r.stderr
    assert live(home) == 'good-1'


def test_an_interrupted_transfer_never_builds_or_activates(pi):
    home, env = pi
    deploy(home, env, 'good-1')
    (home / 'mongla/releases/cut-2').mkdir()            # rsync died before src/
    assert rel(env, 'build', 'cut-2').returncode != 0
    assert rel(env, 'activate', 'cut-2', 'x').returncode != 0
    assert live(home) == 'good-1'


def test_a_running_stack_refuses_without_touching_anything(pi):
    home, env = pi
    deploy(home, env, 'good-1')
    stage(home, 'next-2')
    assert rel(env, 'build', 'next-2').returncode == 0
    dummy = subprocess.Popen(['bash', '-c', 'exec -a mongla_manager sleep 30'])
    try:
        time.sleep(0.2)
        before = sorted(p.name for p in (home / 'mongla/releases').iterdir())
        for args in (('preflight',), ('activate', 'next-2', 'x'), ('rollback',)):
            r = rel(env, *args)
            assert r.returncode != 0 and 'RUNNING' in r.stderr, args
        assert live(home) == 'good-1'
        assert sorted(p.name for p in (home / 'mongla/releases').iterdir()) == before
    finally:
        dummy.kill()


def test_preflight_does_not_refuse_itself(pi):
    """On the vehicle a bare pgrep matched the ssh shell running it, whose
    command line names the workspace."""
    _, env = pi
    r = subprocess.run(['bash', '-c', f'echo mongla_manager >/dev/null; '
                                      f'bash {KUTUM / "release.sh"} preflight'],
                       env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_rollback_goes_back_and_forth_but_never_to_a_legacy_tree(pi):
    home, env = pi
    (home / 'mongla_ws').mkdir()
    deploy(home, env, 'one-1')
    r = rel(env, 'rollback')
    assert r.returncode != 0                     # previous is none: legacy is not one
    deploy(home, env, 'two-2')
    assert rel(env, 'rollback').returncode == 0 and live(home) == 'one-1'
    assert rel(env, 'rollback').returncode == 0 and live(home) == 'two-2'


def test_prune_keeps_active_previous_legacy_and_the_newest(pi):
    home, env = pi
    (home / 'mongla_ws').mkdir()
    for k in range(1, 6):
        deploy(home, env, f'r{k}')
        time.sleep(0.02)
    (home / 'mongla/releases/stale-incomplete').mkdir()
    assert rel(env, 'prune', '1').returncode == 0
    left = sorted(p.name for p in (home / 'mongla/releases').iterdir())
    assert 'r5' in left and 'r4' in left                  # active and previous
    assert any(n.startswith('legacy-') for n in left)
    assert 'r1' not in left and 'stale-incomplete' not in left


def test_a_bad_release_id_is_refused(pi):
    _, env = pi
    for bad in ('../x', '', '-rf'):
        assert rel(env, 'build', bad).returncode != 0


def _probe():
    import importlib.util
    spec = importlib.util.spec_from_file_location('armed_probe', KUTUM / 'armed_probe.py')
    assert spec is not None and spec.loader is not None
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class _HB:
    def __init__(self, base_mode, type_=12, autopilot=0):
        self.base_mode, self.type, self.autopilot = base_mode, type_, autopilot


@pytest.mark.parametrize('devs, hb, code', [
    ([], None, 0),                                # no board: nothing can move
    (['/dev/srot'], _HB(0), 0),                   # DISARMED
    (['/dev/srot'], _HB(128 | 1), 1),             # ARMED
    (['/dev/srot'], None, 2),                     # attached but silent: cannot tell
    (['/dev/srot'], RuntimeError('busy'), 2),     # held by another process
])
def test_the_armed_probe(monkeypatch, devs, hb, code):
    pytest.importorskip('pymavlink')
    m = _probe()
    monkeypatch.setattr(m, 'candidates', lambda: devs)

    def heartbeat(dev, listen_s=0):
        if isinstance(hb, Exception):
            raise hb
        return hb
    monkeypatch.setattr(m, 'board_heartbeat', heartbeat)
    assert m.main() == code


def test_deploy_checks_safety_before_it_writes_anything():
    """The ordering #64 was about: preflight and the armed probe come before
    the first command that changes the vehicle."""
    d = (KUTUM / 'deploy.sh').read_text()
    first_write = min(d.index('r mkdir -p "$REL"'), d.index('rsync -az'))
    assert d.index('preflight < "$REPO_ROOT/kutum/release.sh"') < first_write
    assert d.index('armed_probe.py') < first_write
    assert d.index('release.sh build') < d.index('release.sh activate')
