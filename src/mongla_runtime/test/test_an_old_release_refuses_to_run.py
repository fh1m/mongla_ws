"""After a deploy, a process from an older release must not start (B-153)."""
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from mongla_runtime.release import refuse_other_release

SRC = Path(__file__).resolve().parents[2]
PACKAGE = Path(__file__).resolve().parents[1] / 'mongla_runtime'


def _home(tmp_path, active='new-2'):
    for r in ('old-1', 'new-2'):
        (tmp_path / 'mongla' / 'releases' / r / 'src').mkdir(parents=True)
    live = tmp_path / 'mongla_ws'
    live.symlink_to(tmp_path / 'mongla' / 'releases' / active)
    return live


def _in(tmp_path, release):
    return f'{tmp_path}/mongla/releases/{release}/build/x/y.py'


def test_the_active_release_runs_and_another_does_not(tmp_path):
    live = _home(tmp_path)
    refuse_other_release([_in(tmp_path, 'new-2'), '/opt/ros/jazzy/lib/z.py'], live)
    with pytest.raises(SystemExit, match='old-1.*active one is new-2'):
        refuse_other_release([_in(tmp_path, 'new-2'), _in(tmp_path, 'old-1')], live)


def test_away_from_kutum_nothing_is_refused(tmp_path):
    refuse_other_release(['/home/dev/ws/src/a.py'], tmp_path / 'no_link')


def test_with_no_link_two_releases_are_still_refused(tmp_path):
    refuse_other_release([_in(tmp_path, 'old-1')], tmp_path / 'no_link')
    with pytest.raises(SystemExit):
        refuse_other_release([_in(tmp_path, 'old-1'), _in(tmp_path, 'new-2')], tmp_path / 'no_link')


@pytest.mark.parametrize('release, runs', [('new-2', True), ('old-1', False)])
def test_importing_the_package_from_a_release(tmp_path, release, runs):
    _home(tmp_path)
    where = tmp_path / 'mongla' / 'releases' / release / 'src'
    shutil.copytree(PACKAGE, where / 'mongla_runtime')
    r = subprocess.run([sys.executable, '-c', 'import mongla_runtime'], cwd=tmp_path,
                       env={'HOME': str(tmp_path), 'PYTHONPATH': str(where)},
                       capture_output=True, text=True)
    assert (r.returncode == 0) is runs, r.stderr
    assert runs or 'refusing to run old code' in r.stderr, r.stderr


def test_every_package_that_runs_something_on_the_vehicle_checks():
    """`fetch` and `views` build files and run before the workspace is built, so
    Batighor checks in `serve`; its nodes and launch files import the runtime."""
    missing = []
    for setup in sorted(SRC.glob('*/setup.py')):
        pkg = setup.parent.name
        for module in re.findall(r"=\s*(mongla_[a-z_.]+):", setup.read_text()):
            if pkg == 'mongla_batighor' and not module.endswith('.serve'):
                continue
            top = module.split('.')[0]
            checked = SRC / pkg / top / ('serve.py' if pkg == 'mongla_batighor' else '__init__.py')
            if top != 'mongla_runtime' and 'import mongla_runtime' not in checked.read_text():
                missing.append(module)
    assert not missing, missing
