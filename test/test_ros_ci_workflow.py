"""The ROS job builds and tests the whole stack on both distros, every time (#65).

`.github/workflows/ros.yml` is held to one shape by `tools/ci_contract.py`,
and everything it does is `tools/ci_ros.sh`. These tests run the contract on
the real file, then on each way of quietly weakening it; every mutation must
be refused. The script is held to never turning a failure into a pass.
"""
from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('ci_contract', ROOT / 'tools/ci_contract.py')
assert _spec is not None and _spec.loader is not None
contract = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(contract)

ROS = (ROOT / contract.ROS).read_text()
SCRIPT = (ROOT / 'tools/ci_ros.sh').read_text()


def test_the_real_workflow_keeps_the_contract():
    assert contract.ros_errors(ROS) == []


@pytest.mark.parametrize('old, new', [
    ('distro: [humble, jazzy]', 'distro: [jazzy]'),                 # a distro dropped
    ('fail-fast: false', 'fail-fast: true'),                       # a sibling cancelled
    ('    timeout-minutes: 90\n', '    timeout-minutes: 90\n    if: false\n'),
    ('    timeout-minutes: 90\n', '    timeout-minutes: 90\n    continue-on-error: true\n'),
    ('run: bash tools/ci_ros.sh', 'run: bash tools/ci_ros.sh || true'),
    ('run: bash tools/ci_ros.sh', 'run: echo bash tools/ci_ros.sh'),
    ('ros:${{ matrix.distro }}-ros-base', 'ros:jazzy-ros-base'),     # one image for both
    ('  push:\n    branches: [main]\n', ''),                          # main never checked
    ('  contents: read', '  contents: write'),
    ('    types: [opened, synchronize, reopened, ready_for_review]',
     '    types: [opened]'),
    ('        run: bash tools/ci_ros.sh\n',
     '        run: bash tools/ci_ros.sh\n        env:\n          PYTEST_ADDOPTS: -k nothing\n'),
])
def test_every_weakening_is_refused(old, new):
    assert old in ROS, old
    assert contract.ros_errors(ROS.replace(old, new, 1)), (old, new)


def test_the_script_never_turns_a_failure_into_a_pass():
    code = '\n'.join(l for l in SCRIPT.splitlines() if not l.lstrip().startswith('#'))
    assert 'set -euo pipefail' in code
    assert '|| true' not in code and '|| :' not in code
    pytest_lines = [l for l in code.splitlines() if re.search(r'-m pytest\b', l)]
    assert len(pytest_lines) == 1, pytest_lines
    run = pytest_lines[0].split('pytest', 1)[1]      # pytest's own arguments
    assert 'src/*/test test' in run
    assert not re.search(r'\s-(k|m|x)\b|--deselect|--lf|--maxfail|--ignore', run), run
    assert code.rstrip().endswith('exit "$rc"')
    assert 'rc=${PIPESTATUS[0]}' in code


def test_a_mirror_blip_is_retried_before_it_fails_a_required_check():
    code = '\n'.join(l for l in SCRIPT.splitlines() if not l.lstrip().startswith('#'))
    retries = code.index('Acquire::Retries "5"')
    assert retries < code.index('apt-get update'), 'retries must be set before apt runs'
    assert 'Fix-Missing' not in code and '--fix-missing' not in code
    mirror = code.index('azure.archive.ubuntu.com')
    assert mirror < code.index('apt-get update'), 'the mirror must be set before apt runs'
    # Both layouts: jammy's sources.list (humble) and noble's deb822 (jazzy).
    assert '/etc/apt/sources.list.d/ubuntu.sources' in code and '/etc/apt/sources.list ' in code


def test_an_index_refresh_that_never_succeeds_still_fails_the_job(tmp_path):
    """The retry loop runs for real against an apt-get that always fails: a
    loop whose last command is `sleep` would exit 0 and pass the check."""
    import subprocess
    start = SCRIPT.index('for attempt in')
    loop = SCRIPT[start:SCRIPT.index('done', start) + 4]
    fake = tmp_path / 'apt-get'
    fake.write_text('#!/bin/sh\nexit 100\n')
    fake.chmod(0o755)
    (tmp_path / 'sleep').write_text('#!/bin/sh\nexit 0\n')
    (tmp_path / 'sleep').chmod(0o755)
    env = {'PATH': f'{tmp_path}:/usr/bin:/bin'}
    r = subprocess.run(['bash', '-c', loop + '\necho after'], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 100 and 'after' not in r.stdout
    fake.write_text('#!/bin/sh\nexit 0\n')
    r = subprocess.run(['bash', '-c', loop + '\necho after'], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0 and 'after' in r.stdout


def test_numpy_stays_below_two_in_ci_too():
    assert '-c kutum/constraints.txt' in SCRIPT
    assert "numpy 2 would break cv_bridge" in SCRIPT


def _pin(path, name):
    for line in (ROOT / path).read_text().splitlines():
        m = re.match(rf'{name}\s*==\s*([\w.]+)', line.strip())
        if m:
            return m.group(1)
    return None


def test_ci_resolves_the_vehicles_pymavlink():
    """The decoding tests pin pymavlink's behaviour. An open `>=` let CI pull a
    newer release that decodes SYS_STATUS extended health, so the suite there
    described a library the vehicle does not run. One pin, read from both."""
    vehicle = _pin('kutum/requirements.txt', 'pymavlink')
    assert vehicle, 'kutum/requirements.txt no longer pins pymavlink'
    assert _pin('requirements.txt', 'pymavlink') == vehicle
