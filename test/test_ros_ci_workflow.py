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
    ('    timeout-minutes: 45\n', '    timeout-minutes: 45\n    if: false\n'),
    ('    timeout-minutes: 45\n', '    timeout-minutes: 45\n    continue-on-error: true\n'),
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


def test_numpy_stays_below_two_in_ci_too():
    assert '-c kutum/constraints.txt' in SCRIPT
    assert "numpy 2 would break cv_bridge" in SCRIPT
