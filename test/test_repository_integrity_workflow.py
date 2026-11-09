"""The two required checks must run what they claim, and nothing may skip them.

`tools/ci_contract.py` is an allowlist for both required workflows, and
`tools/run_repo_integrity.py` is the one list of guards. These tests run the
real contract against the real workflows, then against each way of quietly
disabling a gate that #93 named: every mutation must be refused.

Full ROS Jazzy/Humble build and test are a separate requirement, #65.
"""
from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


contract = _load('ci_contract')
runner = _load('run_repo_integrity')

INTEGRITY = (ROOT / contract.INTEGRITY).read_text()
PROVENANCE = (ROOT / contract.PROVENANCE).read_text()
GATE_STEP = '      - name: Run the integrity guards\n'
GATE_RUN = '        run: python tools/run_repo_integrity.py\n'


def test_the_real_workflows_keep_the_contract():
    assert contract.errors(ROOT) == []
    assert subprocess.run([sys.executable, str(ROOT / 'tools/ci_contract.py')],
                          capture_output=True).returncode == 0


def test_every_guard_the_runner_names_exists_and_includes_these_checks():
    assert all((ROOT / g).is_file() for g in runner.GUARDS)
    assert len(set(runner.GUARDS)) == len(runner.GUARDS)
    for me in ('test/test_repository_integrity_workflow.py', 'test/test_provenance_check.py'):
        assert me in runner.GUARDS


def _sub(text, old, new):
    assert old in text, f'fixture drifted: {old!r}'
    return text.replace(old, new, 1)


INTEGRITY_MUTATIONS = {
    'job if, false by expression':
        lambda t: _sub(t, '    runs-on: ubuntu-24.04\n',
                       '    if: ${{ always() && false }}\n    runs-on: ubuntu-24.04\n'),
    'step if':
        lambda t: _sub(t, GATE_RUN, GATE_RUN + "        if: github.actor == 'nobody'\n"),
    'echo instead of running':
        lambda t: _sub(t, GATE_RUN, '        run: echo python -m pytest test/x.py\n'),
    'or true':
        lambda t: _sub(t, GATE_RUN, '        run: python tools/run_repo_integrity.py || true\n'),
    'chained command':
        lambda t: _sub(t, GATE_RUN, '        run: python tools/run_repo_integrity.py; exit 0\n'),
    'job continue-on-error':
        lambda t: _sub(t, '    timeout-minutes: 15\n',
                       '    timeout-minutes: 15\n    continue-on-error: true\n'),
    'step continue-on-error':
        lambda t: _sub(t, GATE_RUN, GATE_RUN + '        continue-on-error: true\n'),
    'gate step removed':
        lambda t: _sub(t, GATE_STEP + GATE_RUN, ''),
    'pull_request trigger removed':
        lambda t: _sub(t, '  pull_request:\n', '  workflow_dispatch:\n'),
    'path filter hides changes':
        lambda t: _sub(t, '    types: [opened, synchronize, reopened, ready_for_review]\n',
                       '    types: [opened, synchronize, reopened, ready_for_review]\n'
                       "    paths: ['docs/**']\n"),
    'push filter widened':
        lambda t: _sub(t, '    branches: [main]\n', "    branches: [main]\n    paths-ignore: ['src/**']\n"),
    'job id renamed':
        lambda t: _sub(t, '  source-integrity:\n', '  source-integrity-old:\n'),
    'job name renamed':
        lambda t: _sub(t, '    name: Source integrity / active-code and documentation\n',
                       '    name: Source integrity\n'),
    'second job':
        lambda t: t + '  other:\n    runs-on: ubuntu-24.04\n    steps: []\n',
    'PYTEST_ADDOPTS on the gate':
        lambda t: _sub(t, GATE_RUN, GATE_RUN + "        env:\n          PYTEST_ADDOPTS: '-k nothing'\n"),
    'workflow-level env':
        lambda t: _sub(t, 'permissions:\n', "env:\n  PYTEST_ADDOPTS: '-k nothing'\npermissions:\n"),
    'checkout of another commit':
        lambda t: _sub(t, '          persist-credentials: false\n',
                       '          persist-credentials: false\n          ref: main\n'),
    'unknown action replaces the gate':
        lambda t: _sub(t, GATE_RUN, '        uses: someone/always-green@v1\n'),
    'write permission':
        lambda t: _sub(t, '  contents: read\n', '  contents: write\n'),
}


@pytest.mark.parametrize('name', sorted(INTEGRITY_MUTATIONS))
def test_each_way_to_quietly_disable_the_gate_is_refused(name):
    bad = INTEGRITY_MUTATIONS[name](INTEGRITY)
    assert bad != INTEGRITY
    assert contract.integrity_errors(bad), f'{name}: accepted'


PROVENANCE_MUTATIONS = {
    'job if':
        lambda t: _sub(t, '    runs-on: ubuntu-24.04\n', "    if: false\n    runs-on: ubuntu-24.04\n"),
    'provenance step removed':
        lambda t: _sub(t, '        run: python3 tools/provenance_check.py\n', '        run: true\n'),
    'contract step removed':
        lambda t: _sub(t, '        run: python3 tools/ci_contract.py\n', '        run: echo skipped\n'),
    'body pasted into the shell':
        lambda t: _sub(t, '        run: python3 tools/provenance_check.py\n',
                       '        run: echo "${{ github.event.pull_request.body }}"\n'),
    'comment step widened to always':
        lambda t: _sub(t, '        if: failure()\n', '        if: always()\n'),
    'edited trigger dropped':
        lambda t: _sub(t, 'reopened, edited,', 'reopened,'),
}


@pytest.mark.parametrize('name', sorted(PROVENANCE_MUTATIONS))
def test_each_way_to_quietly_disable_the_provenance_check_is_refused(name):
    bad = PROVENANCE_MUTATIONS[name](PROVENANCE)
    assert bad != PROVENANCE
    assert contract.provenance_errors(bad), f'{name}: accepted'


def test_the_contract_reads_the_parsed_yaml_not_comments():
    """A comment naming the gate command is not the gate."""
    bad = _sub(INTEGRITY, GATE_STEP + GATE_RUN,
               '      # run: python tools/run_repo_integrity.py\n')
    assert any('exactly one step' in e for e in contract.integrity_errors(bad))
    assert yaml.load(INTEGRITY, Loader=yaml.BaseLoader)['on']['pull_request']


def _run_runner(tmp_path, files):
    for rel, body in files.items():
        (tmp_path / rel).write_text(body)
    code = ('import importlib.util, sys, pathlib\n'
            f'spec = importlib.util.spec_from_file_location("r", {str(ROOT / "tools/run_repo_integrity.py")!r})\n'
            'm = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n'
            f'sys.exit(m.main(guards={tuple(files)!r}, root=pathlib.Path({str(tmp_path)!r})))\n')
    return subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)


def test_the_runner_passes_real_guards(tmp_path):
    r = _run_runner(tmp_path, {'test_a.py': 'def test_a():\n    assert True\n'})
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.parametrize('body, why', [
    ('import pytest\ndef test_a():\n    pass\n@pytest.mark.skip("off")\ndef test_b():\n    pass\n',
     'FAIL skipped'),
    ('import pytest\n@pytest.mark.parametrize("x", [])\ndef test_a(x):\n    pass\n'
     'def test_b():\n    pass\n', 'FAIL skipped'),
    ('def helper():\n    pass\n', 'FAIL no passing test'),
    ('def test_a():\n    assert False\n', 'FAIL'),
])
def test_the_runner_fails_a_guard_that_checks_nothing(tmp_path, body, why):
    r = _run_runner(tmp_path, {'test_a.py': 'def test_ok():\n    pass\n', 'test_b.py': body})
    assert r.returncode != 0 and why in r.stdout, r.stdout


def test_the_runner_fails_a_missing_guard(tmp_path):
    r = subprocess.run(
        [sys.executable, '-c',
         'import importlib.util, sys, pathlib\n'
         f'spec = importlib.util.spec_from_file_location("r", {str(ROOT / "tools/run_repo_integrity.py")!r})\n'
         'm = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n'
         f'sys.exit(m.main(guards=("test_gone.py",), root=pathlib.Path({str(tmp_path)!r})))\n'],
        capture_output=True, text=True)
    assert r.returncode == 1 and 'missing' in r.stdout


def test_ai_provenance_rechecks_when_pr_body_is_edited():
    """A required check that only ran on push would stay red after the
    description is fixed; the contract requires `edited` too."""
    doc = yaml.load(PROVENANCE, Loader=yaml.BaseLoader)
    assert {'opened', 'synchronize', 'reopened', 'edited'} <= set(doc['on']['pull_request']['types'])
