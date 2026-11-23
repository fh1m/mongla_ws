"""CI must not paste attacker-controlled text into a shell.

A workflow that read the pull request body carried this until 2026-09-30 (issue #54):

    PR_BODY="${{ github.event.pull_request.body }}"

`${{ }}` is substituted by the Actions runner BEFORE bash parses the script, so
the pull request description became shell SOURCE. Anyone able to open a pull
request -- which on a public repository is anyone -- could run commands on the
runner with the job's `GITHUB_TOKEN` in the environment.

The safe form passes the value through `env:`. The runner sets a variable and
bash never parses its contents, so `"$PR_BODY"` is exactly the bytes the author
typed.

The static test catches a new workflow, or a new step, written the old way.
The behavioural test that ran a hostile body through the old step went with
that step, when no required workflow read the body any more (2026-11-24).
"""

import pathlib
import re

import pytest

yaml = pytest.importorskip('yaml')

ROOT = pathlib.Path(__file__).resolve().parents[3]
WORKFLOWS = sorted((ROOT / '.github' / 'workflows').glob('*.y*ml'))

# Contexts an outsider controls: the PR body, title, branch names, an issue
# comment, a fork's repo description. `github.event.*` covers all of them, and
# `inputs.*` covers a workflow_dispatch field.
UNTRUSTED = re.compile(r'\$\{\{\s*(github\.event\b|inputs\.|github\.head_ref\b)')


def _run_steps(wf: dict):
    for job_name, job in (wf.get('jobs') or {}).items():
        for step in job.get('steps') or []:
            if isinstance(step, dict) and step.get('run'):
                yield job_name, step


def test_no_workflow_interpolates_untrusted_input_into_a_run_block():
    assert WORKFLOWS, 'no workflows found -- has .github/workflows moved?'
    offenders = []
    for path in WORKFLOWS:
        wf = yaml.safe_load(path.read_text(encoding='utf-8'))
        for job_name, step in _run_steps(wf):
            for hit in UNTRUSTED.findall(step['run']):
                offenders.append(f'{path.name}:{job_name}:{step.get("name", "?")} -> {hit}')
    assert not offenders, (
        'attacker-controlled text is interpolated into a shell script: '
        f'{offenders}. Pass it through `env:` and read it as "$VAR" -- an '
        'environment variable is data, a `${{ }}` substitution is source code.'
    )


def test_every_workflow_declares_least_privilege_permissions():
    """An undeclared token scope depends on a repo setting this file cannot see."""
    missing = [p.name for p in WORKFLOWS
               if 'permissions' not in (yaml.safe_load(p.read_text(encoding='utf-8')) or {})
               and not all('permissions' in j for j in
                           (yaml.safe_load(p.read_text(encoding='utf-8')) or {})
                           .get('jobs', {}).values())]
    assert not missing, (
        f'{missing} declare no `permissions:`, so the GITHUB_TOKEN scope is '
        'whatever the repository default happens to be. State it in the file.')
