"""CI must not paste attacker-controlled text into a shell.

`.github/workflows/poc-check.yml` carried this until 2026-09-30 (issue #54):

    PR_BODY="${{ github.event.pull_request.body }}"

`${{ }}` is substituted by the Actions runner BEFORE bash parses the script, so
the pull request description became shell SOURCE. Anyone able to open a pull
request -- which on a public repository is anyone -- could run commands on the
runner with the job's `GITHUB_TOKEN` in the environment.

The safe form passes the value through `env:`. The runner sets a variable and
bash never parses its contents, so `"$PR_BODY"` is exactly the bytes the author
typed.

Two tests, because each catches a different mistake:

* the STATIC one catches a new workflow, or a new step, written the old way;
* the BEHAVIOURAL one runs the real script out of the real YAML against a
  hostile body and proves the bytes stay data -- so if someone "simplifies" the
  `env:` block back into the `run:` block, the demonstration fails rather than
  the reasoning.
"""

import pathlib
import re
import subprocess

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


HOSTILE = (
    '## \U0001f916 AI Provenance\n'
    '| 1 | @someone | https://x | insight |\n'
    '"; touch /tmp/mongla_poc_injection_marker; echo "'
    "'; touch /tmp/mongla_poc_injection_marker2; echo '"
    '$(touch /tmp/mongla_poc_injection_marker3)\n'
    '`touch /tmp/mongla_poc_injection_marker4`\n'
)


def test_the_provenance_script_treats_a_hostile_body_as_data(tmp_path):
    """Run the shipped `run:` script with a body that tries four ways to escape."""
    wf = yaml.safe_load((ROOT / '.github' / 'workflows' / 'poc-check.yml')
                        .read_text(encoding='utf-8'))
    step = next(s for _, s in _run_steps(wf) if 'Provenance' in (s.get('name') or ''))

    # The body must reach the script through `env:`, not through the script text.
    assert 'PR_BODY' in (step.get('env') or {}), (
        'the PR body is no longer passed through `env:` -- if it moved back into '
        'the `run:` block, this is the injection from issue #54 again')

    markers = [tmp_path / f'marker{n}' for n in range(1, 5)]
    body = HOSTILE.replace('/tmp/mongla_poc_injection_marker', str(tmp_path / 'marker'))
    script = tmp_path / 'step.sh'
    script.write_text(step['run'], encoding='utf-8')
    out = tmp_path / 'github_output'
    out.touch()

    r = subprocess.run(
        ['bash', str(script)], capture_output=True, text=True,
        env={'PATH': '/usr/bin:/bin', 'PR_BODY': body, 'GITHUB_OUTPUT': str(out)})

    fired = [m.name for m in markers if m.exists()]
    assert not fired, f'the hostile body EXECUTED: {fired} created. stderr={r.stderr}'
    # And it still classifies correctly: a populated table is a pass.
    assert r.returncode == 0, f'rc={r.returncode} stdout={r.stdout} stderr={r.stderr}'
    assert 'status=pass' in out.read_text(encoding='utf-8'), out.read_text()
