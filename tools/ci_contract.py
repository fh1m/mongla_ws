#!/usr/bin/env python3
"""The shape the two required workflows must keep, as an allowlist.

    python3 tools/ci_contract.py [--root DIR]

Exits 1 and prints each violation. Both required checks run it: the
integrity job through its tests, and the provenance job directly, so a
change that skips one job is still caught by the other (#93).

Why an allowlist and not a list of bad patterns: GitHub reports a job
skipped by an `if:` as Success, and a skipped required check does not block
a merge (GitHub docs, "Troubleshooting required status checks"). An `if:`
can be false in endless spellings (`${{ always() && false }}`), and a `run:`
can mention pytest without running it (`echo python -m pytest ...`). So the
contract does not try to evaluate either: no `if:` at all on a gating job or
step, only known actions, and only exact commands.

Limit: a single pull request that edits both workflows and this file can
still pass both checks on itself. Closing that needs a check whose code the
pull request cannot change (#96).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

INTEGRITY = '.github/workflows/repository-integrity.yml'
PROVENANCE = '.github/workflows/poc-check.yml'
ROS = '.github/workflows/ros.yml'

INTEGRITY_JOB = ('source-integrity', 'Source integrity / active-code and documentation')
PROVENANCE_JOB = ('poc-check', 'Verify AI Provenance')
# One job, two required checks: 'ROS humble' and 'ROS jazzy' (#65).
ROS_JOB = ('ros', 'ROS ${{ matrix.distro }}')
ROS_STRATEGY = {'fail-fast': 'false', 'matrix': {'distro': ['humble', 'jazzy']}}
ROS_CONTAINER = 'ros:${{ matrix.distro }}-ros-base'
RUN_ROS = 'bash tools/ci_ros.sh'

RUN_GATE = 'python tools/run_repo_integrity.py'
RUN_PROVENANCE = 'python3 tools/provenance_check.py'
RUN_CONTRACT = 'python3 tools/ci_contract.py'
PROVENANCE_ENV = {'PR_BODY': '${{ github.event.pull_request.body }}',
                  'PR_AUTHOR': '${{ github.event.pull_request.user.login }}'}
# A pip install of quoted requirement specifiers and nothing else: no
# `;`, `&&` or `||` can sit outside the quotes.
PIP = re.compile(r"python3? -m pip install --disable-pip-version-check( '[^']+')+")

# Allowed actions and the only inputs each may take. Checkout may not take
# `ref:`, which would test some other commit than the candidate.
ACTIONS = {
    'actions/checkout': {'persist-credentials'},
    'actions/setup-python': {'python-version'},
}
COMMENT_ACTION = 'actions/github-script'

TOP_KEYS = {'name', 'on', 'permissions', 'jobs'}
JOB_KEYS = {'name', 'runs-on', 'timeout-minutes', 'steps'}
STEP_KEYS = {'name', 'id', 'uses', 'with', 'run', 'env'}


def _load(text: str):
    try:
        doc = yaml.load(text, Loader=yaml.BaseLoader)
    except yaml.YAMLError as exc:
        return None, [f'invalid workflow YAML: {exc}']
    if not isinstance(doc, dict):
        return None, ['workflow must be a mapping']
    return doc, []


def _common(doc, job_id, job_name, pr_types, job_keys=JOB_KEYS):
    """Errors shared by both workflows; returns (errors, steps)."""
    errs = [f'top-level key {k!r} is not allowed' for k in sorted(set(doc) - TOP_KEYS)]
    on = doc.get('on')
    if not isinstance(on, dict) or 'pull_request' not in on:
        errs.append("missing required event 'pull_request'")
        on = on if isinstance(on, dict) else {}
    pr = on.get('pull_request') or {}
    if not isinstance(pr, dict) or set(pr) - {'types'}:
        errs.append('pull_request may not carry filters (paths, branches)')
        pr = pr if isinstance(pr, dict) else {}
    missing = set(pr_types) - set(pr.get('types') or [])
    if missing:
        errs.append(f'pull_request types missing {sorted(missing)}')
    jobs = doc.get('jobs')
    if not isinstance(jobs, dict) or set(jobs) != {job_id}:
        return errs + [f'jobs must be exactly {job_id!r}'], []
    job = jobs[job_id]
    if not isinstance(job, dict):
        return errs + [f'{job_id} must be a mapping'], []
    errs += [f'job key {k!r} is not allowed (no if:, no continue-on-error)'
             for k in sorted(set(job) - job_keys)]
    if job.get('name') != job_name:
        errs.append(f'job name must stay {job_name!r}: it is the required check')
    steps = job.get('steps')
    if not isinstance(steps, list) or not all(isinstance(s, dict) for s in steps):
        return errs + ['steps must be a list of mappings'], []
    return errs, steps


# Every action is pinned to a full commit SHA: a tag can be moved to other
# code after review, a SHA cannot. Dependabot proposes the updates (#95).
PINNED = re.compile(r'[^@\s]+@[0-9a-f]{40}')


def pin_errors(text: str) -> list[str]:
    """Any `uses:` in any workflow that is not pinned to a commit SHA."""
    return [f'action not pinned to a commit SHA: {u}'
            for u in re.findall(r'^\s*-?\s*uses:\s*(\S+)', text, re.M)
            if not PINNED.fullmatch(u)]


def _action(step):
    uses = step.get('uses', '')
    return uses.split('@', 1)[0] if uses else ''


def _check_step(step, runs, errs, allow_failure_comment=False):
    action = _action(step)
    if allow_failure_comment and action == COMMENT_ACTION:
        if step.get('if') != 'failure()':
            errs.append('the failure comment may only run on failure()')
        return
    errs.extend(f'step key {k!r} is not allowed' for k in sorted(set(step) - STEP_KEYS))
    if action:
        if action not in ACTIONS:
            errs.append(f'action {step["uses"]!r} is not on the allowlist')
        else:
            extra = set(step.get('with') or {}) - ACTIONS[action]
            if extra:
                errs.append(f'{action} input(s) {sorted(extra)} not allowed')
        if 'run' in step:
            errs.append('a step may not both use an action and run a command')
        return
    run = str(step.get('run', '')).strip()
    if not (run in runs or PIP.fullmatch(run)):
        errs.append(f'run {run!r} is not an allowed command')


def integrity_errors(text: str) -> list[str]:
    doc, errs = _load(text)
    if doc is None:
        return errs
    errs, steps = _common(doc, *INTEGRITY_JOB, ('opened', 'synchronize', 'reopened'))
    push = (doc.get('on') or {}).get('push')
    if not isinstance(push, dict) or push != {'branches': ['main']}:
        errs.append("push must be exactly 'branches: [main]'")
    if doc.get('permissions') != {'contents': 'read'}:
        errs.append('permissions must be exactly contents: read')
    for step in steps:
        if 'env' in step:
            errs.append('no env on a gating step (PYTEST_ADDOPTS can deselect)')
        _check_step(step, {RUN_GATE}, errs)
    runs = [str(s.get('run', '')).strip() for s in steps]
    if runs.count(RUN_GATE) != 1:
        errs.append(f'exactly one step must run {RUN_GATE!r}')
    return errs


def ros_errors(text: str) -> list[str]:
    """The ROS build-and-test job (#65): both distros, every time, one command."""
    doc, errs = _load(text)
    if doc is None:
        return errs
    errs, steps = _common(doc, *ROS_JOB, ('opened', 'synchronize', 'reopened'),
                          job_keys=JOB_KEYS | {'strategy', 'container'})
    push = (doc.get('on') or {}).get('push')
    if not isinstance(push, dict) or push != {'branches': ['main']}:
        errs.append("push must be exactly 'branches: [main]'")
    if doc.get('permissions') != {'contents': 'read'}:
        errs.append('permissions must be exactly contents: read')
    job = (doc.get('jobs') or {}).get(ROS_JOB[0]) or {}
    if isinstance(job, dict):
        if job.get('strategy') != ROS_STRATEGY:
            errs.append('strategy must be exactly fail-fast: false over [humble, jazzy]: '
                        'a dropped distro or a cancelled sibling is a check that never ran')
        if job.get('container') != ROS_CONTAINER:
            errs.append(f'container must be exactly {ROS_CONTAINER!r}')
    for step in steps:
        if 'env' in step:
            errs.append('no env on a gating step (PYTEST_ADDOPTS can deselect)')
        _check_step(step, {RUN_ROS}, errs)
    runs = [str(s.get('run', '')).strip() for s in steps]
    if runs.count(RUN_ROS) != 1:
        errs.append(f'exactly one step must run {RUN_ROS!r}')
    return errs


def provenance_errors(text: str) -> list[str]:
    doc, errs = _load(text)
    if doc is None:
        return errs
    errs, steps = _common(doc, *PROVENANCE_JOB,
                          ('opened', 'synchronize', 'reopened', 'edited'))
    for step in steps:
        run = str(step.get('run', '')).strip()
        env = step.get('env')
        if env is not None and not (run == RUN_PROVENANCE and env == PROVENANCE_ENV):
            errs.append('only the provenance step takes env: PR_BODY and PR_AUTHOR, '
                        'from the event')
        _check_step(step, {RUN_PROVENANCE, RUN_CONTRACT}, errs, allow_failure_comment=True)
    runs = [str(s.get('run', '')).strip() for s in steps]
    for need in (RUN_PROVENANCE, RUN_CONTRACT):
        if runs.count(need) != 1:
            errs.append(f'exactly one step must run {need!r}')
    return errs


def errors(root: Path) -> list[str]:
    out = []
    for path in sorted((root / '.github/workflows').glob('*.y*ml')):
        rel = path.relative_to(root)
        out += [f'{rel}: {e}' for e in pin_errors(path.read_text())]
    for rel, check in ((INTEGRITY, integrity_errors), (PROVENANCE, provenance_errors),
                       (ROS, ros_errors)):
        path = root / rel
        if not path.is_file():
            out.append(f'{rel}: missing')
            continue
        out += [f'{rel}: {e}' for e in check(path.read_text())]
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Check the required workflows keep their shape.')
    ap.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1],
                    help='repository whose workflows to check')
    errs = errors(ap.parse_args(argv).root)
    for e in errs:
        print(f'FAIL {e}')
    print('ok' if not errs else f'{len(errs)} violation(s)')
    return 1 if errs else 0


if __name__ == '__main__':
    sys.exit(main())
