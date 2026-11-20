"""Every GitHub workflow passes actionlint.

The workflows are the required checks. A mistyped expression, an unknown
`needs:` job or a step output read before it is set does not fail when the
file is pushed: it fails, or worse silently evaluates empty, on the run that
needed it. actionlint reads them as GitHub will, and runs shellcheck over each
`run:` block at the same warning level as `test_shell_scripts_pass_shellcheck`.
Measured on 2026-11-20: 57 ms for all five workflows; one info-level note
(an unquoted `$GUARDS`, a list split on purpose), so warning level is the bar.

    SHELLCHECK_OPTS='-S warning' actionlint .github/workflows/*.yml
"""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_every_workflow_passes_actionlint():
    exe = shutil.which('actionlint')
    if not exe:
        pytest.skip('actionlint is not installed (the integrity gate installs actionlint-py)')
    flows = sorted((ROOT / '.github' / 'workflows').glob('*.yml'))
    assert len(flows) >= 4, f'only {len(flows)} workflows found: wrong root?'
    env = dict(os.environ, SHELLCHECK_OPTS='-S warning')
    got = subprocess.run([exe, '-no-color'] + [str(f) for f in flows], cwd=ROOT,
                         capture_output=True, text=True, env=env)
    assert got.returncode == 0, 'actionlint findings:\n' + got.stdout[-6000:] + got.stderr[-2000:]
