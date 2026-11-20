"""Every tracked Python file: no undefined name, no unused one, no silent redefinition.

The pyflakes rules (ruff `F`) plus syntax errors (`E9`), nothing about style.
Those are the defects a test can miss: a name misspelt on a branch no test
takes is a NameError on the vehicle, an unused variable is often an assertion
someone meant to write, and a redefinition hides the first definition. On
2026-11-20 the repository had 143 unused imports, 10 unused variables and 3
redefinitions, none of them yet a bug; this keeps it at none.

A deliberate exception is a `# noqa: F...` on the line, with the reason.

    ruff check --isolated --select F,E9 $(git ls-files '*.py')
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RULES = ['--isolated', '--select', 'F,E9', '--target-version', 'py310',
         '--no-cache', '--output-format', 'concise']


def _ruff():
    exe = shutil.which('ruff')
    if exe:
        return [exe]
    try:
        import ruff  # noqa: F401 -- only asking whether the wheel is installed
    except ImportError:
        pytest.skip('ruff is not installed (the integrity gate installs it)')
    return [sys.executable, '-m', 'ruff']


def _tracked_python():
    out = subprocess.run(['git', 'ls-files', '*.py'], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout.split()
    return [p for p in out if (ROOT / p).is_file()]


def test_every_name_resolves_and_is_used():
    files = _tracked_python()
    assert len(files) > 100, f'only {len(files)} Python files found: wrong root?'
    got = subprocess.run(_ruff() + ['check'] + RULES + files, cwd=ROOT,
                         capture_output=True, text=True)
    assert got.returncode == 0, (
        'pyflakes-class findings (fix them, or `# noqa: F...` with the reason):\n'
        + got.stdout[-6000:] + got.stderr[-2000:])
