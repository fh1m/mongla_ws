"""The repository holds source, docs and their tools, and nothing generated.

A clone should look like the project, not like the last machine that worked
on it. Two rules, both read off `git ls-files`:

* the top level is a fixed list. A new top-level entry is a decision, so it
  is a line here with its reason, not a file that appears;
* nothing tracked is build output, a cache, a capture or an editor leftover.

Local tool state (agent caches, code indexes) is ignored in .gitignore, and
the last test checks the directories we know tools create are ignored.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

TOP_LEVEL = {
    # the stack and its tests
    'src', 'test', 'tools', 'config', 'kutum',
    'conftest.py', 'pytest.ini', 'requirements.txt', 'build_mongla.sh',
    # the book, the site and the project's own records
    'docs', '.claude', 'CLAUDE.md', 'README.md', 'AUTHORS.md', 'CITATION.cff',
    'LICENSE', 'SECURITY.md',
    # repository plumbing
    '.github', '.gitignore', '.beads', '.vscode',
}

# Generated, cached, captured or left behind by an editor or a merge.
FORBIDDEN_SUFFIXES = ('.pyc', '.pyo', '.so', '.o', '.tlog', '.tlog.raw', '.db3',
                      '.mcap', '.bag', '.orig', '.rej', '.bak', '.swp', '~')
FORBIDDEN_PARTS = {'__pycache__', '.pytest_cache', '.ruff_cache', '.mypy_cache',
                   'build', 'install', 'log', 'logs', '.codegraph', '.omp',
                   '.remember', '.ipynb_checkpoints', 'node_modules'}

# Directories local tools create in the workspace. Each must be ignored, or
# the first `git add -A` commits it.
LOCAL_TOOL_DIRS = ('build/', 'install/', 'log/', '.codegraph/', '.omp/',
                   '.remember/', '.pytest_cache/', '__pycache__/')


def tracked():
    r = subprocess.run(['git', 'ls-files', '-z'], cwd=ROOT, capture_output=True)
    if r.returncode != 0:
        pytest.skip('not a git checkout')
    return [p for p in r.stdout.decode().split('\0') if p]


def test_the_top_level_is_the_agreed_list():
    extra = sorted({p.split('/', 1)[0] for p in tracked()} - TOP_LEVEL)
    assert not extra, (f'new top-level entries {extra}: put them under an existing '
                       f'directory, or add them to TOP_LEVEL with the reason')


def test_nothing_generated_is_tracked():
    bad = [p for p in tracked()
           if p.endswith(FORBIDDEN_SUFFIXES) or FORBIDDEN_PARTS & set(p.split('/')[:-1])]
    assert not bad, f'generated or leftover files are tracked: {bad[:20]}'


@pytest.mark.parametrize('d', LOCAL_TOOL_DIRS)
def test_local_tool_state_is_ignored(d):
    r = subprocess.run(['git', 'check-ignore', '-q', '--no-index', d + 'probe'],
                       cwd=ROOT)
    assert r.returncode == 0, f'{d} is not in .gitignore'
