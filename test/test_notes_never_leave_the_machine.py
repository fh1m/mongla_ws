"""The pre-push hook refuses refs/notes/* (owner ruling 2026-11-24).

Authorship notes are a local debugging record. The repository is public, and
on 2026-11-24 490 notes commits had to be rewritten out of it.
"""
import pathlib
import subprocess

HOOK = pathlib.Path(__file__).resolve().parents[1] / '.beads' / 'hooks' / 'pre-push'
Z = '0' * 40


def _run(line):
    return subprocess.run(['/bin/sh', str(HOOK), 'origin', 'https://example.invalid/x.git'],
                          input=line, text=True, capture_output=True,
                          env={'PATH': '/usr/bin:/bin'})


def test_a_notes_ref_is_refused():
    r = _run(f'refs/notes/ai {"1" * 40} refs/notes/ai {Z}\n')
    assert r.returncode == 1 and 'refs/notes' in r.stderr


def test_a_branch_passes_the_guard():
    r = _run(f'refs/heads/main {"1" * 40} refs/heads/main {Z}\n')
    assert 'refusing' not in r.stderr
