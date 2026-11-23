"""The commit hook refuses an assistant session link (owner ruling 2026-11-24).

The repositories are public; a session link in a commit message, a staged file
or a pushed commit is refused before it leaves the machine.
"""
import json
import pathlib
import subprocess
import sys

HOOK = pathlib.Path(__file__).resolve().parents[1] / '.claude' / 'hooks' / 'commit_guard.py'
LINK = 'claude.ai/code/' + 'session_0123'      # split so this file holds no link
TRAILER = 'Claude-' + 'Session:'                # split for the same reason


def _run(cmd, cwd):
    payload = json.dumps({'tool_input': {'command': cmd}})
    return subprocess.run([sys.executable, str(HOOK)], input=payload, text=True,
                          capture_output=True, cwd=cwd,
                          env={'CLAUDE_PROJECT_DIR': str(cwd), 'PATH': '/usr/bin:/bin'})


def test_a_commit_message_with_a_session_link_is_refused(tmp_path):
    subprocess.run(['git', 'init', '-q'], cwd=tmp_path, check=True)
    r = _run(f'git commit -m "x\n\n{TRAILER} https://{LINK}"', tmp_path)
    assert r.returncode == 2 and 'session link' in r.stderr


def test_a_staged_file_with_a_session_link_is_refused(tmp_path):
    subprocess.run(['git', 'init', '-q'], cwd=tmp_path, check=True)
    (tmp_path / 'n.md').write_text(f'see https://{LINK}\n')
    subprocess.run(['git', 'add', 'n.md'], cwd=tmp_path, check=True)
    r = _run('git commit -m "notes"', tmp_path)
    assert r.returncode == 2 and 'session link' in r.stderr


def test_a_plain_commit_passes(tmp_path):
    subprocess.run(['git', 'init', '-q'], cwd=tmp_path, check=True)
    assert _run('git commit -m "plain"', tmp_path).returncode == 0
