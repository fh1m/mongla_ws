"""`stop_tree` must leave NO descendant running. Behavioural, not textual.

This guards a defect that survived three fixes because each one reported
success while doing nothing:

  1. reset the inherited SIGINT-ignore and SIGINT the launcher -- it did not
     exit in 20 s, the escalation sent SIGTERM, and `ros2 launch` does not
     forward SIGTERM, so its nodes were orphaned;
  2. `setsid cmd &` and signal the process group -- os.setsid() failed, the
     shim died at once, `$!` named a pid that had never existed, and stop()
     signalled nothing while returning 0;
  3. and in every case the script printed "stopped", which was true of the
     launcher and false of the five nodes still holding the Hailo.

So this test asserts on PROCESSES, never on the script's own output or on the
text of the library. A test that reads the source would have passed for all
three broken versions.

The tree here is deliberately built the way the real one is: children
backgrounded inside a NON-INTERACTIVE shell, which makes them inherit SIGINT
set to ignore. That is the condition the whole library exists for, and a tree
whose children die on SIGINT would not test it.
"""
import os
import pathlib
import signal
import subprocess
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
LIB = ROOT / 'tools' / '_record_lib.sh'


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except (ProcessLookupError, PermissionError):
        return False
    return True


def _descendants(pid: int) -> list[int]:
    out = subprocess.run(['pgrep', '-P', str(pid)],
                         capture_output=True, text=True)
    return [int(x) for x in out.stdout.split()]


def _reap(pids):
    for p in pids:
        try:
            os.kill(p, signal.SIGKILL)
        except OSError:
            pass


@pytest.fixture
def tree():
    """A parent with three children that IGNORE SIGINT, as the real one does."""
    proc = subprocess.Popen(
        ['bash', '-c', 'sleep 300 & sleep 300 & sleep 300 & wait'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    kids = []
    for _ in range(50):                       # wait for the children to exist
        kids = _descendants(proc.pid)
        if len(kids) == 3:
            break
        time.sleep(0.1)
    assert len(kids) == 3, f'fixture never started three children: {kids}'
    yield proc, kids
    _reap([proc.pid] + kids)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _run(script: str, timeout: int = 90):
    # STOP_TREE_GRACE_S shortens only the WAIT before escalating, never what is
    # signalled. The children here ignore SIGINT deliberately, so the full
    # production grace period would be burned three times for no extra
    # assurance.
    env = dict(os.environ, STOP_TREE_GRACE_S='2')
    return subprocess.run(['bash', '-c', f'source "{LIB}"\n{script}'],
                          capture_output=True, text=True, timeout=timeout,
                          cwd=ROOT, env=env)


@pytest.mark.skipif(not LIB.is_file(), reason='_record_lib.sh is gone')
def test_stop_tree_kills_every_descendant(tree):
    proc, kids = tree
    _run(f'stop_tree {proc.pid} fixture')
    time.sleep(1)
    survivors = [p for p in kids if _alive(p)]
    assert not survivors, (
        f'{len(survivors)} of 3 children SURVIVED stop_tree: {survivors}. '
        f'This is the defect: the parent dies, its children are reparented to '
        f'init, and a re-walk of `pgrep -P` can never find them again.')
    # The parent must be reaped before it can be called dead: it is a direct
    # child of this process, so between being killed and being waited for it
    # is a ZOMBIE, and os.kill(pid, 0) succeeds on a zombie. Checking without
    # reaping measures bookkeeping, not the code under test.
    proc.wait(timeout=10)
    assert not _alive(proc.pid), 'the parent itself survived stop_tree'


@pytest.mark.skipif(not LIB.is_file(), reason='_record_lib.sh is gone')
def test_stop_tree_reports_survivors_rather_than_its_own_exit(tree):
    """"Launcher stopped" was true during every run that left nodes behind."""
    proc, _ = tree
    r = _run(f'stop_tree {proc.pid} fixture')
    said = r.stdout + r.stderr
    assert 'surviving' in said.lower() or 'SURVIVE' in said, (
        'stop_tree no longer says anything about survivors; reporting its own '
        f'exit is what hid this defect three times. Got: {said!r}')


@pytest.mark.skipif(not LIB.is_file(), reason='_record_lib.sh is gone')
def test_a_non_recursive_kill_tree_leaves_the_children(tree):
    """The injection. Break the recursion and the children must survive.

    A guard that has never failed against the real defect is not a guard, so
    this reproduces it deliberately: with kill_tree signalling only the pid it
    was given, the children outlive it -- which is exactly what happened on
    the vehicle, five times over.
    """
    proc, kids = tree
    _run(f'''
descendants() {{ :; }}                       # the snapshot finds nobody
stop_tree {proc.pid} fixture
''')
    time.sleep(1)
    survivors = [p for p in kids if _alive(p)]
    assert survivors, (
        'a NON-recursive kill_tree left no orphans, so this test cannot '
        'detect the defect it exists for -- the fixture is wrong, not the code')
