"""A node holding a camera or the Hailo cannot outlive its parent (2026-10-03).

Driven, not grepped: a child calls `die_with_parent`, its parent is killed,
and the child must be gone.
"""
import os
import subprocess
import sys
import textwrap
import time

import pytest

SRC = os.path.join(os.path.dirname(__file__), '..', '..', 'mongla_localization')


def _running(pid: int) -> bool:
    """A zombie is dead. Where orphans are not reaped -- a container whose
    pid 1 is not an init, as in CI -- the killed child stays a zombie, and
    `kill(pid, 0)` would call it alive."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] not in ('Z', 'X')
    except OSError:
        return True


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='Linux only')
def test_the_child_is_terminated_when_its_parent_dies(tmp_path):
    pidfile = tmp_path / 'child.pid'
    parent = textwrap.dedent(f'''
        import subprocess, sys, time
        child = subprocess.Popen([sys.executable, '-c', {textwrap.dedent(f"""
            import sys, time, os
            sys.path.insert(0, {SRC!r})
            from mongla_localization.orphan import die_with_parent
            die_with_parent()
            open({str(pidfile)!r}, 'w').write(str(os.getpid()))
            time.sleep(60)
        """)!r}])
        time.sleep(60)
    ''')
    p = subprocess.Popen([sys.executable, '-c', parent])
    deadline = time.monotonic() + 10
    while not pidfile.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    child = int(pidfile.read_text())
    p.kill()
    p.wait()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if not _running(child):
            return
        time.sleep(0.05)
    os.kill(child, 9)
    pytest.fail('the child outlived its parent')


def test_every_long_running_node_asks_for_it():
    """Every node a launch file starts. Interactive tools (calibration, the
    check_* utilities) run in a terminal and are left out on purpose."""
    import importlib
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools'))
    from code_text import code_of
    for name in ('mongla_vision.camera_node', 'mongla_vision.detector_node',
                 'mongla_vision.detector_dual_node', 'mongla_vision.lock_node',
                 'mongla_vision.tracker_node', 'mongla_vision.flow.flow_node',
                 'mongla_vision.depth.depth_estimation_node',
                 'mongla_vision.web.mission_web_node',
                 'mongla_vision.utils.display_node', 'mongla_vision.vision_node',
                 'mongla_localization.localization_node',
                 'mongla_localization.pnp_node',
                 'mongla_localization.pose_fuse_node'):
        mod = importlib.import_module(name)
        assert 'die_with_parent()' in code_of(mod.main), name
