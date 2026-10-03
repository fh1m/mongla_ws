"""An orphaned manager stops itself (found on the vehicle, 2026-10-03).

`ros2 launch` died on SIGTERM and left the manager running, holding the
board's port and feeding its heartbeat, so the board's companion-loss
failsafe could never fire.
"""
import os
import types

import pytest

pytest.importorskip('rclpy')


def _rig():
    import mongla_manager.auv_manager_node as amn
    calls = []
    log = types.SimpleNamespace(error=lambda *a: calls.append('logged'))
    node = types.SimpleNamespace(get_logger=lambda: log)
    ex = types.SimpleNamespace(shutdown=lambda timeout_sec=None: calls.append('shutdown'))
    return amn, node, ex, calls


def test_a_dead_parent_triggers_the_shutdown_path():
    amn, node, ex, calls = _rig()
    handler = amn._orphan_guard(node, ex, parent_pid=os.getppid() + 999999)
    handler(10, None)
    assert calls == ['logged', 'shutdown']


def test_a_stray_signal_with_the_parent_alive_does_nothing():
    """PDEATHSIG fires on the forking THREAD's exit; a live parent must not
    disarm a running vehicle."""
    amn, node, ex, calls = _rig()
    handler = amn._orphan_guard(node, ex, parent_pid=os.getppid())
    handler(10, None)
    assert calls == []


def test_the_kernel_hook_is_set_on_linux():
    import sys
    import mongla_manager.auv_manager_node as amn
    if not sys.platform.startswith('linux'):
        pytest.skip('PR_SET_PDEATHSIG is Linux-only')
    import signal
    assert amn._set_parent_death_signal(signal.SIGUSR1) is True
    amn._set_parent_death_signal(0)            # leave the test process as found
