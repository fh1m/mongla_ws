"""Ctrl-C during a mission is an OPERATOR ABORT, never a failed step.

Two defects, both of which ended with a mission that kept driving after the
operator hit Ctrl-C:

1. `MonglaClient.send()` turned KeyboardInterrupt into `MoveFailed` -- an
   ordinary Exception -- and `retry` / `selector` / `never_fails` / the vision
   verbs all contain Exception BY DESIGN. Ctrl-C skipped one step.
2. rclpy's default SIGINT handler shuts the context down, so even when the
   interrupt did reach `_abort_sequence`, its cancel/stop/disarm could not be
   published (`publisher's context is invalid`, measured on Jazzy).
"""

import os
import signal
import subprocess
import sys
import textwrap
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from mongla_planner import cli, mission
from mongla_planner.client import MonglaClient
from mongla_planner.resilience import never_fails, retry, selector


class _Future:
    def __init__(self, done=True, result=None):
        self._done, self._result = done, result

    def done(self):
        return self._done

    def result(self):
        return self._result


def _client_whose_wait_is_interrupted(monkeypatch):
    """A client whose result-wait raises KeyboardInterrupt (the operator's
    Ctrl-C), and whose cancel completes."""
    monkeypatch.setattr('mongla_planner.client.ActionClient',
                        lambda *a, **k: MagicMock())
    result_future = _Future(done=True, result=MagicMock())
    interrupted = {'once': False}

    def spin(node, future, timeout_sec=None):
        # Ctrl-C lands during the RESULT wait -- the long one.
        if future is result_future and not interrupted['once']:
            interrupted['once'] = True
            raise KeyboardInterrupt
    monkeypatch.setattr('mongla_planner.client.rclpy.spin_until_future_complete',
                        spin)
    c = MonglaClient(MagicMock())
    gh = MagicMock(); gh.accepted = True
    gh.get_result_async.return_value = result_future
    gh.cancel_goal_async.return_value = _Future(done=True)
    c._client.send_goal_async.return_value = _Future(done=True, result=gh)
    return c, gh


def test_send_reraises_ctrl_c_after_cancelling(monkeypatch):
    c, gh = _client_whose_wait_is_interrupted(monkeypatch)
    with pytest.raises(KeyboardInterrupt):
        c.send('move_forward', duration=5)
    gh.cancel_goal_async.assert_called()     # the goal is cancelled first
    assert c._active_goal_handle is None


@pytest.mark.parametrize('wrap', [
    lambda f: retry(f, times=3),
    lambda f: never_fails(f),
    lambda f: selector([('precise', f), ('blind', lambda: True)]),
])
def test_ctrl_c_escapes_every_containment_wrapper(monkeypatch, wrap):
    """The wrappers contain FAILURES. An abort is not one."""
    c, _ = _client_whose_wait_is_interrupted(monkeypatch)
    with pytest.raises(KeyboardInterrupt):
        wrap(lambda: c.send('move_forward', duration=5))


@pytest.mark.parametrize('module', [mission, cli])
def test_entry_points_keep_the_context_alive_on_ctrl_c(monkeypatch, module):
    """The runner and the CLI must not let rclpy shut the context on SIGINT."""
    seen = {}

    class _Stop(Exception):
        pass

    def fake_init(*a, **k):
        seen.update(k)
        raise _Stop
    monkeypatch.setattr(module.rclpy, 'init', fake_init)
    monkeypatch.setattr(sys, 'argv', ['x', 'stop'])
    with pytest.raises(_Stop):
        if module is mission:
            name = sorted(mission.discover())[0]
            module.main([name])
        else:
            module.main()
    from rclpy.signals import SignalHandlerOptions
    assert seen.get('signal_handler_options') == SignalHandlerOptions.NO


def test_why_the_default_handler_is_refused():
    """The measurement behind the fix, re-run: under rclpy's default handler a
    publish after Ctrl-C fails; with NO handler it succeeds."""
    script = textwrap.dedent('''
        import os, signal, sys, threading, rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        opt = getattr(SignalHandlerOptions, sys.argv[1])
        rclpy.init(signal_handler_options=opt)
        n = rclpy.create_node('ctrl_c_probe')
        p = n.create_publisher(String, 'ctrl_c_probe', 1)
        threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
        try:
            rclpy.spin_until_future_complete(n, rclpy.task.Future(), timeout_sec=5)
            print('NO_INTERRUPT')
        except KeyboardInterrupt:
            try:
                p.publish(String(data='x')); print('PUBLISH_OK')
            except Exception:
                print('PUBLISH_FAILS')
    ''')

    def run(opt):
        out = subprocess.run([sys.executable, '-c', script, opt],
                             capture_output=True, text=True, timeout=30,
                             env=dict(os.environ))
        return out.stdout.strip().splitlines()[-1] if out.stdout.strip() else out.stderr

    assert run('NO') == 'PUBLISH_OK'
    assert run('ALL') == 'PUBLISH_FAILS'
