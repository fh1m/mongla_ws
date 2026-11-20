"""Ctrl-C stops every node cleanly.

On Jazzy, SIGINT shuts the context down and the executor raises
`ExternalShutdownException` out of `spin()`. The node mains catch only
KeyboardInterrupt, so on Kutum every node of a launch printed a traceback and
exited 1 on a normal stop, and the launch reported each one as died: an
operator could not tell a stop from a crash.
"""
import types

import pytest

pytest.importorskip('rclpy')
from rclpy.executors import ExternalShutdownException   # noqa: E402

from mongla_runtime import spin as S                # noqa: E402


def test_an_external_shutdown_returns_from_spin(monkeypatch):
    shut = []

    class _Ex:
        def add_node(self, n):
            pass

        def spin(self):
            raise ExternalShutdownException()

        def shutdown(self):
            shut.append(True)

    monkeypatch.setattr(S, 'make_executor', lambda *a, **k: (_Ex(), 'events'))
    node = types.SimpleNamespace(get_logger=lambda: types.SimpleNamespace(
        info=lambda *_: None))
    S.spin(node)
    assert shut == [True]
