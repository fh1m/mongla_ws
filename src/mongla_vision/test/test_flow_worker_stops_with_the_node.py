"""The flow worker ends quietly when the node's context has shut down.

At Ctrl-C the camera process's flow thread was mid-frame; its publish failed
on the closed context and it logged `ERROR [FLOW ] worker: RCLError(...)`
for every frame still in flight, which reads as a fault on a clean stop.
"""
import threading
import types

import numpy as np
import pytest

pytest.importorskip('rclpy')
from mongla_vision.flow.flow_node import FlowVelocityNode   # noqa: E402


def test_a_failure_after_shutdown_ends_the_loop_without_an_error():
    errors = []
    n = types.SimpleNamespace(
        _stop=threading.Event(), _slot_evt=threading.Event(),
        _slot_lock=threading.Lock(), _slot=(np.zeros((8, 8), np.uint8), 1.0, 1),
        _admit_frame_size=lambda wh: True,
        context=types.SimpleNamespace(ok=lambda: False),
        get_logger=lambda: types.SimpleNamespace(error=errors.append))
    n._slot_evt.set()

    def boom(*a):
        raise RuntimeError("Failed to publish: publisher's context is invalid")
    n._process = boom
    t = threading.Thread(target=FlowVelocityNode._flow_loop, args=(n,), daemon=True)
    t.start()
    t.join(1.0)
    assert not t.is_alive()
    assert errors == []


def test_the_lock_ladder_ends_without_a_fault_after_shutdown(monkeypatch):
    """Same race in the lock ladder: rclpy.ok() passed, then the context
    closed mid-tick and the publish failed, logged as `ladder fault #1` on
    every stop of the vision graph on Kutum."""
    from mongla_vision import lock_node
    monkeypatch.setattr(lock_node.rclpy, 'ok', lambda: True)
    errors = []

    def boom():
        raise RuntimeError("Failed to publish: publisher's context is invalid")
    n = types.SimpleNamespace(
        _stopping=threading.Event(), _ladder_tick=boom,
        context=types.SimpleNamespace(ok=lambda: False),
        get_logger=lambda: types.SimpleNamespace(error=errors.append))
    t = threading.Thread(target=lock_node.LockNode._loop, args=(n,), daemon=True)
    t.start()
    t.join(1.0)
    assert not t.is_alive()
    assert errors == []
