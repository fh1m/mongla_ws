"""A ladder fault logged and carried on, but published nothing, so the last
state on /lock (LOCKED included) stood while the lock was dead (audit
2026-11-24). The fault path must publish NOT LOCKED."""
from types import SimpleNamespace

from mongla_vision import lock_node


def test_a_ladder_fault_publishes_not_locked():
    sent = []
    ticks = iter([True, True, False])
    me = SimpleNamespace(
        _stopping=SimpleNamespace(is_set=lambda: not next(ticks)),
        context=SimpleNamespace(ok=lambda: True),
        get_logger=lambda: SimpleNamespace(error=lambda *_: None),
        _header=object(),
        _publish=lambda st, h: sent.append(st),
    )

    def boom():
        raise RuntimeError('onnx hiccup')
    me._ladder_tick = boom
    orig = lock_node.rclpy.ok
    lock_node.rclpy.ok = lambda: True
    try:
        lock_node.LockNode._loop(me)
    finally:
        lock_node.rclpy.ok = orig
    assert sent and not sent[0].have_target
