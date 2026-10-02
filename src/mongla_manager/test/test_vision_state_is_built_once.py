"""A duplicate VisionState is closed, not leaked (issue #21 item 1)."""
import threading
import types

import pytest

pytest.importorskip('rclpy')


def test_the_loser_of_a_concurrent_first_build_is_closed(monkeypatch):
    import mongla_manager.auv_manager_node as amn
    built, closed = [], []
    gate = threading.Barrier(2)

    class _VS:
        def __init__(self, *a, **k):
            built.append(self)

        def close(self):
            closed.append(self)

    def _ready(vstate, **k):
        gate.wait(2.0)                # both builders are past the first lock

    monkeypatch.setattr(amn, 'VisionState', _VS)
    monkeypatch.setattr(amn, 'wait_vision_state_ready', _ready)
    log = types.SimpleNamespace(info=lambda *a, **k: None,
                                warning=lambda *a, **k: None)
    node = types.SimpleNamespace(_vision_lock=threading.Lock(),
                                 _vision_states={}, get_logger=lambda: log)
    got = []
    ts = [threading.Thread(target=lambda: got.append(
        amn.AUVManagerNode._vision_state_for(node, 'forward'))) for _ in range(2)]
    for t in ts:
        t.start()
    for t in ts:
        t.join(3.0)
    assert len(built) == 2 and got[0] is got[1]
    assert closed == [b for b in built if b is not got[0]]
