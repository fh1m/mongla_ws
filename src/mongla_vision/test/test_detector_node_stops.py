"""Destroying a detector node stops its worker and closes every model it loaded.

A composed camera that fails to open destroys its detector and keeps the
process. Before `destroy_node` existed the inference worker kept looping and
each model's network group stayed configured on the Hailo-8 until exit
(second-pass audit 2026-11-19, F10). The node is stood up for real; only the
model factory is a stand-in, because CI has no model.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter as P            # noqa: E402


class _Det:
    def __init__(self, **kw):
        self.closed = 0

    def infer(self, frame):
        return []

    def close(self):
        self.closed += 1

    def __getattr__(self, name):                        # setters the node may call
        return lambda *a, **k: None


def test_destroy_stops_the_worker_and_closes_every_model(monkeypatch):
    import mongla_vision.detector_node as dn
    made = []

    def fake(**kw):
        made.append(_Det(**kw))
        return made[-1]
    monkeypatch.setattr(dn, 'make_detector', fake)
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        node = dn.DetectorNode(parameter_overrides=[
            P('models', value='a=one,b=two'), P('active_model', value='a')])
        assert len(made) == 2 and node._worker.is_alive()
        node.destroy_node()
        assert not node._worker.is_alive(), 'the worker outlived its node'
        assert [d.closed for d in made] == [1, 1], 'a loaded model was left configured'
    finally:
        if started:
            rclpy.shutdown()
