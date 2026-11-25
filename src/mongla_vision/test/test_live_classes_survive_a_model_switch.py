"""A live `classes` set survives an `active_model` switch (audit vl_10 F12)."""
from rclpy.parameter import Parameter

from mongla_vision import detector_setup
from mongla_vision.detector_node import DetectorNode


class _Det:
    def __init__(self):
        self.allow = 'launch'

    def update_allowlist(self, a):
        self.allow = a


class _Log:
    def __getattr__(self, _):
        return lambda *a, **k: None


def _node(monkeypatch):
    n = object.__new__(DetectorNode)
    a, b = _Det(), _Det()
    n._registry = {'a': a, 'b': b}
    n._stem_to_key = {}
    n._single_model_name = None
    n._det, n._active_name, n._pending_allowlist = a, 'a', None
    n._extra, n._extra_names = [], []
    n.get_logger = lambda: _Log()
    monkeypatch.setattr(detector_setup, 'changes', lambda node, ps: ps)
    monkeypatch.setattr(DetectorNode, '_live_refusal', lambda self, ps: None)
    for name in ('_publish_classes', '_publish_vision_info', '_publish_model_classes'):
        monkeypatch.setattr(DetectorNode, name, lambda self, *a, **k: None, raising=False)
    return n, b


def test_the_switched_to_model_takes_the_live_filter(monkeypatch):
    n, b = _node(monkeypatch)
    n._on_parameter_change([Parameter('classes', value='gate')])
    n._on_parameter_change([Parameter('active_model', value='b')])
    assert n._det is b and b.allow == ['gate']
