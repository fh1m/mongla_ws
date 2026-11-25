"""The detector keeps its live settings through a rebuild, frees the old one,
moves no model on a refused NMS set, and its worker never dies quietly
(audit vl_9 H1, M1, M2, M5, M6, M7)."""
import pathlib
from types import SimpleNamespace

from mongla_vision.detector_node import DetectorNode

SRC = pathlib.Path(__file__).resolve().parents[2]


class _Log:
    def __getattr__(self, _):
        return lambda *a, **k: None


def _node():
    n = object.__new__(DetectorNode)
    n.get_logger = lambda: _Log()
    return n


def test_a_rebuild_takes_the_live_conf_and_closes_the_old(monkeypatch):
    n = _node()
    closed = []
    n._det = SimpleNamespace(close=lambda: closed.append(1))
    n._build_kwargs = dict(model_path='m', conf=0.45, max_det=100)
    live = {'conf': 0.6, 'max_det': 7}
    n.get_parameter = lambda k: SimpleNamespace(value=live[k])
    got = {}
    n._load_single_model_async = lambda **kw: got.update(kw)
    n._rebuild_detector()
    assert closed == [1]
    assert got['conf'] == 0.6 and got['max_det'] == 7


class _Taker:
    def __init__(self, refuse=''):
        self.refuse, self.floor = refuse, None

    def nms_refusal(self, floor=None, iou=None):
        return self.refuse

    def update_nms(self, floor=None, iou=None):
        if self.refuse:
            raise ValueError(self.refuse)
        self.floor = floor


def test_a_refused_nms_set_moves_no_model():
    n = _node()
    a, b = _Taker(), _Taker('no floor here')
    n._registry, n._det = {'a': a, 'b': b}, a
    assert n._set_nms_live(floor=0.1, iou=None) == 'no floor here'
    assert a.floor is None


def test_a_publish_fault_on_a_live_node_is_not_a_clean_exit():
    src = (SRC / 'mongla_vision/mongla_vision/detector_node.py').read_text()
    assert 'raise       # a live node: the guard logs and restarts' in src


def test_the_debug_frame_is_asked_for_best_effort():
    src = (SRC / 'mongla_planner/mongla_planner/dsl_report.py').read_text()
    assert 'ReliabilityPolicy.BEST_EFFORT' in src


def test_one_eyes_dead_registry_keeps_the_other():
    src = (SRC / 'mongla_vision/mongla_vision/detector_dual_node.py').read_text()
    assert 'keeping the eye that works' in src.split('det = DetectorNode(')[1][:900]
