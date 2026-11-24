"""A second model's association-only boxes never steer; max_det 0 is no blackout;
an unread NMS floor is said (audit vl_8 F1, F2, F3)."""
from types import SimpleNamespace

from mongla_vision.detection.detector import Detection
from mongla_vision.detection.hailo import HailoDetector
from mongla_vision.detector_node import DetectorNode


def _d(score, name='gate'):
    return Detection(class_id=0, class_name=name, score=score, xyxy=(1.0, 1.0, 9.0, 9.0))


def test_a_secondary_models_weak_box_goes_to_candidates_only():
    n = object.__new__(DetectorNode)
    n._extra = [SimpleNamespace(infer=lambda f: [_d(0.20), _d(0.80)],
                                acting_conf=lambda: 0.45)]
    n.get_logger = lambda: SimpleNamespace(warning=lambda *a: None)
    strong, weak = n._merge_extra(object(), [])
    assert [d.score for d in strong] == [0.80] and [d.score for d in weak] == [0.20]


def test_max_det_zero_is_clamped_not_a_blackout():
    h = object.__new__(HailoDetector)
    h.update_max_det(0)
    assert h._max_det == 1


def test_an_unread_floor_is_said():
    said = []
    h = object.__new__(HailoDetector)
    h._log = SimpleNamespace(warn=said.append, info=said.append)
    h._floor, h._blocking, h._names, h._allow_ids = None, False, [], None
    h._path = '/nonexistent/model.hef'
    h._warn_conf(0.15)
    assert any('could not be read' in s for s in said)
