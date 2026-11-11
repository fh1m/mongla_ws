"""The detector node's NMS parameters, set during a run.

`nms_floor` and `nms_iou` reach every loaded model or the parameter is
rejected with the reason; a live `iou` is rejected, because nothing reads it
after a model is built and accepting it reported a change nothing made.
"""
import types

import pytest

pytest.importorskip('rcl_interfaces')
from rclpy.parameter import Parameter                       # noqa: E402

from mongla_vision.detector_setup import DetectorSetupMixin  # noqa: E402


class _Det:
    def __init__(self, refuse=''):
        self.calls, self._refuse = [], refuse

    def update_nms(self, floor=None, iou=None):
        if self._refuse:
            raise ValueError(self._refuse)
        self.calls.append((floor, iou))


def _node(registry=None, det=None, params=None):
    log = types.SimpleNamespace(info=lambda m: None, error=lambda m: None,
                                warning=lambda m: None)
    values = {'nms_floor': 0.05, 'nms_iou': 0.0, 'assoc_conf': 0.0}
    values.update(params or {})
    node = types.SimpleNamespace(
        _registry=registry or {}, _det=det, get_logger=lambda: log,
        get_parameter=lambda n: types.SimpleNamespace(value=values[n]))
    for name in ('_on_parameter_change', '_set_nms_live', '_apply_nms',
                 '_apply_assoc_conf'):
        setattr(node, name, types.MethodType(
            getattr(DetectorSetupMixin, name), node))
    return node


def _set(node, name, value):
    return node._on_parameter_change(
        [Parameter(name, Parameter.Type.DOUBLE, value)])


def test_a_live_floor_reaches_every_loaded_model():
    a, b = _Det(), _Det()
    result = _set(_node(registry={'gate': a, 'bin': b}), 'nms_floor', 0.02)
    assert result.successful
    assert a.calls == [(0.02, None)] and b.calls == [(0.02, None)]


def test_a_live_overlap_threshold_reaches_the_model():
    det = _Det()
    assert _set(_node(det=det), 'nms_iou', 0.5).successful
    assert det.calls == [(None, 0.5)]


def test_a_refusal_rejects_the_parameter_with_its_reason():
    result = _set(_node(det=_Det(refuse='no NMS output to set')),
                  'nms_floor', 0.02)
    assert not result.successful and 'no NMS output' in result.reason


def test_no_model_that_can_take_it_rejects_the_parameter():
    result = _set(_node(det=object()), 'nms_iou', 0.5)
    assert not result.successful and 'no loaded model' in result.reason


def test_a_live_iou_is_rejected_and_names_the_parameter_that_works():
    det = _Det()
    result = _set(_node(det=det), 'iou', 0.5)
    assert not result.successful and 'nms_iou' in result.reason
    assert det.calls == []


def test_as_compiled_is_a_launch_value_not_a_live_one():
    det = _Det()
    assert not _set(_node(det=det), 'nms_iou', 0.0).successful
    assert det.calls == []


def test_the_launch_values_reach_a_model_built_later():
    det = _Det()
    _node(params={'nms_floor': 0.02, 'nms_iou': 0.6})._apply_assoc_conf(det)
    assert det.calls == [(0.02, 0.6)]


def test_the_defaults_leave_a_new_model_as_it_was_built():
    det = _Det()
    _node()._apply_assoc_conf(det)
    assert det.calls == []
