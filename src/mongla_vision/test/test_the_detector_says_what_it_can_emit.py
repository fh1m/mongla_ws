"""The detector publishes the classes its running models can emit.

The allowlist topic (`classes_filter`) says what is kept, not what exists, so
nothing could check a course or mission naming a class the model never emits:
robosub26 names red_pipe, bin, torpedo and octagon, and no shipped model emits
them. `model_classes` is the set to check against.
"""
from types import SimpleNamespace as NS

from mongla_vision.detection.detector import Detector
from mongla_vision.detector_publish import DetectorPublishMixin


class _Det(Detector):
    def __init__(self, names):
        self._names = names


def _node(det, extras=()):
    out = []
    n = NS(_det=det, _registry={k: d for k, d in extras}, _extra_names=[k for k, _ in extras],
           _pub_model_classes=NS(publish=lambda m: out.append(m.data)))
    DetectorPublishMixin._publish_model_classes(n)
    return out


def test_the_primary_model_classes_are_published_sorted():
    assert _node(_Det({0: 'shark', 1: 'gate', 2: 'shark'})) == ['gate,shark']


def test_extras_are_included():
    out = _node(_Det({0: 'gate'}), extras=[('bins', _Det({0: 'bin', 1: 'lid'}))])
    assert out == ['bin,gate,lid']


def test_no_model_yet_is_an_empty_set_not_a_guess():
    assert _node(None) == ['']
    assert Detector().class_names() == []


def test_a_model_that_cannot_say_its_names_does_not_take_the_detector_down():
    class _Broken(Detector):
        def class_names(self):
            raise RuntimeError('no table')
    warned = []
    out = []
    n = NS(_det=_Broken(), _registry={'x': _Det({0: 'gate'})}, _extra_names=['x'],
           _pub_model_classes=NS(publish=lambda m: out.append(m.data)),
           get_logger=lambda: NS(warning=warned.append))
    DetectorPublishMixin._publish_model_classes(n)
    assert out == ['gate'] and warned
