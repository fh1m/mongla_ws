"""Which model produced the box you are holding?

A mission switches the detector mid-run -- gate, then rescue, then red_pipe --
and a consumer acting on a box a PREVIOUS model produced is steering at the
wrong thing while every node looks healthy. Nothing said which model any
message came from; the console polls a PARAMETER at 1 Hz, which cannot answer
that about a message.

`vision_msgs/VisionInfo` is the standard carrier, latched, with
`database_version` bumped on every switch so a consumer comparing two reads
KNOWS a switch happened rather than guessing.
"""
import types
from pathlib import Path

import pytest

_SRC = (Path(__file__).resolve().parents[1] / 'duburi_vision'
        / 'detector_node.py')


def _stub(active='gate_rescue_repair'):
    """The pieces `_publish_vision_info` actually touches, and nothing else.

    Constructing a real DetectorNode needs a model on disk; the weights are
    gitignored. The method under test is small and its dependencies are
    explicit, so they are supplied directly -- the live end of this is checked
    on the vehicle, where the models exist.
    """
    from rclpy.time import Time
    from duburi_vision.detector_node import DetectorNode
    sent = []
    obj = types.SimpleNamespace(
        _active_name=active, _single_model_name=None, _model_epoch=0,
        _pub_vinfo=types.SimpleNamespace(publish=sent.append),
        get_clock=lambda: types.SimpleNamespace(now=Time))
    obj.publish = types.MethodType(DetectorNode._publish_vision_info, obj)
    return obj, sent


def test_the_model_name_is_published_and_is_the_STEM():
    pytest.importorskip('rclpy')
    obj, sent = _stub()
    obj.publish()
    assert len(sent) == 1
    assert sent[0].database_location == 'gate_rescue_repair', (
        'the model identity here must be the stem this stack uses everywhere '
        'else -- set_model, ClassRef and the .engine sidecar all key on it')


def test_every_switch_bumps_the_version():
    """A consumer compares two reads. Equal versions must mean no switch."""
    pytest.importorskip('rclpy')
    obj, sent = _stub()
    for _ in range(3):
        obj.publish()
    versions = [m.database_version for m in sent]
    assert versions == sorted(set(versions)) and len(set(versions)) == 3, (
        f'versions {versions} -- a repeated or unordered version makes a '
        f'switch invisible to the comparison this field exists for')


def test_it_is_latched_and_published_at_startup_and_on_every_switch():
    """A late joiner gets the model; a switch republishes it.

    Wiring, read off the source: the numeric behaviour above is driven, and the
    end-to-end publish is verified on the vehicle where the models exist.
    """
    src = _SRC.read_text()
    assert "VisionInfo, f'{ns_out}/vision_info', qos.LATCHED" in src, (
        'vision_info is not latched -- a consumer joining after the detector '
        'would never learn the model')
    assert src.count('self._publish_vision_info()') >= 2, (
        'vision_info must be published at startup AND on every model switch')
    i = src.index("elif p.name == 'active_model':")
    j = src.index("elif p.name == 'paused':", i)
    assert '_publish_vision_info()' in src[i:j], (
        'a live model switch does not republish the model identity')
