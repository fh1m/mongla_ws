"""A distance started with no heading stays in the body frame for the whole run.

`calc_distance` projects body velocity onto the axis taken at `start`. With no
heading then, the axis was an invented 0 deg, and a heading arriving later was
mixed against it, rotating the distance by a number nobody measured.
"""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from mongla_vision.flow.flow_setup import FlowSetupMixin     # noqa: E402

_ROOT = Path(__file__).resolve().parents[1] / 'mongla_vision' / 'flow'


class _Acc:
    def start(self, axis, lateral):
        self.axis, self.lateral = axis, lateral


class _Node(FlowSetupMixin):
    def __init__(self, yaw):
        self._yaw_deg = yaw
        self._acc = _Acc()
        self.warned = []

    def _reset_lk(self):
        pass

    def get_logger(self):
        return SimpleNamespace(warn=self.warned.append, info=lambda *_a: None)


def test_no_heading_at_start_is_the_body_frame_and_says_so():
    n = _Node(None)
    n._on_control(SimpleNamespace(data='start'))
    assert n._acc_heading is False and n._acc.axis == 0.0
    assert n.warned, 'a body-frame distance must be announced'


def test_a_heading_at_start_is_the_axis():
    n = _Node(90.0)
    n._on_control(SimpleNamespace(data='start'))
    assert n._acc_heading is True
    assert n._acc.axis == pytest.approx(math.pi / 2)


def test_the_frame_step_reads_the_heading_only_when_the_start_had_one():
    src = (_ROOT / 'flow_frame.py').read_text()
    seg = src[src.index('if self._acc.active:'):src.index('self._publish_distance(')]
    assert "_acc_heading" in seg
