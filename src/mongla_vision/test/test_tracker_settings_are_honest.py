"""The tracker's settings: what ships is what the ledger says, and a live
change it cannot apply is refused, not accepted and ignored.

Until 2026-11-06 `config/tracker.yaml` said `track_buffer: 150` and the ledger
quoted it, while nothing loaded that file and the node shipped 60 -- 2.0 s of
lost-track life against a 2.418 s p99 bar. And `ros2 param set` on any tracker
setting but `classes` printed success and changed nothing.
"""
import re
from pathlib import Path

import pytest

SRC = (Path(__file__).resolve().parents[1] / 'mongla_vision' / 'tracker_node.py').read_text()
GAP_P99_S = 2.418          # measured-bars.md, the lock ladder


def _declared(name):
    m = re.search(rf"declare_parameter\('{name}',\s*([0-9.]+)\)", SRC)
    assert m, f'{name} is not declared with a literal default in tracker_node.py'
    return float(m.group(1))


def test_track_buffer_covers_the_p99_gap_in_seconds():
    """Roboflow keeps a lost track int(frame_rate/30 * track_buffer) frames,
    which is track_buffer / 30 seconds at any rate."""
    assert _declared('track_buffer') / 30.0 >= GAP_P99_S


def test_there_is_no_preset_file_to_disagree_with_the_node():
    assert not (Path(__file__).resolve().parents[1] / 'config' / 'tracker.yaml').exists()


class _P:
    def __init__(self, name, value):
        self.name, self.value = name, value


def test_a_live_change_the_tracker_cannot_apply_is_refused():
    pytest.importorskip('rclpy')
    from mongla_vision.tracker_node import TrackerNode
    res = TrackerNode._on_param_change(object(), [_P('track_buffer', 90)])
    assert res.successful is False and 'restart' in res.reason


def test_the_shipped_defaults_do_not_warn_and_an_asked_value_above_the_floor_does(monkeypatch):
    """Every launch printed "high_conf_det_threshold 0.60 is above the detector
    floor 0.45 -- clamped": the node's default disagreed with the detector it
    is launched beside, so the warning meant nothing and trained the operator
    to skip warnings (seen on Kutum, 2026-11-19). Unset now means the floor."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.impl.rcutils_logger import RcutilsLogger
    from rclpy.parameter import Parameter as P
    from mongla_vision.tracker_node import TrackerNode
    said = []
    real = RcutilsLogger.warning

    def spy(self, msg, *a, **k):
        said.append(str(msg))
        return real(self, msg, *a, **k)
    monkeypatch.setattr(RcutilsLogger, 'warning', spy)
    monkeypatch.setattr(RcutilsLogger, 'warn', spy, raising=False)
    started = not rclpy.ok()
    if started:
        rclpy.init()
    try:
        for overrides, expect in (({'detector_conf': 0.45}, False),
                                  ({'detector_conf': 0.45, 'high_conf_det_threshold': 0.6}, True)):
            said.clear()
            n = TrackerNode(parameter_overrides=[P(k, value=v) for k, v in overrides.items()])
            n.destroy_node()
            assert any('clamped' in m for m in said) is expect, (overrides, said)
    finally:
        if started:
            rclpy.shutdown()
