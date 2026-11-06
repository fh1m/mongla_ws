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
