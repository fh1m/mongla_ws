"""switch_camera refuses a camera whose detector is not running BEFORE
pausing anything, so a typo cannot leave the vehicle with no detector."""
import types

from mongla_vision.utils import switch_camera as sc


def test_a_missing_target_pauses_nothing(monkeypatch):
    calls = []
    monkeypatch.setattr(sc.rclpy, 'init', lambda: None)
    monkeypatch.setattr(sc.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(sc, 'Node', lambda name: types.SimpleNamespace(
        get_node_names=lambda: ['mongla_detector_forward', 'mongla_detector_downward'],
        destroy_node=lambda: None))
    monkeypatch.setattr(sc, '_set_paused',
                        lambda node, det, paused, t: calls.append((det, paused)) or True)
    assert sc.main(['fowrard']) == 2
    assert calls == []
    assert sc.main(['downward']) == 0
    assert calls == [('/mongla_detector_forward', True),
                     ('/mongla_detector_downward', False)]
