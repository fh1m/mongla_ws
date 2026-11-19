"""The operator's live panels print an absent number as '--'.

`MonglaState` carries NaN for a depth the board did not send, and flow
refuses a height it cannot measure. The overlay printed them as '+nan m',
where the strip in `draw_strip` already prints '--': one rule, both views.
"""
import types

import numpy as np

from mongla_vision.utils import display_draw


def _texts(monkeypatch, fn, *a, **k):
    seen = []
    monkeypatch.setattr(display_draw, 'pil_text',
                        lambda img, t, *x, **y: seen.append(t))
    fn(*a, **k)
    return seen


def test_an_absent_depth_is_dashes(monkeypatch):
    st = types.SimpleNamespace(depth_m=float('nan'), armed=False)
    seen = _texts(monkeypatch, display_draw._draw_mission_panel,
                  np.zeros((480, 640, 3), np.uint8), camera='forward', fps=30.0,
                  primary=None, native_w=640, native_h=480, deadband=0.05,
                  state=st)
    assert not any('nan' in t for t in seen)
    assert any(t.startswith('-- m') for t in seen)


def test_an_absent_flow_height_is_dashes(monkeypatch):
    seen = _texts(monkeypatch, display_draw._draw_distance_panel,
                  np.zeros((480, 640, 3), np.uint8),
                  [1.25, float('nan'), 8, 1.0])
    assert not any('nan' in t for t in seen)
    assert 'height -- m' in seen
