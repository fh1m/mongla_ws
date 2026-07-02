"""Tests for draw_video HUD overlays -- the bounded primary-target trail and a
render smoke test. The trail replaced supervision's laggy per-ID TraceAnnotator;
its contract is: bounded length, appends the primary centre, and CLEARS on target
loss (so it never smears -- the operator's complaint)."""

import numpy as np
import pytest

pytest.importorskip("supervision")   # render_video_section lazy-imports it

from duburi_vision import draw_video
from duburi_vision.detection.detector import Detection


def _det(cx, cy, w=60, h=60, cls='fire', cid=1, score=0.9):
    return Detection(class_id=cid, class_name=cls, score=score,
                     xyxy=(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))


def _frame():
    return np.zeros((480, 640, 3), dtype=np.uint8)


def test_trail_bounded_and_appends(monkeypatch):
    draw_video._TRAIL.clear()
    out = _frame()
    for i in range(_TRAIL_over := draw_video._TRAIL_LEN + 10):
        draw_video._update_and_draw_trail(out, 'main', (100 + i, 200), sf=1.0)
    dq = draw_video._TRAIL['main']
    assert len(dq) == draw_video._TRAIL_LEN, 'trail must be bounded to _TRAIL_LEN'
    assert dq[-1] == (100 + _TRAIL_over - 1, 200), 'newest point kept at the tail'


def test_trail_clears_on_target_loss():
    # The anti-smear contract: a None point (target lost) drops the whole trail so
    # it never redraws a stale path (the laggy/messy behaviour we replaced).
    draw_video._TRAIL.clear()
    out = _frame()
    for i in range(10):
        draw_video._update_and_draw_trail(out, 'main', (100 + i, 200), sf=1.0)
    assert len(draw_video._TRAIL['main']) == 10
    draw_video._update_and_draw_trail(out, 'main', None, sf=1.0)
    assert len(draw_video._TRAIL['main']) == 0, 'trail must clear on target loss'


def test_trail_keys_are_isolated():
    # Primary and secondary (side-by-side) use different keys -> separate buffers.
    draw_video._TRAIL.clear()
    out = _frame()
    draw_video._update_and_draw_trail(out, 'main', (10, 10), sf=1.0)
    draw_video._update_and_draw_trail(out, 'secondary', (20, 20), sf=1.0)
    assert draw_video._TRAIL['main'][-1] == (10, 10)
    assert draw_video._TRAIL['secondary'][-1] == (20, 20)


def test_render_video_section_runs_and_copies():
    # Smoke: the overlay renders without error, returns a same-shape copy, and does
    # not mutate the input frame (callers rely on the copy).
    frame = _frame()
    dets = [_det(320, 240), _det(400, 300, cls='blood', cid=0)]
    out = draw_video.render_video_section(frame, dets, deadband=0.05)
    assert out is not frame and out.shape == frame.shape
    assert not np.array_equal(out, frame), 'overlays must have been drawn'
    assert np.array_equal(frame, _frame()), 'input frame must be untouched'


def test_render_no_detections_still_ok():
    out = draw_video.render_video_section(_frame(), [], deadband=0.05)
    assert out.shape == (480, 640, 3)


def test_render_draw_trail_false_skips_trail():
    draw_video._TRAIL.clear()
    draw_video.render_video_section(_frame(), [_det(320, 240)],
                                    draw_trail=False, trail_key='main')
    assert 'main' not in draw_video._TRAIL, 'draw_trail=False must not touch the trail'
