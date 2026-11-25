"""Defects found reading the 19 files no audit shard was assigned (2026-11-24)."""
import ast
import re
import math
import pathlib

import numpy as np
import pytest

from mongla_vision import continuity as C
from mongla_vision.detection import hef_file, seg_decode as S

SRC = pathlib.Path(__file__).resolve().parents[2]


def _proto_left_half():
    """160x160 prototype whose first channel is high where network x < 60."""
    proto = np.zeros((160, 160, S.MASK_DIM), np.uint8)
    proto[:, :15, 0] = 255                  # proto col 15 = network x 60
    return proto


def test_a_mask_past_the_canvas_edge_stays_where_the_object_is():
    # Box from network x=-40 to 120: the object (x < 60) fills box cols 40..100.
    coeff = np.zeros((1, S.MASK_DIM), np.float32)
    coeff[0, 0] = 1.0
    q = S.Quant(zp=128.0, scale=1.0)        # threshold: u8 >= 128
    box = np.array([[-40.0, 100.0, 120.0, 200.0]], np.float32)
    m = S.masks_for(coeff, _proto_left_half(), q, box, size=640)[0]
    assert m.shape == (100, 160)
    assert m[50, 20] == 0                   # x = -20: off the canvas, nothing
    assert m[50, 90] == 1                   # x = 50: on the object
    assert m[50, 110] == 0                  # x = 70: past it


def test_an_interior_mask_edge_is_where_the_prototype_puts_it():
    coeff = np.zeros((1, S.MASK_DIM), np.float32)
    coeff[0, 0] = 1.0
    q = S.Quant(zp=128.0, scale=1.0)
    box = np.array([[10.0, 10.0, 110.0, 110.0]], np.float32)
    m = S.masks_for(coeff, _proto_left_half(), q, box, size=640)[0]
    edge = int(np.argmin(m[50]))            # first column that is off
    assert abs((10 + edge) - 60) <= 2       # network x of the edge


def test_the_hud_draws_no_target_as_absent_not_aligned():
    from mongla_vision import draw_widgets as W
    img = np.zeros((40, 200, 3), np.uint8)
    W.needle_gauge(img, 10, 10, 180, 20, float('nan'))   # raised before
    W.confidence_bar(img, 10, 10, 180, 20, float('nan'))
    W.sparkline(img, 10, 10, 180, 20, [0.1, float('nan'), 0.3])
    assert not (img == W.C_OK).all(axis=2).any(), 'a green "aligned" fill for no target'
    strip = (SRC / 'mongla_vision/mongla_vision/draw_strip.py').read_text()
    assert "ex = ey = float('nan')" in strip


def test_an_unknown_video_length_is_not_shown_as_finished():
    from mongla_vision import draw_widgets as W
    shown = []
    real = W.pil_text
    W.pil_text = lambda img, text, *a, **k: shown.append(text)
    try:
        W.video_progress(np.zeros((20, 400, 3), np.uint8), 0, 0, 400, 20, 120, 0, 30.0)
    finally:
        W.pil_text = real
    assert not any('100%' in s for s in shown), shown


def test_the_absence_before_the_first_sighting_is_not_a_reacquisition():
    obs = [C.Obs(t=i * 0.1, seen=i >= 5) for i in range(10)]
    assert C.reacquire_times(obs) == []
    assert C.find_gaps(obs) == []


def _const(path, name):
    tree = ast.parse((SRC / path).read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f'{name} not in {path}')


def test_continuity_ladder_matches_the_code():
    ladder = dict((name.split(' (')[0], v) for name, v in C.LADDER)
    ctl = 'mongla_control/mongla_control/'
    assert ladder['freshness zero'] == _const(ctl + 'motion_vision/freshness.py', 'VISION_FRESH_ZERO_S')
    cmds = (SRC / (ctl + 'commands.py')).read_text()
    for key in ('coast_s', 'lost_grace_s'):
        found = [float(v) for v in re.findall(rf"'{key}': ([0-9.]+)", cmds)]
        assert found and all(v == ladder[key] for v in found), (key, found)
    kalman = (SRC / 'mongla_vision/mongla_vision/tracking/kalman.py').read_text()
    assert f"max_predict_s: float = {ladder['kalman predict']}" in kalman
    node = (SRC / 'mongla_vision/mongla_vision/tracker_node.py').read_text()
    assert f"'track_buffer',                {int(ladder['track_buffer'] * 30)})" in node


def test_a_malformed_sidecar_says_so(tmp_path):
    (tmp_path / 'm.yaml').write_text('names: [unclosed\n')
    with pytest.raises(ValueError, match='unreadable'):
        hef_file.load_class_index(str(tmp_path / 'm.hef'))
    assert hef_file.load_class_index(str(tmp_path / 'absent.hef')) is None


def test_one_sidecar_reader():
    yolo = (SRC / 'mongla_vision/mongla_vision/detection/yolo.py').read_text()
    assert 'def _load_class_index' not in yolo
    assert math.isfinite(1.0)
