"""The in-place letterbox must be the SAME PIXELS as the one it replaced.

`_letterbox_into_bound` writes only the interior of the buffer the chip reads
and repaints the grey bars solely when the source frame size changes. Both
halves of that are silent when wrong: identical-looking output with a stale
border, or a border of the previous frame's image data that the detector is
free to find objects in. No chip is needed to test either -- the method touches
`self._in_buf` and `self._size` and nothing else.
"""
import numpy as np
import pytest

from duburi_vision.detection.hailo import HailoDetector, letterbox

SIZE = 640


class _Buffered:
    """The two attributes `_letterbox_into_bound` actually uses."""
    def __init__(self, size=SIZE):
        self._size = size
        self._in_buf = np.zeros((size, size, 3), np.uint8)
        self._pad_geometry = None

    run = HailoDetector._letterbox_into_bound


def _frame(h, w, seed=0):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, (h, w, 3), dtype=np.uint8)


@pytest.mark.parametrize('h,w', [(1080, 810), (480, 640), (640, 640), (720, 1280)])
def test_same_pixels_as_the_canvas_letterbox(h, w):
    frame = _frame(h, w)
    b = _Buffered()
    scale, px, py = b.run(frame)
    want, s2, px2, py2 = letterbox(frame, SIZE)
    assert (scale, px, py) == pytest.approx((s2, px2, py2))
    assert np.array_equal(b._in_buf, want)


def test_a_frame_of_a_new_shape_repaints_the_bars():
    """Without the repaint the new frame sits inside the OLD frame's image."""
    b = _Buffered()
    b.run(_frame(480, 640, seed=1))          # wide: bars top and bottom
    b.run(_frame(1080, 810, seed=2))         # tall: bars left and right
    want, _, _, _ = letterbox(_frame(1080, 810, seed=2), SIZE)
    assert np.array_equal(b._in_buf, want)


def test_the_bars_are_grey_and_not_black():
    """114 is the value the model was trained and compiled against."""
    b = _Buffered()
    b.run(_frame(1080, 810))
    assert b._in_buf[0, 0].tolist() == [114, 114, 114]
    assert b._in_buf[SIZE - 1, SIZE - 1].tolist() == [114, 114, 114]


def test_repainting_is_skipped_when_the_shape_is_unchanged():
    """The whole point is that the bars are painted once, not every frame."""
    b = _Buffered()
    b.run(_frame(1080, 810, seed=3))
    geom = b._pad_geometry
    # Vandalise a bar. A frame of the same shape must NOT repair it -- if it
    # does, the memset is still happening every frame and the optimisation is
    # not in effect.
    b._in_buf[0, 0] = (7, 7, 7)
    b.run(_frame(1080, 810, seed=4))
    assert b._pad_geometry is geom
    assert b._in_buf[0, 0].tolist() == [7, 7, 7]
