"""`image_msg` is the stack's only way between an Image message and an array.

It replaced `cv_bridge`, whose C extension tied every vision node to the
NumPy it was compiled against (measured-bars 77, 78). These tests state what
the replacement must do with no reference to the thing it replaced, and then
compare the two wherever `cv_bridge` still imports.
"""
from __future__ import annotations

import array
import re
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('sensor_msgs')
from sensor_msgs.msg import Image                              # noqa: E402

from mongla_vision import image_msg as im                      # noqa: E402

SRC = Path(__file__).resolve().parents[2]
RNG = np.random.default_rng(7)
BGR = RNG.integers(0, 255, (36, 64, 3), dtype=np.uint8)
MONO = BGR[:, :, 1].copy()
F32 = RNG.random((36, 64), dtype=np.float32)


@pytest.mark.parametrize('arr, enc', [(BGR, 'bgr8'), (BGR, 'rgb8'),
                                      (MONO, 'mono8'), (F32, '32FC1')])
def test_an_array_survives_the_round_trip(arr, enc):
    msg = im.to_msg(arr, enc)
    assert (msg.height, msg.width, msg.encoding) == (36, 64, enc)
    assert msg.step * msg.height == len(msg.data)
    back = im.to_array(msg, enc)
    assert back.dtype == arr.dtype and back.shape == arr.shape
    assert (back == arr).all()


def test_channel_order_is_converted_not_relabelled():
    """A known pixel: blue in BGR must read as blue in RGB."""
    px = np.zeros((2, 2, 3), np.uint8)
    px[0, 0] = (255, 10, 20)                       # B, G, R
    rgb = im.to_array(im.to_msg(px, 'bgr8'), 'rgb8')
    assert tuple(rgb[0, 0]) == (20, 10, 255)
    assert tuple(im.to_array(im.to_msg(px, 'rgb8'), 'bgr8')[0, 0]) == (20, 10, 255)


def test_gray_becomes_three_equal_channels_and_back():
    bgr = im.to_array(im.to_msg(MONO, 'mono8'), 'bgr8')
    assert bgr.shape == (36, 64, 3) and (bgr[:, :, 0] == MONO).all()
    assert im.to_array(im.to_msg(BGR, 'bgr8'), 'mono8').shape == (36, 64)


def test_reading_in_the_messages_own_encoding_copies_nothing():
    msg = im.to_msg(BGR, 'bgr8')
    view = im.to_array(msg, 'bgr8')
    assert np.shares_memory(view, np.frombuffer(msg.data, np.uint8))


def test_filling_a_message_does_not_alias_the_callers_array():
    src = BGR.copy()
    msg = im.to_msg(src, 'bgr8')
    src[:] = 0
    assert (im.to_array(msg) == BGR).all()


def test_the_message_holds_a_byte_array_the_middleware_takes_as_is():
    assert isinstance(im.to_msg(BGR, 'bgr8').data, array.array)
    assert im.to_msg(BGR, 'bgr8').data.typecode == 'B'


def test_row_padding_is_dropped():
    msg = Image(height=2, width=3, encoding='mono8', step=5)
    msg.data = array.array('B', [1, 2, 3, 99, 99, 4, 5, 6, 99, 99])
    assert im.to_array(msg).tolist() == [[1, 2, 3], [4, 5, 6]]


def test_a_big_endian_float_image_reads_as_the_same_numbers():
    msg = Image(height=1, width=2, encoding='32FC1', step=8, is_bigendian=1)
    msg.data = array.array('B', np.array([1.5, -2.25], '>f4').tobytes())
    assert im.to_array(msg).tolist() == [[1.5, -2.25]]


def test_a_non_contiguous_array_is_sent_whole():
    msg = im.to_msg(BGR[:, ::2], 'bgr8')
    assert (im.to_array(msg) == BGR[:, ::2]).all()


@pytest.mark.parametrize('bad', ['bgra8', 'yuv422', '16UC1', ''])
def test_an_encoding_the_stack_does_not_use_is_refused(bad):
    with pytest.raises(ValueError):
        im.to_msg(BGR, bad)
    msg = im.to_msg(BGR, 'bgr8')
    msg.encoding = bad
    with pytest.raises(ValueError):
        im.to_array(msg)


def test_an_array_that_is_not_that_encoding_is_refused():
    with pytest.raises(ValueError):
        im.to_msg(MONO, 'bgr8')                    # one channel, three claimed
    with pytest.raises(ValueError):
        im.to_msg(BGR.astype(np.float32), 'bgr8')  # wrong dtype
    with pytest.raises(ValueError):
        im.to_array(im.to_msg(F32, '32FC1'), 'bgr8')   # no such conversion


def test_a_truncated_message_is_refused():
    msg = im.to_msg(BGR, 'bgr8')
    msg.data = msg.data[:100]
    with pytest.raises(ValueError):
        im.to_array(msg)


_AGAINST_CV_BRIDGE = r"""
import sys
import numpy as np
from cv_bridge import CvBridge
from mongla_vision import image_msg as im
rng = np.random.default_rng(7)
bgr = rng.integers(0, 255, (36, 64, 3), dtype=np.uint8)
mono = bgr[:, :, 1].copy()
f32 = rng.random((36, 64), dtype=np.float32)
b = CvBridge()
for arr, enc in ((bgr, 'bgr8'), (bgr, 'rgb8'), (mono, 'mono8'), (f32, '32FC1')):
    theirs, ours = b.cv2_to_imgmsg(arr, encoding=enc), im.to_msg(arr, enc)
    assert bytes(theirs.data) == bytes(ours.data) and theirs.step == ours.step, enc
    for want in ((enc,) if enc == '32FC1' else ('bgr8', 'rgb8', 'mono8')):
        assert (b.imgmsg_to_cv2(theirs, want) == im.to_array(ours, want)).all(), (enc, want)
print('AGREES')
"""


def test_it_agrees_with_cv_bridge_where_cv_bridge_still_works():
    """In a child process: beside a NumPy it was not built for, `cv_bridge`
    does not raise, it takes the interpreter down."""
    import subprocess
    import sys
    r = subprocess.run([sys.executable, '-c', _AGAINST_CV_BRIDGE],
                       capture_output=True, text=True, timeout=120)
    if 'AGREES' in r.stdout:
        return
    if 'AssertionError' in r.stderr:
        pytest.fail(f'image_msg and cv_bridge disagree: {r.stderr[-300:]}')
    pytest.skip(f'cv_bridge does not work on this machine (exit {r.returncode})')


def test_nothing_in_the_stack_imports_cv_bridge():
    """One import brings back the NumPy coupling for the whole process."""
    users = []
    for path in list(SRC.glob('*/*/**/*.py')) + list((SRC.parent / 'tools').rglob('*.py')):
        if '/test/' in path.as_posix():
            continue
        if re.search(r'^\s*(from|import)\s+cv_bridge\b', path.read_text(), re.M):
            users.append(path.relative_to(SRC.parent).as_posix())
    assert not users, f'{users}: use mongla_vision.image_msg'
    for xml in SRC.glob('*/package.xml'):
        assert 'cv_bridge' not in xml.read_text(), f'{xml} still depends on cv_bridge'
