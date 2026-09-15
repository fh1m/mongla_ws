"""The flare order through the camera: decoded only when framed and repeated."""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from duburi_vision.optical_order import OrderDecoder, classify_frame  # noqa: E402

BGR = {'R': (40, 40, 255), 'B': (255, 60, 30), 'Y': (30, 230, 255)}


def _frame(sym, distractor=False):
    img = np.full((240, 320, 3), (70, 90, 60), np.uint8)       # murky pool
    if distractor:
        img[40:200, 20:40] = (30, 30, 150)                        # a red prop, not a light
    if sym != '.':
        cv2.circle(img, (200, 120), 12, BGR[sym], -1)
    return img


def _schedule(order, repeats, fps=30.0, on=0.6, off=0.4, gap=2.0, lead=2.0):
    seq = [('.', lead)]
    for _ in range(repeats):
        for s in order:
            seq += [(s, on), ('.', off)]
        seq += [('.', gap)]
    out, t = [], 0.0
    for sym, dur in seq:
        for _ in range(int(round(dur * fps))):
            out.append((t, sym))
            t += 1.0 / fps
    return out


def _run(frames):
    d = OrderDecoder()
    for t, s in frames:
        d.feed(t, s)
    return d.order


def test_frames_classify_by_the_light_not_the_prop():
    assert classify_frame(_frame('.', distractor=True)) == '.'
    for s in 'RBY':
        assert classify_frame(_frame(s, distractor=True)) == s


def test_the_order_is_decoded_after_two_identical_framed_messages():
    assert _run(_schedule(('R', 'B', 'Y'), repeats=2)) == ('R', 'B', 'Y')
    assert _run(_schedule(('Y', 'R', 'B'), repeats=2)) == ('Y', 'R', 'B')


def test_one_message_is_not_enough():
    assert _run(_schedule(('R', 'B', 'Y'), repeats=1)) is None


def test_a_misread_flash_does_not_decode_a_wrong_order_and_repeats_recover():
    frames = _schedule(('R', 'B', 'Y'), repeats=2)
    # the 2nd message's second colour (B) is misread as R for its whole flash
    start = int(round((2.0 + 3 * 1.0 + 2.0 + 1.0) * 30))
    frames = [(t, 'R' if start <= i < start + 18 else s) for i, (t, s) in enumerate(frames)]
    assert _run(frames) is None
    more = _schedule(('R', 'B', 'Y'), repeats=2, lead=0.0)
    t0 = frames[-1][0] + 1 / 30
    assert _run(frames + [(t0 + t, s) for t, s in more]) == ('R', 'B', 'Y')


def test_dropped_frames_at_low_fps_still_decode():
    frames = _schedule(('B', 'Y', 'R'), repeats=2, fps=10.0)
    rng = np.random.default_rng(0)
    kept = [f for f in frames if rng.random() > 0.15]
    assert _run(kept) == ('B', 'Y', 'R')


def test_a_steady_colour_is_not_a_flash():
    """A red prop held in view for 3 s where the first flash should be must not
    stand in for 'R', however well the rest of the message frames it."""
    seq, t = [], 0.0
    plan = [('.', 2.0)] + [('R', 3.0), ('.', 0.4), ('B', 0.6), ('.', 0.4), ('Y', 0.6), ('.', 2.0)] * 2
    for sym, dur in plan:
        for _ in range(int(round(dur * 30))):
            seq.append((t, sym)); t += 1 / 30
    assert _run(seq) is None


def test_end_to_end_through_the_classifier():
    d = OrderDecoder()
    for t, s in _schedule(('Y', 'B', 'R'), repeats=2, fps=15.0):
        d.feed(t, classify_frame(_frame(s, distractor=True)))
    assert d.order == ('Y', 'B', 'R')
