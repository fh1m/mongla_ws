"""A blinded camera must not read as an empty scene -- and ordinary footage
must never read as blind. The second half runs on the REAL archive when it is
present (dev box), and skips elsewhere rather than passing on sim frames."""
import glob
import os

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from duburi_vision.seeing import (  # noqa: E402
    COVERED, FROZEN, FROZEN_FRAMES, GLARE, HOLD_FRAMES, OK, WASHOUT, Seeing,
    classify,
)

RNG = np.random.default_rng(0)
# Absolute: inside the distrobox $HOME is not /home/fh1m, and expanduser
# silently pointed nowhere, so this control SKIPPED and looked like a pass.
ARCHIVE = '/home/fh1m/Work/Projects/Duburi/2025/raw_videos'


def scene(noise=True):
    """A textured pool-like frame: gradient + blobs + sensor noise."""
    y, x = np.mgrid[0:480, 0:640].astype(np.float32)
    im = 90 + 60 * np.sin(x / 37.0) * np.cos(y / 53.0) + 0.1 * x
    if noise:
        im = im + RNG.normal(0, 6, im.shape)
    return np.clip(im, 0, 255).astype(np.uint8)


def _jitter(im):
    """Break exact equality without uint8 WRAP: 255 + 1 is 0, which turned a
    glare frame dark in the first draft of these tests."""
    return np.clip(im.astype(np.int16) - RNG.integers(0, 2, im.shape), 0, 255).astype(np.uint8)


def feed(s, frame, n):
    for _ in range(n):
        s.observe(frame if frame is not None else scene())
    return s.state


def test_a_textured_scene_is_ok():
    assert feed(Seeing(), None, 10) == OK


def test_a_covered_lens_is_COVERED():
    s = Seeing()
    feed(s, None, 5)
    cap = (scene().astype(np.float32) * 0.05 + 5).astype(np.uint8)
    cap = cap + RNG.integers(0, 2, cap.shape, dtype=np.uint8)   # not frozen
    assert classify(cv2.resize(cap, (160, 120))) == COVERED


def test_a_heavy_veil_is_WASHOUT():
    veil = (scene().astype(np.float32) * 0.2 + 200 * 0.8).astype(np.uint8)
    assert classify(cv2.resize(veil, (160, 120))) == WASHOUT


def test_sun_glare_is_GLARE():
    glare = np.clip(scene().astype(np.int32) + 160, 0, 255).astype(np.uint8)
    assert classify(cv2.resize(glare, (160, 120))) == GLARE


def test_a_frozen_driver_is_FROZEN_after_the_run():
    s = Seeing()
    f = scene()
    for i in range(FROZEN_FRAMES - 1):
        s.observe(f)
        assert s.state != FROZEN, i
    s.observe(f)
    assert s.state == FROZEN


def test_one_bad_frame_does_not_flip_the_state():
    s = Seeing()
    feed(s, None, 5)
    glare = np.clip(scene().astype(np.int32) + 160, 0, 255).astype(np.uint8)
    s.observe(glare)
    assert s.state == OK
    for _ in range(HOLD_FRAMES - 1):
        s.observe(_jitter(glare))
    assert s.state == GLARE


def test_it_recovers():
    s = Seeing()
    glare = np.clip(scene().astype(np.int32) + 160, 0, 255).astype(np.uint8)
    for _ in range(5):
        s.observe(_jitter(glare))
    assert s.state == GLARE
    feed(s, None, HOLD_FRAMES)
    assert s.state == OK


@pytest.mark.skipif(not os.path.isdir(ARCHIVE), reason='real archive not on this machine')
def test_REAL_FOOTAGE_never_reads_as_blind():
    """⛔ The negative control that decides the thresholds. Every clip, every
    frame of its first 20 s: not one may leave OK."""
    bad = []
    n = 0
    for clip in sorted(glob.glob(ARCHIVE + '/**/*.mkv', recursive=True)):
        cap = cv2.VideoCapture(clip)
        s = Seeing()
        for _ in range(600):
            ok, f = cap.read()
            if not ok:
                break
            n += 1
            if s.observe(f) != OK:
                bad.append((os.path.basename(clip), s.state))
                break
    assert n > 5000
    assert bad == [], bad
