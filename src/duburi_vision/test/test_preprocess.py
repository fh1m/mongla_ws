"""Underwater contrast preprocessing: it must help, and never silently do nothing.

MEASURED, on real RoboSub 2025 footage, 600 frames of the gate approach at
conf 0.15 -- the clip where the target was effectively invisible:

    preprocessing        presence   mean score
    none                   10.7 %      0.226
    unsharp mask           38.0 %      0.329
    CLAHE (LAB, clip 2)    56.3 %      0.401
    CLAHE (YUV, clip 3)    56.2 %      0.410     <- shipped

...and no cost where it is not needed: bin 100 % -> 100 %, octagon 100 % ->
100 % with a HIGHER mean score. On the Pi, 640x360, one thread: 3.78 ms
(YUV) against 7.07 (LAB) for the same result.

These tests pin the properties that survive a rewrite -- that it enhances
local contrast, preserves shape and colour ordering, is cheap, and that a
mistyped setting RAISES rather than quietly meaning 'off'.
"""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip('cv2')

from duburi_vision.detection.preprocess import (      # noqa: E402
    DEFAULT_CLIP, make_clahe, make_preprocessor,
)


def _washed_out(w=640, h=360):
    """A low-contrast frame with a faint target -- the underwater case.

    Values crushed into a narrow band around mid-grey is exactly what
    backscatter does, and it is why a global stretch is the wrong tool: the
    haze is not uniform across the frame.
    """
    rng = np.random.default_rng(7)
    img = np.full((h, w, 3), 128, np.uint8)
    img[:, :, :] = (128 + rng.integers(-6, 6, (h, w, 3))).astype(np.uint8)
    img[150:210, 280:360] = 140          # the faint 'target'
    return img


def test_it_actually_increases_local_contrast():
    """The mechanism. If the standard deviation does not rise, nothing was
    enhanced and the 3.78 ms bought nothing."""
    f = _washed_out()
    out = make_clahe()(f)
    before = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).std()
    after = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY).std()
    assert after > before * 1.5, (before, after)


def test_the_target_stands_out_MORE_than_before():
    """Contrast for its own sake is not the point -- separating the target
    from its background is."""
    f = _washed_out()
    out = make_clahe()(f)

    def sep(img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float)
        return abs(g[150:210, 280:360].mean() - g[0:60, 0:60].mean())

    assert sep(out) > sep(f)


def test_shape_and_dtype_are_preserved():
    """It runs mid-pipeline; a changed shape or dtype breaks the letterbox
    silently rather than loudly."""
    f = _washed_out()
    out = make_clahe()(f)
    assert out.shape == f.shape and out.dtype == f.dtype


def test_colour_ordering_survives():
    """BGR in, BGR out. A swapped conversion looks plausible on grey test
    data and destroys a colour-cued detector on the vehicle -- the slalom
    pipes are found by being RED."""
    f = np.zeros((64, 64, 3), np.uint8)
    f[:, :, 2] = 200                       # pure red in BGR
    out = make_clahe()(f)
    b, g, r = out[..., 0].mean(), out[..., 1].mean(), out[..., 2].mean()
    assert r > b and r > g, f'red channel no longer dominant: {(b, g, r)}'


def test_off_returns_None_not_an_identity_function():
    """So the hot loop can skip the call, and so 'is it on' is answerable by
    looking at one attribute."""
    for name in ('off', 'none', '', 'FALSE'):
        assert make_preprocessor(name) is None


def test_an_unknown_name_RAISES_rather_than_meaning_off():
    """A typo that silently means 'off' is how a setting reaches nothing and
    the measurement does not move -- this repo has shipped that failure four
    times (device_path into **_, the unloaded YAML table, the fps override,
    the QoS mismatch)."""
    with pytest.raises(ValueError):
        make_preprocessor('clache')          # plausible typo
    with pytest.raises(ValueError):
        make_preprocessor('histogram')


def test_the_clip_limit_actually_reaches_the_operator():
    """A parameter that does not change the output is a parameter nobody can
    use. Higher clip = more amplification."""
    f = _washed_out()
    soft = cv2.cvtColor(make_clahe(1.0)(f), cv2.COLOR_BGR2GRAY).std()
    hard = cv2.cvtColor(make_clahe(6.0)(f), cv2.COLOR_BGR2GRAY).std()
    assert hard > soft, (soft, hard)


def test_the_shipped_default_is_the_measured_best():
    """3.0 came from the sweep on real footage (56.2 % presence, mean score
    0.410 -- the best of 1.5 / 2.0 / 3.0), not from a guess."""
    assert DEFAULT_CLIP == 3.0


def test_it_is_cheap_enough_for_the_hot_path():
    """Budget check. The photon-to-detections chain is 18.0 ms; this must be
    a few ms, not tens. Loose bound -- it is a smoke test against an
    accidental per-frame CLAHE object or a full-resolution conversion, not a
    benchmark (the Pi number, 3.78 ms, is in the module docstring)."""
    f = _washed_out()
    fn = make_clahe()
    fn(f)
    t0 = time.perf_counter()
    for _ in range(20):
        fn(f)
    per = (time.perf_counter() - t0) / 20 * 1000
    assert per < 25.0, f'{per:.1f} ms/frame is too slow for the hot path'


def test_the_clahe_object_is_built_ONCE():
    """Rebuilding it per frame is ~2x the cost for identical output, and this
    runs 77 times a second."""
    calls = {'n': 0}
    real = cv2.createCLAHE

    def counting(*a, **k):
        calls['n'] += 1
        return real(*a, **k)

    cv2.createCLAHE = counting
    try:
        fn = make_clahe()
        f = _washed_out()
        for _ in range(10):
            fn(f)
    finally:
        cv2.createCLAHE = real
    assert calls['n'] == 1, f'built {calls["n"]} times for 10 frames'


def test_a_BOOL_is_accepted_because_launch_produces_one():
    """ROS 2 launch coerces the literal 'off' in a `default_value` to boolean
    False before `declare_parameter` sees it -- which then raises
    InvalidParameterTypeException and kills the whole composed process at
    startup.

    Measured on the vehicle: every node dead, `preprocess:=off` in the launch
    file, and the only clue eleven frames down a traceback. The unit tests
    could not see it because they pass strings, as a human would.
    """
    assert make_preprocessor(False) is None
    assert make_preprocessor(True) is not None


def test_the_launch_default_survives_the_round_trip():
    """'off' is the natural word and it is the one that breaks. 'none' means
    the same to `make_preprocessor` and stays a string through launch."""
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    assert "DeclareLaunchArgument('preprocess', default_value='none')" in launch, \
        "the launch default must not be a value launch coerces to bool"
