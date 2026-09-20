"""The backend seam: extension decides, stems stay portable, boxes come back right.

These run without `hailo_platform` or `ultralytics` -- the factory imports each
backend lazily for exactly that reason, and the box-mapping maths is a pure
function that does not need a chip.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.detection.factory import (      # noqa: E402
    KNOWN_EXTENSIONS, backend_for,
)


# --------------------------------------------------------------------------- #
#  Backend selection
# --------------------------------------------------------------------------- #
def test_extension_picks_the_backend():
    assert backend_for('/models/gate_rescue_repair.hef') == 'hailo'
    assert backend_for('/models/gate_rescue_repair.engine') == 'yolo'
    assert backend_for('/models/gate_rescue_repair.pt') == 'yolo'


def test_hef_is_preferred_over_engine_and_pt():
    """A device has one compiled artifact and it is the fast one: measured
    82.3 Hz (Hailo) vs 20-30 (TensorRT) vs 3-4 (raw PyTorch). If this order ever
    changes, `yolo._resolve_model_path` must change with it -- they walk the
    same list, and disagreeing means resolving one file and loading another."""
    assert KNOWN_EXTENSIONS.index('.hef') < KNOWN_EXTENSIONS.index('.engine')
    assert KNOWN_EXTENSIONS.index('.engine') < KNOWN_EXTENSIONS.index('.pt')


def test_the_resolver_walks_the_same_extension_order():
    """REGRESSION GUARD for the split above. The factory and the resolver each
    keep their own list; a stem that resolves to .hef but dispatches on a list
    that has not heard of .hef would load a Hailo graph through ultralytics."""
    src = (Path(__file__).resolve().parents[1]
           / 'mongla_vision' / 'detection' / 'yolo.py').read_text()
    assert "for ext in ('.hef', '.engine', '.pt'):" in src, (
        'yolo._resolve_model_path no longer walks the factory order')
    assert src.count("for ext in ('.hef', '.engine', '.pt'):") == 2, (
        'both the source-tree and share-dir search roots must agree')


def test_an_explicit_hef_path_is_passed_through():
    """`model:=/abs/path.hef` must not be treated as a stem and re-resolved."""
    from mongla_vision.detection.yolo import _resolve_model_path
    assert _resolve_model_path('/tmp/x/gate.hef') == '/tmp/x/gate.hef'


# --------------------------------------------------------------------------- #
#  Letterbox round-trip -- the part that silently drives the hull the wrong way
# --------------------------------------------------------------------------- #
from mongla_vision.detection.hailo import letterbox   # noqa: E402


def _unletterbox(b, size, scale, pad_x, pad_y):
    """The mapping HailoDetector.infer applies, isolated so it can be tested
    without a chip. HailoRT emits (y1, x1, y2, x2) normalised to the PADDED
    square -- y first, and relative to the canvas rather than the frame."""
    y1, x1, y2, x2 = b
    return ((x1 * size - pad_x) / scale, (y1 * size - pad_y) / scale,
            (x2 * size - pad_x) / scale, (y2 * size - pad_y) / scale)


@pytest.mark.parametrize('w,h', [(1280, 720), (640, 360), (640, 480), (720, 1280)])
def test_a_box_survives_the_letterbox_round_trip(w, h):
    """Put a known rectangle through the letterbox, express it the way the chip
    would, map it back, and it must land where it started.

    This is the check worth having: an un-letterbox that drops the pads maps
    every box onto a stretched frame. It looks completely plausible -- boxes
    still track the object -- and it is wrong by the bar width, which on a
    vision-servoed hull is a steady-state aiming error."""
    size = 640
    frame = np.zeros((h, w, 3), np.uint8)
    _, scale, pad_x, pad_y = letterbox(frame, size)

    x1, y1, x2, y2 = 0.25 * w, 0.30 * h, 0.55 * w, 0.70 * h
    # forward: frame -> padded canvas -> normalised, in the chip's y-first order
    nb = ((y1 * scale + pad_y) / size, (x1 * scale + pad_x) / size,
          (y2 * scale + pad_y) / size, (x2 * scale + pad_x) / size)
    got = _unletterbox(nb, size, scale, pad_x, pad_y)
    assert got == pytest.approx((x1, y1, x2, y2), abs=1e-6)


def test_dropping_the_padding_is_detectably_wrong():
    """Verifies the test above can FAIL -- a round-trip that passes under a
    broken mapping is testing nothing."""
    size, w, h = 640, 1280, 720
    _, scale, pad_x, pad_y = letterbox(np.zeros((h, w, 3), np.uint8), size)
    assert pad_y > 0, 'a 16:9 frame must letterbox to non-zero vertical pads'
    x1, y1 = 0.25 * w, 0.30 * h
    nb_y = (y1 * scale + pad_y) / size
    wrong = nb_y * size / scale          # pads ignored
    assert abs(wrong - y1) > 1.0


def test_letterbox_is_square_and_aspect_preserving():
    out, scale, pad_x, pad_y = letterbox(np.zeros((720, 1280, 3), np.uint8), 640)
    assert out.shape == (640, 640, 3)
    assert scale == pytest.approx(640 / 1280)
    assert pad_x == 0 and pad_y > 0      # wide frame: bars top and bottom


# --------------------------------------------------------------------------- #
#  The sidecar is mandatory on this backend
# --------------------------------------------------------------------------- #
def test_a_hef_without_a_sidecar_refuses_to_construct(tmp_path):
    """A .pt carries model.names; a .hef carries nothing. Without the sidecar
    there is no id->label map, the allowlist matches nothing, and the detector
    returns [] every frame while every health check passes -- the documented
    silent-failure mode. Fail at construction instead."""
    hailo = pytest.importorskip(
        'mongla_vision.detection.hailo',
        reason='module imports numpy/cv2 only; hailo_platform is lazy')
    fake = tmp_path / 'nosidecar.hef'
    fake.write_bytes(b'not a real hef')
    with pytest.raises(FileNotFoundError) as exc:
        hailo.HailoDetector(model_path=str(fake))
    assert 'sidecar' in str(exc.value).lower()
    assert 'nosidecar.yaml' in str(exc.value)     # names the file to create
