"""The segmentation backend keeps the contract the detection backend keeps.

`HailoSegDetector` decodes raw heads on the CPU; `HailoDetector` reads the
chip's NMS. Three places they parted:

* the association band: boxes between `assoc_conf` and `conf` reach the
  tracker and the lock ladder (never `detections`). The seg path decoded at
  `conf`, so with a seg model that band was always empty;
* a mask on a box that runs off the frame was RESIZED onto the clipped box,
  squashing the off-frame part into view, instead of cropped;
* `nms_iou` live: refused, because there is no chip NMS -- while the seg
  path runs its own NMS on an IoU nothing could change.
"""
import threading
import types

import numpy as np
import pytest

from mongla_vision.detection import seg_decode
from mongla_vision.detection.hailo_seg import HailoSegDetector


def _seg(monkeypatch, decoded):
    d = HailoSegDetector.__new__(HailoSegDetector)
    d._ready, d._layout, d._infer_lock = True, object(), threading.Lock()
    cim = types.SimpleNamespace(wait_for_async_ready=lambda **k: None,
                                run_async=lambda b: types.SimpleNamespace(wait=lambda t: None))
    d._configured = lambda: cim
    d._letterbox_into_bound = lambda f: (1.0, 0.0, 0.0)
    d._bindings, d._heads, d._proto_name, d._bufs = None, [], 'p', {'p': None}
    d._conf, d._assoc_conf, d._allow_ids = 0.5, 0.2, None
    d._iou, d._max_det, d._want_masks, d._names = 0.45, 100, True, {0: 'gate'}
    d._path, d._log = 'x_seg.hef', None
    seen = {}

    def fake(layout, heads, proto, **kw):
        seen.update(kw)
        return decoded
    monkeypatch.setattr(seg_decode, 'decode', fake)
    return d, seen


def test_the_seg_path_decodes_down_to_the_association_floor(monkeypatch):
    empty = (np.zeros((0, 4), np.float32), np.zeros(0, np.float32),
             np.zeros(0, np.int64), [])
    d, seen = _seg(monkeypatch, empty)
    d.infer(np.zeros((100, 100, 3), np.uint8))
    assert seen['conf'] == pytest.approx(0.2)


def test_a_mask_running_off_the_frame_is_cropped_not_squashed(monkeypatch):
    # Box spans x -50..50 in a 100 px frame. Its mask is 1 only on the part
    # OFF the frame; what is left in view must be empty.
    m = np.zeros((100, 100), np.uint8)
    m[:, :50] = 1
    decoded = (np.array([[-50.0, 0.0, 50.0, 100.0]], np.float32),
               np.array([0.9], np.float32), np.array([0]), [m])
    d, _ = _seg(monkeypatch, decoded)
    (det,) = d.infer(np.zeros((100, 100, 3), np.uint8))
    assert det.xyxy == (0.0, 0.0, 50.0, 100.0)
    assert det.mask.shape == (100, 50)
    assert int(det.mask.sum()) == 0


def test_nms_iou_is_live_on_the_seg_path_and_the_floor_is_refused(monkeypatch):
    d, _ = _seg(monkeypatch, None)
    d.update_nms(iou=0.3)
    assert d._iou == pytest.approx(0.3)
    with pytest.raises(ValueError):
        d.update_nms(floor=0.1)
    with pytest.raises(ValueError):
        d.update_nms(iou=1.5)
