"""YoloDetector tests with ultralytics + torch mocked.

Verifies the bits we wrote (allowlist filter, threshold pass-through,
empty-result safety, dataclass shape) without ever loading a real model.
"""

import sys
import types
import numpy as np
import pytest


# ----- shared fakes ------------------------------------------------------- #

class _FakeBoxes:
    def __init__(self, xyxy, conf, cls):
        import torch
        self.xyxy = torch.tensor(xyxy, dtype=torch.float32)
        self.conf = torch.tensor(conf, dtype=torch.float32)
        self.cls  = torch.tensor(cls,  dtype=torch.float32)

    def __len__(self):
        return self.xyxy.shape[0]


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeYOLO:
    """Stand-in for ultralytics.YOLO. Returns a fixed set of boxes."""
    NEXT_BOXES = None      # set by tests before instantiation
    NAMES      = {0: 'person', 1: 'cat', 2: 'dog', 3: 'backpack'}

    def __init__(self, *_, **__):
        self.names = dict(self.NAMES)

    def to(self, _device):
        return self

    def predict(self, _frame, **_kwargs):
        return [_FakeResult(self.NEXT_BOXES)]


@pytest.fixture
def patched(monkeypatch):
    # Real torch is OK if installed; tests need it for the tensor shims above.
    pytest.importorskip('torch')

    fake_ultra = types.ModuleType('ultralytics')
    fake_ultra.YOLO = _FakeYOLO
    monkeypatch.setitem(sys.modules, 'ultralytics', fake_ultra)
    return fake_ultra


def _make(monkeypatch, **overrides):
    """Build a YoloDetector with the patched fake. select_device is forced to cpu
    so we don't need a real CUDA stack."""
    from duburi_vision.detection import yolo
    monkeypatch.setattr(yolo, 'select_device',
                        lambda requested='cuda:0', logger=None: 'cpu')
    kwargs = dict(model_path='unused.pt', device='cpu', warmup=False,
                  class_allowlist=('person',))
    kwargs.update(overrides)
    return yolo.YoloDetector(**kwargs)


def test_empty_result_returns_empty_list(patched, monkeypatch):
    _FakeYOLO.NEXT_BOXES = _FakeBoxes(
        xyxy=np.zeros((0, 4)), conf=np.zeros((0,)), cls=np.zeros((0,)))
    det = _make(monkeypatch)
    out = det.infer(np.zeros((640, 640, 3), dtype=np.uint8))
    assert out == []


def test_allowlist_filters_other_classes(patched, monkeypatch):
    _FakeYOLO.NEXT_BOXES = _FakeBoxes(
        xyxy=[[10, 10, 50, 50],
              [60, 60, 100, 100],
              [110, 10, 150, 50]],
        conf=[0.9, 0.8, 0.7],
        cls=[0, 1, 0],          # person, cat, person
    )
    det = _make(monkeypatch, class_allowlist=('person',))
    out = det.infer(np.zeros((200, 200, 3), dtype=np.uint8))
    assert len(out) == 2
    assert all(d.class_name == 'person' for d in out)


def test_no_allowlist_keeps_everything(patched, monkeypatch):
    _FakeYOLO.NEXT_BOXES = _FakeBoxes(
        xyxy=[[0, 0, 10, 10], [0, 0, 20, 20], [0, 0, 30, 30]],
        conf=[0.5, 0.5, 0.5],
        cls=[0, 1, 3],
    )
    det = _make(monkeypatch, class_allowlist=None)
    out = det.infer(np.zeros((100, 100, 3), dtype=np.uint8))
    assert {d.class_name for d in out} == {'person', 'cat', 'backpack'}


def test_detection_dataclass_geometry(patched, monkeypatch):
    _FakeYOLO.NEXT_BOXES = _FakeBoxes(
        xyxy=[[10, 20, 50, 80]], conf=[0.99], cls=[0])
    det = _make(monkeypatch)
    out = det.infer(np.zeros((100, 100, 3), dtype=np.uint8))
    d = out[0]
    assert d.class_id == 0
    assert d.class_name == 'person'
    assert d.score == pytest.approx(0.99, abs=1e-5)
    assert d.width  == pytest.approx(40.0)
    assert d.height == pytest.approx(60.0)
    assert d.cx == pytest.approx(30.0)
    assert d.cy == pytest.approx(50.0)
    assert d.area == pytest.approx(40.0 * 60.0)


def test_infer_handles_none_frame(patched, monkeypatch):
    _FakeYOLO.NEXT_BOXES = _FakeBoxes(
        xyxy=[[0, 0, 1, 1]], conf=[0.5], cls=[0])
    det = _make(monkeypatch)
    assert det.infer(None) == []
    assert det.infer(np.zeros((0, 0, 3), dtype=np.uint8)) == []
