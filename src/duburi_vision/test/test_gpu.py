"""select_device tests -- mock torch so they pass on CI without a GPU."""

import sys
import types
import pytest


@pytest.fixture
def fake_torch(monkeypatch):
    """Install a tiny fake torch into sys.modules so select_device imports
    it without pulling the real (heavy) wheel."""
    mod = types.ModuleType('torch')
    mod.__version__ = 'fake-2.11'
    mod.version = types.SimpleNamespace(cuda='fake-12.8')

    class _Cuda:
        _avail = True

        @classmethod
        def is_available(cls):
            return cls._avail

        @classmethod
        def get_device_name(cls, idx):
            return f'FakeGPU#{idx}'

    mod.cuda = _Cuda
    monkeypatch.setitem(sys.modules, 'torch', mod)
    return mod


def test_cuda_requested_and_available(fake_torch):
    fake_torch.cuda._avail = True
    from duburi_vision.detection.gpu import select_device
    assert select_device('cuda:0') == 'cuda:0'


def test_cuda_requested_but_missing_raises(fake_torch):
    fake_torch.cuda._avail = False
    from duburi_vision.detection.gpu import select_device
    with pytest.raises(RuntimeError) as exc:
        select_device('cuda:0')
    msg = str(exc.value)
    assert 'CUDA' in msg or 'cuda' in msg.lower()
    assert "device='cpu'" in msg, 'must point user at the cpu workaround'


def test_cpu_explicit(fake_torch):
    from duburi_vision.detection.gpu import select_device
    assert select_device('cpu') == 'cpu'


def test_auto_falls_back_to_cpu(fake_torch):
    fake_torch.cuda._avail = False
    from duburi_vision.detection.gpu import select_device
    assert select_device('auto') == 'cpu'


def test_auto_uses_cuda_when_present(fake_torch):
    fake_torch.cuda._avail = True
    from duburi_vision.detection.gpu import select_device
    assert select_device('auto') == 'cuda:0'


def test_unknown_device_raises(fake_torch):
    from duburi_vision.detection.gpu import select_device
    with pytest.raises(ValueError):
        select_device('vulkan')
