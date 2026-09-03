"""Two detectors, one chip: the activation arbiter.

The chip allows exactly one ACTIVE network group. Everything else about
sharing it was measured on hardware and is recorded in `hailo.py`; what these
tests hold is the part that can go wrong in software and would be a core dump
rather than an exception on the vehicle:

  * a second detector must CONSTRUCT (the old code activated in __init__, so it
    could not),
  * exactly one may hold the activation at any moment,
  * the handover must be atomic under a MultiThreadedExecutor,
  * close() must not release the DEVICE while another detector is using it.

`hailo_platform` is stubbed. That is not a weaker test here: the invariant is
about our bookkeeping, and the real runtime answers a double activation with a
segfault, which no test can catch.
"""
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# --------------------------------------------------------------------------- #
#  A chip that COUNTS activations and refuses a second one
# --------------------------------------------------------------------------- #
class _Chip:
    """Models the one property that matters: activation is exclusive.

    The real device does not raise on a double activation, it dumps core. This
    raises instead so a failure is a readable assertion rather than a dead
    worker.
    """

    def __init__(self):
        self.active = None
        self.releases = 0
        self.devices = 0


CHIP = _Chip()


class _Act:
    def __init__(self, owner):
        self.owner = owner

    def __enter__(self):
        # The real swap costs ~4.15 ms. Modelling it as instantaneous is what
        # made the first version of the thread test USELESS: it passed with the
        # lock removed, because the unguarded window was too narrow for the GIL
        # to interleave. A test that cannot fail is not evidence.
        time.sleep(0.002)
        assert CHIP.active is None, (
            f'{self.owner} activated while {CHIP.active} still holds the chip '
            f'-- on hardware this is a segfault')
        CHIP.active = self.owner
        return self

    def __exit__(self, *a):
        CHIP.active = None
        CHIP.releases += 1


class _Ng:
    def __init__(self, name):
        self.name = name

    def create_params(self):
        return {}

    def activate(self, _p=None):
        return _Act(self.name)


class _Pipe:
    def __init__(self, ng, *_a):
        self.ng = ng

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def infer(self, _feed):
        assert CHIP.active == self.ng.name, (
            f'inferred on {self.ng.name} while {CHIP.active} is active')
        return {'out': np.array([[[]]], dtype=object)}


class _VDevice:
    def __init__(self, *_a, **_k):
        CHIP.devices += 1

    def configure(self, hef, _cfg):
        return [_Ng(hef.stem)]

    def release(self):
        raise AssertionError('the shared device must never be released by a '
                             'single detector: another may still hold a graph '
                             'configured on it')


class _Info:
    def __init__(self, name, size=640):
        self.name = name
        self.shape = (size, size, 3)


class _HEF:
    def __init__(self, path):
        self.stem = Path(path).stem

    def get_input_vstream_infos(self):
        return [_Info('in')]

    def get_output_vstream_infos(self):
        return [_Info('out')]


def _install_stub():
    m = types.ModuleType('hailo_platform')
    m.HEF = _HEF
    m.VDevice = _VDevice
    m.InferVStreams = _Pipe
    m.HailoStreamInterface = types.SimpleNamespace(PCIe=0)
    m.ConfigureParams = types.SimpleNamespace(create_from_hef=lambda h, **k: {})
    m.InputVStreamParams = types.SimpleNamespace(make=lambda ng, **k: {})
    m.OutputVStreamParams = types.SimpleNamespace(make=lambda ng, **k: {})
    m.FormatType = types.SimpleNamespace(UINT8=0, FLOAT32=1)
    sys.modules['hailo_platform'] = m


@pytest.fixture
def hailo(tmp_path, monkeypatch):
    _install_stub()
    global CHIP
    CHIP = _Chip()
    import importlib
    from duburi_vision.detection import hailo as mod
    importlib.reload(mod)
    monkeypatch.setattr(mod, '_ACTIVE', None, raising=False)
    monkeypatch.setattr(mod, '_DEVICE', None, raising=False)
    return mod


def _model(tmp_path, stem, names):
    (tmp_path / f'{stem}.yaml').write_text(
        'names:\n' + ''.join(f'  {i}: {n}\n' for i, n in enumerate(names)))
    p = tmp_path / f'{stem}.hef'
    p.write_bytes(b'\0')
    return str(p)


def _pair(mod, tmp_path):
    a = mod.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                          class_allowlist=None, warmup=False)
    b = mod.HailoDetector(model_path=_model(tmp_path, 'dwn', ['fire']),
                          class_allowlist=None, warmup=False)
    return a, b


FRAME = np.zeros((360, 640, 3), np.uint8)


# --------------------------------------------------------------------------- #
def test_a_second_detector_can_be_constructed(hailo, tmp_path):
    """This is the whole bug. `vision_dual` launches two of these, and the old
    code activated in __init__ -- so the dual-camera path could never start."""
    a, b = _pair(hailo, tmp_path)
    assert a is not b


def test_one_device_serves_both(hailo, tmp_path):
    """A VDevice per detector is HAILO_OUT_OF_PHYSICAL_DEVICES (74) on the
    second, measured. One per process."""
    _pair(hailo, tmp_path)
    assert CHIP.devices == 1


def test_nothing_is_activated_until_the_first_infer(hailo, tmp_path):
    _pair(hailo, tmp_path)
    assert CHIP.active is None


def test_inferring_takes_the_chip(hailo, tmp_path):
    a, _ = _pair(hailo, tmp_path)
    a.infer(FRAME)
    assert CHIP.active == 'fwd'


def test_the_other_detector_evicts_it(hailo, tmp_path):
    a, b = _pair(hailo, tmp_path)
    a.infer(FRAME)
    b.infer(FRAME)
    assert CHIP.active == 'dwn'


def test_staying_on_one_detector_does_not_swap(hailo, tmp_path):
    """The mission model -- one camera live, the other paused -- must cost
    nothing. A swap per frame is 4.15 ms against a 10.18 ms frame."""
    a, _ = _pair(hailo, tmp_path)
    for _ in range(20):
        a.infer(FRAME)
    assert CHIP.releases == 0


def test_alternating_swaps_once_per_change_not_once_per_frame(hailo, tmp_path):
    a, b = _pair(hailo, tmp_path)
    for _ in range(5):
        a.infer(FRAME)
    for _ in range(5):
        b.infer(FRAME)
    assert CHIP.releases == 1


def test_two_threads_cannot_both_hold_it(hailo, tmp_path):
    """Two detector nodes in one process are two rclpy callbacks, and on a
    MultiThreadedExecutor they run on different threads. Without the lock both
    can observe `_ACTIVE is not self`, both evict, and both enter."""
    a, b = _pair(hailo, tmp_path)
    errs = []

    def spin(det):
        try:
            for _ in range(40):
                det.infer(FRAME)
        except BaseException as e:                               # noqa: BLE001
            errs.append(e)

    ts = [threading.Thread(target=spin, args=(d,)) for d in (a, b, a, b)]
    for t in ts:
        t.start()
    for t in ts:
        t.join(30)
    assert not errs, errs[0]


def test_close_does_not_release_the_shared_device(hailo, tmp_path):
    """Pulling the device out from under a detector that still has a graph
    configured on it is a segfault during shutdown -- the hardest kind to
    read. The stub raises so this is an assertion instead."""
    a, b = _pair(hailo, tmp_path)
    a.infer(FRAME)
    a.close()                                    # must not raise
    b.infer(FRAME)                               # and b must still work
    assert CHIP.active == 'dwn'


def test_closing_the_holder_frees_the_chip(hailo, tmp_path):
    a, _ = _pair(hailo, tmp_path)
    a.infer(FRAME)
    a.close()
    assert CHIP.active is None


def test_a_closed_detector_stops_inferring(hailo, tmp_path):
    a, _ = _pair(hailo, tmp_path)
    a.close()
    assert a.infer(FRAME) == []


def test_competition_is_reported_rather_than_left_to_be_discovered(hailo, tmp_path):
    """73.8 Hz across two cameras reads as 'the chip got slower' unless
    something says the two detectors are fighting over it."""
    warned = []
    a = hailo.HailoDetector(
        model_path=_model(tmp_path, 'fwd', ['gate']), class_allowlist=None,
        warmup=False, logger=types.SimpleNamespace(
            info=lambda s: None, warn=warned.append))
    b = hailo.HailoDetector(model_path=_model(tmp_path, 'dwn', ['fire']),
                            class_allowlist=None, warmup=False)
    for _ in range(hailo._SWAP_WARN_AT * 2):
        a.infer(FRAME)
        b.infer(FRAME)
    assert any('competing for the chip' in w for w in warned)
