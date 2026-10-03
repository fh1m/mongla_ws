"""Two detectors and the XFeat anchors, one chip: HailoRT's scheduler shares it.

Until 2026-10-03 the chip was shared by hand -- one ACTIVE network group,
handed back and forth under a process-wide lock held across each inference.
Measured live with the lock ladders composed into the detector process, that
lock turned every GIL stall into an idle chip: 69 -> 47 Hz combined at an
unchanged ~10.5 ms of chip time per call. The ROUND_ROBIN scheduler now owns
activation (measured 46.1 + 46.1 Hz for the two detectors, stable with four
groups and a configure/close mid-run).

What these tests hold is the contract with the scheduler, which on hardware
fails as a core dump or a silent stall rather than an exception:

  * one VDevice per process, created WITH the ROUND_ROBIN scheduler;
  * `activate()` is never called (invalid with the scheduler);
  * two models are inferring AT THE SAME TIME without waiting on each other
    -- no Python lock spans another model's inference;
  * one model's bound buffers are never used by two calls at once;
  * each group is configured ONCE (per-call configure filled the chip's SRAM);
  * close() never releases the shared device.

`hailo_platform` is stubbed; the stub raises where the hardware would crash.
"""
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


ROUND_ROBIN = 'ROUND_ROBIN'


class _Chip:
    def __init__(self):
        self.devices = 0
        self.scheduler = None
        self.configures = 0
        self.inflight = set()        # models mid-inference right now
        self.overlap = False         # two models were mid-inference together
        # Set by a test to hold one model's inference open until released.
        self.hold = {}


CHIP = _Chip()


class _Job:
    """`wait()` RELEASES THE GIL on hardware; `time.sleep` models that."""

    def __init__(self, cim):
        self.cim = cim

    def wait(self, _ms):
        name = self.cim.name
        CHIP.inflight.add(name)
        if len(CHIP.inflight) > 1:
            CHIP.overlap = True
        gate = CHIP.hold.get(name)
        if gate is not None:
            assert gate.wait(5), f'{name} was held and never released'
        time.sleep(0.002)
        CHIP.inflight.discard(name)
        self.cim.busy = False


class _Bindings:
    def __init__(self):
        ns = types.SimpleNamespace(set_buffer=lambda b: None)
        self._in = self._out = ns

    def input(self):
        return self._in

    def output(self, _name=None):
        return self._out


class _Cim:
    def __init__(self, name):
        self.name = name
        self.busy = False
        self.exited = False

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.exited = True
        return False

    def activate(self):
        raise AssertionError('activate() with the scheduler enabled is invalid '
                             '-- HailoRT owns activation')

    def deactivate(self):
        raise AssertionError('deactivate() with the scheduler enabled')

    def create_bindings(self):
        return _Bindings()

    def wait_for_async_ready(self, timeout_ms=0):
        pass

    def run_async(self, _bindings):
        assert not self.exited, f'{self.name} inferred after close'
        assert not self.busy, (
            f'two calls on {self.name} at once -- its bound buffers would be '
            f'overwritten mid-read')
        self.busy = True
        return _Job(self)


class _InferModel:
    _CAP = 5          # boxes per class in the stub; the real HEF uses 100

    def __init__(self, name, nclasses):
        self.name = name
        self._nclasses = nclasses

    def input(self):
        return types.SimpleNamespace(set_format_type=lambda t: None)

    def output(self):
        # MEASURED ON THE CHIP, not assumed. A HailoRT NMS output bound
        # through InferModel is a FLAT float32 buffer:
        #
        #     [ count_0, (y1 x1 y2 x2 score) * CAP, count_1, ... ]
        #
        # with a fixed per-class stride. The real model reports (1503,) for
        # 3 classes = 3 * (1 + 100 * 5). This stub is the same layout with
        # CAP=5, so the decoder's stride arithmetic is exercised rather than
        # bypassed.
        #
        # The first version of this stub said (3, 5, 5) -- the shape I
        # expected -- and hardware answered (1503,). A fake that models the
        # API you imagined tests the code you imagined.
        return types.SimpleNamespace(
            set_format_type=lambda t: None,
            shape=(self._nclasses * (1 + self._CAP * 5),))

    def configure(self):
        CHIP.configures += 1
        return _Cim(self.name)


class _VDevice:
    def __init__(self, params=None):
        CHIP.devices += 1
        CHIP.scheduler = getattr(params, 'scheduling_algorithm', None)

    @staticmethod
    def create_params():
        return types.SimpleNamespace(scheduling_algorithm=None)

    def configure(self, hef, _cfg):
        raise AssertionError('legacy configure path used')

    def create_infer_model(self, path):
        # Sized from the SAME sidecar the detector reads. A stub that invents
        # its own class count tests a mismatch that never ships and hides the
        # stride bug that does.
        import yaml
        y = Path(path).with_suffix('.yaml')
        n = len(yaml.safe_load(y.read_text())['names']) if y.exists() else 1
        return _InferModel(Path(path).stem, n)

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
    m.HailoSchedulingAlgorithm = types.SimpleNamespace(ROUND_ROBIN=ROUND_ROBIN)
    m.FormatType = types.SimpleNamespace(UINT8=0, FLOAT32=1)
    sys.modules['hailo_platform'] = m


@pytest.fixture
def hailo(tmp_path, monkeypatch):
    _install_stub()
    global CHIP
    CHIP = _Chip()
    import importlib
    from mongla_vision.detection import hailo as mod
    importlib.reload(mod)
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
    a, b = _pair(hailo, tmp_path)
    assert a is not b


def test_one_device_serves_both(hailo, tmp_path):
    """A VDevice per detector is HAILO_OUT_OF_PHYSICAL_DEVICES (74) on the
    second, measured. One per process."""
    _pair(hailo, tmp_path)
    assert CHIP.devices == 1


def test_the_device_runs_the_ROUND_ROBIN_scheduler(hailo, tmp_path):
    """Without it, two configured groups cannot both run, and the code would
    need the hand-off whose lock idled the chip."""
    a, _ = _pair(hailo, tmp_path)
    a.infer(FRAME)
    assert CHIP.scheduler == ROUND_ROBIN


def test_nothing_is_configured_until_the_first_infer(hailo, tmp_path):
    _pair(hailo, tmp_path)
    assert CHIP.configures == 0


def test_each_group_is_configured_ONCE(hailo, tmp_path):
    """`configure()` allocates on-chip SRAM. Per-call configure filled the
    chip: CONTEXT_SWITCH_STATUS_SRAM_MEMORY_FULL, HAILO_OUT_OF_FW_MEMORY(71),
    then every inference failing forever at 98 % CPU."""
    a, b = _pair(hailo, tmp_path)
    for _ in range(12):
        a.infer(FRAME)
        b.infer(FRAME)
    assert CHIP.configures == 2


def test_two_models_infer_AT_ONCE_and_neither_waits_on_the_other(hailo, tmp_path):
    """THE PROPERTY THAT WAS MISSING. Detector A is held mid-inference; B must
    still complete. Under the old process-wide lock, B waited for A -- and on
    the vehicle A was often waiting for the GIL, not the chip."""
    a, b = _pair(hailo, tmp_path)
    a.infer(FRAME)
    b.infer(FRAME)                       # both configured
    CHIP.hold['fwd'] = threading.Event()
    t = threading.Thread(target=a.infer, args=(FRAME,))
    t.start()
    time.sleep(0.05)                     # A is now inside its wait
    done = threading.Event()
    threading.Thread(target=lambda: (b.infer(FRAME), done.set())).start()
    finished = done.wait(2)
    CHIP.hold['fwd'].set()
    t.join(5)
    assert finished, 'B waited for A: a lock is held across another inference'
    assert CHIP.overlap


def test_one_detector_is_never_entered_twice_at_once(hailo, tmp_path):
    """Its buffers stay bound; a second call would overwrite them mid-read.
    Two executor threads can call the same detector."""
    a, _ = _pair(hailo, tmp_path)
    errs = []

    def spin():
        try:
            for _ in range(30):
                a.infer(FRAME)
        except BaseException as e:                               # noqa: BLE001
            errs.append(e)

    ts = [threading.Thread(target=spin) for _ in range(3)]
    for t in ts:
        t.start()
    for t in ts:
        t.join(30)
    assert not errs, errs[0]


def test_close_does_not_release_the_shared_device(hailo, tmp_path):
    """Pulling the device out from under a model that still has a group
    configured on it is a segfault. The stub raises so this is an assertion."""
    a, b = _pair(hailo, tmp_path)
    a.infer(FRAME)
    a.close()                                    # must not raise
    b.infer(FRAME)                               # and b must still work


def test_close_hands_the_group_back(hailo, tmp_path):
    a, _ = _pair(hailo, tmp_path)
    a.infer(FRAME)
    cim = a._cim
    a.close()
    assert cim.exited and a._cim is None


def test_a_closed_detector_stops_inferring(hailo, tmp_path):
    a, _ = _pair(hailo, tmp_path)
    a.close()
    assert a.infer(FRAME) == []


# --------------------------------------------------------------------------- #
#  The baked NMS floor
# --------------------------------------------------------------------------- #
def _stub_parse_hef(monkeypatch, mod, text):
    import subprocess
    monkeypatch.setattr(
        mod.subprocess, 'run',
        lambda *a, **k: subprocess.CompletedProcess(a[0], 0, stdout=text,
                                                    stderr=''))


def _logger():
    out = {'warn': [], 'info': []}
    return out, types.SimpleNamespace(warn=out['warn'].append,
                                      info=out['info'].append)


def test_the_baked_floor_is_read_from_the_hef(hailo, tmp_path, monkeypatch):
    """There is no Python API for it -- `HEF` exposes stream infos and nothing
    about the post-process -- so it comes off `hailortcli parse-hef`."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    assert hailo.baked_score_threshold('x.hef') == pytest.approx(0.05)


def test_an_unreadable_floor_is_None_not_a_guess(hailo, monkeypatch):
    """A default would be worse than nothing: it would let the warning below
    fire, or not fire, on a number nobody measured."""
    monkeypatch.setattr(hailo.subprocess, 'run',
                        lambda *a, **k: (_ for _ in ()).throw(OSError('no tool')))
    assert hailo.baked_score_threshold('x.hef') is None


def test_asking_below_the_floor_is_reported(hailo, tmp_path, monkeypatch):
    """The floor is invisible otherwise: the HEF drops everything under it
    before the host sees a byte, so a mission believes it lowered the bar."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                        conf=0.02, class_allowlist=None, warmup=False,
                        logger=log)
    assert any('BELOW' in w and '0.050' in w for w in warned['warn'])


def test_the_cuda_operating_point_is_reported_on_an_int8_graph(
        hailo, tmp_path, monkeypatch):
    """0.35-0.45 is the CUDA number, and every launch path ships it. INT8 costs
    ~0.08 of score, so it is ~3x the intended point -- the single most likely
    cause of 'the Hailo model misses things'."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                        conf=0.45, class_allowlist=None, warmup=False,
                        logger=log)
    assert any('intended operating point' in w for w in warned['warn'])


def test_the_intended_point_is_quiet(hailo, tmp_path, monkeypatch):
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                        conf=0.15, class_allowlist=None, warmup=False,
                        logger=log)
    assert warned['warn'] == []


def test_a_stock_model_baked_at_0_2_does_not_get_the_int8_advice(
        hailo, tmp_path, monkeypatch):
    """The advice is 'run at 0.12-0.15', and on a stock HEF that is not
    reachable -- its floor IS 0.200. Repeating it there would be advice the
    operator cannot follow."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.200\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                        conf=0.45, class_allowlist=None, warmup=False,
                        logger=log)
    assert not any('intended operating point' in w for w in warned['warn'])


def test_a_LATER_conf_change_is_checked_too(hailo, tmp_path, monkeypatch):
    """conf is live-tunable from the mission and from the console, so checking
    only at construction covers the case that is least likely to be wrong."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    d = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                            conf=0.15, class_allowlist=None, warmup=False,
                            logger=log)
    d.update_conf(0.01)
    assert any('BELOW' in w for w in warned['warn'])


