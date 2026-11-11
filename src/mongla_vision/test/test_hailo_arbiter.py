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
import subprocess
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
        self.models = []             # every InferModel created
        self.refuse_floor = False    # set_nms_score_threshold raises


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
        CHIP.models.append(self)
        self.nms_floor = None       # what set_nms_score_threshold was given
        self.floor_set_after_configure = False
        self.configured = False

    def _set_floor(self, value):
        if CHIP.refuse_floor:
            raise RuntimeError('HAILO_INVALID_OPERATION')
        self.floor_set_after_configure = self.configured
        self.nms_floor = value

    def _set_iou(self, value):
        self.nms_iou = value

    def input(self):
        return types.SimpleNamespace(set_format_type=lambda t: None)

    def output(self):
        # Measured on the chip, not assumed. A HailoRT NMS output bound
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
            set_nms_score_threshold=self._set_floor,
            set_nms_iou_threshold=self._set_iou,
            shape=(self._nclasses * (1 + self._CAP * 5),))

    def configure(self):
        CHIP.configures += 1
        self.configured = True
        self.configured_at = (self.nms_floor, getattr(self, 'nms_iou', None))
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
    from mongla_vision.detection import hailo as mod, hailo_device
    importlib.reload(hailo_device)
    importlib.reload(mod)
    monkeypatch.setattr(hailo_device, '_DEVICE', None, raising=False)
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
    """The property that was missing. Detector A is held mid-inference; B must
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
    monkeypatch.setattr(
        subprocess, 'run',
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
    monkeypatch.setattr(subprocess, 'run',
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


def _measured(tmp_path, stem, bar):
    """A sidecar carrying a `separation:` block, as negative_clip_check writes."""
    path = _model(tmp_path, stem, ['gate'])
    sidecar = Path(path).with_suffix('.yaml')
    sidecar.write_text(sidecar.read_text() + (
        "separation:\n  gate:\n    measured: '2026-09-28'\n"
        f"    recommended_conf: {'null' if bar is None else bar}\n"))
    return path


def test_the_bar_the_model_measured_is_quiet(hailo, tmp_path, monkeypatch):
    """gate_sharks measured 0.45 and every launch ships 0.45. The retired
    warning called that 'three times the intended operating point' on an INT8
    cost the ledger later retracted, and it fired on every launch."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_measured(tmp_path, 'fwd', 0.45),
                        conf=0.45, class_allowlist=None, warmup=False,
                        logger=log)
    assert warned['warn'] == []


@pytest.mark.parametrize('conf,words', [(0.15, 'empty water'),
                                        (0.75, 'being discarded')])
def test_a_conf_off_the_measured_bar_is_reported_with_it(
        hailo, tmp_path, monkeypatch, conf, words):
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_measured(tmp_path, 'fwd', 0.45),
                        conf=conf, class_allowlist=None, warmup=False,
                        logger=log)
    assert any(words in w and '0.45' in w for w in warned['warn'])


def test_a_class_measured_unusable_is_reported(hailo, tmp_path, monkeypatch):
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_measured(tmp_path, 'fwd', None),
                        conf=0.45, class_allowlist=None, warmup=False,
                        logger=log)
    assert any('not evidence' in w for w in warned['warn'])


def test_an_unmeasured_model_is_quiet(hailo, tmp_path, monkeypatch):
    """No `separation:` block means nobody has measured it -- unknown, not
    wrong, and the preflight already names the tool that measures it."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    warned, log = _logger()
    hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                        conf=0.45, class_allowlist=None, warmup=False,
                        logger=log)
    assert warned['warn'] == []


def test_a_graph_compiled_above_the_floor_is_configured_at_the_floor(
        hailo, tmp_path, monkeypatch):
    """A stock graph is compiled at 0.200. Its NMS runs on the host and takes
    a new threshold when the model is configured (measured-bars 79), so the
    detector asks for EMIT_FLOOR before the configure, and 0.10 is then a
    conf this model can run at: no BELOW warning."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.200\n')
    warned, log = _logger()
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.10, class_allowlist=None, warmup=False,
                              logger=log)
    model = CHIP.models[-1]
    assert model.nms_floor == pytest.approx(hailo.EMIT_FLOOR)
    assert det._floor == pytest.approx(hailo.EMIT_FLOOR)
    assert not any('BELOW' in w for w in warned['warn'])
    det.infer(np.zeros((64, 64, 3), np.uint8))
    assert model.configured and not model.floor_set_after_configure


def test_a_graph_already_at_the_floor_is_left_alone(
        hailo, tmp_path, monkeypatch):
    """Ours are compiled at 0.05. Setting the same value again is a call into
    the runtime that can only go wrong; a graph compiled lower keeps its own."""
    for baked in ('0.050', '0.020'):
        _stub_parse_hef(monkeypatch, hailo, f'Score threshold: {baked}\n')
        det = hailo.HailoDetector(
            model_path=_model(tmp_path, f'm{baked[-2:]}', ['gate']),
            conf=0.45, class_allowlist=None, warmup=False)
        assert CHIP.models[-1].nms_floor is None
        assert det._floor == pytest.approx(float(baked))


def test_a_refused_lowering_keeps_and_reports_the_compiled_floor(
        hailo, tmp_path, monkeypatch):
    """If the runtime refuses, the floor is still the compiled one. Reporting
    EMIT_FLOOR then would bring back the silent overrule this log exists for."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.200\n')
    monkeypatch.setattr(CHIP, 'refuse_floor', True)
    warned, log = _logger()
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.10, class_allowlist=None, warmup=False,
                              logger=log)
    assert det._floor == pytest.approx(0.2)
    assert any('could not lower' in w for w in warned['warn'])
    assert any('BELOW' in w and '0.200' in w for w in warned['warn'])


def test_the_nms_changes_on_a_live_model_by_configuring_it_again(
        hailo, tmp_path, monkeypatch):
    """Both values are read at the configure. A change on a running model
    hands the configured model back, and the next inference configures it
    with the new values (measured-bars 79: about 85 ms, once)."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.45, class_allowlist=None, warmup=False)
    frame = np.zeros((64, 64, 3), np.uint8)
    det.infer(frame)
    model, before = CHIP.models[-1], CHIP.configures
    det.update_nms(floor=0.02, iou=0.5)
    assert CHIP.configures == before, 'configured eagerly, under the caller'
    det.infer(frame)
    assert CHIP.configures == before + 1
    assert model.configured_at == (0.02, 0.5)
    assert det._floor == pytest.approx(0.02)
    det.infer(frame)
    assert CHIP.configures == before + 1, 'configured again with no change'


def test_an_nms_change_before_the_first_frame_costs_no_configure(
        hailo, tmp_path, monkeypatch):
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    before = CHIP.configures
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.45, class_allowlist=None, warmup=False)
    det.update_nms(iou=0.6)
    assert CHIP.configures == before
    det.infer(np.zeros((64, 64, 3), np.uint8))
    assert CHIP.configures == before + 1
    assert CHIP.models[-1].configured_at[1] == 0.6


def test_a_floor_raised_and_put_back_leaves_the_other_floors_alone(
        hailo, tmp_path, monkeypatch):
    """On the vehicle a floor raised to 0.5 and returned to 0.05 left the
    detector returning nothing: the change had dragged the association floor
    up with it, and nothing brought it back down."""
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.45, class_allowlist=None, warmup=False)
    det.update_assoc_conf(0.10)
    det.update_nms(floor=0.50)
    det.update_nms(floor=0.05)
    assert det._assoc_conf == pytest.approx(0.10)
    assert det.acting_conf() == pytest.approx(0.45)


@pytest.mark.parametrize('kwargs', [{'floor': 0.0}, {'floor': 1.0},
                                    {'iou': 0.0}, {'iou': 1.5}])
def test_an_nms_value_out_of_range_is_refused(hailo, tmp_path, monkeypatch,
                                              kwargs):
    _stub_parse_hef(monkeypatch, hailo, 'Score threshold: 0.050\n')
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.45, class_allowlist=None, warmup=False)
    with pytest.raises(ValueError):
        det.update_nms(**kwargs)
    assert det._floor == pytest.approx(0.05)


def test_a_graph_with_no_nms_output_refuses_the_change(
        hailo, tmp_path, monkeypatch):
    """A caller must never report a change that did not happen."""
    _stub_parse_hef(monkeypatch, hailo, 'no post-process here\n')
    det = hailo.HailoDetector(model_path=_model(tmp_path, 'fwd', ['gate']),
                              conf=0.45, class_allowlist=None, warmup=False)
    with pytest.raises(ValueError, match='no NMS'):
        det.update_nms(floor=0.02)


def test_a_stock_model_baked_at_0_2_does_not_get_the_int8_advice(
        hailo, tmp_path, monkeypatch):
    """A conf at the bar a model measured gets no operating-point advice,
    whatever the graph was compiled at."""
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


