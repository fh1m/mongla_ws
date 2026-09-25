"""The pre-flight gate must not PASS on hardware that is not there.

⛔ WHY THIS FILE EXISTS. Measured on the vehicle on 2026-09-22, with **both
cameras physically removed and sitting in the lab**, `bringup_check --srot`
reported:

    [PASS] USB cameras    2 USB camera(s): platform-1000800000.codec-video-index0,
                                           platform-1000880000.pisp_be-video-index0

Those are the Raspberry Pi's hardware video **codec** and its **ISP backend**.
They are `/dev/v4l/by-path/*-video-index0` nodes that exist whether or not a
camera is plugged in, and the filter took anything matching that glob. On the
Jetson the defect could not appear, because that platform has no such nodes --
so this shipped the moment the default platform became a Pi.

A pre-flight gate that passes on a hull with no cameras is worse than no gate:
it is the failure this repository keeps naming, a plausible answer standing in
for an absent measurement, in the one tool whose entire job is to refuse.

The same run exposed section K checking for a TensorRT `.engine` and advising
`export_engine --all (ON THE JETSON)`. The default platform is a Pi 5 with a
Hailo-8, where the flight artifact is a `.hef` -- so that check could never
pass, on any correctly-configured vehicle.
"""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager import bringup_check as bc  # noqa: E402

# Exactly what `ls /dev/v4l/by-path/` printed on the vehicle, cameras removed.
PI_PLATFORM_NODES = [
    '/dev/v4l/by-path/platform-1000800000.codec-video-index0',
    '/dev/v4l/by-path/platform-1000880000.pisp_be-video-index0',
]
# A real UVC camera on a USB port, by-path form.
USB_CAMERA_NODES = [
    '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1:1.0-video-index0',
    '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.2:1.0-video-index0',
]


def _with_nodes(monkeypatch, nodes):
    monkeypatch.setattr(bc, 'glob', lambda pat: (
        sorted(nodes) if 'by-path' in pat else []))


# ── J. cameras ───────────────────────────────────────────────────────────────

def test_the_pi_codec_and_isp_are_not_cameras(monkeypatch):
    """THE REGRESSION. These two nodes are present on every Pi 5, always."""
    _with_nodes(monkeypatch, PI_PLATFORM_NODES)

    assert bc._physical_cameras() == []


def test_the_gate_does_not_pass_with_no_cameras_attached(monkeypatch):
    """The line the operator actually reads. It must not say PASS."""
    _with_nodes(monkeypatch, PI_PLATFORM_NODES)

    status, detail = bc._check_cameras()

    assert status != bc.PASS, f'gate passed with no cameras: {detail}'
    assert 'NO USB cameras' in detail


def test_real_usb_cameras_are_still_counted(monkeypatch):
    """The fix must not break the working case -- two cameras on two ports."""
    _with_nodes(monkeypatch, USB_CAMERA_NODES)

    assert len(bc._physical_cameras()) == 2
    assert bc._check_cameras()[0] == bc.PASS


def test_platform_nodes_do_not_pad_a_real_camera_count(monkeypatch):
    """One real camera beside the Pi's own nodes must read as ONE, not three --
    otherwise a single working camera masks the dead one."""
    _with_nodes(monkeypatch, PI_PLATFORM_NODES + USB_CAMERA_NODES[:1])

    assert len(bc._physical_cameras()) == 1
    assert bc._check_cameras()[0] == bc.WARN      # 2 expected, 1 found


# ── K. models ────────────────────────────────────────────────────────────────

def test_a_hef_without_its_yaml_sidecar_fails_rather_than_warns(tmp_path):
    """A missing sidecar means an empty class allowlist, so the detector returns
    [] every frame while the pipeline looks healthy. Silent, and fatal to a run
    -- so it is a FAIL, not a warning."""
    (tmp_path / 'gate.hef').write_bytes(b'x')
    (tmp_path / 'gate.yaml').write_text('names: [gate]')
    (tmp_path / 'bin.hef').write_bytes(b'x')          # no sidecar

    status, detail = bc._check_models_hailo([str(tmp_path)])

    assert status == bc.FAIL
    assert 'bin' in detail and 'sidecar' in detail


def test_hef_with_sidecars_passes(tmp_path):
    for stem in ('gate', 'bin'):
        (tmp_path / f'{stem}.hef').write_bytes(b'x')
        (tmp_path / f'{stem}.yaml').write_text('names: []')

    assert bc._check_models_hailo([str(tmp_path)])[0] == bc.PASS


def _hefs(tmp_path, floors):
    """A model dir where every .hef has its sidecar, so the only thing under
    test is the baked floor."""
    for stem in floors:
        (tmp_path / f'{stem}.hef').write_bytes(b'x')
        (tmp_path / f'{stem}.yaml').write_text('names: [gate]')
    return [str(tmp_path)]


def test_a_baked_floor_above_the_configured_conf_fails_the_gate(
        tmp_path, monkeypatch):
    """⛔ B-58. A HEF compiles its NMS score threshold on-chip and nothing at
    runtime can go below it. `gate_rescue_repair` shipped baked at 0.200
    against a launch default of conf=0.15, so the configured threshold was a
    no-op and the faintest detections -- measured underwater p10 0.167 -- were
    discarded in silicon.

    ⚠ The backend ALREADY warned: `HailoDetector._warn_conf` fires at
    construction with a logger correctly wired from `detector_node`. It
    printed on every boot for weeks and nobody read it. A warning only a human
    scrolling a log can catch is not a gate, which is why the same fact now
    decides a pre-dive verdict.
    """
    floors = {'gate_rescue_repair': 0.20}
    monkeypatch.setattr(bc, '_baked_floor',
                        lambda p: floors[Path(p).stem])
    monkeypatch.setattr(bc, '_CONF_DEFAULT', 0.15)
    status, detail = bc._check_models_hailo(_hefs(tmp_path, floors))
    assert status == bc.FAIL
    assert '0.200' in detail and 'no-op' in detail
    assert 'hailo_compile.sh' in detail, 'a verdict must say how to fix it'


def test_one_bad_model_among_good_ones_still_fails(tmp_path, monkeypatch):
    """⭐ THE ACTUAL SHAPE OF B-58: two models were correct at 0.050 and one
    was not, which is exactly why a spot check of the others confirmed the
    intent and missed the outlier."""
    floors = {'sauvc_sim': 0.05, 'bin_fire_blood': 0.05,
              'gate_rescue_repair': 0.20}
    monkeypatch.setattr(bc, '_baked_floor',
                        lambda p: floors[Path(p).stem])
    monkeypatch.setattr(bc, '_CONF_DEFAULT', 0.15)
    status, detail = bc._check_models_hailo(_hefs(tmp_path, floors))
    assert status == bc.FAIL
    assert 'gate_rescue_repair' in detail
    assert 'sauvc_sim' not in detail, 'name the offender, not the innocent'


def test_the_floor_guard_bites_only_because_of_the_floor(
        tmp_path, monkeypatch):
    """Injection-verify. Same models, same sidecars: dropping the floor to a
    compliant value must flip the verdict, or the FAIL came from something
    else and these tests prove nothing."""
    monkeypatch.setattr(bc, '_CONF_DEFAULT', 0.15)
    dirs = _hefs(tmp_path, {'gate_rescue_repair': 0.0})
    monkeypatch.setattr(bc, '_baked_floor', lambda p: 0.20)
    assert bc._check_models_hailo(dirs)[0] == bc.FAIL
    monkeypatch.setattr(bc, '_baked_floor', lambda p: 0.05)
    assert bc._check_models_hailo(dirs)[0] == bc.PASS


def test_an_unreadable_floor_does_not_fail_the_dive(tmp_path, monkeypatch):
    """`hailortcli` may be absent. A pre-dive check that refused to pass over
    a missing diagnostic binary would get switched off, which is the real
    cost."""
    monkeypatch.setattr(bc, '_baked_floor', lambda p: None)
    monkeypatch.setattr(bc, '_CONF_DEFAULT', 0.15)
    assert bc._check_models_hailo(
        _hefs(tmp_path, {'sauvc_sim': 0}))[0] == bc.PASS


def test_the_conf_default_is_read_from_the_launch_file():
    """⛔ One truth, two copies is the bug. A literal here would keep passing
    after someone changed the launch default -- the exact drift this check
    exists to catch."""
    assert bc._launch_conf_default() == pytest.approx(0.15)


def test_pt_weights_alone_are_a_failure_on_a_hailo_vehicle(tmp_path):
    """Measured on the vehicle: 4 .pt, zero .hef. The .pt path is not the flight
    path on this platform, and reporting that as merely slow would be wrong --
    there is nothing to fly."""
    (tmp_path / 'gate.pt').write_bytes(b'x')

    status, detail = bc._check_models_hailo([str(tmp_path)])

    assert status == bc.FAIL
    assert 'NO .hef' in detail


def test_the_hailo_branch_is_selected_by_the_accelerator_not_the_hostname(
        monkeypatch):
    """A Hailo is detected by /dev/hailo* or hailortcli. Guessing from the host
    name or the ROS distro would silently pick the wrong artifact type."""
    monkeypatch.setattr(bc, 'glob', lambda pat: ['/dev/hailo0']
                        if 'hailo' in pat else [])
    assert bc._hailo_present() is True

    monkeypatch.setattr(bc, 'glob', lambda _pat: [])
    monkeypatch.setattr(bc.shutil, 'which', lambda _n: None)
    assert bc._hailo_present() is False
