"""Bringup checks of the computer itself: ROS, disk, drivers, cameras, models, the Hailo, power.

Each returns (grade, message) and opens no serial port.
"""
from __future__ import annotations
import os
import pathlib
import re
import shutil
import subprocess
from glob import glob
from .bringup_grade import FAIL, PASS, WARN


# --------------------------------------------------------------------------- #
# low-level probes
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
# A. compute / environment
# --------------------------------------------------------------------------- #
def _check_ros_env() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []
    distro = os.environ.get('ROS_DISTRO', '')
    if distro:
        out.append((PASS, 'ROS distro', distro))
    else:
        out.append((FAIL, 'ROS_DISTRO unset',
                    'source /opt/ros/humble/setup.bash + install/setup.bash'))
    dom = os.environ.get('ROS_DOMAIN_ID', '(unset -> 0)')
    out.append((PASS, 'ROS_DOMAIN_ID', str(dom)))

    # The three packages the mission path imports. A broken build shows here
    # BEFORE `start` fails with a confusing traceback.
    for pkg in ('mongla_control', 'mongla_interfaces', 'mongla_planner'):
        try:
            __import__(pkg)
            out.append((PASS, f'import {pkg}', 'ok'))
        except Exception as exc:  # noqa: BLE001 - report, never raise
            out.append((FAIL, f'import {pkg}',
                        f'{type(exc).__name__}: {str(exc).splitlines()[0]} '
                        '-- rebuild: ./build_mongla.sh'))
    return out


# Graded here and by the manager's live health report (kutum_health.py), so the
# preflight and the run cannot disagree about the same disk.
DISK_FAIL_GB = 1.0
DISK_WARN_GB = 3.0


def read_disk_free_gb(path: str = '/') -> float:
    return shutil.disk_usage(path).free / 1e9


def _disk_verdict(free_gb: float) -> tuple[str, str]:
    if free_gb < DISK_FAIL_GB:
        return FAIL, f'{free_gb:.1f} GB free -- logging/models will fail'
    if free_gb < DISK_WARN_GB:
        return WARN, f'{free_gb:.1f} GB free -- low for bag/log capture'
    return PASS, f'{free_gb:.1f} GB free'


def _check_disk() -> tuple[str, str]:
    try:
        free_gb = read_disk_free_gb()
    except OSError as exc:
        return WARN, f'could not stat /: {exc}'
    return _disk_verdict(free_gb)


# --------------------------------------------------------------------------- #
# B. vision dependencies (pi-and-env-traps.md E1-E3)
# --------------------------------------------------------------------------- #
def _check_vision_deps() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []

    # numpy<2 -- numpy 2 breaks the cv_bridge / cv2 ABI => every vision node
    # dies with `_ARRAY_API not found` (pi-and-env-traps.md E1).
    try:
        import numpy
        ver = numpy.__version__
        major = int(ver.split('.')[0])
        if major >= 2:
            out.append((FAIL, 'numpy version', f'{ver} -- MUST be <2 '
                        '(numpy 2 kills every vision node: _ARRAY_API). '
                        'pip install "numpy<2" (1.26.4)'))
        else:
            out.append((PASS, 'numpy version', f'{ver} (<2, ABI ok)'))
    except Exception as exc:  # noqa: BLE001
        out.append((WARN, 'numpy import', f'{exc}'))

    # cv2 must be the SYSTEM build (GUI-capable); a pip opencv-python shadows
    # it headless and kills vision_display (pi-and-env-traps.md E2).
    try:
        import cv2
        path = getattr(cv2, '__file__', '')
        if 'dist-packages' in path or '/usr/lib' in path:
            out.append((PASS, 'cv2 (system)', f'{cv2.__version__}'))
        else:
            out.append((WARN, 'cv2 not system build',
                        f'{cv2.__version__} @ {path} -- pip opencv may shadow '
                        'the GUI build; uninstall pip opencv-python*'))
    except Exception as exc:  # noqa: BLE001
        out.append((FAIL, 'cv2 import', f'{exc} -- vision cannot run'))

    # pip opencv-python shadow detector (independent of the import path check).
    try:
        import importlib.metadata as _md
        shadows = [d for d in ('opencv-python', 'opencv-contrib-python',
                               'opencv-python-headless')
                   if _pkg_present(_md, d)]
        if shadows:
            out.append((WARN, 'pip opencv installed',
                        f'{", ".join(shadows)} -- shadows system cv2; '
                        'pip uninstall them (pi-and-env-traps.md E2)'))
    except Exception:
        pass

    # trackers 2.5.0 PyPI wheel is a broken dud; 2.4.0 is the good one (E3).
    try:
        import importlib.metadata as _md
        tver = _md.version('trackers')
        if tver.startswith('2.5'):
            out.append((WARN, 'trackers version',
                        f'{tver} -- the 2.5.0 wheel is a broken 9.7 kB dud; '
                        'pip install "trackers==2.4.0" --no-deps (E3)'))
        else:
            out.append((PASS, 'trackers version', tver))
    except Exception:
        out.append((WARN, 'trackers not installed',
                    'OC-SORT/ByteTrack HUD unavailable; '
                    'pip install "trackers==2.4.0" --no-deps'))
    return out


def _pkg_present(md, name: str) -> bool:
    try:
        md.version(name)
        return True
    except Exception:
        return False


# --------------------------------------------------------------------------- #
# C. serial drivers (the board's CH340 + brltty trap, pi-and-env-traps.md E5)
# --------------------------------------------------------------------------- #
def _check_serial_drivers() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []

    # ch341 must exist for the SROT board's CH340 to get a /dev/ttyUSB* node.
    # A fresh kernel can ship without it.
    have_ch341 = subprocess.run(['modinfo', 'ch341'],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL).returncode == 0 \
        if shutil.which('modinfo') else None
    if have_ch341 is None:
        out.append((PASS, 'ch341 driver', 'modinfo absent -- skipped'))
    elif have_ch341:
        out.append((PASS, 'ch341 driver', 'available (the board can enumerate)'))
    else:
        out.append((WARN, 'ch341 driver MISSING',
                    'the SROT board will have no /dev/ttyUSB* -- this kernel '
                    'lacks ch341 (pi-and-env-traps.md E5)'))

    # brltty greedily grabs the CH340 (1a86:7523) as a braille display.
    try:
        installed = subprocess.run(['dpkg', '-l', 'brltty'],
                                   capture_output=True, text=True).stdout
        if '\nii  brltty' in installed or installed.startswith('ii  brltty'):
            out.append((WARN, 'brltty installed',
                        'steals the SROT board\'s CH340 via usbfs -- '
                        'sudo apt-get purge -y brltty (pi-and-env-traps.md E5)'))
        else:
            out.append((PASS, 'brltty absent', 'the board\'s CH340 is not hijacked'))
    except (FileNotFoundError, OSError):
        pass  # not a dpkg system
    return out


# --------------------------------------------------------------------------- #
# J. cameras
# --------------------------------------------------------------------------- #
def _physical_cameras() -> list[str]:
    """Distinct USB video capture devices, keyed by stable by-path port.

    A single UVC camera exposes several /dev/videoN nodes (capture + metadata);
    counting by-path *ports* (strip the interface + -video-indexN suffix)
    collapses those to one entry per physical camera and is reboot-stable.
    """
    ports: dict[str, str] = {}
    for link in sorted(glob('/dev/v4l/by-path/*-video-index0')):
        base = os.path.basename(link)
        # ONLY USB. A Raspberry Pi exposes its hardware video codec and its
        # ISP backend as by-path video nodes that exist whether or not a camera
        # is plugged in -- `platform-1000800000.codec-video-index0` and
        # `platform-1000880000.pisp_be-video-index0`. Measured on the vehicle
        # 2026-09-22 with BOTH cameras physically removed: this check reported
        # "[PASS] 2 USB camera(s)". A pre-flight gate that passes on a hull with
        # no cameras is worse than no gate.
        if '-usb-' not in base:
            continue
        key = base.split('-video-index')[0]      # includes :1.x interface
        port = key.rsplit(':', 1)[0]              # drop interface -> USB port
        ports.setdefault(port, base)
    return list(ports.values())


def _check_cameras() -> tuple[str, str]:
    cams = _physical_cameras()
    n = len(cams)
    if n == 0:
        plat = [os.path.basename(l) for l in
                sorted(glob('/dev/v4l/by-path/*-video-index0'))]
        if plat:
            return WARN, (f'NO USB cameras. The {len(plat)} /dev/video* by-path '
                          f'node(s) present are the platform codec/ISP, not '
                          f'capture devices -- cameras unplugged?')
        raw = sorted(glob('/dev/video*'))
        if raw:
            return WARN, (f'{len(raw)} /dev/video* node(s) but no by-path link '
                          '-- non-UVC or udev issue; run v4l2-ctl --list-devices')
        return WARN, 'no /dev/video* devices -- cameras unplugged?'
    detail = f'{n} USB camera(s): ' + ', '.join(
        p.split('.usb-')[-1] if '.usb-' in p else p for p in cams)
    if n >= 2:
        return PASS, detail
    return WARN, detail + '  (2 expected: forward + downward)'


# --------------------------------------------------------------------------- #
# K. vision models (Hailo .hef + sidecar)
# --------------------------------------------------------------------------- #
def _models_dirs() -> list[str]:
    """All plausible model dirs (src dev tree + installed share), existing only.

    The .pt weights are gitignored and usually live in the src tree; the
    installed share/ copy is often empty. Scan both and merge.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands: list[str] = []
    # Walk up from this file (installed under .../mongla_ws/install/...) and
    # from the cwd looking for a workspace root that has src/mongla_vision/models.
    for start in (here, os.getcwd()):
        d = start
        for _ in range(8):
            probe = os.path.join(d, 'src', 'mongla_vision', 'models')
            if os.path.isdir(probe):
                cands.append(probe)
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory('mongla_vision'), 'models'))
    except Exception:
        pass
    seen: list[str] = []
    for c in cands:
        rp = os.path.realpath(c)
        if os.path.isdir(rp) and rp not in seen:
            seen.append(rp)
    return seen


def _hailo_present() -> bool:
    """Is a Hailo accelerator on this machine?"""
    if glob('/dev/hailo*'):
        return True
    return shutil.which('hailortcli') is not None


def _check_models() -> tuple[str, str]:
    dirs = _models_dirs()
    if not dirs:
        return WARN, 'models dir not found -- vision detection disabled'
    if _hailo_present():
        return _check_models_hailo(dirs)
    # A dev box: the .pt path replays footage, it is not the vehicle path.
    stems = sorted({os.path.splitext(os.path.basename(p))[0]
                    for d in dirs for p in glob(os.path.join(d, '*.pt'))})
    if not stems:
        return WARN, (f'no Hailo here and no .pt weights in {", ".join(dirs)} '
                      '-- vision detection disabled')
    return WARN, (f'no Hailo accelerator on this host: {len(stems)} .pt model(s) '
                  'will run on CPU/CUDA (~3-4 Hz on CPU). Fine for footage '
                  'replay; the vehicle runs the .hef')


def _check_sensor_offsets() -> tuple[str, str]:
    """Are the sensors' positions relative to the IMU measured? (#19, #27)

    Warn, never fail. An unmeasured offset leaves its correction OFF -- the
    behaviour before the correction existed -- and the vehicle flies as it
    always has. This line exists so the cost is SEEN at the pool, where the
    only fix (a tape measure) is available, rather than inferred later from a
    sideways drift on every turn. The same verdict the manager's
    `sensor_offsets` health line gives, from the same file and the same words.
    """
    try:
        from mongla_localization import frames
        from mongla_manager import health_reporters as hr
        h = hr.sensor_offsets(frames.load())
    except Exception as exc:                        # noqa: BLE001
        return WARN, f'frames.yaml unreadable: {exc!r}'
    return (PASS if h.ok else WARN), h.evidence


def depth_floor_verdict(nodes: dict) -> tuple[str, str]:
    """PASS / WARN for the manager's depth floor, from the deck file's `nodes:`."""
    raw = (nodes.get('mongla_manager') or {}).get('shield_max_depth_m', 0.0)
    try:
        floor = float(raw)
    except (TypeError, ValueError):
        return WARN, f'shield_max_depth_m is {raw!r}, not a number'
    if floor > 0.0:
        return PASS, f'the manager surfaces a hull past {floor:.2f} m'
    return WARN, ('no depth floor: set shield_max_depth_m under nodes: '
                  'mongla_manager: in ~/.mongla/bringup.yaml, to the depth '
                  'the hull must never pass in this venue')


def _check_depth_floor() -> tuple[str, str]:
    """Is there a depth floor below the mission? (#52)

    Warn, never fail: a bench has no floor to set. The board has none of its
    own until upstream ask Y lands, so on the deck this line is the only
    place an unset floor is seen before the hull is wet.
    """
    try:
        from mongla_localization import launch_defaults as ld
        path = ld.config_path()
        nodes = ld.read_nodes(path) if path is not None else {}
    except Exception as exc:                        # noqa: BLE001
        return WARN, f'bringup.yaml unreadable: {exc!r}'
    return depth_floor_verdict(nodes)


def _check_model_separation() -> tuple[str, str]:
    """Can the model you are about to fly TELL THE PROP FROM THE WATER?

    The defect this exists for. Every other check in this file asks whether
    a model LOADS. None asked whether it WORKS, and for eight months the
    vehicle's default forward graph was one that claims a gate on 90.8 % of
    gate-free frames -- +0.0 points of separation between gate-present and
    gate-absent footage at every bar measured (B-59). It loaded perfectly. It
    ran at full rate. Every check passed.

    A graph that hallucinates is worse than no graph: the mission acts on it.

    The measurement lives in the sidecar, beside the weights it describes,
    so it cannot be separated from them by a copy or a rename. Written by
    `tools/studies/negative_clip_check.py`; `recommended_conf: null` means MEASURED AND
    FOUND UNUSABLE for that class, which is different from absent.

    A model with no `separation:` block is unknown, not bad. Most graphs have
    not been through the tool yet, and reporting them as failures would train
    the operator to ignore this line. Unknown is a WARN naming the tool.
    """
    dirs = _models_dirs()
    if not dirs:
        return WARN, 'models dir not found -- cannot check separation'
    try:
        import yaml
    except Exception:                                            # noqa: BLE001
        return WARN, 'pyyaml missing -- cannot read model sidecars'

    refused, unmeasured, good = [], [], []
    seen = set()
    for d in dirs:
        for p_ in sorted(glob(os.path.join(d, '*.yaml'))):
            stem = os.path.splitext(os.path.basename(p_))[0]
            if stem in seen:
                continue
            seen.add(stem)
            try:
                doc = yaml.safe_load(open(p_)) or {}
            except Exception:                                    # noqa: BLE001
                continue
            sep = doc.get('separation')
            if not isinstance(sep, dict):
                unmeasured.append(stem)
                continue
            for cls, row in sep.items():
                if not isinstance(row, dict) or row.get('measured') is None:
                    continue
                if row.get('recommended_conf') is None:
                    refused.append(f'{stem}:{cls}')
                else:
                    good.append(f'{stem}:{cls}@{row["recommended_conf"]}')
    if refused:
        return FAIL, (
            f'MEASURED AND UNUSABLE for {", ".join(refused)} -- the sidecar '
            f'records separation and says do not fly this class on this '
            f'graph. Usable: {", ".join(good) if good else "NONE"}. Run both '
            f'with fwd_models:=<good>,<other> and take the class from the '
            f'good one.')
    if good:
        return PASS, (f'{len(good)} class(es) with a measured bar: '
                      f'{", ".join(good)}'
                      + (f'; {len(unmeasured)} graph(s) unmeasured'
                         if unmeasured else ''))
    return WARN, ('no model carries a measured separation -- a graph that '
                  'loads is not a graph that works. '
                  'python3 tools/studies/negative_clip_check.py --model <m> '
                  '--class <c> --positive <clip> --negative <clip>')


def _check_build_freshness() -> tuple[str, str]:
    """Is the vehicle running the code that is in `src/`?

    It usually is not, and nothing said so. `mongla_vision` resolves to
    `build/mongla_vision/mongla_vision/...`, not `src/`. On 2026-09-25 the
    B-61 fix was rsynced into `src/`, verified by a probe that inserts `src/`
    on `sys.path`, and reported as "verified on the vehicle" -- while the
    flight path was still importing the broken copy out of `build/`. The
    rsync proved nothing.

    That is the same shape as every §9 defect: the code was correct, the test
    passed, and the vehicle was running something else.

    A source file NEWER than its build copy is the signal, not file
    contents: `--symlink-install` makes many packages share an inode, in
    which case the mtimes match and this passes trivially, which is correct --
    a symlinked package cannot go stale.
    """
    ws = pathlib.Path(__file__).resolve().parents[3]
    src, build = ws / 'src', ws / 'build'
    if not build.is_dir():
        return PASS, 'no build/ tree -- running from source'
    stale = []
    for s in src.rglob('*.py'):
        if '/test/' in str(s) or '/launch/' in str(s):
            continue
        rel = s.relative_to(src)
        pkg = rel.parts[0]
        b = build / pkg / pathlib.Path(*rel.parts[1:])
        if not b.exists():
            continue
        try:
            if s.stat().st_mtime > b.stat().st_mtime + 1.0:
                stale.append(str(rel))
        except OSError:
            continue
    if stale:
        shown = ', '.join(sorted(stale)[:4])
        more = f' (+{len(stale) - 4} more)' if len(stale) > 4 else ''
        pkgs = sorted({p.split('/')[0] for p in stale})
        return FAIL, (
            f'{len(stale)} source file(s) NEWER than the build copy the '
            f'vehicle imports: {shown}{more}. The running code is not your '
            f'code. Rebuild: colcon build --packages-select '
            f'{" ".join(pkgs)} --symlink-install')
    return PASS, 'build tree matches src'


# Stock Model Zoo builds. Not ours to recompile, not flown: `yolov11n` is
# documented in bringup.launch.py as "ROBOSUB-tested pretrained, sim/bench".
# Its 0.200 floor is correct for what it is, and failing a dive over it is how
# a gate gets ignored.
_STOCK_MODELS = ('yolov11n', 'yolov8n', 'yolo26_nano_pretrained')


def _baked_floor(hef_path: str) -> 'float | None':
    """The NMS score threshold compiled into a HEF, or None if unreadable.

    Shells out to `hailortcli parse-hef` because there is no Python API for
    it: `HEF` exposes stream infos and nothing about the post-process. Best
    effort -- a pre-dive check that crashed because a diagnostic binary was
    missing would be worse than one that reports what it could read.
    """
    try:
        out = subprocess.run(['hailortcli', 'parse-hef', hef_path],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:                                            # noqa: BLE001
        return None
    m = re.search(r'Score threshold:\s*([0-9.]+)', out)
    return float(m.group(1)) if m else None


def _launch_conf_default() -> float:
    """Read the shipped `conf` default out of the launch file.

    NOT a literal here. One truth, two copies is the bug: a hardcoded 0.15
    would keep passing after someone changed the launch default, which is
    exactly the drift this check exists to catch.
    """
    try:
        import mongla_manager
        p = (pathlib.Path(mongla_manager.__file__).resolve().parents[1]
             / 'launch' / 'bringup.launch.py')
        if not p.exists():                       # installed share/ layout
            from ament_index_python.packages import get_package_share_directory
            p = (pathlib.Path(get_package_share_directory('mongla_manager'))
                 / 'launch' / 'bringup.launch.py')
        m = re.search(r"DeclareLaunchArgument\(\s*'conf'\s*,\s*"
                      r"default_value='([0-9.]+)'", p.read_text())
        if m:
            return float(m.group(1))
    except Exception:                                            # noqa: BLE001
        pass
    return 0.15


_CONF_DEFAULT = _launch_conf_default()


def _check_models_hailo(dirs: list[str]) -> tuple[str, str]:
    """On Hailo the accelerated artifact is a `.hef`, compiled offline.

    A `.hef` without its `<stem>.yaml` sidecar is worse than no model. The
    sidecar carries the class allowlist; missing, the allowlist is empty and the
    detector returns `[]` every frame while the whole pipeline looks healthy.
    That is a silent failure, so it is reported as a FAIL rather than a warning.
    """
    hefs: dict[str, bool] = {}
    # A link to nothing is not a model (2026-10-03, on the vehicle). A
    # symlink-install tree keeps links to models deleted from source; counted
    # as engines they made a retired model fail this gate while nothing could
    # load it. They are named instead, so the stale tree is visible.
    dangling = sorted({os.path.basename(p) for d in dirs
                       for p in glob(os.path.join(d, '*'))
                       if os.path.islink(p) and not os.path.exists(p)})
    for d in dirs:
        for p in glob(os.path.join(d, '*.hef')):
            if not os.path.exists(p):
                continue
            stem = os.path.splitext(os.path.basename(p))[0]
            sidecar = os.path.exists(os.path.join(d, stem + '.yaml'))
            hefs[stem] = hefs.get(stem, False) or sidecar
    if not hefs:
        pts = {os.path.splitext(os.path.basename(p))[0]
               for d in dirs for p in glob(os.path.join(d, '*.pt'))}
        if pts:
            return FAIL, (f'Hailo present but NO .hef in {", ".join(dirs)} '
                          f'({len(pts)} .pt found). The .pt path is not the '
                          f'flight path on this platform -- compile the models')
        return WARN, f'no .hef and no .pt in {", ".join(dirs)} -- vision disabled'
    orphans = [s for s, ok in hefs.items() if not ok]
    if orphans:
        return FAIL, (f'{len(orphans)}/{len(hefs)} .hef have NO <stem>.yaml '
                      f'sidecar ({", ".join(orphans)}) -- the class allowlist '
                      f'is empty, so the detector returns [] every frame while '
                      f'looking healthy')
    # The baked NMS floor vs THE CONFIGURED conf. A HEF compiles its NMS
    # score threshold on-chip; nothing at runtime can go below it. The Hailo
    # backend already WARNS about this at construction -- and that warning
    # fired on every boot for `gate_rescue_repair`, baked at 0.200 against a
    # launch default of 0.15, and nobody read it (B-58). A warning that only a
    # human scrolling a log can catch is not a gate, so it is GRADED here.
    floors = {}
    for d in dirs:
        for p in glob(os.path.join(d, '*.hef')):
            if not os.path.exists(p):
                continue
            th = _baked_floor(p)
            if th is not None:
                floors[os.path.splitext(os.path.basename(p))[0]] = th
    # Stock models are not ours to recompile, and failing a dive over one is
    # how a gate gets ignored. `yolov11n` is Hailo's own Model Zoo build,
    # documented in bringup.launch.py as "ROBOSUB-tested pretrained,
    # sim/bench" -- it is not a flight model, and its 0.200 floor is correct
    # for what it is. Ours are the ones that must be compiled at 0.05.
    high = {s: t for s, t in floors.items()
            if t > _CONF_DEFAULT + 1e-6 and s not in _STOCK_MODELS}
    stock_high = {s: t for s, t in floors.items()
                  if t > _CONF_DEFAULT + 1e-6 and s in _STOCK_MODELS}
    if high:
        worst = ', '.join(f'{s} {t:.3f}' for s, t in sorted(high.items()))
        return FAIL, (
            f'{len(high)}/{len(floors)} .hef bake an NMS floor ABOVE the '
            f'configured conf {_CONF_DEFAULT:.2f} ({worst}) -- everything '
            f'below the bake was discarded on-chip and no runtime value '
            f'brings it back, so conf is a no-op on those models. Recompile: '
            f'tools/bench/hailo_compile.sh <model> --nms-score-th 0.05')
    seen = (f', floors {min(floors.values()):.3f}-{max(floors.values()):.3f}'
            if floors else '')
    if stock_high:
        names = ', '.join(f'{s} {v:.3f}' for s, v in sorted(stock_high.items()))
        return WARN, (
            f'{len(hefs)} Hailo model(s), each with its .yaml sidecar{seen}. '
            f'stock build(s) bake a floor above conf {_CONF_DEFAULT:.2f} '
            f'({names}) -- correct for a Model Zoo model and not flown, so a '
            f'note rather than a gate. Every model of OURS is at or below the '
            f'configured conf.')
    if dangling:
        return WARN, (f'{len(hefs)} Hailo model(s), each with its .yaml '
                      f'sidecar{seen}; {len(dangling)} DANGLING link(s) to '
                      f'deleted files ({", ".join(dangling)}) -- a stale '
                      f'symlink-install tree; rebuild with build_mongla.sh, '
                      f'which prunes them')
    return PASS, (f'{len(hefs)} Hailo model(s), each with its .yaml '
                  f'sidecar{seen}')


# --------------------------------------------------------------------------- #
# L. Raspberry Pi power / throttling
# --------------------------------------------------------------------------- #
# `vcgencmd get_throttled` is a bitmask. The LOW half is "happening RIGHT NOW",
# the HIGH half is "has happened since boot" -- and the sticky half is the one
# that matters at a preflight, because a brown-out that already cost you frames
# clears itself the moment the load drops. Bit meanings are the firmware's:
_THROTTLE_BITS = {
    0:  ('under-voltage', True),
    1:  ('ARM frequency capped', True),
    2:  ('currently throttled', True),
    3:  ('soft temperature limit', False),
    16: ('under-voltage HAS OCCURRED', True),
    17: ('ARM frequency cap HAS OCCURRED', True),
    18: ('throttling HAS OCCURRED', True),
    19: ('soft temperature limit HAS OCCURRED', False),
}


def _throttled_verdict(raw: int) -> tuple[str, str]:
    """Grade a `get_throttled` bitmask. Pure, so it is testable without a Pi.

    A clean reading is not a certificate. The load that matters -- eight
    thrusters drawing while both cameras and the Hailo run -- cannot be applied
    on a bench, and the sticky bits only record what has ALREADY happened. This
    reports; it does not clear the hull for water.
    """
    if raw == 0:
        return PASS, ('0x0 -- no under-voltage or throttling since boot. NOTE: '
                      'the thrusters were not drawing; this is not a load test')
    now = [n for b, (n, _) in _THROTTLE_BITS.items() if b < 16 and raw >> b & 1]
    past = [n for b, (n, _) in _THROTTLE_BITS.items() if b >= 16 and raw >> b & 1]
    serious = any(sev for b, (_, sev) in _THROTTLE_BITS.items() if raw >> b & 1)
    what = '; '.join(now + past) or f'unknown bits in {raw:#x}'
    if now:
        return FAIL, (f'{raw:#x} ACTIVE NOW: {what} -- the PSU or the cable '
                      f'cannot hold 5 V under this load. Fix before water: a '
                      f'brown-out mid-run costs the run')
    return (WARN if serious else PASS), (
        f'{raw:#x} since boot: {what} -- not happening now, but it HAS. '
        f'Suspect the supply or the USB-C cable under load')


def read_throttled() -> int:
    """The `get_throttled` bitmask. Raises FileNotFoundError off a Pi and
    ValueError on output it cannot read; up to 3 s, so never on an executor."""
    out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                         text=True, timeout=3).stdout.strip()
    if '=' not in out:
        raise ValueError(f'could not parse `vcgencmd get_throttled` output: {out[:60]!r}')
    try:
        return int(out.split('=', 1)[1].strip(), 0)
    except ValueError:
        raise ValueError(f'non-numeric throttle value: {out[:60]!r}') from None


def _check_pi_power() -> tuple[str, str]:
    try:
        raw = read_throttled()
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return PASS, 'not a Raspberry Pi (vcgencmd absent) -- skipped'
    except ValueError as exc:
        return WARN, str(exc)
    return _throttled_verdict(raw)
