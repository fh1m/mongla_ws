"""Where calibrations and captures live on disk, and which camera is which.

Camera discovery (`camera_identity`, `list_cameras`), the calibration
directory and the per-medium capture directories, and the library of
installed calibrations. `guide.py` serves these; nothing here opens a socket.
Loaded by path like the other siblings, so it must not import the package.
"""
import glob
import json
import os
import re
import subprocess
import time

try:
    from . import solver                     # the installed package
except ImportError:                          # loaded by path from guide.py (tools/)
    import importlib.util
    _spec = importlib.util.spec_from_file_location(
        'mongla_calib_solver',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'solver.py'))
    solver = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(solver)


def camera_identity(dev):
    """Who this camera actually IS -- card name and USB VID:PID.

    This is the field that would have caught the wrong-camera bug. The one
    calibration this project held was named for one camera, described a
    second, and was wired to a third, for four days, and nothing could see it
    because the file did not record what it was taken with. `applies_to`
    fixed the wiring half; this fixes the provenance half -- a calibration
    now says which physical unit produced it, so a swapped camera is
    DETECTABLE rather than merely unlucky.
    """
    out = {'device': dev}
    try:
        r = subprocess.run(['v4l2-ctl', '-d', dev, '--info'],
                           capture_output=True, text=True, timeout=5)
        for line in (r.stdout or '').splitlines():
            if 'Card type' in line:
                out['card'] = line.split(':', 1)[1].strip()
            elif 'Bus info' in line:
                out['bus'] = line.split(':', 1)[1].strip()
    except Exception:
        pass
    try:
        real = os.path.realpath(dev)
        node = os.path.basename(real)
        base = f'/sys/class/video4linux/{node}/device/../'
        for key, fn in (('usb_vid', 'idVendor'), ('usb_pid', 'idProduct'),
                        ('usb_serial', 'serial')):
            try:
                out[key] = open(base + fn).read().strip()
            except Exception:
                pass
    except Exception:
        pass
    return out


def list_cameras(cal_dir):
    """Every capture-capable camera, with the calibration it has (or lacks).

    The competition-day question is not "what is my focal length", it is
    "which of my cameras can I trust right now". This answers that in one
    place: the units present, what each one is, whether a calibration exists
    for it, and whether that calibration was taken on THIS unit.
    """
    cams = []
    for dev in sorted(glob.glob('/dev/video*'),
                      key=lambda d: int(''.join(c for c in d if c.isdigit()) or 0)):
        try:
            r = subprocess.run(['v4l2-ctl', '-d', dev, '--list-formats'],
                               capture_output=True, text=True, timeout=5)
            txt = r.stdout or ''
            # Version-robust: this v4l2-ctl prints "[0]: 'MJPG' (...)" and no
            # "Pixel Format" string at all, which an earlier check assumed --
            # it found ZERO cameras on a machine with one plugged in. Require
            # a capture type AND at least one enumerated format.
            if 'Video Capture' not in txt or not re.search(r'\[\d+\]:', txt):
                continue                     # metadata node, not a stream
        except Exception:
            continue
        ident = camera_identity(dev)
        # USB ONLY. The Pi exposes ~18 `pispbe` ISP pipeline nodes that
        # enumerate formats and are not cameras; listing them buries the two
        # devices that matter under a page of noise. A USB vendor id is the
        # thing every real camera here has and no ISP node does.
        if not ident.get('usb_vid'):
            continue
        entry = {**ident, 'calibration': None, 'match': None}
        best, best_match = None, None
        for f in sorted(glob.glob(os.path.join(cal_dir, '*.json'))):
            try:
                d = json.load(open(f))
            except Exception:
                continue
            info = {
                'file': os.path.basename(f),
                'applies_to': d.get('applies_to'),
                'captured': d.get('captured'),
                'fx': (d.get('camera_matrix') or [[None]])[0][0],
                'hfov_air': d.get('hfov_deg_air'),
                'hfov_water': d.get('hfov_deg_water'),
                'max_ere_px': d.get('max_ere_px'),
                'views': d.get('views_used'),
            }
            # Three states, not two. "confirmed" needs the USB id recorded
            # AT CAPTURE and matching now. "unverified" is a file that plausibly
            # belongs but cannot prove it -- which is every calibration taken
            # before identity was recorded, and is exactly the state the
            # wrong-camera bug lived in for four days. Reporting it as
            # "calibrated" would repeat that; reporting it as "none" would
            # throw away a good calibration. So it gets its own word.
            if d.get('usb_vid'):
                if (d['usb_vid'] == ident.get('usb_vid') and
                        d.get('usb_pid') == ident.get('usb_pid')):
                    best, best_match = info, 'confirmed'
                    break
                continue                      # identity recorded and DIFFERENT
            card = (ident.get('card') or '').split(':')[0].strip().lower()
            blob = (str(d.get('camera') or '') + ' ' +
                    ' '.join(d.get('applies_to') or [])).lower()
            if card and card in blob and best is None:
                best, best_match = info, 'unverified'
        entry['calibration'], entry['match'] = best, best_match
        cams.append(entry)
    return cams


def default_calibration_dir():
    """Where calibrations live, found robustly.

    This was wrong when installed, twice, and failed quietly both times.

    First version built `<this file>/../../src/mongla_vision/config/
    calibration`, which is right in the source tree and nonsense in
    `install/.../site-packages`. Download 404'd and the camera panel read NO
    CALIBRATION for a camera that has one.

    Second version walked upward looking for `src/mongla_vision/config/
    calibration` -- and under `ros2 run` on the vehicle it resolved to

        <ws>/build/mongla_vision/mongla_vision/src/mongla_vision/config/calibration

    an EMPTY directory **the tool had created itself** on an earlier run
    whose cwd happened to sit there. `main()` does `os.makedirs` on whatever
    this returns, so a resolver that creates the directory it searches for
    will find its own mistake forever after, and every symptom (empty
    library, 404 download, cameras showing NO CALIBRATION) points at missing
    calibrations rather than at a wrong path.

    So: **a candidate inside a generated tree is never valid.** `build/` and
    `install/` are wiped and rebuilt; a calibration written there is deleted
    by the next `colcon build` and can never be committed, which is the one
    thing this file must not allow. The source tree is the target.
    """
    def usable(cand):
        # Reject anything under a generated tree, however plausible.
        parts = os.path.abspath(cand).split(os.sep)
        return (os.path.isdir(cand)
                and 'build' not in parts and 'install' not in parts)

    here = os.path.dirname(os.path.abspath(__file__))
    starts = [os.getcwd(), here]
    # Under `ros2 run` this file lives inside <ws>/build/... or
    # <ws>/install/...; the workspace root is the parent of that, so name it
    # directly rather than hoping the upward walk survives the decoys.
    parts = here.split(os.sep)
    for gen in ('build', 'install'):
        if gen in parts:
            starts.insert(0, os.sep.join(parts[:parts.index(gen)]))
    for start in starts:
        d = start
        for _ in range(8):
            cand = os.path.join(d, 'src', 'mongla_vision', 'config',
                                'calibration')
            if usable(cand):
                return cand
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
    # Nothing found: a real directory the operator can find, never a path
    # that only looks plausible.
    fallback = os.path.expanduser('~/mongla_calibrations')
    os.makedirs(fallback, exist_ok=True)
    return fallback


def capture_dir(applies_to, medium):
    """Where this camera's frames for THIS medium live.

    The medium is part of the path, not just the filename. Switching to
    water re-execs and `resume()` picks up whatever is in `--out`. With one
    shared folder that is the AIR capture set, and Solve would install it as
    `..._water.json`: a filename saying water over pixels taken in air --
    the exact 1.44x confusion the whole medium flag exists to prevent,
    arriving through the front door.

    Same reasoning as the per-profile split that came before it: a switch
    must not resume into another capture's frames.
    """
    suffix = '' if medium == 'air' else f'_{medium}'
    return os.path.expanduser(f'~/calib_{applies_to}{suffix}')


def library(cal_dir):
    """Every calibration this vehicle holds, with what it is good for.

    A calibration costs an operator ten minutes and a board. Keeping them all
    and being able to re-apply one is the difference between swapping a
    camera in two minutes and recalibrating on competition ground.
    """
    out = []
    for f in sorted(glob.glob(os.path.join(cal_dir, '*.json'))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        K = d.get('camera_matrix') or [[None]]
        out.append({
            'file': os.path.basename(f),
            'applies_to': d.get('applies_to'),
            'medium': d.get('medium', 'air'),
            'captured': d.get('captured'),
            'camera': d.get('camera'),
            'usb_vid': d.get('usb_vid'), 'usb_pid': d.get('usb_pid'),
            'width': d.get('image_width'), 'height': d.get('image_height'),
            'fx': K[0][0],
            'hfov_air': d.get('hfov_deg_air'),
            'hfov_water': d.get('hfov_deg_water'),
            'views': d.get('views_used'),
            'max_ere_px': d.get('max_ere_px'),
            'holdout_rms_px': d.get('holdout_rms_px'),
        })
    return out


def apply_calibration(cal_dir, src_file, dev, applies_to):
    """Bind an existing calibration to a connected camera.

    Applying is the operator asserting "this is the same physical unit",
    and the file must say so rather than pretend it was measured here. So the
    connected camera's identity is written in ALONGSIDE
    `identity_source: operator asserted at apply time` -- never as though it
    had been captured on this unit. The difference matters the day someone
    asks why a calibration claims a camera it never saw.

    Refuses to apply across MEDIA. An in-water calibration standing in for an
    in-air one is a ~1.44x scale error on every range and velocity, which is
    both silent and exactly the size that still looks plausible.
    """
    # A bad request must come back as a REFUSAL, not as a dropped
    # connection. The first live test of this route passed a file that was
    # not there and got an unhandled FileNotFoundError inside do_POST, which
    # the browser sees as the page hanging -- a failure mode indis-
    # tinguishable from the tool being broken.
    src = os.path.join(cal_dir, os.path.basename(src_file or ''))
    try:
        d = json.load(open(src))
    except Exception as e:
        return None, f'{src_file!r}: {e}'
    medium = d.get('medium', 'air')
    ident = camera_identity(dev)
    if not ident.get('usb_vid'):
        return None, f'{dev}: no USB identity, refusing to bind blindly'
    if not applies_to:
        return None, 'no camera profile given'
    d = dict(d)
    d['applies_to'] = [applies_to]
    # State the medium EXPLICITLY even when the source predates the field.
    # A file that merely omits it is read as air by convention, and a
    # convention is not evidence.
    d['medium'] = medium
    d.update({k: v for k, v in ident.items()
              if k in ('usb_vid', 'usb_pid', 'usb_serial', 'card', 'bus')})
    d['identity_source'] = 'operator asserted at apply time'
    d['applied_from'] = os.path.basename(src_file)
    d['applied_on'] = time.strftime('%Y-%m-%d %H:%M')
    dest = os.path.join(cal_dir, solver.calibration_filename(
        applies_to, d.get('image_width'), d.get('image_height'), medium))
    if os.path.exists(dest) and os.path.abspath(dest) != os.path.abspath(src):
        # Never overwrite a calibration without keeping the old one. It cost
        # somebody a board and ten minutes.
        os.replace(dest, dest + '.bak-' + time.strftime('%H%M%S'))
    json.dump(d, open(dest, 'w'), indent=2)
    return os.path.basename(dest), None
