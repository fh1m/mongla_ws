"""Which calibration belongs to which camera -- resolved ONCE, from the data.

Why this is a module and not a line in each launch file.

`vision_pi.launch.py` wires calibrations by passing an explicit path per
camera. `vision.launch.py` -- the launch `bringup.launch.py` includes, i.e.
the one an operator is told to run -- passed nothing, so `camera_node`
published CameraInfo with `k` all zero on the mission path. The obvious fix
was to copy the wiring into the second launch file. That would have put the
camera-to-calibration map in three places (`vision`, `vision_dual`,
`vision_pi`) and left the fourth one to be discovered later, which is the
defect this package has already produced four times (`device_path` into
`**_`, the unloaded YAML profile table, `ros2 param set` on
construction-time params, and the calibration binding itself).

Each calibration ALREADY declares the camera it describes, in its own
`applies_to`, and `camera_node` ALREADY knows its own `profile`. So the
binding is derivable and belongs nowhere but here: every launch file, and
every future one, gets it by construction.

An explicit `calibration` param still wins -- `vision_pi.launch.py` keeps
working unchanged, and an operator can always point at a file by hand.

Note on why `forward` finds nothing and that is correct: `forward` and
`pi_forward` are DIFFERENT PHYSICAL CAMERAS (a generic USB role vs
the Fantech on the Pi). We hold calibrations for the two Pi units only, so
`camera:=forward` resolves to '' and says so. Binding it to the Pi file
because the names look alike is exactly the round-38 defect
(`test_calibration_binding.py`) in a new costume.
"""
import glob
import json
import os

_SUBDIR = os.path.join('config', 'calibration')


def _candidate_dirs():
    """The deck first, then the installed share, then the source tree.

    The deck (`~/mongla/deck/calibration`) is where a fit made on the vehicle
    is written; it outlives releases, and searched first it is never shadowed
    by the shipped file of the same name (bead edm).


    The share and the source tree both, because the node runs from the install tree on the vehicle and the
    tests read the source tree -- and a resolver that works in only one of
    those is how a config comes to reach nothing.
    """
    dirs = []
    deck = os.path.expanduser('~/mongla/deck/calibration')
    if os.path.isdir(deck):
        dirs.append(deck)
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(
            get_package_share_directory('mongla_vision'), _SUBDIR))
    except Exception:
        pass
    # mongla_vision/calibration/binding.py -> mongla_vision/ -> package root
    dirs.append(os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), _SUBDIR))
    return dirs


def calibration_for_profile(profile: str) -> str:
    """Path of the calibration whose `applies_to` names `profile`, or ''.

    Empty rather than a guess: `camera_node` treats '' as "no calibration"
    and publishes size-only CameraInfo with a warning, which is honest. A
    wrong calibration is worse than none -- it produces confident bearings
    that are wrong by a fixed factor and nothing logs a fault.
    """
    profile = (profile or '').strip()
    if not profile:
        return ''
    for d in _candidate_dirs():
        for path in sorted(glob.glob(os.path.join(d, '*.json'))):
            try:
                with open(path) as fh:
                    c = json.load(fh)
                applies = c.get('applies_to') or []
            except Exception:
                continue          # a malformed file must not hide a good one
            # The published K is the in-AIR lens: every consumer applies the
            # port itself, so a water fit (port already inside) would be
            # refracted twice.
            if c.get('medium', 'air') != 'air':
                continue
            if isinstance(applies, list) and profile in applies:
                return path
    return ''


def device_usb_id(device) -> tuple:
    """(vendor, product) of the USB camera behind `device`, or ('', '').

    Read from sysfs, so it names the unit actually plugged in, not the
    one a symlink was meant for. '' when it cannot be told (a file, a topic,
    a non-USB device): unknown, never a guess.
    """
    try:
        dev = f'/dev/video{int(device)}' if str(device).isdigit() else str(device)
        name = os.path.basename(os.path.realpath(dev))
        d = os.path.realpath(f'/sys/class/video4linux/{name}/device')
        for _ in range(6):
            v, p = os.path.join(d, 'idVendor'), os.path.join(d, 'idProduct')
            if os.path.isfile(v) and os.path.isfile(p):
                return (open(v).read().strip().lower(), open(p).read().strip().lower())
            d = os.path.dirname(d)
    except Exception:
        pass
    return ('', '')


def pairing(cal: dict, device_id: tuple) -> str:
    """Does this calibration belong to the unit plugged in?

    'match'      the USB id recorded at capture is the connected one;
    'different'  it was recorded on ANOTHER unit: not to be used;
    'unverified' the file recorded no id (taken before ids were kept);
    'unknown'    the connected unit's id cannot be read.
    """
    vid, pid = (str(cal.get('usb_vid') or '').lower(), str(cal.get('usb_pid') or '').lower())
    if not device_id or not device_id[0]:
        return 'unknown'
    if not vid:
        return 'unverified'
    return 'match' if (vid, pid) == tuple(device_id) else 'different'

