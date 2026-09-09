"""Which calibration belongs to which camera -- resolved ONCE, from the data.

⛔ WHY THIS IS A MODULE AND NOT A LINE IN EACH LAUNCH FILE.

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

NOTE ON WHY `forward` FINDS NOTHING AND THAT IS CORRECT: `forward` and
`pi_forward` are DIFFERENT PHYSICAL CAMERAS (Blue Robotics on the Jetson vs
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
    """Installed share first, then the source tree.

    Both, because the node runs from the install tree on the vehicle and the
    tests read the source tree -- and a resolver that works in only one of
    those is how a config comes to reach nothing.
    """
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(
            get_package_share_directory('duburi_vision'), _SUBDIR))
    except Exception:
        pass
    # duburi_vision/calibration/binding.py -> duburi_vision/ -> package root
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
                    applies = json.load(fh).get('applies_to') or []
            except Exception:
                continue          # a malformed file must not hide a good one
            if isinstance(applies, list) and profile in applies:
                return path
    return ''
