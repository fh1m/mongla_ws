"""Where each tool is, relative to the camera that aims it.

`config/tool_geometry.yaml` holds the numbers and the sign convention; this
resolves a tool name to an offset in metres and to the pixel correction that
puts the TARGET ON THE TOOL'S AXIS instead of the camera's.

⛔ THE DEFECT THIS EXISTS FOR. Every vision align centres the target on the
CAMERA axis and then actuates a device mounted somewhere else. The axes are
parallel, so **the miss equals the offset at every range** -- it does not shrink
as you close in. A 10 cm offset misses a 4.75 cm-radius torpedo opening from
1 m and from 3 m alike, and nothing logged a fault because from the camera's
point of view the shot was perfectly centred.

⛔ WHY THIS IS A TABLE AND NOT AN OPERATOR PARAMETER -- the `target_width_m`
lesson, which shipped a metric path that had never once run because the number
defaulted to 0.0. A quantity that must be typed correctly, per mission, under
pressure, for an answer whose wrongness is invisible is not a parameter. Same
reasoning, same shape, same file layout.

⛔ THE CORRECTION IS RANGE-DEPENDENT, which is the part that makes this more
than a constant. To sit on the tool's axis the target must appear at

    u = cx + fx * x / Z        v = cy + fy * y / Z

so the pixel shift is LARGE up close and small far away -- exactly inverted
from the intuition that says a fixed mount is a fixed error. `Z` comes from
`lock_node`'s `target_pose`, which is metric only since the flat-port
refraction fix; before that this correction could not have been computed
correctly even with the offsets measured.
"""
import math
import os

_CACHE = {}


def _load():
    if _CACHE:
        return _CACHE
    import yaml
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, '..', 'config', 'tool_geometry.yaml'),
                 os.path.join(here, 'config', 'tool_geometry.yaml')):
        cand = os.path.abspath(cand)
        if os.path.isfile(cand):
            with open(cand) as fh:
                _CACHE.update((yaml.safe_load(fh) or {}).get('tools', {}) or {})
            break
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('mongla_vision')
            path = os.path.join(share, 'config', 'tool_geometry.yaml')
            if os.path.isfile(path):
                with open(path) as fh:
                    _CACHE.update((yaml.safe_load(fh) or {}).get('tools', {}) or {})
        except Exception:                                   # noqa: BLE001
            pass
    return _CACHE


def offset_m(tool: str):
    """(x, y, z) metres in the camera optical frame, or None if unknown.

    x +right, y +DOWN (image-Y), z +forward -- the camera's convention, not the
    world's, because the correction it feeds is a pixel offset.
    """
    e = _load().get(str(tool or '').strip())
    if not e:
        return None
    return (float(e.get('x', 0.0)), float(e.get('y', 0.0)), float(e.get('z', 0.0)))


def is_measured(tool: str) -> bool:
    """False when the entry is a placeholder reading zero.

    ⛔ A ZERO OFFSET AND AN UNMEASURED ONE ARE THE SAME NUMBER AND NOT THE SAME
    CLAIM. `camera_forward` is exactly zero from itself; `torpedo` is zero
    because nobody has held a tape to it. Collapsing those would make the
    warning that matters unprintable.
    """
    e = _load().get(str(tool or '').strip())
    return bool(e) and not bool(e.get('unmeasured', False))


def camera_for(tool: str):
    """Which camera aims this tool, or None."""
    e = _load().get(str(tool or '').strip())
    return (e or {}).get('camera') or None


def known_tools():
    return sorted(_load())


def pixel_offset(tool: str, *, fx: float, fy: float, range_m: float):
    """Signed (du, dv) px the target must sit at to be on the TOOL's axis.

    Returns None when it cannot be computed -- unknown tool, no calibration, or
    no range. Refusing is the point: the caller then aims the camera, as the
    stack always did, and says so. A guessed correction moves the aim point
    with false confidence, which is strictly worse than a known-absent one.
    """
    off = offset_m(tool)
    if off is None or not (fx > 0.0 and fy > 0.0):
        return None
    if not (range_m and math.isfinite(range_m) and range_m > 1e-3):
        return None
    x, y, _z = off
    return (fx * x / float(range_m), fy * y / float(range_m))
