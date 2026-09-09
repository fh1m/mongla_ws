"""Committed prop widths, keyed by the class name the detector emits.

`config/target_geometry.yaml` holds the numbers and their rulebook sources;
this resolves a class name to one width in metres.

⛔ WHY A MODULE AND NOT A LAUNCH PARAMETER. `target_width_m` was an operator
parameter defaulting to 0.0, so the 6-DoF path published
`ok=false, reason='target_width_m unset'` and the metric branch had never run
on the vehicle. A number that must be typed correctly, per mission, under
pressure, for an answer whose wrongness is invisible is not a parameter —
it is a defect waiting for a pool day. The parameter still WINS when set,
because a measured prop beats a rulebook nominal.

The width is that of WHAT THE DETECTOR BOXES. See the YAML's header.
"""
import os

_CACHE = {}


def _load():
    if _CACHE:
        return _CACHE
    import yaml
    for d in _candidate_dirs():
        p = os.path.join(d, 'target_geometry.yaml')
        if os.path.isfile(p):
            try:
                with open(p) as fh:
                    _CACHE.update(yaml.safe_load(fh) or {})
            except Exception:
                continue          # a malformed file must not hide a good one
            if _CACHE:
                return _CACHE
    return _CACHE


def _candidate_dirs():
    """Installed share first, then the source tree.

    Both, because the node runs from the install tree on the vehicle and the
    tests read the source tree — the same reuse boundary `calibration/binding`
    documents.
    """
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(
            get_package_share_directory('duburi_vision'), 'config'))
    except Exception:
        pass
    dirs.append(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))
    return dirs


def width_for(class_name: str, competition: str = '') -> float:
    """True width in metres of what the detector boxes for `class_name`.

    0.0 when unknown — never a guess. `target_pose` already refuses on a
    non-positive width with a stated reason, so an unlisted class degrades to
    exactly the behaviour it has today rather than to a confident wrong range.

    `competition` narrows the lookup; empty searches all and requires the name
    to be unambiguous. A class defined differently in two competitions with the
    same name would otherwise resolve by dict order.
    """
    name = (class_name or '').strip()
    if not name:
        return 0.0
    data = _load()
    comps = ([competition] if competition else list(data))
    hits = []
    for c in comps:
        entry = (data.get(c) or {}).get(name)
        if isinstance(entry, dict) and entry.get('width_m'):
            hits.append(float(entry['width_m']))
    if not hits:
        return 0.0
    if len(set(hits)) > 1:
        return 0.0        # ambiguous across competitions: refuse, do not pick
    return hits[0]


def describe(class_name: str, competition: str = '') -> dict:
    """The whole entry, for logging what a number is and where it came from."""
    name = (class_name or '').strip()
    data = _load()
    for c in ([competition] if competition else list(data)):
        entry = (data.get(c) or {}).get(name)
        if isinstance(entry, dict):
            return dict(entry)
    return {}
