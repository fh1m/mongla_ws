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

★ OUR POOL IS NOT THE VENUE. We practise against props we built, and ours are
not the handbook's sizes -- a home-cut gate is a different width from a RoboSub
gate, and a range computed from the handbook number against our prop is wrong by
exactly that ratio. So the committed table is a DEFAULT, never a hardcoding:

    DUBURI_TARGET_GEOMETRY=~/my_props.yaml        (a file, or a directory
                                                   holding target_geometry.yaml)
    ~/.duburi/target_geometry.yaml                (picked up with no env var)

Either file is merged OVER the committed table, per class, so you override the
one prop you re-cut and inherit the rest. Same nesting, same keys:

    robosub:
      gate:
        width_m: 1.82
        boxes: whole-gate
        source: "our pool prop, tape-measured 2026-09-10"

`describe()` returns whichever entry won, so a log line says which number the
vehicle actually used and where it came from. An override with no `source:` is
still honoured -- refusing a measured number over missing paperwork would be the
worse failure -- but `overridden: true` is stamped on it so the provenance of a
surprising range is one lookup away.

⛔ AND: the committed defaults come from the ORGANISERS' documents, never from
our simulator. The two SAUVC entries taken from the sim arena spec carry
`unquoted: true` for that reason. A sim prop's size is our own guess wearing a
number's clothes.
"""
import os

_CACHE = {}


def _load():
    if _CACHE:
        return _CACHE
    table = {}
    for d in _candidate_dirs():
        p = os.path.join(d, 'target_geometry.yaml')
        loaded = _read(p)
        if loaded:
            table = loaded
            break
    for p in _override_paths():
        _merge(table, _read(p), p)
    _CACHE.update(table)
    return _CACHE


def _read(path):
    if not os.path.isfile(path):
        return {}
    import yaml
    try:
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return {}                 # a malformed file must not hide a good one


def _override_paths():
    """Where an operator may put OUR pool's prop sizes, lowest priority first."""
    paths = [os.path.expanduser('~/.duburi/target_geometry.yaml')]
    env = os.environ.get('DUBURI_TARGET_GEOMETRY', '').strip()
    if env:
        env = os.path.expanduser(env)
        paths.append(os.path.join(env, 'target_geometry.yaml')
                     if os.path.isdir(env) else env)
    return paths


def _merge(table, extra, origin):
    """Per-CLASS merge, not per-file.

    A whole-file replace would mean overriding one prop silently deletes every
    other -- the class you did not re-cut would resolve to 0.0 and the pose path
    would refuse, on a pool day, for a prop you never touched.
    """
    for comp, entries in (extra or {}).items():
        if not isinstance(entries, dict):
            continue
        dst = table.setdefault(comp, {})
        for name, entry in entries.items():
            if not isinstance(entry, dict) or not entry.get('width_m'):
                continue
            merged = dict(entry)
            merged['overridden'] = True
            merged.setdefault('source', 'operator override: %s' % origin)
            dst[name] = merged


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
