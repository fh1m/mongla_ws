"""Committed prop widths, keyed by the class name the detector emits.

`config/target_geometry.yaml` holds the numbers and their rulebook sources;
this resolves a class name to one width in metres.

Why a module and not a launch parameter. `target_width_m` was an operator
parameter defaulting to 0.0, so the 6-DoF path published
`ok=false, reason='target_width_m unset'` and the metric branch had never run
on the vehicle. A number that must be typed correctly, per mission, under
pressure, for an answer whose wrongness is invisible is not a parameter —
it is a defect waiting for a pool day. The parameter is gone: it outlived
the class it was measured for when a mission switched targets. A measured
prop beats a rulebook nominal by going in the deck override under its class.

The width is that of WHAT THE DETECTOR BOXES. See the YAML's header.

our pool is not the venue. We practise against props we built, and ours are
not the handbook's sizes -- a home-cut gate is a different width from a RoboSub
gate, and a range computed from the handbook number against our prop is wrong by
exactly that ratio. So the committed table is a DEFAULT, never a hardcoding:

    MONGLA_TARGET_GEOMETRY=~/my_targets.yaml      (a file, or a directory
                                                   holding target_geometry.yaml)
    ~/.mongla/target_geometry.yaml                (picked up with no env var)

this is a target library, not a competition table. The competition groupings
below are only where the shipped defaults happen to be SOURCED from; they are
not what the capability is for. The vehicle's actual claim is: name any label,
state how wide the thing you box is, and the hull can reach it metrically from
one camera. So an override file needs no competition key at all --

    red_pipe:
      width_m: 0.0334
      boxes: single-pipe
      source: "our pool prop, calipered 2026-09-10"

-- a flat `label: {width_m: ...}` mapping is read as its own group. Train a
model on a new object, add one line here, and every metric consumer (range,
6-DoF pose, tool aim, the standoff obliquity gate) works on it with no code
change. Nested-by-competition also still works, for overriding a shipped entry
in place.

Merging is PER LABEL, so overriding the one prop you re-cut inherits the rest
rather than zeroing them. `describe()` returns whichever entry won, so a log
line says which number the vehicle used and where it came from. An override
with no `source:` is still honoured -- refusing a measured number over missing
paperwork would be the worse failure -- but `overridden: true` is stamped on it
so the provenance of a surprising range is one lookup away.

an edited override is picked up live. The override files are stat'd on each
lookup and re-read when their mtime or size changes, so correcting a prop width
mid-session does NOT need a node restart -- which matters because the moment
you discover the number is wrong is the moment you are standing at the pool
with the vehicle in the water. Staleness here is not a performance question:
a correction that silently does not apply is worse than no correction, because
the operator believes it did.

What still needs an event. `lock_node` captures the width into
`self._target_w_m`, and it re-resolves on every target-class change
(`_retarget_width`), not on a timer. So an edit lands at the next class switch
-- or immediately, by re-pinning `target_class`:

    ros2 param set /mongla_lock_forward target_class ''
    ros2 param set /mongla_lock_forward target_class gate

The committed table inside the package is a different matter: it is read from
the INSTALLED share, so editing the source copy still needs a colcon build.
That is what the override files are for.

AND: the committed defaults come from the ORGANISERS' documents, never from
our simulator. The two SAUVC entries are read off a rulebook figure and carry
no `unquoted` key; only a robosub entry sets it. A sim prop's size is our own
guess wearing a number's clothes.
"""
import os

_CACHE = {}


def _load():
    # Called FIRST and unconditionally: it records the stamps as well as
    # comparing them. Behind `_CACHE and ...` Python short-circuits it away on
    # the very load that should establish the baseline, so the next lookup saw
    # an empty `_STAMPS`, called it "changed", and re-read the whole table
    # once for nothing.
    changed = _overrides_changed()
    if _CACHE and not changed:
        return _CACHE
    _CACHE.clear()
    del _ERRORS[:]
    del _REJECTED[:]
    table = {}
    for d in _candidate_dirs():
        p = os.path.join(d, 'target_geometry.yaml')
        loaded = _read(p)
        if loaded:
            table = loaded
            break
    for p in _override_paths():
        _merge(table, _read(p), p)
    src, widths = _course_widths()
    if widths:
        _merge(table, widths, src)
    _CACHE.update(table)
    return _CACHE



_STAMPS: dict = {}


def _overrides_changed() -> bool:
    """Has an operator edited an override since we last read it?

    Stat only -- two files, ~1 us -- and compared on (mtime_ns, size) rather
    than mtime alone, because an edit that lands inside the same filesystem
    timestamp tick is exactly the rushed pool-day correction this exists to
    catch. Called per lookup, and lookups happen on a target-class change, not
    per frame.
    """
    now = {}
    for path in _override_paths() + [p for p in [_course_widths()[0]] if p]:
        try:
            st = os.stat(path)
            now[path] = (st.st_mtime_ns, st.st_size)
        except OSError:
            now[path] = None        # absent is a state, and it can change
    if now == _STAMPS:
        return False
    _STAMPS.clear()
    _STAMPS.update(now)
    return True


def _read(path):
    """Parse one table, RECORDING any failure instead of only surviving it.

    Swallowing the exception is right -- a malformed file must not hide a good
    one -- but swallowing it SILENTLY is how a stray indent in the committed
    YAML made every class resolve to 0.0 with nothing logged. That happened
    during this very change and looked exactly like an unrelated regression:
    the table is not a file the vehicle can afford to lose quietly.
    """
    if not os.path.isfile(path):
        return {}
    import yaml
    try:
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception as exc:
        _ERRORS.append((path, str(exc).splitlines()[0] if str(exc) else repr(exc)))
        return {}


_ERRORS = []


def load_errors():
    """Files that failed to parse, so a node can say so instead of reading 0.0."""
    _load()
    return list(_ERRORS)


def _course_widths():
    """(course file, {class: {width_m, source}}) from the deck's course, or (None, {}).

    The venue's course is its one file (`~/mongla/deck/courses/<name>.yaml`),
    so a prop measured there sets the width the pose path ranges on, on top
    of the shipped table and the older override file.
    """
    try:
        from mongla_runtime.launch_defaults import deck_value
        name = deck_value('course')
        if not name:
            return None, {}
        from mongla_localization.course_map import load_course
        c = load_course(name)
    except Exception as exc:                            # noqa: BLE001
        _ERRORS.append(('course', str(exc).splitlines()[0] if str(exc) else repr(exc)))
        return None, {}
    return c.source, {cls: {'width_m': w, 'source': f'measured in course {c.name}'}
                      for cls, w in c.widths_by_class().items()}


def _override_paths():
    """Where an operator may put OUR pool's prop sizes, lowest priority first."""
    paths = [os.path.expanduser('~/.mongla/target_geometry.yaml')]
    env = os.environ.get('MONGLA_TARGET_GEOMETRY', '').strip()
    if env:
        env = os.path.expanduser(env)
        paths.append(os.path.join(env, 'target_geometry.yaml')
                     if os.path.isdir(env) else env)
    return paths


# A width outside this band is a unit error, not a target. The floor is 1 cm
# because the SMALLEST thing we already steer on is a 1 in PVC slalom pipe at
# 0.0334 m -- a 5 cm floor was the first draft and it rejected that pipe, which
# is the whole reason this constant carries its measurement. The ceiling is 5 m,
# wider than either pool is deep.
#
# What this catches: `width_m: 182` for a 1.82 m gate, the classic centimetre
# slip, which would otherwise put every range off by 100x with nothing logging
# a fault. What it CANNOT catch, said plainly rather than left to be
# discovered: `3.34` for `0.0334` lands inside the band and reads as a plausible
# 3.34 m object. A range bound cannot check a unit; only the `source:` line and
# the logged width can. Read the `[LOCK ] target width` line on the deck.
_MIN_WIDTH_M = 0.01
_MAX_WIDTH_M = 5.0


def _merge(table, extra, origin):
    """Per-LABEL merge, accepting flat `label:` or nested `group: {label:}`.

    Flat is the general case: name any object, give its width, done. Nested
    exists so a shipped entry can be overridden in place under its own group.

    A whole-file replace would mean overriding one prop silently deletes every
    other -- the label you did not re-cut would resolve to 0.0 and the pose path
    would refuse, on a pool day, for a prop you never touched.
    """
    if not extra:
        return
    if not isinstance(extra, dict):
        # Valid YAML that is not a table (a list, a bare number): recorded
        # like a parse failure, never raised into the lock's start-up.
        _ERRORS.append((origin, 'not a mapping of label: {width_m: ...} '
                                '(got %s)' % type(extra).__name__))
        return
    for key, value in extra.items():
        if not isinstance(value, dict):
            _REJECTED.append((key, value, origin))     # `red_pipe: 0.0334`
            continue
        if _is_entry(value):                    # flat: this IS a target
            _put(table, 'custom', key, value, origin)
            continue
        for name, entry in value.items():       # nested: a group of targets
            if isinstance(entry, dict):
                _put(table, key, name, entry, origin)
            else:
                # `red_pipe: {width: 0.0334}` reads as a group whose member
                # `width` is no target: the operator's correction, dropped.
                _REJECTED.append(('%s.%s' % (key, name), entry, origin))


def _is_entry(value):
    return 'width_m' in value


def _put(table, group, name, entry, origin):
    """Install one override, DISPLACING the shipped entry of the same label.

    Not merely adding it. A flat override lands in its own group, and
    `width_for` searches every group and REFUSES when two disagree -- so
    overriding a shipped class from a flat file would have made that class
    resolve to 0.0, silently turning off the prop the operator was trying to
    correct. Found by running it on the vehicle, not by reading it: `hole`
    took the override's 0.095 while `describe` still reported the handbook as
    its source, because the committed entry was still there and matched first.
    One label means one entry, in one place, whichever file it came from.
    """

    try:
        w = float(entry.get('width_m'))
    except (TypeError, ValueError):
        # Reported like an implausible width, or the lock's start-up warning
        # is silent and the shipped width flies in its place.
        _REJECTED.append((name, entry.get('width_m'), origin))
        return
    if not (_MIN_WIDTH_M <= w <= _MAX_WIDTH_M):
        # Ignored, not obeyed -- same choice as a malformed file. Falling back
        # to a sourced default is recoverable; a 100x range error is not.
        _REJECTED.append((name, w, origin))
        return
    merged = dict(entry)
    merged['width_m'] = w
    merged['overridden'] = True
    merged['override_from'] = origin
    merged.setdefault('source', 'operator override: %s' % origin)
    for g in list(table):
        if g != group:
            table[g].pop(name, None)
    table.setdefault(group, {})[name] = merged


_REJECTED = []


def rejected_overrides():
    """Overrides ignored (an implausible width, or an entry that is not a
    `label: {width_m: ...}` mapping), so a node can log them.

    A silently-dropped override looks exactly like an override that was never
    read, which is the single most confusing failure this file could have.
    """
    _load()
    return list(_REJECTED)


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
            get_package_share_directory('mongla_vision'), 'config'))
    except Exception:
        pass
    dirs.append(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))
    return dirs


def width_for(class_name: str, competition: str = '') -> float:
    """True width in metres of what the detector boxes for `class_name`.

    0.0 when unknown — never a guess. The lock publishes no pose for a
    non-positive width, so an unlisted class gives no pose rather than a
    confident wrong range.

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
