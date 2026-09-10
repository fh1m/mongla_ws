"""The table is keyed by class names; nothing checked they were OUR class names.

`target_geometry.yaml` was written from the rulebooks and keyed by RULEBOOK PROP
names, while the detectors emit MODEL CLASS names, and the two had never been
compared. Of the five classes the two competition models actually produce, only
`gate` resolved: `rescue`, `repair`, `fire` and `blood` all returned 0.0, so the
6-DoF path published `ok=false, reason='target_width_m unset'` for them -- no
width, no range, and therefore no tool-aim correction and no standoff obliquity
gate. Every test passed throughout, because every test named a class that
happened to be covered.

The check that would have caught it is this one: read the class names off the
shipped model sidecars and demand each one resolve.

Also pinned here: an operator's own prop sizes override the committed defaults,
because our pool props are not the venue's props.
"""
import contextlib
import os
import tempfile

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_MODELS = os.path.join(_PKG, 'models')

# Sidecars for models that are not competition props. `yolov11n` is the COCO
# checkpoint (80 classes of person/car/dog); the `*sim*` stems are simulator
# models, and the whole point of the sim-vs-handbook rule is that we do not
# take prop sizes from the simulator.
_NOT_COMPETITION = ('yolov11n', 'sauvc_sim', 'sim_sauvc_v1')


def _geometry():
    """A fresh view of the table.

    `_CACHE`, `_ERRORS` and `_REJECTED` are all module-global and all three
    survive between tests: the broken-override test below leaves an entry in
    `_ERRORS`, which then read as 'the COMMITTED table failed to parse' in a
    later test. Clear all three, not just the cache.
    """
    import duburi_vision.target_geometry as tg
    tg._CACHE.clear()
    del tg._ERRORS[:]
    del tg._REJECTED[:]
    return tg


def _shipped_classes():
    """Every class a competition model emits -- ON DISK **OR** TRACKED IN GIT.

    On-disk alone is not enough, and that hole is live right now: the two
    sidecars carrying the classes a pool run actually steers on --
    `slalom_red_pipe` (`red_pipe`) and `torpedo_blood_hole` (`torpedo`, `blood`,
    `hole`) -- are deleted from the working tree while still tracked. A census
    that only listed `models/*.yaml` could never fail on them, which is the same
    shape of blind spot as keying the table by rulebook names: the guard would
    be green about precisely the classes nobody had checked.
    """
    out = {}
    for stem, text in _sidecar_texts().items():
        if stem in _NOT_COMPETITION:
            continue
        names = (yaml.safe_load(text) or {}).get('names') or {}
        if names:
            out[stem] = [str(v).strip() for v in names.values()]
    return out


def _sidecar_texts():
    texts = {}
    for fn in sorted(os.listdir(_MODELS)):
        stem, ext = os.path.splitext(fn)
        if ext == '.yaml':
            with open(os.path.join(_MODELS, fn)) as fh:
                texts[stem] = fh.read()
    for stem, text in _tracked_sidecars().items():
        texts.setdefault(stem, text)
    return texts


def _tracked_sidecars():
    """Sidecars git still tracks, including ones deleted from the working tree."""
    import subprocess
    root = os.path.dirname(os.path.dirname(_PKG))
    rel = 'src/duburi_vision/models'
    try:
        listed = subprocess.run(
            ['git', '-C', root, 'ls-files', rel + '/*.yaml'],
            capture_output=True, text=True, timeout=20)
        if listed.returncode != 0:
            return {}
        out = {}
        for path in listed.stdout.split():
            got = subprocess.run(['git', '-C', root, 'show', 'HEAD:' + path],
                                 capture_output=True, text=True, timeout=20)
            if got.returncode == 0:
                out[os.path.splitext(os.path.basename(path))[0]] = got.stdout
        return out
    except Exception:
        return {}          # no git (an install tree): fall back to disk alone


def test_the_models_we_ship_are_actually_being_read():
    """A guard over an empty set passes forever."""
    sidecars = _shipped_classes()
    assert sidecars, 'no competition model sidecars found in %s' % _MODELS
    flat = [c for classes in sidecars.values() for c in classes]
    assert len(flat) >= 5, flat


def test_every_shipped_model_class_has_a_width():
    """Every emitted class resolves, or the TABLE says why it cannot.

    `no_published_dimension: true` is the one accepted answer for a zero, and it
    lives in the YAML beside the class rather than in a list here -- so the
    reason travels with the number, and adding a class cannot be waved through
    by editing the test.
    """
    tg = _geometry()
    missing = []
    for stem, classes in _shipped_classes().items():
        for cls in classes:
            if tg.width_for(cls) > 0.0:
                continue
            if tg.describe(cls).get('no_published_dimension') is True:
                continue
            missing.append('%s -> %s' % (stem, cls))
    assert not missing, (
        'these classes the detector emits resolve to NO width, so their pose, '
        'range, tool-aim and obliquity gate cannot compute: %s' % missing)


def test_a_width_is_plausible_for_a_pool_prop():
    """A typo of 3.0 for 0.3 is invisible until the hull stops 10x too early.

    The band is the module's own, not a second copy: a guard that disagreed
    with the code would either pass what the loader rejects or reject what it
    accepts. The first draft of this test hardcoded 0.05 and would have failed
    the 0.0334 m slalom pipe the loader is required to accept.
    """
    tg = _geometry()
    for stem, classes in _shipped_classes().items():
        for cls in classes:
            w = tg.width_for(cls)
            if w == 0.0:
                continue                 # covered by the resolve test above
            assert tg._MIN_WIDTH_M <= w <= tg._MAX_WIDTH_M, (
                '%s -> %s = %s m' % (stem, cls, w))


def test_our_pool_prop_size_overrides_the_handbook(monkeypatch):
    """Our gate is not RoboSub's gate, and the operator must be able to say so."""
    tg = _geometry()
    stock = tg.width_for('gate')
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'robosub': {'gate': {'width_m': 1.82,
                                                 'boxes': 'whole-gate'}}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
        assert tg.width_for('gate') != stock
        # and the override says it is one, so a surprising range is traceable
        assert tg.describe('gate').get('overridden') is True
    tg._CACHE.clear()


def test_overriding_one_prop_does_not_delete_the_others(monkeypatch):
    """A whole-file replace would zero every class the operator did not re-cut."""
    tg = _geometry()
    others = {c: tg.width_for(c) for c in ('rescue', 'repair', 'fire', 'blood')}
    assert all(v > 0 for v in others.values()), others
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'robosub': {'gate': {'width_m': 1.82}}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        for cls, was in others.items():
            assert tg.width_for(cls) == pytest.approx(was), cls
    tg._CACHE.clear()


def test_a_directory_works_as_well_as_a_file(monkeypatch):
    """Pointing at a folder is what an operator does by reflex."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, 'target_geometry.yaml'), 'w') as fh:
            yaml.safe_dump({'robosub': {'gate': {'width_m': 1.82}}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', d)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
    tg._CACHE.clear()


def test_a_broken_override_does_not_erase_the_defaults(monkeypatch):
    """Fail to the handbook, never to zero: 0.0 stops the mission silently."""
    tg = _geometry()
    stock = tg.width_for('gate')
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            fh.write('robosub: {gate: {width_m: [oops\n')
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(stock)
    tg._CACHE.clear()


def test_simulator_sourced_numbers_are_marked_as_such():
    """Organisers' documents are the source; the sim is our own guess."""
    tg = _geometry()
    entry = tg.describe('target_mat')
    assert entry, 'target_mat vanished from the table'
    assert entry.get('unquoted') is True, (
        'a width taken from the simulator arena spec must be flagged, so it is '
        'never mistaken for a rulebook figure: %s' % entry)


def test_the_committed_table_parses():
    """A stray indent made EVERY class resolve to 0.0, and nothing said so.

    `_read` swallows a parse error on purpose, so one bad file cannot hide a
    good one -- but that means a broken committed table degrades to an empty
    dict, which is indistinguishable from 'this class is not listed'. It
    happened while writing this round's own change. Two guards: the file must
    parse, and the failure must be REPORTABLE rather than only survivable.
    """
    tg = _geometry()
    assert not tg.load_errors(), tg.load_errors()
    assert tg._load(), 'the committed target table loaded EMPTY'


def test_a_parse_failure_is_reported_not_just_survived(monkeypatch):
    tg = _geometry()
    stock = tg.width_for('gate')
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            fh.write('gate:\n  width_m: 1.82\n   boxes: bad indent\n')
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        del tg._ERRORS[:]
        assert tg.width_for('gate') == pytest.approx(stock)   # survived
        assert any(path in p for p, _ in tg.load_errors())     # and reported
    tg._CACHE.clear()
    del tg._ERRORS[:]


def test_an_implausible_override_is_reported_not_just_dropped(monkeypatch):
    """A silently-dropped override looks exactly like one that was never read."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 182}}, fh)     # cm typed as m
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        del tg._REJECTED[:]
        assert tg.width_for('gate') != pytest.approx(182.0)
        assert [n for n, _, _ in tg.rejected_overrides()] == ['gate']
    tg._CACHE.clear()
    del tg._REJECTED[:]


def test_a_one_inch_pipe_is_an_acceptable_target():
    """The smallest thing we steer on must survive the plausibility floor.

    The first draft floored at 0.05 m and would have rejected the 0.0334 m
    slalom pipe -- the constant now carries that measurement as its reason.
    """
    tg = _geometry()
    assert tg._MIN_WIDTH_M <= 0.0334
    assert tg.width_for('red_pipe') == pytest.approx(0.0334)


def test_an_override_displaces_the_shipped_entry_it_replaces(monkeypatch):
    """Found on the vehicle: the width changed but the PROVENANCE did not.

    A flat override lands in its own group while the shipped entry stays in
    `robosub`, and `width_for` searches every group and refuses when two
    disagree -- so overriding a shipped class from a flat file would have made
    it resolve to 0.0, turning off the very prop the operator was correcting.
    `describe` matched the shipped entry first and reported the handbook as the
    source of a number the handbook never gave.
    """
    tg = _geometry()
    assert tg.width_for('gate') == pytest.approx(3.0)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 1.82, 'boxes': 'whole-gate',
                                     'source': 'our pool prop, tape-measured'}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)   # not 0.0
        d_ = tg.describe('gate')
        assert d_.get('overridden') is True
        assert d_.get('source') == 'our pool prop, tape-measured'
        assert path in d_.get('override_from', '')
        # and the shipped entry is gone, not shadowed: exactly one survives
        table = tg._load()
        assert sum('gate' in g for g in table.values()) == 1
    tg._CACHE.clear()


@contextlib.contextmanager
def _lock_node():
    """A LockNode with no ROS graph, for driving its class-change paths.

    ⛔ rclpy CONTEXT IS PROCESS-GLOBAL, so leaving it initialised breaks other
    files. The first draft called `rclpy.init()` and only destroyed the node;
    `test_flow_node`'s module fixture then called `rclpy.init()` again and ALL
    31 of its tests errored -- a file this round never touched, which passed
    perfectly on its own. Shut down exactly what we started, and only that.
    """
    import rclpy
    from duburi_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = LockNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def test_the_width_FOLLOWS_the_class_through_a_whole_mission():
    """Seven classes, one node -- the width must re-resolve on every switch.

    A pool run locks gate -> rescue -> red_pipe -> torpedo -> blood -> fire,
    switching the detector's class as it goes. If the width were captured once
    at construction, every leg after the first would scale its range by the
    FIRST prop's width: a 3.0 m gate applied to a 0.0334 m pipe is a 90x range
    error that publishes a confident number and logs nothing.

    Driven, not read. `test_lock_node_ACTUALLY_consults_the_table` asserts the
    lookup and the assignment sit close together in the source, which proves
    adjacency and says nothing about whether the width tracks the class.
    """
    pytest.importorskip('rclpy')
    from std_msgs.msg import String
    with _lock_node() as node:
        expect = [('gate', 3.0), ('rescue', 0.305), ('red_pipe', 0.0334),
                  ('torpedo', 0.6096), ('blood', 0.305), ('fire', 0.305)]
        seen = []
        for cls, _ in expect:
            node._on_classes_filter(String(data=cls))
            seen.append((cls, node._target_w_m))
        assert seen == [(c, pytest.approx(w)) for c, w in expect], seen


def test_a_class_with_no_width_does_not_inherit_the_previous_one():
    """`hole` has no published dimension, and must not reuse `blood`'s 0.305."""
    pytest.importorskip('rclpy')
    from std_msgs.msg import String
    with _lock_node() as node:
        node._on_classes_filter(String(data='blood'))
        assert node._target_w_m == pytest.approx(0.305)
        node._on_classes_filter(String(data='hole'))
        assert node._target_w_m == 0.0, (
            'the pose must REFUSE for a class with no width, not carry the '
            'previous class\'s width into a confident wrong range')


def test_the_no_env_var_path_is_the_one_the_deck_warning_names(monkeypatch):
    """`~/.duburi/target_geometry.yaml` had never once been exercised.

    Every other override test goes through DUBURI_TARGET_GEOMETRY, and the
    vehicle check did too -- while the warning `lock_node` prints on the deck
    tells the operator to use this path. An untested path in a message we hand
    the operator under pressure is the same defect class as a knob wired to
    nothing.
    """
    tg = _geometry()
    monkeypatch.delenv('DUBURI_TARGET_GEOMETRY', raising=False)
    with tempfile.TemporaryDirectory() as home:
        os.makedirs(os.path.join(home, '.duburi'))
        with open(os.path.join(home, '.duburi', 'target_geometry.yaml'), 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 1.82}}, fh)
        monkeypatch.setenv('HOME', home)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
        assert tg.width_for('rescue') == pytest.approx(0.305)
    tg._CACHE.clear()


def test_an_edited_override_applies_without_a_restart(monkeypatch):
    """The moment you discover a width is wrong is the moment the vehicle is
    in the water. A correction that silently does not apply until a restart is
    worse than no correction, because the operator believes it did."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 1.82}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 2.44}}, fh)
        assert tg.width_for('gate') == pytest.approx(2.44), (
            'the edit did not take -- an override read once per process needs '
            'a node restart, which is exactly when nobody can do one')
    tg._CACHE.clear()


def test_deleting_an_override_returns_to_the_committed_default(monkeypatch):
    """Absent is a state too, and it can be reached by an operator undoing a
    change. Falling back to the handbook is the recoverable direction."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 1.82}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
        os.remove(path)
        assert tg.width_for('gate') == pytest.approx(3.0)
    tg._CACHE.clear()


def test_an_unchanged_file_is_not_re_read(monkeypatch):
    """Change-reactive, not poll-and-parse. Re-parsing YAML on every lookup
    would put a file read on the path a class switch takes."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            yaml.safe_dump({'gate': {'width_m': 1.82}}, fh)
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        tg.width_for('gate')
        reads = []
        real = tg._read
        monkeypatch.setattr(tg, '_read', lambda p: (reads.append(p), real(p))[1])
        for _ in range(10):
            tg.width_for('gate')
        assert reads == [], f'{len(reads)} re-reads with nothing changed'
    tg._CACHE.clear()


def test_an_edit_inside_one_timestamp_tick_is_still_caught(monkeypatch):
    """Two writes in the same filesystem tick differ in SIZE, and a rushed
    pool-day correction is exactly the edit that lands inside one tick."""
    tg = _geometry()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'mine.yaml')
        with open(path, 'w') as fh:
            fh.write('gate: {width_m: 1.82}\n')
        monkeypatch.setenv('DUBURI_TARGET_GEOMETRY', path)
        tg._CACHE.clear()
        assert tg.width_for('gate') == pytest.approx(1.82)
        st = os.stat(path)
        with open(path, 'w') as fh:
            fh.write('gate: {width_m: 2.4400}\n')      # a different LENGTH
        os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns))   # same mtime
        assert tg.width_for('gate') == pytest.approx(2.44)
    tg._CACHE.clear()
