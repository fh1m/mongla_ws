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
    import duburi_vision.target_geometry as tg
    tg._CACHE.clear()
    return tg


def _competition_sidecars():
    out = {}
    for fn in sorted(os.listdir(_MODELS)):
        stem, ext = os.path.splitext(fn)
        if ext != '.yaml' or stem in _NOT_COMPETITION:
            continue
        with open(os.path.join(_MODELS, fn)) as fh:
            names = (yaml.safe_load(fh) or {}).get('names') or {}
        if names:
            out[stem] = [str(v).strip() for v in names.values()]
    return out


def test_the_models_we_ship_are_actually_being_read():
    """A guard over an empty set passes forever."""
    sidecars = _competition_sidecars()
    assert sidecars, 'no competition model sidecars found in %s' % _MODELS
    flat = [c for classes in sidecars.values() for c in classes]
    assert len(flat) >= 5, flat


def test_every_shipped_model_class_has_a_width():
    tg = _geometry()
    missing = []
    for stem, classes in _competition_sidecars().items():
        for cls in classes:
            if tg.width_for(cls) <= 0.0:
                missing.append('%s -> %s' % (stem, cls))
    assert not missing, (
        'these classes the detector emits resolve to NO width, so their pose, '
        'range, tool-aim and obliquity gate cannot compute: %s' % missing)


def test_a_width_is_plausible_for_a_pool_prop():
    """A typo of 3.0 for 0.3 is invisible until the hull stops 10x too early."""
    tg = _geometry()
    for stem, classes in _competition_sidecars().items():
        for cls in classes:
            w = tg.width_for(cls)
            assert 0.05 <= w <= 5.0, '%s -> %s = %s m' % (stem, cls, w)


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
