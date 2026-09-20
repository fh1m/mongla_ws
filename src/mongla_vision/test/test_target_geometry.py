"""The committed geometry table: reachable, sourced, and agreeing with the sim.

⛔ WHY THIS FILE EXISTS. `target_width_m` was an operator parameter defaulting
to 0.0, so `lock_node`'s 6-DoF branch published
`ok=false, reason='target_width_m unset'` and the metric path had never run on
the vehicle. A number that must be typed correctly per mission, under
pressure, for an answer whose wrongness is invisible, is a defect waiting for
a pool day.

Three things can go wrong and only one of them is loud:

  * the file is not installed -> every class resolves to 0.0 on a clean
    deploy and the metric path silently stays refused (the round-1 defect, in
    a new file);
  * a width is the prop's largest dimension rather than the width of WHAT THE
    DETECTOR BOXES -> every range is wrong by a fixed factor, with nothing
    logging a fault;
  * this table and the simulator's arena spec disagree -> a rulebook change
    lands in one and not the other, and sim-tuned behaviour stops matching the
    pool.

Reads the files; no ROS, no camera.
"""
import pathlib

import pytest

yaml = pytest.importorskip('yaml')

_PKG = pathlib.Path(__file__).resolve().parents[1]
_TABLE = _PKG / 'config' / 'target_geometry.yaml'
# The simulator is a SEPARATE colcon workspace (sim/COLCON_IGNORE), so it can
# never be imported from here -- but it can be read, and that is the whole
# point of this cross-check.
# _PKG is src/mongla_vision, so parents[1] is the WORKSPACE ROOT and the sim
# lives beside src/. This was parents[2] and resolved one level too high, so
# the cross-check below skipped silently -- "a cross-check that compares
# nothing always passes", which is the failure its own assertion warns about.
_SIM_SPEC = _PKG.parents[1] / 'sim' / 'src' / 'mongla_sim_worlds' / 'spec'


def _table():
    return yaml.safe_load(_TABLE.read_text())


def test_every_entry_states_a_positive_width_and_its_SOURCE():
    for comp, props in _table().items():
        for name, e in props.items():
            assert isinstance(e, dict), f'{comp}.{name} is not a mapping'
            w = e.get('width_m')
            if e.get('no_published_dimension') is True:
                # The one accepted way to have no width: the organisers never
                # published one. `hole` is the case -- the handbook dimensions
                # the torpedo board and not its openings. The entry still has
                # to say what it boxes and cite the document that omits it, so
                # the gap is a recorded fact rather than a missing line, and
                # `align(fwd=)` (angular size, a ratio) is unaffected either way.
                assert w is None, (
                    f'{comp}.{name} claims no published dimension AND states '
                    f'width_m={w!r}. Pick one.')
                continue
            assert isinstance(w, (int, float)) and w > 0, (
                f'{comp}.{name} has width_m={w!r}. A non-positive width makes '
                f'target_pose refuse, which is the state this table exists to '
                f'end. If the organisers never published one, say so with '
                f'`no_published_dimension: true`.')
            assert e.get('source'), (
                f'{comp}.{name} states {w} m with no source. An unsourced '
                f'dimension cannot be re-checked against a rulebook revision.')
            assert e.get('boxes'), (
                f'{comp}.{name} does not say WHAT the detector boxes. The '
                f'width is the reference REGION\'s, not the prop\'s largest '
                f'dimension -- without this the next reader cannot tell '
                f'whether 1.50 is a gate span or a post diameter.')


def test_a_diameter_entry_is_not_secretly_a_span():
    """The failure that hides. A pole-shaped prop boxed by the detector gives
    a bbox one DIAMETER wide; using the span between two poles instead makes
    every range wrong by a fixed factor, always in the flattering direction."""
    for comp, props in _table().items():
        for name, e in props.items():
            if e.get('boxes') == 'diameter':
                assert e['width_m'] <= 0.7, (
                    f'{comp}.{name} is marked `boxes: diameter` but is '
                    f'{e["width_m"]} m wide. That is span-sized, not '
                    f'pole-sized -- check which one the detector actually '
                    f'boxes.')


def test_the_classes_MATCH_a_shipped_model():
    """A table keyed by names no detector emits is unreachable by
    construction. Checks the SAUVC block against the model YAML we ship."""
    model = _PKG / 'models' / 'sauvc_sim.yaml'
    if not model.is_file():
        pytest.skip('sauvc_sim.yaml not present (weights are gitignored)')
    names = set((yaml.safe_load(model.read_text()) or {}).get('names', {}).values())
    assert names, 'the model YAML declares no class names'
    table = set(_table()['sauvc'])
    unknown = sorted(table - names)
    assert not unknown, (
        f'target_geometry names SAUVC classes the shipped model does not '
        f'emit: {unknown}. Those entries can never be looked up. Known: '
        f'{sorted(names)}')


def test_it_AGREES_with_the_simulator_arena_spec_where_both_state_a_number():
    """One truth, two files. They serve different purposes -- the sim spec
    builds a world, this feeds a pose solver -- but where both name the same
    physical dimension they must not disagree, or a rulebook change lands in
    one and sim-tuned behaviour stops matching the pool."""
    spec = _SIM_SPEC / 'sauvc.yaml'
    if not spec.is_file():
        pytest.skip('simulator arena spec not present')
    sim = (yaml.safe_load(spec.read_text()) or {}).get('props', {})
    table = _table()['sauvc']
    # (table class, sim prop, sim key) -- only where BOTH describe the same
    # physical quantity.
    pairs = [
        ('qual_gate',     'qualification_gate', 'width'),
        ('final_gate',    'final_gate',         'width'),
        ('orange_flare',  'orange_flare',       'diameter'),
        ('flare_red',     'bump_flare',         'diameter'),
        ('drum_red',      'drum',               'diameter'),
        ('starting_zone', 'starting_zone',      'size'),
    ]
    checked = 0
    bad = []
    for cls, prop, key in pairs:
        if prop not in sim or key not in (sim.get(prop) or {}):
            continue
        checked += 1
        a, b = float(table[cls]['width_m']), float(sim[prop][key])
        if abs(a - b) > 1e-6:
            bad.append(f'{cls}: table={a} sim {prop}.{key}={b}')
    assert checked >= 4, (
        f'only {checked} dimensions were comparable -- has the sim spec '
        f'changed shape? A cross-check that compares nothing always passes.')
    assert not bad, (
        'the geometry table and the simulator arena spec disagree:\n  '
        + '\n  '.join(bad))


def test_lock_node_ACTUALLY_consults_the_table():
    """The table existing is not the same as it being reached -- that gap is
    how four configs in this package came to reach nothing."""
    src = (_PKG / 'mongla_vision' / 'lock_node.py').read_text()
    assert 'target_geometry' in src, (
        'lock_node never consults the committed table, so target_width_m '
        'still defaults to 0.0 and the 6-DoF path stays refused.')
    i = src.index('target_width_m')
    window = src[i:i + 1500]
    # Assert the CALL and the ASSIGNMENT, not the name. Injecting
    # `w = 0.0` in place of `w = width_for(self._cls)` left the import line
    # intact, so a bare `'width_for' in window` still passed against code
    # where the lookup had been removed -- the test was reading an import
    # statement, not behaviour.
    assert 'width_for(self._cls)' in window, (
        'lock_node does not CALL width_for on the locked class (an import '
        'line is not a lookup), so target_width_m stays 0.0 and the 6-DoF '
        'path is refused exactly as before.')
    assert 'self._target_w_m = w' in window, (
        'width_for is called but its result is never assigned to the width '
        'the pose solver reads.')


def test_an_explicit_parameter_still_WINS():
    """A measured prop beats a rulebook nominal -- that is what SAUVC's
    +/- 5 % tolerance exists to allow for. The fallback must not overwrite an
    operator's measurement."""
    src = (_PKG / 'mongla_vision' / 'lock_node.py').read_text()
    i = src.index('target_width_m')
    window = src[i:i + 1500]
    assert 'self._target_w_m <= 0.0' in window, (
        'the table is consulted unconditionally, so it overrides an explicit '
        'target_width_m -- discarding a measurement in favour of a nominal.')
