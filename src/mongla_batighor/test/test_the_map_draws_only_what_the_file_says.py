"""The pool map (instruments `poolMap`) draws what the course file claims, and no more.

Run in node against the served panel, with a DOM that records what is drawn:
- a prop the file calls a nominal is drawn dashed amber and named "(nominal)";
- Mongla is always on the map once there is a pose: solid red when the
  filter is anchored in the pool, hollow amber and labelled "by its start"
  when it is only placed by where it started, so an assumption never looks
  like a measurement;
- placing by the start turns odometry into the pool frame correctly;
- a go_to target is circled at its own tolerance.
"""
import json
import pathlib
import shutil
import subprocess

import pytest

pytestmark = pytest.mark.skipif(not shutil.which('node'), reason='node not installed (the Pi, CI)')

PANELS = pathlib.Path(__file__).resolve().parents[1] / 'panels'

HARNESS = r"""
const drawn = [];
function node(tag) { const n = { tag, attrs: {}, kids: [], setAttribute(k, v) { this.attrs[k] = String(v); },
  append(...k) { this.kids.push(...k); }, addEventListener() {} }; drawn.push(n); return n; }
global.document = { createElementNS: (_, t) => node(t), createElement: (t) => node(t), querySelector: () => null,
  head: { append() {} } };
const module = { exports: {} };
const CAMERAS = ["forward", "downward"], VERBS = {}, TUNABLE = {}, THEME_CSS = "";
%(src)s
const [course, pose, target] = JSON.parse(process.argv[process.argv.length - 1]);
module.exports.poolMap(course, pose, [], [], target);
process.stdout.write(JSON.stringify(drawn.map((n) => ({ tag: n.tag, attrs: n.attrs,
  text: n.kids.filter((k) => typeof k === "string").join("") }))));
"""

COURSE = {'course': 't', 'pool': {'outline': [[0, -5], [0, 5], [20, 5], [20, -5]], 'floor': []},
          'props': [{'name': 'gate', 'x_m': 10, 'y_m': 0, 'bearing_deg': 0, 'width_m': 1.5,
                     'measured': True},
                    {'name': 'flare', 'x_m': 6, 'y_m': 2, 'bearing_deg': None, 'width_m': None,
                     'measured': False}],
          'unplaced': [{'name': 'drum'}]}


def _draw(pose, target=None):
    src = (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()
    out = subprocess.run(['node', '-e', HARNESS % {'src': src}, json.dumps([COURSE, pose, target])],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def _hulls(drawn):
    # The hull is the one polygon in Mongla's inks; the pool outline is sea-blue.
    return [n for n in drawn if n['tag'] == 'polygon'
            and n['attrs'].get('stroke') != 'var(--mb-sea)']


def test_a_nominal_prop_says_so_and_a_measured_one_is_solid():
    d = _draw(None)
    texts = {n['text'] for n in d if n['tag'] == 'text'}
    assert 'flare (nominal)' in texts and 'gate' in texts
    assert not any('drum' in t for t in texts), 'an unplaced prop was drawn'
    flare = [n for n in d if n['tag'] == 'circle' and n['attrs'].get('stroke') == 'var(--mb-amber)']
    assert flare and all(n['attrs'].get('stroke-dasharray') for n in flare)


def test_mongla_is_drawn_as_what_its_position_is():
    assert _hulls(_draw(None)) == []
    pose = {'x': 3.0, 'y': 1.0, 'yaw': 0.0, 'sigma': 0.3}
    (solid,) = _hulls(_draw({**pose, 'placed': False}))
    assert solid['attrs']['fill'] == 'var(--mb-red)'
    d = _draw({**pose, 'placed': True})
    (hollow,) = _hulls(d)
    assert hollow['attrs']['fill'] == 'none' and hollow['attrs']['stroke'] == 'var(--mb-amber)'
    assert 'by its start' in {n['text'] for n in d if n['tag'] == 'text'}


def test_placing_by_the_start_turns_odometry_into_the_pool():
    """Started at odom (1, 2) heading east: a metre further east is a metre
    toward the gate (+x), and its heading there is bearing 0."""
    out = subprocess.run(['node', '-e', HARNESS.split('const [course')[0] % {
        'src': (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()}
        + 'process.stdout.write(JSON.stringify(module.exports.placeByStart('
          '{x: 1, y: 3, yaw: Math.PI / 2}, {x: 1, y: 2, yaw: Math.PI / 2})));'],
        capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    p = json.loads(out.stdout)
    assert abs(p['x'] - 1.0) < 1e-9 and abs(p['y']) < 1e-9 and abs(p['yaw']) < 1e-9, p


def test_a_go_to_is_circled_at_its_tolerance():
    d = _draw(None, {'x': 12.0, 'y': -2.0, 'tol': 0.5})
    rings = [n for n in d if n['tag'] == 'circle' and n['attrs'].get('stroke') == 'var(--mb-red)'
             and n['attrs'].get('cx') == '-2' and n['attrs'].get('cy') == '-12']
    assert rings and rings[0]['attrs']['r'] == '0.5'


ROWS = r"""
function node(tag) { return { tag, attrs: {}, kids: [], setAttribute(k, v) { this.attrs[k] = String(v); },
  append(...k) { this.kids.push(...k); }, addEventListener() {} }; }
global.document = { createElementNS: (_, t) => node(t), createElement: (t) => node(t), querySelector: () => null,
  head: { append() {} } };
const module = { exports: {} };
const CAMERAS = ["forward", "downward"], VERBS = {}, TUNABLE = {}, THEME_CSS = "";
%(src)s
const course = JSON.parse(process.argv[process.argv.length - 1]);
const text = (n) => typeof n === "string" ? n : (n.kids || []).map(text).join(" ");
process.stdout.write(module.exports.drawMap({ course }, false).map(text).join("\n"));
"""


def _rows(course):
    src = (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()
    out = subprocess.run(['node', '-e', ROWS % {'src': src}, json.dumps(course)],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return out.stdout


def test_a_blank_venue_says_how_to_fill_it_and_a_filled_one_does_not():
    """Kutum on the real board, 2026-11-22: the deck's pool was the blank
    venue and the Map showed only dashes, while flow refused every interval."""
    blank = {'course': 'pool', 'pool': {'depth_m': None, 'floor': []}, 'props': []}
    assert 'blank venue no floor, so flow refuses every interval' in _rows(blank)
    filled = {'course': 'pool', 'pool': {'depth_m': 2.0, 'floor': []}, 'props': []}
    assert 'blank venue' not in _rows(filled)
