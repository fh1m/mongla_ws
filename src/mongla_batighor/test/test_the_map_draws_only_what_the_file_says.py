"""The pool map (instruments `poolMap`) draws what the course file claims, and no more.

Run in node against the served panel, with a DOM that records what is drawn:
- a prop the file calls a nominal is drawn dashed amber and named "(nominal)";
- the hull is drawn only once the filter is anchored in the pool frame: its
  odom position is in another frame, and on the map it would sit among the
  props as if it were placed;
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
const [course, loc, target] = JSON.parse(process.argv[process.argv.length - 1]);
module.exports.poolMap(course, loc, [], [], target);
process.stdout.write(JSON.stringify(drawn.map((n) => ({ tag: n.tag, attrs: n.attrs,
  text: n.kids.filter((k) => typeof k === "string").join("") }))));
"""

COURSE = {'course': 't', 'pool': {'outline': [[0, -5], [0, 5], [20, 5], [20, -5]], 'floor': []},
          'props': [{'name': 'gate', 'x_m': 10, 'y_m': 0, 'bearing_deg': 0, 'width_m': 1.5,
                     'measured': True},
                    {'name': 'flare', 'x_m': 6, 'y_m': 2, 'bearing_deg': None, 'width_m': None,
                     'measured': False}],
          'unplaced': [{'name': 'drum'}]}


def _draw(loc, target=None):
    src = (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()
    out = subprocess.run(['node', '-e', HARNESS % {'src': src}, json.dumps([COURSE, loc, target])],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def _hull(drawn):
    return [n for n in drawn if n['tag'] == 'polygon' and n['attrs'].get('fill') == 'var(--mb-red)']


def test_a_nominal_prop_says_so_and_a_measured_one_is_solid():
    d = _draw(None)
    texts = {n['text'] for n in d if n['tag'] == 'text'}
    assert 'flare (nominal)' in texts and 'gate' in texts
    assert not any('drum' in t for t in texts), 'an unplaced prop was drawn'
    flare = [n for n in d if n['tag'] == 'circle' and n['attrs'].get('stroke') == 'var(--mb-amber)']
    assert flare and all(n['attrs'].get('stroke-dasharray') for n in flare)


def test_the_hull_is_on_the_map_only_once_anchored_in_the_pool():
    loc = {'frame': 'odom_ned', 'anchored': False, 'x_m': 3.0, 'y_m': 1.0, 'yaw_deg': 0.0,
           'sigma_xy_m': 0.3}
    assert _hull(_draw(loc)) == [], 'an unanchored hull was drawn among the props'
    assert len(_hull(_draw({**loc, 'frame': 'pool', 'anchored': True}))) == 1


def test_a_go_to_is_circled_at_its_tolerance():
    d = _draw(None, {'x': 12.0, 'y': -2.0, 'tol': 0.5})
    rings = [n for n in d if n['tag'] == 'circle' and n['attrs'].get('stroke') == 'var(--mb-red)'
             and n['attrs'].get('cx') == '-2' and n['attrs'].get('cy') == '-12']
    assert rings and rings[0]['attrs']['r'] == '0.5'
