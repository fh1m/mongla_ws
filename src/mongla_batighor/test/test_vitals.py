"""The Vitals drawing says only what was heard, and says it while it is fresh.

Each case is built so its answer is fixed before the code runs: a state
three seconds old is stale, a demand older than two seconds is no demand,
a board line nobody graded is stale, never "ok".
"""
import json
import pathlib
import shutil
import subprocess

import pytest

PANELS = pathlib.Path(__file__).resolve().parents[1] / 'panels'
NEEDS_NODE = pytest.mark.skipif(not shutil.which('node'), reason='node not installed (the Pi, CI)')


def _run(probe: str):
    from mongla_batighor.panels import verbs
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],TUNABLE={},THEME_CSS="";'
           f'const VERBS={json.dumps(verbs())};')
    js = (pre + (PANELS / 'common.js').read_text() + (PANELS / 'instruments' / 'extension.js').read_text()
          + '\nconst X=module.exports;\n' + probe)
    out = subprocess.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


@NEEDS_NODE
def test_fresh_state_is_read_and_old_state_is_said_old():
    out = _run(r'''
const now = 100000;
const v = { armed: true, mode: "AUTO", yaw_deg: 92, depth_m: -1.2, battery_voltage: NaN };
const fresh = X.vitalsOf({ v, vAt: now - 500 }, now);
const old = X.vitalsOf({ v, vAt: now - 3000 }, now);
const none = X.vitalsOf({}, now);
process.stdout.write(JSON.stringify([fresh.stale, fresh.armed, fresh.mode, fresh.below, fresh.yaw,
  old.stale, Math.round(old.ageS), none.stale, none.below]));''')
    # Depth is altitude, negative below the surface: 1.2 m below. Absence is
    # null (NaN through JSON), never 0.
    assert out == [False, True, 'AUTO', 1.2, 92, True, 3, True, None]


@NEEDS_NODE
def test_a_demand_not_repeated_is_no_demand():
    out = _run(r'''
const now = 100000;
const s = { v: {}, vAt: now, demand: { x: 0.6, y: -0.3 } };
const fresh = X.vitalsOf({ ...s, dAt: now - 1000 }, now);
const old = X.vitalsOf({ ...s, dAt: now - 2500 }, now);
process.stdout.write(JSON.stringify([fresh.surge, fresh.sway, old.surge === undefined, old.sway === undefined]));''')
    assert out == [0.6, -0.3, True, True]


@NEEDS_NODE
def test_each_board_line_is_its_own_grade_and_an_ungraded_one_is_stale():
    out = _run(r'''
const now = 100000;
const health = [{ name: "mongla: barometer", level: 2, message: "PROM CRC failed", at: now },
                { name: "mongla: board_link", level: 0, message: "heartbeat fresh", at: now }];
const g = X.vitalsOf({ v: {}, vAt: now, health }, now);
process.stdout.write(JSON.stringify(g.health.map(([k, lvl]) => [k, lvl])));''')
    grades = dict(out)
    assert grades['barometer'] == 'err'
    assert grades['link'] == 'ok'
    assert grades['leak'] == 'stale' and grades['kill line'] == 'stale'
    assert len(out) == 6


@NEEDS_NODE
def test_esc_speeds_only_while_fresh_and_scaled_to_the_fastest():
    out = _run(r'''
const now = 100000;
const s = { v: {}, vAt: now, rpm: [0, 1500, -3000] };
const fresh = X.vitalsOf({ ...s, rAt: now - 200 }, now);
const old = X.vitalsOf({ ...s, rAt: now - 5000 }, now);
process.stdout.write(JSON.stringify([fresh.rpm, fresh.peak, old.rpm]));''')
    assert out == [[0, 1500, -3000], 3000, []]


@NEEDS_NODE
def test_a_link_reads_as_bars_and_no_number_is_no_bars():
    out = _run(r'''
const L = [0.15, 0.5, 2];
process.stdout.write(JSON.stringify([0.05, 0.15, 0.3, 1.0, 5, NaN, undefined].map((x) => X.signalBars(x, L))));''')
    assert out == [4, 4, 3, 2, 1, 0, 0]


@NEEDS_NODE
def test_the_watch_strip_shows_nothing_vitals_shows():
    """Vitals draws arming, mode, depth, heading, packs, position and health;
    the pilot console's strip keeps the rest, the link to Kutum among it."""
    out = _run(r'''
const { cells } = X.statusOf({}, 100000);
process.stdout.write(JSON.stringify([X.WATCH_CELLS, cells.map(([k]) => k)]));''')
    watch, every = out
    assert set(watch) <= set(every)
    assert not set(watch) & {'vehicle', 'mode', 'depth', 'heading', 'battery', 'navigation', 'health'}
    assert 'to kutum' in watch
