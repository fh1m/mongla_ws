"""The Intent instrument: a verb's goal, whether its error closes, and what stops it.

Every case here has a known answer: the error series, the health row and the
lock are built so the verdict is fixed before the code runs.
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


def test_the_board_flag_is_the_verb_table_the_reference_reads():
    """One truth: `on_the_board` asks the driver's verb table; the generated
    reference parses the same table. They must name the same verbs."""
    import importlib.util
    from mongla_batighor.panels import verbs
    root = pathlib.Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('gen_reference', root / 'tools' / 'ci' / 'gen_reference.py')
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    board = {n for n, v in verbs().items() if v['board']}
    assert board == gen.board_verbs()
    assert {'set_depth', 'move_forward', 'turn', 'stop'} <= board
    assert not board & {'surface', 'vision_align', 'go_to', 'arm'}


@NEEDS_NODE
def test_a_closing_error_reads_closing_and_a_flat_one_does_not():
    out = _run(r'''
const g = { what: "depth", err: -0.4, unit: "m" };
const now = 10000;
const closing = [{t:7000,e:0.8},{t:8500,e:0.6},{t:10000,e:0.4}];
const flat = [{t:7000,e:0.4},{t:8500,e:0.42},{t:10000,e:0.4}];
const young = [{t:9500,e:0.8},{t:10000,e:0.4}];
process.stdout.write(JSON.stringify([X.verdictOf(g, closing, now).word, X.verdictOf(g, flat, now).word,
  X.verdictOf(g, young, now).word, X.verdictOf({...g, err: NaN}, closing, now).word]));''')
    assert out == ['closing', 'not closing', 'starting', 'unknown']


@NEEDS_NODE
def test_a_vision_error_inside_the_verbs_own_default_band_is_inside():
    """err_px is not set by the sender: the verb's default (40 px) is the band."""
    out = _run(r'''
const d = { verb: "vision_align", fields: {}, since: 1 };
const f = { ...VERBS.vision_align.defaults, ...d.fields };
const s = { fb: { phase: "EXECUTING", err_x_px: -30, err_y_px: NaN } };
const g = X.goalsOf(d, f, s)[0];
const s2 = { fb: { phase: "EXECUTING", err_x_px: 55, err_y_px: 12 } };
const g2 = X.goalsOf(d, f, s2)[0];
process.stdout.write(JSON.stringify([g.band, g.err, X.verdictOf(g, [], 1).word, g2.err, X.verdictOf(g2, [], 1).word]));''')
    assert out == [40, 30, 'inside', 55, 'starting']


@NEEDS_NODE
def test_goals_are_measured_the_way_each_verb_means_them():
    out = _run(r'''
const v = { depth_m: -0.6, yaw_deg: 350, armed: true };
const depth = X.goalsOf({verb:"set_depth"}, {target:-1.0}, {v})[0];
const turn = X.goalsOf({verb:"turn"}, {target:10}, {v})[0];
const go = X.goalsOf({verb:"go_to"}, {x_m:3, y_m:4, tolerance_m:0.5}, {v, odom:{pose:{pose:{position:{x:0,y:0}}}}})[0];
const prog = X.goalsOf({verb:"move_forward"}, {}, {v, fb:{phase:"move_forward", current_value:0.25}});
const none = X.goalsOf({verb:"move_forward"}, {}, {v, fb:{phase:"EXECUTING", current_value:-0.6}});
process.stdout.write(JSON.stringify([+depth.err.toFixed(3), turn.err, go.err, go.band,
  prog.map((g) => [g.what, g.now]), none.length]));''')
    # Depth 0.4 m shallow of target; 350 to 10 deg is 20 deg the short way;
    # (0,0) to (3,4) is 5 m; the board's progress counts only when the
    # feedback's phase is the verb (the vision pump's EXECUTING is not).
    assert out == [0.4, -20, 5, 0.5, [["board's progress", 25]], 0]


@NEEDS_NODE
def test_what_stops_a_verb_is_said_from_evidence_only():
    out = _run(r'''
const now = 100000;
const healthy = [{ name: "mongla: barometer", level: 0, message: "ok", at: now }];
const sick = [{ name: "mongla: barometer", level: 2, message: "PROM CRC failed", at: now }];
const base = { v: { armed: true, depth_m: -0.2 }, vAt: now };
const w = (verb, s) => X.cannotOf({ verb }, { ...VERBS[verb].defaults }, s, now);
process.stdout.write(JSON.stringify({
  fine: w("set_depth", { ...base, health: healthy }),
  baro: w("set_depth", { ...base, health: sick }),
  surfaceBaro: w("surface", { ...base, health: sick }),
  disarmed: w("move_forward", { ...base, v: { armed: false }, health: healthy }),
  stale: w("set_depth", { ...base, vAt: now - 5000, health: healthy }),
  lost: w("vision_align", { ...base, health: healthy, lock: { forward: { detections: [] } }, lockAt: { forward: now } }),
  silent: w("vision_align", { ...base, health: healthy }),
  late: w("set_depth", { ...base, health: healthy, tasks: [{ task: "gate", state: "running", elapsed_s: 70, deadline_s: 60, at: now }] }),
}));''')
    assert out['fine'] == []
    assert len(out['baro']) == 1 and 'barometer unhealthy' in out['baro'][0] and 'AUTO' in out['baro'][0]
    assert out['surfaceBaro'] == []                     # surface is SURFACE mode, not AUTO
    assert out['disarmed'] == ['disarmed: the board moves nothing']
    assert 'no /mongla/state for 5 s' in out['stale'][0]
    assert out['lost'] == ['forward lock is lost: the verb steers on nothing']
    assert 'forward lock silent since start' in out['silent'][0]
    assert out['late'] == ['task gate is past its deadline']


@NEEDS_NODE
def test_the_panel_keeps_each_goals_error_and_forgets_it_with_the_verb():
    out = _run(r'''
const I = X.INSTRUMENTS.Intent, s = {};
const doing = (verb, since, fields) => ({ data: JSON.stringify({ now: { verb, since, fields }, recent: [], stamp: since }) });
I.fold(s, "/mongla/doing", doing("set_depth", 1, { target: -1 }));
I.fold(s, "/mongla/state", { armed: true, depth_m: -0.3, yaw_deg: 0 });
const kept = s.hist.depth.length;
I.fold(s, "/mongla/doing", doing("turn", 2, { target: 90 }));
process.stdout.write(JSON.stringify([kept, Object.keys(s.hist), s.hist.heading.length]));''')
    # The depth history went with set_depth; the turn's heading starts fresh.
    assert out == [1, ['heading'], 1]
