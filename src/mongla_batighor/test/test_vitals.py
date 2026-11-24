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


@NEEDS_NODE
def test_a_caption_says_what_its_detector_does_and_goes_stale_when_the_overlay_does():
    """The strip under a picture (overlay.caption on <camera>/hud): running is
    calm, paused is amber with why, silent and not paused is the alarm, and a
    caption older than CAPTION_STALE_S is said stale, not kept as current."""
    out = _run(r'''
const now = 100000, s = {};
const say = (cam, c, at) => X.foldCaption(s, { data: JSON.stringify(c) }, cam, at);
say("forward", { state: "running", rate_hz: 20, late_ms: 68, model: "m", size: [640, 480], alarm: false }, now - 500);
say("downward", { state: "silent", silent_s: 4.2, model: "m", size: null, alarm: true }, now - 200);
const run = X.captionOf(s, "forward", now), dead = X.captionOf(s, "downward", now);
const old = X.captionOf(s, "forward", now + 5000), never = X.captionOf({}, "forward", now);
process.stdout.write(JSON.stringify([run.level, run.word, run.rate, run.size, dead.level, dead.why,
  old.level, old.state, never.state, never.level]));''')
    assert out == ['ok', 'detecting', 20, '640x480', 'err', '4 s, NOT paused',
                   'stale', 'stale', 'unheard', 'stale']


@NEEDS_NODE
def test_a_long_reason_is_cut_to_its_room_and_marked():
    out = _run(r'''
const long = "heading not anchored: no prop seen since start and flow refused";
process.stdout.write(JSON.stringify([X.fitText(long, X.NAV_WHY_CHARS), X.fitText("ok", 38), X.NAV_WHY_CHARS]));''')
    cut, short, room = out
    assert len(cut) == room and cut.endswith('…') and short == 'ok'


@NEEDS_NODE
def test_attitude_reads_the_boards_frd_quaternion_and_absent_is_null():
    """Roll 10 deg (starboard down), pitch 15 deg (nose up), yaw 30, packed
    the way node_support._quat_from_rpy packs them (Z-Y-X)."""
    import math
    r, p, y = (math.radians(a) / 2 for a in (10, 15, 30))
    q = {'w': math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
         'x': math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
         'y': math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
         'z': math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y)}
    out = _run(f'''
const q = {json.dumps(q)};
const a = X.attitudeOf({{ orientation: q, orientation_covariance: [0.01] }});
const none = X.attitudeOf({{ orientation: q, orientation_covariance: [-1] }});
process.stdout.write(JSON.stringify([a.roll, a.pitch, none]));''')
    assert out[0] == pytest.approx(10, abs=1e-6) and out[1] == pytest.approx(15, abs=1e-6)
    assert out[2] is None


@NEEDS_NODE
def test_depth_rate_is_down_positive_and_needs_a_second_of_history():
    out = _run(r'''
const now = 100000;
const going = [[now - 2000, 1.0], [now - 1000, 1.25], [now, 1.5]];
const short = [[now - 500, 1.0], [now, 1.1]];
process.stdout.write(JSON.stringify([X.depthRate(going, now), X.depthRate(short, now), X.depthRate([], now)]));''')
    assert out[0] == pytest.approx(0.25) and out[1] is None and out[2] is None


@NEEDS_NODE
def test_the_horizon_tilts_against_the_roll_and_drops_when_the_nose_rises():
    out = _run(r'''
const drawn = [];
global.document = { createElementNS: (_, t) => { const n = { tag: t, a: {}, setAttribute(k, v) { this.a[k] = String(v); },
  append() {}, addEventListener() {} }; drawn.push(n); return n; }, createElement: (t) => ({ append() {}, setAttribute() {} }) };
const now = Date.now();
const s = { v: { armed: false, mode: "", yaw_deg: 0, depth_m: -1 }, vAt: now, att: { roll: 10, pitch: 15 }, aAt: now };
X.vitalsSvg(X.vitalsOf(s, now));
process.stdout.write(JSON.stringify(drawn.filter((n) => (n.a.transform || "").includes("translate(0")).map((n) => n.a.transform)));''')
    (tf,) = out
    assert tf.startswith('rotate(-10 ') and tf.endswith('translate(0 25)'), tf


@NEEDS_NODE
def test_the_chain_drawing_says_live_idle_silent_and_not_sent_apart():
    """chainBoxes, on a chain.py message built so each answer is fixed: a
    frame heard at 26 Hz is live with its ages; a detector silent with a
    reason is idle; a tracker silent with none is the alarm; one message is
    heard but has no rate; a stage nothing places is not drawn; what the
    board does not send is drawn absent with its ask."""
    out = _run(r'''
const st = (o) => ({ n: 0, hz: 0, age_ms_p50: null, age_ms_p95: null, last_s: null, label: "", idle: "", ask: "", ...o });
const c = { order: ["frame:forward", "detect:forward", "track:forward", "lock:forward", "nowhere"],
  stages: { "frame:forward": st({ n: 132, hz: 26.4, age_ms_p50: 46, age_ms_p95: 67, last_s: 0.01, label: "forward frame" }),
            "detect:forward": st({ label: "forward detect", idle: "until a mission asks" }),
            "track:forward": st({ label: "forward track" }),
            "lock:forward": st({ n: 1, hz: 0, age_ms_p50: 40, age_ms_p95: 40, last_s: 0.5, label: "forward lock" }),
            nowhere: st({ n: 5, hz: 1, last_s: 0.1, label: "unplaced" }) },
  not_sent: [{ label: "mixer", ask: "upstream B: saturation" }, { label: "unknown", ask: "x" }] };
const b = X.chainBoxes(c);
process.stdout.write(JSON.stringify(b.map((x) => [x.key, x.state, x.hz, x.p50, x.why])));''')
    assert out == [['frame:forward', 'live', 26.4, 46, ''],
                   ['detect:forward', 'idle', None, None, 'until a mission asks'],
                   ['track:forward', 'silent', None, None, ''],
                   ['lock:forward', 'live', None, 40, ''],
                   ['mixer', 'absent', None, None, 'upstream B: saturation']]
