// The instruments: Mongla's own gauges, one Lichtblick panel each, from one table.
//
// Plain DOM and SVG, no build; panels.py puts CAMERAS and panels/common.js
// above this file, and theme/batighor.css gives the look. Each instrument
// names only the topics it draws, all of them small (never an image or a
// detection stream), and redraws at most ten times a second and only when
// something changed, plus once a second so an age keeps counting. Opening
// one costs the vehicle almost nothing.
//
// To add an instrument: one entry in INSTRUMENTS (its topics, how to fold a
// message into its state, how to draw it), and `instrument('<Name>')` in
// mongla_batighor/views.py where it belongs. Nothing else changes.

const RUNGS = ["detection", "held", "follow", "anchor", "lost"];
const RUNG_COLOUR = { detection: "var(--mb-red)", held: "var(--mb-amber)", follow: "var(--mb-glass)", anchor: "var(--mb-violet)", lost: "var(--mb-ink-3)" };
const LEVEL = ["ok", "warn", "err", "stale"];
// What each level is called on screen: STALE in /diagnostics means the
// manager could not tell, which an operator reads as "unknown".
const LEVEL_WORD = { ok: "ok", warn: "warn", err: "fail", stale: "unknown" };
const REDRAW_MS = 100;
const MISSION_ALIVE_S = 1.5;  // the runner beats at 2 Hz; silent this long is no mission (helm.MISSION_ALIVE_S)
const TRACK_S = 120;
// The manager grades /diagnostics once a second (node_setup.py), every row
// each time: a row not repeated for this long is unknown, not its last word.
const HEALTH_STALE_S = 3;
// Anything published faster than once a second is stale after this.
const FRESH_S = 2;          // the navigation plan view keeps two minutes of track
const SVG = "http://www.w3.org/2000/svg";

const lockTopic = (c) => `/mongla/vision/${c}/lock`;
const hudTopic = (c) => `/mongla/vision/${c}/hud`;
const seeingTopic = (c) => `/mongla/vision/${c}/seeing`;
const modelTopic = (c) => `/mongla/vision/${c}/vision_info`;
// What the running model can emit and what the mission filters for, both
// latched by the detector (detector_publish.py, detector_node.py).
const classesTopic = (c) => `/mongla/vision/${c}/model_classes`;
const filterTopic = (c) => `/mongla/vision/${c}/classes_filter`;
const names = (s) => String(s == null ? "" : s).split(",").map((x) => x.trim()).filter(Boolean);

// A filter naming a class the model never emits is a mission steering on
// nothing: the detector logs it and reports an empty frame, every frame
// (CLAUDE.md, the models are development harnesses). Pure; exported.
function classesOf(emits, filter) {
  if (emits == null) return null;
  const e = names(emits), f = names(filter);
  return { emits: e, filter: f, missing: f.filter((x) => !e.includes(x)) };
}
const STAGES_TOPIC = "/mongla/batighor/stages";     // overlay.py, once a second
const ageS = (t) => (t ? (Date.now() - t) / 1000 : Infinity);
// When the message being folded arrived. A fold stamps what it keeps with
// this, never with the clock, so a message replayed to a panel that mounted
// late keeps its own age (see LAST in makePanel).
let HEARD = 0;
const heard = () => HEARD || Date.now();
// The bridge's own time for the message being folded (Lichtblick's
// receiveTime: when Kutum's bridge took it), or null.
let SENT = null;
const sent = () => SENT;

function svg(tag, attrs, ...kids) {
  const e = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
  for (const k of kids) if (k) e.append(k);
  return e;
}

// Exported for the test: which rung a lock message says holds the target.
function rungOf(lock) {
  const d = lock && lock.detections && lock.detections[0];
  if (!d) return { rung: "lost", id: "", authority: null };
  const r = d.results && d.results[0];
  const hyp = r && (r.hypothesis || r);
  return { rung: hyp ? String(hyp.class_id) : "lost", id: d.id || "", authority: hyp ? hyp.score : null };
}

// -- small drawings ------------------------------------------------------

// A compass: north up, the heading as a needle; "--" when there is none.
function compass(deg) {
  const ok = fin(deg);
  const a = ok ? (deg * Math.PI) / 180 : 0;
  const x = 40 + 30 * Math.sin(a), y = 40 - 30 * Math.cos(a);
  const ticks = [0, 90, 180, 270].map((t) => {
    const r = (t * Math.PI) / 180;
    return svg("line", { x1: 40 + 34 * Math.sin(r), y1: 40 - 34 * Math.cos(r), x2: 40 + 38 * Math.sin(r), y2: 40 - 38 * Math.cos(r), stroke: "var(--mb-ink-3)" });
  });
  return svg("svg", { viewBox: "0 0 80 80", class: "dial" },
    svg("circle", { cx: 40, cy: 40, r: 38, fill: "var(--mb-sunk)", stroke: "var(--mb-rule-lit)" }),
    ...ticks,
    svg("text", { x: 40, y: 14, "text-anchor": "middle", "font-size": 8, fill: "var(--mb-ink-3)" }, "N"),
    ok && svg("line", { x1: 40, y1: 40, x2: x, y2: y, stroke: "var(--mb-red)", "stroke-width": 2.5, "stroke-linecap": "round" }),
    svg("circle", { cx: 40, cy: 40, r: 2.5, fill: "var(--mb-ink-2)" }));
}

// A vertical depth gauge from the surface to `floorM`, the vehicle as a tick.
function depthGauge(depthBelow, floorM) {
  const ok = fin(depthBelow);
  const f = Math.max(floorM, ok ? depthBelow : 0, 0.5);
  const y = ok ? 6 + (68 * Math.max(0, depthBelow)) / f : null;
  return svg("svg", { viewBox: "0 0 26 80", class: "gauge" },
    svg("rect", { x: 9, y: 6, width: 8, height: 68, rx: 2, fill: "var(--mb-sunk)", stroke: "var(--mb-rule-lit)" }),
    svg("line", { x1: 4, y1: 6, x2: 22, y2: 6, stroke: "var(--mb-sea)" }),
    ok && svg("rect", { x: 9, y: 6, width: 8, height: y - 6, fill: "var(--mb-sea)", opacity: 0.35 }),
    ok && svg("line", { x1: 4, y1: y, x2: 22, y2: y, stroke: "var(--mb-red)", "stroke-width": 2 }));
}

// A bar from the middle: -1..1, the demand's own range.
function centreBar(v) {
  const ok = fin(v);
  const w = ok ? Math.min(1, Math.abs(v)) * 50 : 0;
  return el("div", { class: "bar centre", style: "position:relative" },
    el("b", { style: `position:absolute;${v < 0 ? "right" : "left"}:50%;width:${w}%;${ok ? "" : "display:none"}` }),
    el("i", { style: "position:absolute;left:50%;top:0;bottom:0;width:1px;background:var(--mb-ink-3)" }));
}

// The plan view: the filter's track, north up, metres, the vehicle last.
function planView(track, fixes) {
  const W = 220, H = 150, pad = 10;
  const pts = track.filter((p) => fin(p.x) && fin(p.y));
  if (!pts.length) return el("div", { class: "note" }, "no track yet");
  // NED: x north (up), y east (right).
  let n0 = Infinity, n1 = -Infinity, e0 = Infinity, e1 = -Infinity;
  for (const p of [...pts, ...fixes]) { n0 = Math.min(n0, p.x); n1 = Math.max(n1, p.x); e0 = Math.min(e0, p.y); e1 = Math.max(e1, p.y); }
  const span = Math.max(n1 - n0, e1 - e0, 1);
  const s = Math.min(W - 2 * pad, H - 2 * pad) / span;
  const cx = (e0 + e1) / 2, cn = (n0 + n1) / 2;
  const X = (p) => W / 2 + (p.y - cx) * s, Y = (p) => H / 2 - (p.x - cn) * s;
  const last = pts[pts.length - 1];
  const grid = [];
  const step = span > 20 ? 5 : span > 5 ? 1 : 0.5;
  for (let g = Math.ceil(e0 / step) * step; g <= e1; g += step) grid.push(svg("line", { x1: X({ y: g, x: 0 }), y1: 0, x2: X({ y: g, x: 0 }), y2: H, stroke: "var(--mb-rule)" }));
  for (let g = Math.ceil(n0 / step) * step; g <= n1; g += step) grid.push(svg("line", { x1: 0, y1: Y({ x: g, y: 0 }), x2: W, y2: Y({ x: g, y: 0 }), stroke: "var(--mb-rule)" }));
  return svg("svg", { viewBox: `0 0 ${W} ${H}`, width: "100%", style: "flex:none;min-height:120px;max-height:260px;background:var(--mb-sunk);border:1px solid var(--mb-rule);border-radius:3px" },
    ...grid,
    svg("polyline", { points: pts.map((p) => `${X(p).toFixed(1)},${Y(p).toFixed(1)}`).join(" "), fill: "none", stroke: "var(--mb-sea)", "stroke-width": 1.5 }),
    ...fixes.map((f) => svg("circle", { cx: X(f), cy: Y(f), r: 3, fill: "none", stroke: "var(--mb-amber)" })),
    svg("circle", { cx: X(last), cy: Y(last), r: 4, fill: "var(--mb-red)" }),
    svg("text", { x: W - 4, y: H - 4, "text-anchor": "end", "font-size": 8, fill: "var(--mb-ink-3)" }, `grid ${step} m`));
}

// Where the filter's odometry puts Mongla in the pool frame, before any anchor,
// if it started at `start` facing bearing 0: the course origin is the centre of
// the starting zone and +x points at the gate (courses/*.yaml `frame:`). An
// assumption the map draws as one, hollow and amber, never as a measurement.
// Exported for the test.
function placeByStart(p, start) {
  if (!start || !fin(p.x) || !fin(p.y)) return null;
  const c = Math.cos(-start.yaw), s = Math.sin(-start.yaw);
  const dx = p.x - start.x, dy = p.y - start.y;
  return { x: c * dx - s * dy, y: s * dx + c * dy, yaw: fin(p.yaw) ? p.yaw - start.yaw : null };
}

// The pool map: the course file's pool frame, north (bearing 0, +x) up and
// +y right, in metres. A prop is drawn where the file puts it and as what the
// file claims: solid when measured, dashed amber when a rulebook nominal; a
// prop with no position is listed by the caller, never placed. Mongla is
// always on it: solid red once the filter is anchored in the pool, hollow
// amber where its start says it is before that (`pose.placed`).
// `pose`: {x, y, yaw (rad), sigma, placed}; `track`, `fixes`: pool-frame points
// ({x, y}, a fix with {fx, fy} also: where the filter was when it took it).
// `opts.vel`: [{vx, vy, ink}] pool-frame m/s, drawn as two-second arrows.
// Exported for the test.
// A label's dark halo, so a wall, a grid line or another mark crossing it
// does not cut the words.
const HALO = (u) => ({ stroke: "var(--mb-d0)", "stroke-width": 2.5 * u, "paint-order": "stroke", "stroke-linejoin": "round" });

function poolMap(course, pose, track, fixes, target, opts) {
  const C = course || {}, P = C.pool || {}, O = opts || {};
  const props = C.props || [];
  // The rulebook's zones (course.py): where a prop may be, never steered by.
  const zones = (C.zones || []).filter((z) => (z.points || []).length === 2);
  const here = pose && fin(pose.x) && fin(pose.y) ? pose : null;
  const pts = [...(P.outline || []).map(([x, y]) => ({ x, y })), ...(P.floor || []).map(([x, y]) => ({ x, y })),
    ...zones.flatMap((z) => z.points.map(([x, y]) => ({ x, y }))),
    ...props.map((p) => ({ x: p.x_m, y: p.y_m })), ...(here ? track : []), ...(here ? fixes : []),
    ...(here ? [here] : []), ...(target ? [target] : [])].filter((p) => fin(p.x) && fin(p.y));
  if (!pts.length) return null;
  let n0 = Infinity, n1 = -Infinity, e0 = Infinity, e1 = -Infinity;
  for (const p of pts) { n0 = Math.min(n0, p.x); n1 = Math.max(n1, p.x); e0 = Math.min(e0, p.y); e1 = Math.max(e1, p.y); }
  const span = Math.max(n1 - n0, e1 - e0, 4);
  const pad = span * 0.08;
  const cn = (n0 + n1) / 2, ce = (e0 + e1) / 2;
  // `opts.px`, the drawing's size on screen: the map takes the panel's shape
  // (a square pool drawing in a wide panel left two thirds of it black) and
  // `u` is one screen pixel, so a name is 12 px at any panel size; at half
  // height the names were 7 px (Kutum, 2026-11-23). Without it, a square
  // drawn as if 300 px wide.
  const px = O.px && O.px.w > 0 && O.px.h > 0 ? O.px : null;
  // Each axis by its own extent: the longer side used for both left the
  // 50 x 25 m olympic pool a third of a wide panel (2026-11-24).
  const sE = Math.max(e1 - e0, 4) + 2 * pad, sN = Math.max(n1 - n0, 4) + 2 * pad;
  const m = px ? Math.max(sE / px.w, sN / px.h) : (span + 2 * pad) / 300;
  const hw = px ? (m * px.w) / 2 : span / 2 + pad, hh = px ? (m * px.h) / 2 : span / 2 + pad;
  n0 = cn - hh; n1 = cn + hh; e0 = ce - hw; e1 = ce + hw;
  const W = e1 - e0;
  // SVG in metres: east right, north up (the y axis flipped by drawing at -x).
  const X = (p) => p.y, Y = (p) => -p.x;
  const u = m;                            // one screen pixel, in metres
  // The grid's step from what is drawn, not the panel: a wide panel drew
  // 5 m squares on a 9 m pool (2026-11-23).
  const drawn = span + 2 * pad;
  const step = drawn > 40 ? 5 : drawn > 12 ? 2 : drawn > 5 ? 1 : 0.5;
  const kids = [];
  for (let g = Math.ceil(e0 / step) * step; g <= e1; g += step) kids.push(svg("line", { x1: g, y1: -n1, x2: g, y2: -n0, stroke: "var(--mb-rule)", "stroke-width": u }));
  for (let g = Math.ceil(n0 / step) * step; g <= n1; g += step) kids.push(svg("line", { x1: e0, y1: -g, x2: e1, y2: -g, stroke: "var(--mb-rule)", "stroke-width": u }));
  // Zones under everything else, dashed amber and labelled small: an area
  // as its box, a line as itself; "(figure)" when only the rulebook's
  // not-to-scale figure places it.
  for (const z of zones) {
    const [[a0, b0], [a1, b1]] = z.points;
    const dash = `${4 * u},${3 * u}`, ink = "var(--mb-amber)";
    kids.push(z.kind === "line"
      ? svg("line", { x1: b0, y1: -a0, x2: b1, y2: -a1, stroke: ink, "stroke-width": 1.5 * u, "stroke-dasharray": dash, opacity: 0.8 })
      : svg("rect", { x: Math.min(b0, b1), y: -Math.max(a0, a1), width: Math.abs(b1 - b0), height: Math.abs(a1 - a0),
        fill: "rgba(248,135,1,0.05)", stroke: ink, "stroke-width": u, "stroke-dasharray": dash, opacity: 0.8 }));
    // Named inside its north-west corner, and only where the box holds the
    // name (about 6 px a character at 10 px): centred, the olympic pool's
    // eight names piled into one blur on a small map (2026-11-24).
    const words = z.name.replace(/_/g, " ") + (z.source === "rulebook" ? "" : " (figure)");
    const room = z.kind === "line" ? Infinity : Math.abs(b1 - b0);
    if (words.length * 6 * u <= room && (z.kind === "line" || Math.abs(a1 - a0) >= 14 * u)) {
      kids.push(svg("text", { x: Math.min(b0, b1) + 3 * u, y: -Math.max(a0, a1) + 11 * u, "font-size": 10 * u, fill: ink,
        opacity: 0.85, ...HALO(u) }, svg("title", {}, z.note || ""), words));
    }
  }
  if ((P.outline || []).length >= 3) {
    kids.push(svg("polygon", { points: P.outline.map(([x, y]) => `${y},${-x}`).join(" "), fill: "rgba(90,141,255,0.06)", stroke: "var(--mb-sea)", "stroke-width": 1.5 * u }));
  }
  for (const [x, y, d] of P.floor || []) {
    kids.push(svg("circle", { cx: y, cy: -x, r: 2 * u, fill: "var(--mb-teal)" }));
    // Left of the dot: prop names are written to the right of theirs, and a
    // sounding beside a prop read as one label (bench course, 2026-11-21).
    kids.push(svg("text", { x: y - 5 * u, y: -x + 4 * u, "font-size": 11 * u, fill: "var(--mb-teal)", "text-anchor": "end", ...HALO(u) }, `${num(d, 2)} m`));
  }
  const text = (x, y, s, fill, anchor) => svg("text", { x, y, "font-size": 12 * u, fill, "text-anchor": anchor || "start", ...HALO(u) }, s);
  for (const p of props) {
    const ink = p.measured ? "var(--mb-ink)" : "var(--mb-amber)";
    const dash = p.measured ? "" : `${3 * u},${2 * u}`;
    const b = fin(p.bearing_deg) ? (p.bearing_deg * Math.PI) / 180 : null;
    if (b !== null && fin(p.width_m)) {
      // A gate: its posts across the direction its face points.
      const h = p.width_m / 2, ax = -Math.sin(b), ay = Math.cos(b);
      kids.push(svg("line", { x1: p.y_m - ay * h, y1: -(p.x_m - ax * h), x2: p.y_m + ay * h, y2: -(p.x_m + ax * h), stroke: ink, "stroke-width": 3 * u, "stroke-dasharray": dash, "stroke-linecap": "round" }));
    } else {
      kids.push(svg("circle", { cx: p.y_m, cy: -p.x_m, r: 4 * u, fill: "none", stroke: ink, "stroke-width": 1.5 * u, "stroke-dasharray": dash }));
    }
    if (b !== null) {
      const L = 14 * u;
      kids.push(svg("line", { x1: p.y_m, y1: -p.x_m, x2: p.y_m + L * Math.sin(b), y2: -(p.x_m + L * Math.cos(b)), stroke: ink, "stroke-width": u }));
    }
    kids.push(text(p.y_m + 6 * u, -p.x_m - 6 * u, p.measured ? p.name : `${p.name} (nominal)`, ink));
  }
  if (here) {
    const ink = here.placed ? "var(--mb-amber)" : "var(--mb-red)";
    const tr = track.filter((p) => fin(p.x) && fin(p.y));
    if (tr.length > 1) kids.push(svg("polyline", { points: tr.map((p) => `${X(p)},${Y(p)}`).join(" "), fill: "none", stroke: here.placed ? "var(--mb-amber)" : "var(--mb-sea)", "stroke-width": 1.5 * u, "stroke-dasharray": here.placed ? `${3 * u},${2 * u}` : "" }));
    for (const f of fixes) {
      // A fix and, on the localization map, the correction it asked for:
      // from where the filter was to where the prop put it.
      if (O.rays && fin(f.fx) && fin(f.fy)) kids.push(svg("line", { x1: f.fy, y1: -f.fx, x2: f.y, y2: -f.x, stroke: "var(--mb-amber)", "stroke-width": u }));
      kids.push(svg("circle", { cx: f.y, cy: -f.x, r: 3 * u, fill: "none", stroke: "var(--mb-amber)", "stroke-width": u }));
    }
    if (fin(here.sigma) && here.sigma > 0) kids.push(svg("circle", { cx: here.y, cy: -here.x, r: here.sigma, fill: here.placed ? "none" : "rgba(255,42,20,0.08)", stroke: ink, "stroke-width": u, "stroke-dasharray": `${2 * u},${2 * u}` }));
    for (const v of O.vel || []) {
      if (!fin(v.vx) || !fin(v.vy)) continue;
      kids.push(svg("line", { x1: here.y, y1: -here.x, x2: here.y + 2 * v.vy, y2: -(here.x + 2 * v.vx), stroke: v.ink, "stroke-width": 1.5 * u, "stroke-linecap": "round" }));
      kids.push(svg("circle", { cx: here.y + 2 * v.vy, cy: -(here.x + 2 * v.vx), r: 1.5 * u, fill: v.ink }));
    }
    const yaw = fin(here.yaw) ? here.yaw : 0;
    const hull = [[10, 0], [-6, 5], [-3, 0], [-6, -5]].map(([f, r]) => {
      const n = here.x + u * (f * Math.cos(yaw) - r * Math.sin(yaw)), e = here.y + u * (f * Math.sin(yaw) + r * Math.cos(yaw));
      return `${e},${-n}`;
    });
    kids.push(svg("polygon", here.placed
      ? { points: hull.join(" "), fill: "none", stroke: "var(--mb-amber)", "stroke-width": 1.5 * u }
      : { points: hull.join(" "), fill: "var(--mb-red)", stroke: "var(--mb-d0)", "stroke-width": u }));
    if (here.placed) kids.push(text(here.y + 8 * u, -here.x + 12 * u, "by its start", "var(--mb-amber)"));
    if (target) kids.push(svg("line", { x1: here.y, y1: -here.x, x2: target.y, y2: -target.x, stroke: "var(--mb-red)", "stroke-width": u, "stroke-dasharray": `${4 * u},${3 * u}` }));
  }
  if (target) {
    const r = fin(target.tol) && target.tol > 0 ? target.tol : 6 * u;
    kids.push(svg("circle", { cx: target.y, cy: -target.x, r, fill: "none", stroke: "var(--mb-red)", "stroke-width": 1.5 * u }));
    kids.push(svg("line", { x1: target.y - r, y1: -target.x, x2: target.y + r, y2: -target.x, stroke: "var(--mb-red)", "stroke-width": u }));
    kids.push(svg("line", { x1: target.y, y1: -target.x - r, x2: target.y, y2: -target.x + r, stroke: "var(--mb-red)", "stroke-width": u }));
    kids.push(text(target.y + r + 3 * u, -target.x, "go_to", "var(--mb-red)"));
  }
  // The scale and north, where an operator looks for them.
  kids.push(svg("line", { x1: e1 - pad / 2 - step, y1: -n0 - pad / 3, x2: e1 - pad / 2, y2: -n0 - pad / 3, stroke: "var(--mb-ink-2)", "stroke-width": 2 * u }));
  kids.push(text(e1 - pad / 2, -n0 - pad / 3 - 4 * u, `${step} m`, "var(--mb-ink-2)", "end"));
  kids.push(text(e0 + pad / 2, -n1 + pad / 2 + 4 * u, "N  bearing 0", "var(--mb-ink-3)"));
  return svg("svg", { viewBox: `${e0} ${-n1} ${W} ${n1 - n0}`, preserveAspectRatio: "xMidYMid meet", class: "map" }, ...kids);
}

// The Map instruments' shared state: the course, the filter, where Mongla
// started (for drawing it before the anchor), its track and its fixes.
const MAP_TOPICS = ["/mongla/batighor/course", "/mongla/localization/state", "/mongla/localization/fix", "/mongla/doing"];
const rad = (d) => (fin(d) ? (d * Math.PI) / 180 : null);

const FLOW_CAL = "/mongla/vision/downward/flow_calibration";
function foldMap(s, topic, m) {
  s.track = s.track || []; s.fixes = s.fixes || [];
  if (topic === "/mongla/batighor/course") { try { s.course = JSON.parse(m.data); } catch { /* keep */ } }
  // The flow's scale, whole: 'calibrated' or 'uncalibrated: <why>'. The flow
  // view's state panel clips it after a few words.
  else if (topic === FLOW_CAL) s.flowCal = m.data;
  else if (topic === "/mongla/localization/state") {
    // A frame change (the anchor, or a filter restart) starts the track and
    // the start again: a track across it would join two frames.
    if (!s.loc || s.loc.frame !== m.frame) {
      s.track = []; s.fixes = [];
      s.start = m.frame === "pool" ? null : { x: 0, y: 0, yaw: rad(m.yaw_deg) || 0, by: "the filter's origin, heading when first seen" };
    }
    s.loc = m; s.lAt = heard();
    const p = mapPose(s);
    if (p) {
      const last = s.track[s.track.length - 1];
      if (!last || Math.hypot(last.x - p.x, last.y - p.y) > 0.02) s.track.push({ x: p.x, y: p.y, ox: m.x_m, oy: m.y_m, t: s.lAt });
      while (s.track.length && s.lAt - s.track[0].t > 10 * TRACK_S * 1000) s.track.shift();
    }
  } else if (topic === "/mongla/localization/fix") {
    const p = m.point;
    if (p) {
      const was = mapPose(s);
      s.fixes.push({ x: p.x, y: p.y, fx: was ? was.x : NaN, fy: was ? was.y : NaN, t: heard() });
      if (s.fixes.length > 40) s.fixes.shift();
    }
  } else if (topic === "/mongla/odom") { s.odom = m; s.odomAt = heard(); }
  // Flow publishes only an interval it accepted: a refused run is silence.
  else if (topic === "/mongla/vision/downward/velocity") { s.flow = m; s.flowAt = heard(); }
  else { try { s.doing = JSON.parse(m.data); } catch { /* keep */ } }
}

// Mongla in the pool frame: the filter's, once anchored; else placed by its start.
function mapPose(s) {
  const loc = s.loc;
  if (!loc || !fin(loc.x_m) || !fin(loc.y_m)) return null;
  if (loc.anchored && loc.frame === "pool") return { x: loc.x_m, y: loc.y_m, yaw: rad(loc.yaw_deg), sigma: loc.sigma_xy_m, placed: false };
  const p = placeByStart({ x: loc.x_m, y: loc.y_m, yaw: rad(loc.yaw_deg) }, s.start);
  return p && { ...p, sigma: loc.sigma_xy_m, placed: true };
}

function drawMap(s, forLocalization) {
  const c = s.course, loc = s.loc;
  if (!c) return [el("div", { class: "note" }, "waiting for /mongla/batighor/course (latched; it comes a few seconds after connecting). Still empty: is Batighor's course node running (batighor:=true)?")];
  const now = s.doing && s.doing.now;
  const f = now && now.verb === "go_to" ? now.fields || {} : null;
  const target = f && fin(f.x_m) && fin(f.y_m) ? { x: f.x_m, y: f.y_m, tol: f.tolerance_m } : null;
  const pose = mapPose(s);
  // The filter's last word stays drawn, said to be old: never "anchored" from silence.
  const silent = loc && ageS(s.lAt) > FRESH_S;
  const anchored = pose && !pose.placed && !silent;
  const out = [el("div", { class: "row" },
    el("b", {}, c.course || "no course"), el("span", { class: "note" }, c.from === "mission" ? "from the mission" : "from the deck"),
    !loc ? el("span", { class: "stale" }, "no localization")
      : silent ? el("span", { class: "stale" }, `localization silent ${num(ageS(s.lAt), 0)} s: last position drawn`)
      : anchored ? el("span", { class: "live" }, "anchored in the pool")
        : el("span", { class: "warn" }, `not anchored: drawn by its start (${s.start ? s.start.by : "--"})`))];
  if (c.error) { out.push(el("div", { class: "err" }, c.error)); return out; }
  const vel = [];
  if (forLocalization && pose && fin(pose.yaw)) {
    // Body-frame velocities (FRD), turned into the pool by the drawn heading.
    const turn = (vx, vy) => ({ vx: Math.cos(pose.yaw) * vx - Math.sin(pose.yaw) * vy, vy: Math.sin(pose.yaw) * vx + Math.cos(pose.yaw) * vy });
    const ot = ageS(s.odomAt) <= 1 && s.odom && s.odom.twist && s.odom.twist.twist && s.odom.twist.twist.linear;
    if (ot) vel.push({ ...turn(ot.x, ot.y), ink: "var(--mb-sea)" });
    const ft = ageS(s.flowAt) <= 1 && s.flow && s.flow.twist && s.flow.twist.twist && s.flow.twist.twist.linear;
    if (ft) vel.push({ ...turn(ft.x, ft.y), ink: "var(--mb-teal)" });
  }
  const map = poolMap(c, pose, s.track || [], s.fixes || [], target, { rays: forLocalization, vel, px: s.mapPx });
  out.push(map || el("div", { class: "note" }, "nothing placed yet: survey the pool (course_survey --corner, --prop, --floor)"));
  const un = c.unplaced || [];
  const where = pose ? `${num(pose.x, 2)}, ${num(pose.y, 2)} m  heading ${num(fin(pose.yaw) ? (pose.yaw * 180) / Math.PI : NaN, 0)} deg` : "--";
  const lastFix = (s.fixes || []).at(-1);
  const rows = [
    ["position", anchored ? where : pose ? `${where} (${silent ? "old" : "by its start"})` : "--", anchored ? "live" : "warn"],
    ["filter frame", loc ? `${loc.frame || "--"}  ${num(loc.x_m, 2)}, ${num(loc.y_m, 2)} m` : "--"],
    ["sigma", loc ? `${num(loc.sigma_xy_m, 2)} m  yaw ${num(loc.sigma_yaw_deg, 1)} deg` : "--"],
    ["usable", !loc || silent ? "--" : loc.usable ? "yes" : `no: ${loc.reason || "--"}`, loc && !silent && loc.usable ? "live" : "warn"],
    ["aiding", loc ? loc.aiding || "--" : "--"],
    ["going to", target ? `${num(target.x, 2)}, ${num(target.y, 2)} m within ${num(target.tol, 2, " m")}` : "--", target ? "live" : ""],
  ];
  if (forLocalization) {
    rows.push(["flow scale", s.flowCal || "--", s.flowCal === "calibrated" ? "live" : s.flowCal ? "warn" : ""],
      ["arrows", "2 s ahead: filter (sea), flow (teal); none while its source is silent", ageS(s.flowAt) > 1 ? "warn" : ""],
      ["last fix", lastFix ? `moved the estimate ${num(Math.hypot(lastFix.x - lastFix.fx, lastFix.y - lastFix.fy), 2, " m")}, ${num(ageS(lastFix.t), 0)} s ago` : "none yet"],
      ["motion", loc ? loc.motion || "--" : "--"],
      ["input ages", loc ? `imu ${num(loc.imu_age_s, 2)}  depth ${num(loc.depth_age_s, 2)}  flow ${num(loc.flow_age_s, 2)} s` : "--"]);
  } else {
    rows.push(["placed", `${(c.props || []).length} props, ${(c.props || []).filter((p) => !p.measured).length} of them nominal`],
      ["unplaced", un.length ? un.map((p) => p.name).join(", ") : "none", un.length ? "warn" : ""],
      ["pool", `depth ${num(c.pool && c.pool.depth_m, 2, " m")}  tile ${num(c.pool && c.pool.tile_m, 3, " m")}  floor stop ${num(c.pool && c.pool.shield_max_depth_m, 2, " m")}`],
      // A blank venue holds every mission back (flow has no floor): say how to fill it.
      ...(c.pool && c.pool.depth_m == null && !(c.pool.floor || []).length
        ? [["blank venue", "no floor, so flow refuses every interval: at the pool, `ros2 run mongla_localization course_survey`", "warn"]] : []),
      ["file", c.source || "--"]);
  }
  out.push(kv(...rows));
  return out;
}

// Built once above the redrawn part, so a click is never lost to a redraw:
// the operator says "Mongla is at the start now, facing the gate".
function startControl(s, done) {
  const b = el("button", {
    title: "Mongla sits in the starting zone, facing bearing 0 (the gate): draw it from here until it anchors",
    onclick: () => {
      const loc = s.loc;
      if (!loc || !fin(loc.x_m)) return;
      s.start = { x: loc.x_m, y: loc.y_m, yaw: rad(loc.yaw_deg) || 0, by: `set by the operator ${new Date().toLocaleTimeString()}` };
      s.track = [];
      done();
    },
  }, "At the start now");
  return el("div", { class: "row" }, b, el("span", { class: "note" }, "before the anchor, Mongla is drawn from where it started"));
}

// Seconds as "1:05", the way a run clock reads.
const clock = (s) => (fin(s) ? `${s < 0 ? "-" : ""}${Math.floor(Math.abs(s) / 60)}:${String(Math.floor(Math.abs(s) % 60)).padStart(2, "0")}` : "--");
const STATE_CLASS = { running: "live", done: "ok", refused: "warn", canceled: "warn", failed: "err", abandoned: "warn" };
const fieldsText = (f) => Object.entries(f || {}).map(([k, v]) => `${k}=${v === null ? "--" : typeof v === "number" ? +v.toFixed(3) : v}`).join("  ");

// The mission's lines, numbered, the running one marked and scrolled to once
// each time it moves. Pure but for that one scroll; exported for the test.
function missionText(s) {
  if (!s.src) return [el("div", { class: "note" }, "no /mongla/mission/source: the runner publishes the mission's text when one starts")];
  const live = ageS(s.aliveAt) < MISSION_ALIVE_S && s.missionName === s.src.name;
  const at = s.at && s.at.line;
  const file = String(s.src.file || "").split("/").pop();
  const head = el("div", { class: "row pin" }, el("b", { class: live ? "live" : "" }, s.src.name),
    el("span", { class: "note" }, `${file}${live ? "" : ", ended"}`),
    at ? el("span", { class: live ? "live" : "" }, `line ${at} sent ${s.at.cmd}, ${num(ageS(s.at.t), 0)} s ago`)
      : el("span", { class: "note" }, "no verb sent yet"));
  let mark = null;
  const rows = s.lines.map((tx, i) => {
    const n = i + 1;
    const r = el("div", { class: `ln${n === at ? " now" : ""}${n === s.src.first_line ? " entry" : ""}` },
      el("span", { class: "no" }, String(n)), el("span", { class: "tx" }, tx || " "));
    if (n === at) mark = r;
    return r;
  });
  const key = at ? `${at}@${s.at.t}` : `entry@${s.src.first_line}`;
  if (s.scrolled !== key && typeof requestAnimationFrame === "function") {
    s.scrolled = key;
    const target = mark || rows[Math.max(0, (s.src.first_line || 1) - 1)];
    requestAnimationFrame(() => target && target.scrollIntoView && target.scrollIntoView({ block: "center" }));
  }
  return [head, el("div", { class: "code" }, ...rows)];
}

function kv(...pairs) {
  return el("div", { class: "kv" }, ...pairs.flatMap(([k, v, cls]) => [el("span", {}, k), el("span", cls ? { class: cls } : {}, v)]));
}

// `filter` keeps the rows naming it; a leading "!" keeps every other row.
function healthRows(rows, filter) {
  const not = filter.startsWith("!"), key = not ? filter.slice(1) : filter;
  const shown = rows.filter((r) => !key || r.name.includes(key) !== not)
    .sort((a, b) => (b.level - a.level) || a.name.localeCompare(b.name));
  return shown.length
    // The level in words in its own column: colour alone left the six
    // unknown rows looking exactly like the good ones (2026-11-23).
    ? el("div", { class: "health" }, ...shown.flatMap((r) => {
      const lvl = LEVEL[Math.min(r.level, 3)] || "stale";
      return [
        el("i", { class: `lvl ${lvl}` }, LEVEL_WORD[lvl]),
        el("span", { class: "name" }, r.name.replace(/^mongla: /, "")),
        el("span", { class: r.level ? lvl : "", title: r.message }, r.message)];
    }))
    : el("div", { class: "note" }, "no /diagnostics rows yet: the manager grades health once it runs");
}

// The rows as of `now`. A dead manager's last green stayed green: a row not
// repeated for HEALTH_STALE_S reads unknown, with its age and its last word.
// Exported for the test.
function healthNow(s, now) {
  return (s.health || []).map((r) => {
    const age = (now - (r.at || 0)) / 1000;
    return age > HEALTH_STALE_S ? { ...r, level: 3, message: `silent ${num(age, 0)} s (last: ${r.message})` } : r;
  });
}
// The manager is heard while any of its rows is fresh: what a latched
// record of its own (/mongla/doing) is current by.
const managerHeard = (s, now) => (s.health || []).some((r) => (now - (r.at || 0)) / 1000 <= HEALTH_STALE_S);

function foldDiagnostics(s, m) {
  s.health = s.health || [];
  for (const st of m.status || []) {
    const i = s.health.findIndex((h) => h.name === st.name);
    const row = { name: st.name, level: typeof st.level === "number" ? st.level : Number(st.level), message: st.message, at: heard() };
    if (i < 0) s.health.push(row); else s.health[i] = row;
  }
}

// The Vehicle card's content, centred in its panel by `.vehicle`. Under the
// state, the board lines an operator checks before arming, graded by the
// manager (/diagnostics): absent means not graded yet, never fine.
const BOARD_LINES = [["link", "board_link"], ["leak", "leak"], ["kill line", "thruster_power"],
  ["barometer", "barometer"], ["heading ref", "heading_ref"], ["safety latch", "safety_latch"]];

function vehicleBody(s, v, h, below) {
  // The manager names each row "mongla: <line>".
  const rows = healthNow(s, Date.now());
  const graded = (key) => rows.find((r) => r.name === key || r.name.endsWith(`: ${key}`));
  const board = BOARD_LINES.map(([label, key]) => {
    const r = graded(key);
    if (!r) return [label, "--", "stale", "not graded yet"];
    const lvl = LEVEL[Math.min(r.level, 3)];
    // A good line says "ok"; only a line with something to say spends the room.
    // One word a line, the manager's sentence on hover (Health has it whole).
    const word = LEVEL_WORD[lvl];
    return [label, word, lvl === "ok" ? "good" : lvl, r.message];
  });
  return [
    el("div", { class: "row" },
      ageS(s.at) > FRESH_S
        ? el("span", { class: "warn" }, `${v.armed ? "ARMED" : "disarmed"}, ${num(ageS(s.at), 0)} s old`)
        : el("span", { class: v.armed ? "armed" : "ok" }, v.armed ? "ARMED" : "disarmed"),
      el("span", {}, v.mode || "--"),
      h.mission && el("span", { class: "live" }, `mission ${h.mission}`)),
    el("div", { class: "row gauges" },
      compass(v.yaw_deg), depthGauge(below, 3),
      el("div", { class: "readouts" },
        el("span", { class: "big" }, num(below, 2), el("span", { class: "unit" }, "m deep")),
        el("span", { class: "big" }, num(v.yaw_deg, 0), el("span", { class: "unit" }, "deg")),
        el("span", { class: "big" }, num(v.battery_voltage, 1), el("span", { class: "unit" }, "V")))),
    kv(ageS(s.aliveAt) < MISSION_ALIVE_S ? ["mission", `running ${s.missionName || ""}`, "live"] : ["mission", "none"],
      ["pilot", h.pilot || "nobody"], ["state age", `${num(ageS(s.at), 1)} s`, ageS(s.at) > 2 ? "warn" : ""],
      // Silence is "no order", never a default (node_vision._publish_flare_order).
      ["flare order", s.flare ? (s.flare.colours || []).join("-") : "none latched", s.flare ? "live" : ""]),
    el("div", { class: "rule" }, "board"),
    // One cell per line, label and word together: cells wrap whole, so a
    // narrow card shows fewer per row and never cuts a word.
    el("div", { class: "board" }, ...board.map(([k, val, cls, full]) =>
      el("span", { class: "cell", title: full }, el("span", {}, k), el("b", { class: cls || "" }, val)))),
  ];
}

// How late this screen is: arrival here minus the time Kutum's bridge took the
// vehicle state (Lichtblick's receiveTime; the state's header stamp only when
// there is none). The header is the board's capture time, which is data age,
// not this screen's; on 2026-11-23 the strip read a steady 0.56 s from it
// while a raw socket in the same page read 17 ms. foxglove_bridge
// sends a client its messages in arrival order; when the pictures outrun the
// link, every message waits behind them, each still arriving, each seconds
// old, so a gap between arrivals cannot see it (2026-11-23). Arrival minus
// stamp is that wait plus the two clocks' offset; the least of it over the
// last minute is the offset plus the link's own latency, so the difference
// is the queue. A lag that lasts longer than the window becomes the new
// floor and reads 0: a minute is longer than any queue the bridge holds.
// Exported for the test.
const LAG_WINDOW_MS = 60000;
function noteLag(s, at, stamp) {
  if (!stamp || !fin(stamp.sec)) return;
  const off = at - (stamp.sec * 1000 + (stamp.nanosec ?? stamp.nsec ?? 0) / 1e6);
  if (!fin(off)) return;
  const w = (s.lagWin = (s.lagWin || []).filter(([t]) => at - t <= LAG_WINDOW_MS));
  w.push([at, off]);
  let least = Infinity;
  for (const [, o] of w) least = Math.min(least, o);
  s.lag = (off - least) / 1000;
}

// When a screen reports its lag to the helm: every 2 s, and not in its first
// 15 s, while a page that has just loaded lags as every panel mounts; one new
// screen must not cut every screen's video for the half minute it takes to
// climb back. Exported for the test.
const REPORT_MS = 2000, WARM_MS = 15000;
// This page's own short id, so the helm's log and every strip can say which
// screen cut the shared video; the operator closes that one.
const SCREEN = Math.floor(Math.random() * 0x10000).toString(16).padStart(4, "0");
// The shared video's scale, and which screen cut it: "this screen" when it
// was this one, so the operator knows which to close.
function videoWords(scale, by, me = SCREEN) {
  if (!fin(scale)) return "video --";
  if (scale >= 1 || !by) return `video x${scale}`;
  return `video x${scale} set by ${by === me ? "this screen" : `screen ${by}`}`;
}

function shouldReport(s, now) {
  // A lag frozen at the state's last arrival is not this screen's lag now.
  if (!fin(s.lag) || !(now - (s.vAt || 0) <= FRESH_S * 1000)) return false;
  if (!fin(s.mountAt)) s.mountAt = now;
  if (now - s.mountAt < WARM_MS || now - (s.lagSent || -Infinity) < REPORT_MS) return false;
  s.lagSent = now;
  return true;
}

// A link's quality as four bars, from the number that says it: three
// thresholds, at or under the first is four bars, over the last is one; no
// number is no bars, never four. Lag in seconds (the strip's warn at 0.5 and
// error at 2 are the second and third edges); the board's drops in percent
// (warn over 1). Exported for the test.
const LAG_BARS = [0.15, 0.5, 2], DROP_BARS = [0.1, 0.5, 1];
function signalBars(x, edges) {
  if (!fin(x)) return 0;
  return 4 - edges.filter((e) => x > e).length;
}

// One status cell: its name, its value, and, for a link, the bars.
function statusCell([k, val, lvl, bars]) {
  const meter = bars === undefined ? null
    : el("i", { class: `sig s${bars}`, title: `${bars} of 4` }, el("span"), el("span"), el("span"), el("span"));
  return el("div", { class: `cell ${lvl}` }, el("small", {}, k), el("b", {}, meter, val));
}

// The strip's fold, and what every strip does each draw besides its cells:
// mark the console's tabs, and report this screen's lag to the helm, which
// sets the viewers' video for the slowest screen (link.py). Whichever strip
// a console carries (Status, or Watch on the pilot console) does both: a
// console with neither had no tab marks and never reported its lag.
function statusFold(s, topic, m) {
  if (topic === "/mongla/state") { s.v = m; s.vAt = heard(); noteLag(s, heard(), sent() || (m.header && m.header.stamp)); }
  else if (topic === "/mongla/batighor/helm_state") { try { const h = JSON.parse(m.data); s.video = h.video; s.videoBy = h.video_by || ""; } catch { /* keep */ } }
  else if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); }
  else if (topic === "/mongla/mission/task") { try { s.task = JSON.parse(m.data); } catch { /* keep */ } }
  else if (topic === "/mongla/doing") { try { s.doing = JSON.parse(m.data); } catch { /* keep */ } }
  else if (topic === "/diagnostics") foldDiagnostics(s, m);
  else if (topic === "/mongla/localization/state") { s.loc = m; s.locAt = heard(); }
  else if (topic === "/mongla/board_link") { s.link = m; s.linkAt = heard(); }
  else {
    const c = CAMERAS.find((x) => topic === lockTopic(x));
    if (c) { s.locks = s.locks || {}; s.locks[c] = { m, at: heard() }; }
  }
}

function statusDuties(s, tabs) {
  if (typeof callAny === "function" && shouldReport(s, Date.now())) {
    callAny("/mongla/batighor/helm", { who: "", op: "link", cmd: "", mission: "", confirms: [],
      fields_json: JSON.stringify({ lag_s: Math.round(s.lag * 1000) / 1000, screen: SCREEN }) }).catch(() => { /* no helm */ });
  }
  if (typeof document !== "undefined" && document.querySelectorAll) {
    for (const tab of document.querySelectorAll('[data-testid="toolbar-tab"]')) {
      const lvl = tabs[tab.getAttribute("title")] || "";
      if (tab.dataset.mb !== lvl) tab.dataset.mb = lvl;
    }
  }
}

const STATUS_TOPICS = ["/mongla/state", "/mongla/mission_alive", "/mongla/mission/task", "/mongla/doing", "/diagnostics",
  "/mongla/localization/state", "/mongla/board_link", "/mongla/batighor/helm_state"];
// What the pilot console's strip keeps: the rest is on Vitals, below.
const WATCH_CELLS = ["mission", "verb", "lock", "link", "to kutum"];

// A camera's caption, from the overlay's `<camera>/hud` (overlay.caption):
// what the strip under the picture says. Pure: exported for the test. The
// rate's last half minute is kept for the sparkline; a caption older than
// CAPTION_STALE_S is the overlay gone quiet, said, never the last word kept
// as if current.
const CAPTION_STALE_S = 3, RATE_KEEP_S = 30;
const CAPTION_WORDS = {
  running: "detecting", paused: "paused", silent: "silent", none: "no detector", unread: "silent",
};
function captionOf(s, cam, now) {
  const c = s.caps && s.caps[cam];
  if (!c) return { cam, state: "unheard", word: "no overlay", level: "stale", rates: [] };
  const ageS = (now - c.at) / 1000;
  const rates = (c.rates || []).filter(([t]) => now - t <= RATE_KEEP_S * 1000);
  if (ageS > CAPTION_STALE_S) {
    return { cam, state: "stale", word: `overlay silent ${num(ageS, 0)} s`, level: "stale", model: c.m.model, rates };
  }
  const m = c.m;
  const level = m.alarm ? "err" : m.state === "paused" ? "warn" : "ok";
  const why = m.state === "paused" ? `${num(m.silent_s, 0)} s, runs when a mission asks`
    : m.state === "running" ? "" : m.state === "none" ? "nothing publishes its detections"
      : `${num(m.silent_s, 0)} s, NOT paused`;
  return { cam, state: m.state, word: CAPTION_WORDS[m.state] || m.state, level, why, model: m.model || "",
    size: m.size ? `${m.size[0]}x${m.size[1]}` : "--", rate: m.rate_hz, late: m.late_ms, rates };
}

function foldCaption(s, m, cam, at) {
  let c;
  try { c = JSON.parse(m.data); } catch { return; }
  s.caps = s.caps || {};
  const was = s.caps[cam] || { rates: [] };
  const rates = was.rates.filter(([t]) => at - t <= RATE_KEEP_S * 1000);
  if (fin(c.rate_hz)) rates.push([at, c.rate_hz]);
  else if (c.state !== "running") rates.push([at, 0]);
  s.caps[cam] = { m: c, at, rates };
}

// The rate's last half minute as a line, on its own peak; nothing heard,
// nothing drawn.
function rateSpark(rates, now) {
  if (rates.length < 2) return null;
  const peak = Math.max(1, ...rates.map(([, r]) => r));
  const pts = rates.map(([t, r]) => `${num(100 - (now - t) / (RATE_KEEP_S * 10), 1)},${num(18 - 16 * r / peak, 1)}`);
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 100 20");
  svg.setAttribute("preserveAspectRatio", "none");
  svg.setAttribute("class", "spark");
  const line = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
  line.setAttribute("points", pts.join(" "));
  svg.append(line);
  return svg;
}

function captionStrip(g, now) {
  const chip = (k, v) => el("span", { class: "chip" }, el("small", {}, k), el("b", {}, v));
  return el("div", { class: `cap ${g.level}` },
    el("span", { class: "cam" }, el("i", { class: "eye" }), g.cam),
    el("span", { class: "state" }, el("i", { class: "dot" }), g.word.toUpperCase(),
      g.why ? el("em", {}, g.why) : null),
    g.model !== undefined ? chip("model", g.model || "none named") : null,
    g.state === "running" ? chip("rate", `${num(g.rate, 0)} Hz`) : null,
    g.state === "running" ? chip("behind", `${num(g.late, 0)} ms`) : null,
    typeof document !== "undefined" ? rateSpark(g.rates, now) : null,
    g.size ? chip("frame", g.size) : null);
}

// -- the table -----------------------------------------------------------

// The status strip's cells and the console's tab marks, from one fold. Pure:
// exported for the test. A level is "" (nothing to say), "ok", "live" (it is
// happening: armed, a mission running, a lock held), "warn" or "err".
function statusOf(s, now) {
  const age = (t) => (t ? (now - t) / 1000 : Infinity);
  const v = s.v, cells = [], tabs = {};
  const fresh = v && age(s.vAt) < 2;
  // Gone quiet: the last word stays, with its age, as a warning. A starved
  // link (pictures filling a slow deck connection) showed a calm "disarmed"
  // while nothing had arrived for seconds (2026-11-23).
  const armedWord = !v ? "--" : v.armed ? "ARMED" : "disarmed";
  cells.push(["vehicle", v && !fresh ? `${armedWord}  ${num(age(s.vAt), 1)} s old` : armedWord,
    !v ? "stale" : !fresh ? "warn" : v.armed ? "armed" : "ok"]);
  cells.push(["mode", v ? v.mode || "--" : "--", ""]);
  cells.push(["depth", v && fin(v.depth_m) ? `${num(-v.depth_m, 2)} m` : "--", ""]);
  cells.push(["heading", v ? `${num(v.yaw_deg, 0)} deg` : "--", ""]);
  cells.push(["battery", v ? `${num(v.battery_voltage, 1)} V` : "--", ""]);
  const running = age(s.aliveAt) < MISSION_ALIVE_S;
  const task = s.task && running ? ` / ${s.task.task || s.task.name || ""}` : "";
  cells.push(["mission", running ? `${s.missionName || "running"}${task}` : "none", running ? "live" : ""]);
  const doing = s.doing && s.doing.now;
  // /mongla/doing is latched, sent on change: current only while the manager is heard.
  const heardMgr = managerHeard(s, now);
  cells.push(["verb", !s.doing ? "--" : !heardMgr ? `${doing ? doing.verb : "idle"}, manager silent` : doing ? doing.verb : "idle",
    !s.doing ? "stale" : !heardMgr ? "warn" : doing ? "live" : ""]);
  const rungs = CAMERAS.map((c) => [c, s.locks && age(s.locks[c] && s.locks[c].at) < 2 ? rungOf(s.locks[c].m).rung : null]);
  const held = rungs.filter(([, r]) => r && r !== "lost");
  cells.push(["lock", held.length ? held.map(([c, r]) => `${c} ${r}`).join("  ") : rungs.every(([, r]) => r) ? "lost" : "--", held.length ? "live" : ""]);
  const loc = s.loc && age(s.locAt) < 2 ? s.loc : null;
  const anchored = loc && loc.anchored && loc.frame === "pool";
  cells.push(["navigation", !loc ? "--" : anchored ? `anchored  ${num(loc.sigma_xy_m, 2)} m` : loc.usable ? `${loc.aiding || "aided"}` : `unusable`, !loc ? "stale" : anchored ? "ok" : loc.usable ? "" : "warn"]);
  // ROS levels: 0 ok, 1 warn, 2 error, 3 stale. A row with no data is
  // unknown, never a fault: the bench has no barometer, not a broken one.
  const rows = healthNow(s, now);
  const count = (lvl) => rows.filter((r) => r.level === lvl).length;
  const [errs, warns, unknown] = [count(2), count(1), count(3)];
  const said = [errs && `${errs} error${errs > 1 ? "s" : ""}`, warns && `${warns} warning${warns > 1 ? "s" : ""}`,
    unknown && `${unknown} unknown`].filter(Boolean).join("  ");
  const worst = !rows.length ? -1 : errs ? 2 : warns ? 1 : unknown ? 3 : 0;
  cells.push(["health", worst < 0 ? "--" : said || "all ok", worst < 0 ? "stale" : LEVEL[worst]]);
  const l = s.link && age(s.linkAt) < 3 ? s.link : null;
  cells.push(["link", l ? `${num(100 * l.bytes_per_s / (l.capacity_bytes_per_s || NaN), 0)} %  drops ${num(l.board_drop_pct, 1)} %` : "--", !l ? "stale" : l.board_drop_pct > 1 ? "warn" : "ok",
    l ? signalBars(l.board_drop_pct, DROP_BARS) : 0]);
  // This screen's link to Kutum, apart from the board's line above: the lag
  // and the scale the helm put the viewers' video at for it (link.py).
  // The lag is measured on the state's arrival; with the state silent it is unknown.
  const lag = fresh ? s.lag : NaN;
  const lagLvl = !fin(lag) ? "stale" : lag > 2 ? "err" : lag > 0.5 ? "warn" : "ok";
  cells.push(["to kutum", fin(lag) ? `lag ${num(lag, 2)} s  ${videoWords(s.video, s.videoBy)}` : "--", lagLvl,
    signalBars(lag, LAG_BARS)]);
  tabs.Cockpit = v && v.armed ? "live" : "";
  tabs.Thrust = v && v.armed ? "live" : "";
  tabs.Mission = running ? "live" : "";
  tabs.Vision = held.length ? "live" : "";
  tabs.Map = tabs.Localization = !loc ? "" : anchored ? "ok" : loc.usable ? "" : "warn";
  tabs.Health = worst < 0 ? "" : LEVEL[Math.min(worst, 3)] === "ok" ? "ok" : LEVEL[Math.min(worst, 3)];
  tabs.Link = lagLvl === "err" || lagLvl === "warn" ? lagLvl : !l ? "" : l.board_drop_pct > 1 ? "warn" : "";
  return { cells, tabs };
}

// -- Vitals: the vehicle at a glance, drawn to be read from the corner of an eye
// A redraw replaces the drawing, which restarts a CSS animation; a negative
// delay taken from the clock puts each one back where it was, so the water
// keeps flowing across redraws instead of stuttering at every message.
function phase(ms) { return `animation-delay:-${Date.now() % ms}ms`; }

const HEALTH_FILL = { ok: "var(--mb-teal)", warn: "var(--mb-amber)", err: "var(--mb-red)", stale: "var(--mb-ink-3)" };

// Pure, for the test: what the drawing says, from the folded state.
// Roll and pitch in degrees from /mongla/imu: body FRD, world NED (the
// board's convention, node_telemetry), so roll + is starboard down and pitch
// + is nose up. A covariance of -1 is "no orientation", which is null here.
function attitudeOf(m) {
  const q = m && m.orientation, c = m && m.orientation_covariance;
  if (!q || (c && c[0] < 0) || ![q.w, q.x, q.y, q.z].every(fin)) return null;
  const { w, x, y, z } = q, D = 180 / Math.PI;
  return { roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * D,
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * D };
}

// The last DEPTH_TRACE_S of depth below the surface, for the trace and rate.
const DEPTH_TRACE_S = 60;
function noteDepth(s, depth_m, at) {
  if (!fin(depth_m)) return;
  const h = (s.dh = s.dh || []);
  h.push([at, -depth_m]);
  while (h.length && (at - h[0][0]) / 1000 > DEPTH_TRACE_S) h.shift();
}
// Metres a second, + going down, over the last 2 s; NaN with under 1 s heard.
function depthRate(h, now) {
  const r = (h || []).filter(([t]) => (now - t) / 1000 <= 2);
  if (r.length < 2 || (r[r.length - 1][0] - r[0][0]) / 1000 < 1) return NaN;
  return (r[r.length - 1][1] - r[0][1]) / ((r[r.length - 1][0] - r[0][0]) / 1000);
}

function vitalsOf(s, now) {
  const v = s.v || {};
  const stale = !s.vAt || (now - s.vAt) / 1000 > FRESH_S;
  const d = s.dAt && (now - s.dAt) / 1000 <= FRESH_S ? s.demand || {} : {};
  const rows = healthNow(s, now);
  const health = BOARD_LINES.map(([label, key]) => {
    const r = rows.find((x) => x.name === key || x.name.endsWith(`: ${key}`));
    return [label, r ? LEVEL[Math.min(r.level, 3)] : "stale", r ? r.message : "not graded yet"];
  });
  const rpm = s.rAt && (now - s.rAt) / 1000 <= FRESH_S ? s.rpm || [] : [];
  return {
    stale, ageS: s.vAt ? (now - s.vAt) / 1000 : NaN,
    armed: !!v.armed, mode: v.mode || "", yaw: v.yaw_deg, below: fin(v.depth_m) ? -v.depth_m : NaN,
    surge: d.x, sway: d.y, health, rpm, peak: Math.max(1000, ...rpm.map((r) => Math.abs(r))),
    pack: s.pAt && (now - s.pAt) / 1000 <= FRESH_S && s.pack ? s.pack.voltage : NaN,
    rail: v.battery_voltage,
    att: s.aAt && (now - s.aAt) / 1000 <= FRESH_S ? s.att : null,
    trace: stale ? [] : (s.dh || []).map(([t, d]) => [(t - now) / 1000, d]),
    rate: stale ? NaN : depthRate(s.dh, now),
    // Where the filter puts the hull, and whether it says to trust it.
    nav: s.locAt && (now - s.locAt) / 1000 <= FRESH_S && s.loc ? {
      x: s.loc.x_m, y: s.loc.y_m, sigma: s.loc.sigma_xy_m, frame: s.loc.frame,
      usable: !!s.loc.usable, why: s.loc.reason || s.loc.aiding || "",
    } : null,
  };
}

// The navigation reason's room on the Vitals drawing: from x 400 to the
// divider at 672 less a margin, 250 units at 11 px mono, which renders at
// about 7.2 units a character (38 ran past it on the ScreenPad, 2026-11-24).
const NAV_WHY_CHARS = 34;
// A string cut to `n` characters, the cut marked; whole if it fits.
function fitText(s, n) {
  s = String(s || "");
  return s.length <= n ? s : `${s.slice(0, n - 1).trimEnd()}\u2026`;
}

// The chain drawn: two camera lanes (frame, detect, track, lock) meeting at
// the frame-to-command stage, then the demand and the board's half along the
// bottom, right to left. One box per stage of chain.py's message, placed by
// its key; what the board does not send is a dashed box naming its ask, so
// the gap is on the drawing, not only in a list (bead 318.2: the raw topic
// graph flattened ~90 nodes into one unreadable band).
const CHAIN_AT = {
  "frame:forward": [1, 0], "detect:forward": [2, 0], "track:forward": [3, 0], "lock:forward": [4, 0],
  "frame:downward": [1, 1], "detect:downward": [2, 1], "track:downward": [3, 1], "lock:downward": [4, 1],
  command: [5, 0.5], demand: [5, 2], "board loop": [4, 2], mixer: [3, 2], rpm: [2, 2], imu: [1, 2],
  "camera trigger": [0, 0.5],
};
const CHAIN_LINKS = [
  ["camera trigger", "frame:forward"], ["camera trigger", "frame:downward"],
  ["frame:forward", "detect:forward"], ["detect:forward", "track:forward"], ["track:forward", "lock:forward"],
  ["frame:downward", "detect:downward"], ["detect:downward", "track:downward"], ["track:downward", "lock:downward"],
  ["lock:forward", "command"], ["lock:downward", "command"], ["command", "demand"],
  ["demand", "board loop"], ["board loop", "mixer"], ["mixer", "rpm"],
];
// Each stage as a box: where, what it says, and its state. `live` is heard
// in the window; `idle` is silent with a stated reason (a detector paused
// until a mission asks, a command until a vision verb drives); `silent` is
// silent with none, the alarm; `absent` is not sent by the board at all.
function chainBoxes(c) {
  const out = [];
  if (!c) return out;
  for (const k of c.order || []) {
    const r = c.stages[k], at = CHAIN_AT[k];
    if (!r || !at) continue;
    const quiet = !(r.n > 0) || (fin(r.last_s) && r.last_s > 2);
    const state = !quiet ? "live" : r.idle ? "idle" : "silent";
    out.push({ key: k, at, label: r.label, state, hz: fin(r.hz) && r.n > 1 ? r.hz : null,
      p50: quiet ? null : r.age_ms_p50, p95: quiet ? null : r.age_ms_p95, why: r.idle || r.ask || "" });
  }
  for (const n of c.not_sent || []) {
    const at = CHAIN_AT[n.label];
    if (at) out.push({ key: n.label, at, label: n.label, state: "absent", hz: null, p50: null, p95: null, why: n.ask });
  }
  return out;
}

const CHAIN_W = 120, CHAIN_H = 54, CHAIN_DX = 132, CHAIN_DY = 78;
function chainSvg(boxes) {
  const where = {};
  for (const b of boxes) where[b.key] = [8 + b.at[0] * CHAIN_DX, 8 + b.at[1] * CHAIN_DY];
  const worst = Math.max(1, ...boxes.map((b) => (fin(b.p50) ? b.p50 : 0)));
  const s = svg("svg", { class: "chain", viewBox: `0 0 ${8 + 6 * CHAIN_DX} ${8 + 2 * CHAIN_DY + CHAIN_H + 8}` },
    svg("defs", {}, svg("marker", { id: "chain-head", viewBox: "0 0 8 8", refX: 7, refY: 4, markerWidth: 7, markerHeight: 7, orient: "auto" },
      svg("path", { d: "M0,0 L8,4 L0,8 z", class: "head" }))));
  for (const [a, b] of CHAIN_LINKS) {
    if (!where[a] || !where[b]) continue;
    const [ax, ay] = where[a], [bx, by] = where[b];
    // Edge to edge: across a row from side to side, down from bottom to top.
    let x1 = ax + CHAIN_W, y1 = ay + CHAIN_H / 2, x2 = bx, y2 = by + CHAIN_H / 2;
    if (bx < ax) { x1 = ax; x2 = bx + CHAIN_W; }
    else if (bx === ax) { x1 = x2 = ax + CHAIN_W / 2; y1 = ay + CHAIN_H; y2 = by; }
    s.append(svg("line", { x1, y1, x2, y2, class: "link", "marker-end": "url(#chain-head)" }));
  }
  for (const b of boxes) {
    const [x, y] = where[b.key];
    const words = b.state === "live" ? [fin(b.hz) ? `${num(b.hz, 1)} Hz` : "heard", fin(b.p50) ? `${num(b.p50, 0)} / ${num(b.p95, 0)} ms` : ""]
      : b.state === "idle" ? ["idle", fitText(b.why, 19)]
        : b.state === "absent" ? ["not sent", fitText(b.why.split(":")[0], 19)]
          : ["SILENT", "no reason given"];
    const g = svg("g", { class: `stage ${b.state}` },
      svg("title", {}, `${b.label}: ${b.why || b.state}`),
      svg("rect", { x, y, width: CHAIN_W, height: CHAIN_H, rx: 3 }),
      svg("text", { x: x + 8, y: y + 15, class: "name" }, fitText(b.label, 18)),
      svg("text", { x: x + 8, y: y + 32, class: "big" }, words[0]),
      svg("text", { x: x + 8, y: y + 46, class: "small" }, words[1]));
    if (fin(b.p50)) g.append(svg("rect", { x, y: y + CHAIN_H - 3, width: Math.max(2, (CHAIN_W * b.p50) / worst), height: 3, class: "age" }));
    s.append(g);
  }
  return s;
}

function vitalsSvg(g) {
  const T = (x, y, txt, a = {}) => svg("text", { x, y, ...a }, txt);
  // Depth: a water column from the surface to 3 m (deeper, to the vehicle).
  const floor = Math.max(3, fin(g.below) ? g.below : 0);
  const top = 46, bot = 236, colY = fin(g.below) ? top + ((bot - top) * Math.max(0, g.below)) / floor : null;
  const wave = "M0 0 q6 -4 12 0 t12 0 t12 0 t12 0 t12 0";
  const depth = [
    T(47, 26, "DEPTH", { class: "lbl", "text-anchor": "middle" }),
    svg("rect", { x: 34, y: top, width: 26, height: bot - top, rx: 3, class: "well" }),
    colY !== null && svg("rect", { x: 34, y: top, width: 26, height: colY - top, class: "water" }),
    svg("g", { transform: `translate(30 ${top})` },
      svg("path", { d: wave, class: "wave", style: phase(1600) })),
    colY !== null ? svg("path", { d: `M24 ${colY} l10 -6 v12 z`, class: "hullmark" })
      : T(47, (top + bot) / 2, "NO DEPTH HEARD", { class: "absent", "text-anchor": "middle",
        transform: `rotate(-90 47 ${(top + bot) / 2})` }),
    T(47, 266, fin(g.below) ? g.below.toFixed(2) : "--", { class: "num", "text-anchor": "middle" }),
    T(47, 282, "m", { class: "unit", "text-anchor": "middle" }),
  ];
  // Heading up, as from inside: the ring turns, the hull stays; a red mark
  // is where the nose points.
  const cx = 250, cy = 150, R = 112;
  const ring = [];
  for (let a = 0; a < 360; a += 10) {
    const r = (a * Math.PI) / 180, long = a % 30 === 0;
    ring.push(svg("line", { x1: cx + (R - (long ? 12 : 6)) * Math.sin(r), y1: cy - (R - (long ? 12 : 6)) * Math.cos(r),
      x2: cx + R * Math.sin(r), y2: cy - R * Math.cos(r), class: long ? "tick long" : "tick" }));
  }
  for (const [a, n] of [[0, "N"], [90, "E"], [180, "S"], [270, "W"]]) {
    const r = (a * Math.PI) / 180;
    ring.push(T(cx + (R - 26) * Math.sin(r), cy - (R - 26) * Math.cos(r) + 5, n,
      { class: a === 0 ? "card north" : "card", "text-anchor": "middle",
        transform: `rotate(${fin(g.yaw) ? g.yaw : 0} ${cx + (R - 26) * Math.sin(r)} ${cy - (R - 26) * Math.cos(r)})` }));
  }
  // The hull: 702 x 176 mm, nose up. Demand as water pushed past it.
  const L = 128, W = 32, hx = cx - W / 2, hy = cy - L / 2;
  const arrow = (x1, y1, x2, y2) => svg("line", { x1, y1, x2, y2, class: "flow", style: phase(600), "marker-end": "url(#mb-tip)" });
  const thrust = [];
  if (fin(g.surge) && Math.abs(g.surge) > 0.02) {
    const len = 70 * Math.min(1, Math.abs(g.surge));
    thrust.push(g.surge > 0 ? arrow(cx, hy - 4, cx, hy - 4 - len) : arrow(cx, hy + L + 4, cx, hy + L + 4 + len));
  }
  if (fin(g.sway) && Math.abs(g.sway) > 0.02) {
    const len = 70 * Math.min(1, Math.abs(g.sway)), sx = g.sway > 0 ? hx + W + 4 : hx - 4;
    thrust.push(arrow(sx, cy, sx + Math.sign(g.sway) * len, cy));
  }
  const compass = [
    svg("defs", {}, svg("marker", { id: "mb-tip", viewBox: "0 0 10 10", refX: 5, refY: 5, markerWidth: 5, markerHeight: 5, orient: "auto-start-reverse" },
      svg("path", { d: "M0 0 L10 5 L0 10 z", class: "tip" }))),
    svg("circle", { cx, cy, r: R, class: "ring" }),
    svg("g", { transform: `rotate(${fin(g.yaw) ? -g.yaw : 0} ${cx} ${cy})`, class: fin(g.yaw) ? "" : "unknown" }, ...ring),
    svg("path", { d: `M${cx} ${cy - R - 2} l-7 -12 h14 z`, class: "lubber" }),
    g.armed && svg("rect", { x: hx - 6, y: hy - 6, width: W + 12, height: L + 12, rx: (W + 12) / 2, class: "glow", style: phase(2400) }),
    svg("rect", { x: hx, y: hy, width: W, height: L, rx: W / 2, class: g.armed ? "hull armed" : "hull" }),
    svg("line", { x1: cx, y1: hy + 10, x2: cx, y2: hy + 26, class: "nose" }),
    ...thrust,
    !fin(g.surge) && T(cx, cy + R + 30, "thrust: the board's own move", { class: "lbl", "text-anchor": "middle" }),
  ];
  // The words: what it is, which way, on what.
  const state = g.stale ? (fin(g.ageS) ? `STATE ${Math.round(g.ageS)} s OLD` : "NO STATE") : g.armed ? "ARMED" : "DISARMED";
  const words = [
    T(400, 62, state, { class: `state ${g.stale ? "stale" : g.armed ? "armed" : ""}` }),
    T(400, 92, g.mode ? g.mode.replace("_", " ") : "mode --", { class: `mode ${/AUTO|DEPTH/.test(g.mode) ? "hold" : g.mode === "SURFACE" ? "surf" : ""}` }),
    T(400, 126, "HEADING", { class: "lbl" }),
    T(400, 166, fin(g.yaw) ? `${Math.round(((g.yaw % 360) + 360) % 360)}°` : "--", { class: "big" }),
    T(400, 196, "THRUSTER PACK", { class: "lbl" }), T(530, 196, "RAIL", { class: "lbl" }),
    T(400, 222, fin(g.pack) ? `${g.pack.toFixed(1)} V` : "-- V", { class: "num" }),
    T(530, 222, fin(g.rail) ? `${g.rail.toFixed(1)} V` : "-- V", { class: "num" }),
    // Position, north and east in the filter's frame (pool once anchored).
    T(560, 126, "POSITION", { class: "lbl" }),
    ...(g.nav && fin(g.nav.x) ? [
      T(560, 148, `N ${g.nav.x >= 0 ? " " : ""}${g.nav.x.toFixed(2)}`, { class: "num" }),
      T(560, 170, `E ${g.nav.y >= 0 ? " " : ""}${g.nav.y.toFixed(2)}`, { class: "num" }),
      // Right-aligned to the drawing's edge: "odom_ned" ran past it.
      T(652, 186, fin(g.nav.sigma) ? `\u00b1${g.nav.sigma.toFixed(2)} m ${g.nav.frame}` : g.nav.frame,
        { class: "unit", "text-anchor": "end" }),
    ] : [T(560, 150, g.nav ? "no position yet" : "no filter", { class: "absent" })]),
    // Usable is one line; not usable says so, then why, cut to the width
    // (the whole sentence on hover): one line of both ran off the edge.
    ...(!g.nav ? [] : g.nav.usable ? [T(400, 284, "NAVIGATION USABLE", { class: "nav ok" })] : [
      T(400, 276, "NAVIGATION NOT USABLE", { class: "nav warn" }),
      svg("text", { x: 400, y: 291, class: "unit why" }, svg("title", {}, g.nav.why), fitText(g.nav.why, NAV_WHY_CHARS)),
    ]),
  ];
  // One spinner per ESC the board reports; speed and sense from its rpm.
  const escs = g.rpm.length ? g.rpm.map((r, i) => {
    const x = 410 + i * 28, y = 252, f = Math.abs(r) / g.peak;
    const blades = svg("g", { class: f > 0.01 ? "spin" : "",
      style: f > 0.01 ? `animation-duration:${(0.3 + 1.7 * (1 - f)).toFixed(2)}s;animation-direction:${r < 0 ? "reverse" : "normal"};${phase(1000)}` : "" },
      svg("path", { d: `M${x} ${y - 9} L${x + 3} ${y} L${x - 3} ${y} z M${x} ${y + 9} L${x - 3} ${y} L${x + 3} ${y} z`, class: "blade" }));
    return svg("g", {}, svg("circle", { cx: x, cy: y, r: 11, class: "esc" }), blades);
  }) : [T(400, 256, "ESCs not announced", { class: "lbl" })];
  // The board's six lines along the bottom; a bad one breathes.
  const health = g.health.map(([label, lvl, msg], i) => {
    const x = 14 + i * 128;
    return svg("g", {}, svg("title", {}, msg),
      svg("circle", { cx: x + 6, cy: 306, r: 5, fill: HEALTH_FILL[lvl], class: lvl === "err" || lvl === "warn" ? "alarm" : "", style: phase(1200) }),
      T(x + 16, 311, label, { class: `hl ${lvl}` }));
  });
  // The right column: attitude as a horizon, and depth as a trace with its
  // rate. Neither is on the main screen.
  const ax = 735, ay = 98, AR = 50, K = AR / 30;     // 30 deg of pitch to the rim
  const att = g.att;
  const horizon = [
    T(ax, 26, "ATTITUDE", { class: "lbl", "text-anchor": "middle" }),
    svg("defs", {}, svg("clipPath", { id: "mb-ball" }, svg("circle", { cx: ax, cy: ay, r: AR }))),
    svg("circle", { cx: ax, cy: ay, r: AR, class: "ring" }),
    att ? svg("g", { "clip-path": "url(#mb-ball)" },
      svg("g", { transform: `rotate(${-att.roll} ${ax} ${ay}) translate(0 ${Math.max(-AR, Math.min(AR, att.pitch * K))})` },
        svg("rect", { x: ax - 2 * AR, y: ay, width: 4 * AR, height: 2 * AR, class: "sea" }),
        svg("line", { x1: ax - 2 * AR, y1: ay, x2: ax + 2 * AR, y2: ay, class: "horizon" }),
        ...[-20, -10, 10, 20].map((d) => svg("line", { x1: ax - (d % 20 ? 8 : 14), y1: ay - d * K, x2: ax + (d % 20 ? 8 : 14), y2: ay - d * K, class: "rung" }))))
      : T(ax, ay + 4, "NO ATTITUDE", { class: "absent", "text-anchor": "middle" }),
    svg("path", { d: `M${ax - 30} ${ay} h18 l6 6 l6 -6 h18`, class: "wings" }),
    // Either side of the centre: from the ball's edges, "R -1.0" ran into "P +0.6".
    T(ax - 5, 172, att ? `R ${att.roll >= 0 ? "+" : ""}${att.roll.toFixed(1)}\u00b0` : "R --", { class: "unit att", "text-anchor": "end" }),
    T(ax + 5, 172, att ? `P ${att.pitch >= 0 ? "+" : ""}${att.pitch.toFixed(1)}\u00b0` : "P --", { class: "unit att" }),
  ];
  const tx0 = 690, tx1 = 780, ty0 = 200, ty1 = 254;
  const deepest = Math.max(1, ...g.trace.map(([, d]) => d));
  const tp = g.trace.map(([s, d]) => `${(tx1 + (s / DEPTH_TRACE_S) * (tx1 - tx0)).toFixed(1)},${(ty0 + (Math.max(0, d) / deepest) * (ty1 - ty0)).toFixed(1)}`);
  const trace = [
    T(tx0, 192, `DEPTH ${DEPTH_TRACE_S} s`, { class: "lbl" }),
    svg("rect", { x: tx0, y: ty0, width: tx1 - tx0, height: ty1 - ty0, class: "well" }),
    tp.length > 1 ? svg("polyline", { points: tp.join(" "), class: "trace" })
      : T((tx0 + tx1) / 2, (ty0 + ty1) / 2 + 4, "no depth heard", { class: "absent", "text-anchor": "middle" }),
    T(tx1, ty1 + 12, `${deepest.toFixed(1)} m`, { class: "unit", "text-anchor": "end" }),
    T(tx0, 284, !fin(g.rate) ? "-- m/s" : Math.abs(g.rate) < 0.02 ? "holding" :
      `${g.rate > 0 ? "\u2193" : "\u2191"} ${Math.abs(g.rate).toFixed(2)} m/s`, { class: `num rate${fin(g.rate) && Math.abs(g.rate) >= 0.02 ? " moving" : ""}` }),
  ];
  return svg("svg", { viewBox: "0 0 790 322", class: `vitals${g.stale ? " stale" : ""}`, preserveAspectRatio: "xMidYMid meet" },
    svg("line", { x1: 672, y1: 14, x2: 672, y2: 290, class: "divider" }),
    ...depth, ...compass, ...words, ...escs, ...horizon, ...trace, ...health);
}

const INSTRUMENTS = {
  // The strip above the console's tabs: what an operator must never have to
  // look for, on every tab, and a mark on each tab whose subject needs an eye.
  Status: {
    title: "Status",
    kind: "status",
    topics: [...STATUS_TOPICS, ...CAMERAS.map(lockTopic)],
    fold: statusFold,
    draw(s) {
      const { cells, tabs } = statusOf(s, Date.now());
      statusDuties(s, tabs);
      return cells.map(statusCell);
    },
  },

  Watch: {
    title: "Watch",
    kind: "status watch",
    topics: [...STATUS_TOPICS, ...CAMERAS.map(lockTopic)],
    fold: statusFold,
    draw(s) {
      const { cells, tabs } = statusOf(s, Date.now());
      statusDuties(s, tabs);
      return cells.filter(([k]) => WATCH_CELLS.includes(k)).map(statusCell);
    },
  },

  Caption: {
    title: "Caption",
    kind: "caption",
    topics: [...CAMERAS.map((c) => hudTopic(c))],
    fold(s, topic, m) {
      const cam = CAMERAS.find((c) => topic === hudTopic(c));
      if (cam) foldCaption(s, m, cam, heard());
    },
    draw(s) {
      const now = Date.now();
      // The layout names the camera; a caption placed by hand with none
      // says each camera's, one line each.
      const cams = s.cfg && CAMERAS.includes(s.cfg.camera) ? [s.cfg.camera] : CAMERAS;
      return cams.map((c) => captionStrip(captionOf(s, c, now), now));
    },
  },

  Vitals: {
    title: "Vitals",
    kind: "vitals",
    topics: ["/mongla/state", "/mongla/demand", "/mongla/esc_rpm", "/mongla/thruster_pack", "/diagnostics",
      "/mongla/localization/state", "/mongla/imu"],
    fold(s, topic, m) {
      if (topic === "/mongla/state") { s.v = m; s.vAt = heard(); noteDepth(s, m.depth_m, s.vAt); }
      else if (topic === "/mongla/imu") { s.att = attitudeOf(m); s.aAt = heard(); }
      else if (topic === "/mongla/localization/state") { s.loc = m; s.locAt = heard(); }
      else if (topic === "/mongla/demand") { s.demand = m.vector; s.dAt = heard(); }
      else if (topic === "/mongla/esc_rpm") { s.rpm = Array.from(m.data || []); s.rAt = heard(); }
      else if (topic === "/mongla/thruster_pack") { s.pack = m; s.pAt = heard(); }
      else foldDiagnostics(s, m);
    },
    draw(s) { return [vitalsSvg(vitalsOf(s, Date.now()))]; },
  },

  Vehicle: {
    title: "Vehicle",
    topics: ["/mongla/state", "/mongla/batighor/helm_state", "/mongla/flare_order", "/mongla/mission_alive",
      "/diagnostics"],
    fold(s, topic, m) {
      if (topic === "/mongla/state") { s.v = m; s.at = heard(); }
      else if (topic === "/diagnostics") foldDiagnostics(s, m);
      // The runner's 2 Hz beat, carrying its name.
      else if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); }
      else if (topic === "/mongla/flare_order") { try { s.flare = JSON.parse(m.data); } catch { /* keep */ } }
      else { try { s.helm = JSON.parse(m.data); } catch { /* keep */ } }
    },
    draw(s) {
      const v = s.v, h = s.helm || {};
      if (!v) return [el("div", { class: "note" }, "no /mongla/state: is the manager running?")];
      const below = fin(v.depth_m) ? -v.depth_m : NaN;
      return [el("div", { class: "vehicle" }, ...vehicleBody(s, v, h, below))];
    },
  },
  Vision: {
    title: "Vision",
    topics: [...CAMERAS.flatMap((c) => [lockTopic(c), seeingTopic(c), modelTopic(c), classesTopic(c), filterTopic(c)]),
      "/mongla/vision/active_camera", STAGES_TOPIC],
    fold(s, topic, m) {
      s.lock = s.lock || {}; s.lockAt = s.lockAt || {}; s.seeing = s.seeing || {}; s.model = s.model || {};
      s.emits = s.emits || {}; s.filter = s.filter || {};
      if (topic === "/mongla/vision/active_camera") { s.active = m.data; return; }
      if (topic === STAGES_TOPIC) { try { s.stages = JSON.parse(m.data); s.stagesAt = heard(); } catch { /* keep */ } return; }
      for (const c of CAMERAS) {
        if (topic === lockTopic(c)) { s.lock[c] = m; s.lockAt[c] = heard(); }
        if (topic === seeingTopic(c)) s.seeing[c] = m.data;
        // The detector's own word for which model runs (latched), with its epoch:
        // a mission's model switch shows as the epoch moving.
        if (topic === modelTopic(c)) s.model[c] = `${m.database_location || "--"}  epoch ${m.database_version}`;
        if (topic === classesTopic(c)) s.emits[c] = m.data;
        if (topic === filterTopic(c)) s.filter[c] = m.data;
      }
    },
    draw(s) {
      const out = [];
      for (const c of CAMERAS) {
        // One block per camera; the theme sets them side by side when the tile is wide.
        const cam = [];
        const { rung, id, authority } = rungOf((s.lock || {})[c]);
        const at = (s.lockAt || {})[c];
        const stale = ageS(at) > 1.0;
        const live = s.active === c;
        cam.push(el("div", { class: "row" },
          el("b", { class: live ? "live" : "" }, c), live && el("span", { class: "note" }, "live"),
          stale ? el("span", { class: "stale" }, at ? `lock silent ${num(ageS(at), 1)} s` : "no lock node")
            : el("span", {}, `${id || "--"}  authority ${num(authority, 2)}`)));
        cam.push(el("div", { class: "ladder" }, ...RUNGS.map((r) => {
          const on = !stale && r === rung;
          return el("i", on ? { class: "on", style: `background:${RUNG_COLOUR[r]}` } : {}, r);
        })));
        cam.push(el("div", { class: "bar" }, el("b", { style: `width:${stale || !fin(authority) ? 0 : 100 * Math.min(1, authority)}%;background:${RUNG_COLOUR[rung] || "var(--mb-sea)"}` })));
        const notes = [(s.model || {})[c] && `model ${s.model[c]}`, (s.seeing || {})[c] && `seeing ${s.seeing[c]}`].filter(Boolean);
        if (notes.length) cam.push(el("div", { class: "note" }, notes.join("   ")));
        const k = classesOf((s.emits || {})[c], (s.filter || {})[c]);
        if (k) {
          cam.push(el("div", { class: "note" }, `emits ${k.emits.join(", ") || "nothing: no model loaded"}`));
          cam.push(el("div", { class: k.missing.length ? "err" : "note" },
            k.missing.length ? `filter names ${k.missing.join(", ")}: this model never emits it`
              : `filter ${k.filter.join(", ") || "none, every class"}`));
        }
        // What runs on this camera; an off stage carries the switch that turns
        // it on. The overlay node counts publishers: a topic the graph lists
        // is not a stage that runs (a subscriber alone lists it too).
        // overlay.py says which stages run once a second.
        const st = ageS(s.stagesAt) <= HEALTH_STALE_S ? (s.stages || {})[c] : null;
        if (!st && s.stages) cam.push(el("div", { class: "stale" }, `stages: the overlay node is silent ${num(ageS(s.stagesAt), 0)} s`));
        else if (st) {
          cam.push(el("div", { class: "stages" }, ...st.map((x) => x.on
            ? el("i", { class: "on", title: x.topic }, x.stage)
            : el("i", { class: "off", title: `off: ${x.switch}` }, el("s", {}, x.stage), el("small", {}, x.switch)))));
        } else cam.push(el("div", { class: "note" }, "stages: no word from the overlay node (batighor:=true)"));
        out.push(el("div", { class: "cam" }, ...cam));
      }
      return [el("div", { class: "cams" }, ...out)];
    },
  },

  Navigation: {
    title: "Navigation",
    topics: ["/mongla/odom", "/mongla/localization/aiding", "/mongla/localization/motion", "/mongla/localization/fix"],
    fold(s, topic, m) {
      s.track = s.track || []; s.fixes = s.fixes || [];
      if (topic === "/mongla/odom") {
        // The anchor moves odometry into the pool frame: a track across it
        // would join two frames, and a fix is drawn only in the frame it is in.
        const frame = (m.header && m.header.frame_id) || "";
        if (s.odom && frame !== s.frame) { s.track = []; s.fixes = []; }
        s.frame = frame;
        s.odom = m; s.at = heard();
        const p = m.pose && m.pose.pose && m.pose.pose.position;
        if (p) {
          s.track.push({ x: p.x, y: p.y, t: s.at });
          while (s.track.length && s.at - s.track[0].t > TRACK_S * 1000) s.track.shift();
        }
      } else if (topic === "/mongla/localization/aiding") s.aiding = m.data;
      else if (topic === "/mongla/localization/motion") s.motion = m.data;
      else if (m.point) {
        s.nFixes = (s.nFixes || 0) + 1;
        if (((m.header && m.header.frame_id) || "") === (s.frame || "")) { s.fixes.push({ x: m.point.x, y: m.point.y }); if (s.fixes.length > 20) s.fixes.shift(); }
      }
    },
    draw(s) {
      const o = s.odom;
      const p = o && o.pose && o.pose.pose && o.pose.pose.position;
      const tw = o && o.twist && o.twist.twist && o.twist.twist.linear;
      return [
        planView(s.track || [], s.fixes || []),
        kv(["aiding", s.aiding || "--", s.aiding === "aided" ? "live" : "warn"],
          ["motion", s.motion || "--"],
          ["north, east", p ? `${num(p.x, 2)}, ${num(p.y, 2)} m` : "--"],
          ["speed", tw ? `${num(Math.hypot(tw.x, tw.y), 2)} m/s` : "--"],
          ["prop fixes", String(s.nFixes || 0)],
          ["odom age", `${num(ageS(s.at), 1)} s`, ageS(s.at) > 1 ? "warn" : ""]),
      ];
    },
  },

  // The pool, the props and Mongla on them, for planning a run as much as
  // watching one: course.py's course file, the filter (anchored, or drawn by
  // its start), its fixes, and the go_to the manager is flying (/mongla/doing).
  Map: {
    title: "Map",
    topics: MAP_TOPICS,
    fold: foldMap,
    head: startControl,
    draw: (s) => drawMap(s, false),
  },

  // The same pool, read as the filter's: each fix with the correction it made,
  // the filter's and the flow's velocity as arrows, sigma, and the input ages.
  LocalizationMap: {
    title: "Localization map",
    topics: [...MAP_TOPICS, "/mongla/odom", "/mongla/vision/downward/velocity", FLOW_CAL],
    fold: foldMap,
    head: startControl,
    draw: (s) => drawMap(s, true),
  },

  // The decision layer: the mission, the task it is in and its clock, the
  // verb the manager runs now with what was asked of it, and how each recent
  // one ended (refusals too), whoever sent it.
  Mission: {
    title: "Mission",
    topics: ["/mongla/mission_alive", "/mongla/mission/task", "/mongla/doing", "/diagnostics"],
    fold(s, topic, m) {
      if (topic === "/diagnostics") foldDiagnostics(s, m);
      else if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); }
      else if (topic === "/mongla/mission/task") {
        try {
          const t = JSON.parse(m.data); t.at = heard();
          s.tasks = [t, ...(s.tasks || []).filter((x) => !(x.task === t.task && x.state === "running"))].slice(0, 8);
        } catch { /* keep */ }
      } else { try { s.doing = JSON.parse(m.data); s.dAt = heard(); } catch { /* keep */ } }
    },
    draw(s) {
      const live = ageS(s.aliveAt) < MISSION_ALIVE_S && s.missionName;
      const out = [el("div", { class: "row" }, el("span", { class: "note" }, "mission"),
        live ? el("b", { class: "live" }, s.missionName) : el("span", {}, "none running"))];
      // A task's start is latched: it runs only while its mission beats.
      const task = (s.tasks || []).find((t) => t.state === "running");
      if (task && !live) out.push(el("div", { class: "warn" }, `${task.task}: its mission is not running (no beat for ${num(ageS(s.aliveAt), 0)} s); last said running`));
      else if (task) {
        const el_s = task.elapsed_s + ageS(task.at);
        const left = fin(task.deadline_s) ? task.deadline_s - el_s : NaN;
        out.push(kv(["task", task.task, "live"], ["elapsed", clock(el_s)],
          ["deadline", fin(task.deadline_s) ? `${clock(left)} left of ${clock(task.deadline_s)}` : "none", left < 15 ? "warn" : ""],
          ["run budget", fin(task.budget_left_s) ? `${clock(task.budget_left_s - ageS(task.at))} left` : "off"]));
        if (fin(left)) out.push(el("div", { class: "bar" }, el("b", { style: `width:${Math.max(0, Math.min(100, (100 * left) / task.deadline_s))}%;${left < 15 ? "background:var(--mb-amber)" : ""}` })));
      } else out.push(el("div", { class: "note" }, "no task running"));
      const d = s.doing || {};
      const now = d.now;
      out.push(el("h3", {}, el("span", {}, "doing now")));
      const heardMgr = managerHeard(s, Date.now());
      if (s.doing && !heardMgr) out.push(el("div", { class: "warn" }, "the manager is silent: this is its last word, not what runs now"));
      out.push(now ? kv(["verb", now.verb, heardMgr ? "live" : "warn"], ["asked", fieldsText(now.fields) || "defaults"],
        // Both stamps are the manager's clock: the browser's may differ from Kutum's.
        ["for", clock(d.stamp - now.since + ageS(s.dAt))])
        : el("div", { class: "note" }, s.doing ? "nothing: the manager is idle" : "no /mongla/doing: is the manager running?"));
      out.push(el("h3", {}, el("span", {}, "recent verbs, newest first")));
      out.push(kv(...(d.recent || []).map((r) => [`${r.verb}`, `${r.state}${fin(r.ended - r.since) ? ` ${num(r.ended - r.since, 1)} s` : ""}${r.message ? `: ${r.message}` : ""}`, STATE_CLASS[r.state] || ""])));
      const done = (s.tasks || []).filter((t) => t.state !== "running");
      if (done.length) {
        out.push(el("h3", {}, el("span", {}, "tasks ended")));
        out.push(kv(...done.map((t) => [t.task, `${t.state} after ${clock(t.elapsed_s)}`, STATE_CLASS[t.state.split(":")[0]] || "err"])));
      }
      return out;
    },
  },

  // The plan beside the map: the mission's file as the runner published it,
  // and the line of it that sent the verb running now (mission.MissionText).
  Files: {
    title: "Files",
    kind: "files",
    topics: [],
    head: () => filesPanel(),
    fold() {},
    draw: () => [],
  },

  MissionText: {
    title: "Mission text",
    topics: ["/mongla/mission/source", "/mongla/mission/line", "/mongla/mission_alive"],
    fold(s, topic, m) {
      if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); return; }
      try {
        const d = JSON.parse(m.data);
        if (topic === "/mongla/mission/source") { s.src = d; s.lines = String(d.text || "").split("\n"); s.at = null; }
        else { s.at = { ...d, t: heard() }; }
      } catch { /* keep */ }
    },
    draw: (s) => missionText(s),
  },

  Thrust: {
    title: "Thrust",
    topics: ["/mongla/demand", "/mongla/esc_rpm", "/mongla/state", "/mongla/thruster_pack"],
    fold(s, topic, m) {
      if (topic === "/mongla/demand") { s.demand = m.vector; s.dAt = heard(); }
      else if (topic === "/mongla/esc_rpm") {
        s.rpm = Array.from(m.data || []); s.rAt = heard();
        s.peak = Math.max(s.peak || 1000, ...s.rpm.map((r) => Math.abs(r)));
      } else if (topic === "/mongla/thruster_pack") { s.pack = m; s.pAt = heard(); }
      else { s.state = m; s.sAt = heard(); }
    },
    draw(s) {
      // A demand not repeated is not the demand now: none rather than the last.
      const d = ageS(s.dAt) <= FRESH_S ? s.demand || {} : {};
      const rpm = s.rpm || [];
      return [
        el("div", { class: "note" }, "the demand sent to the board, -1 to 1 (NaN: a board primitive owns it)"),
        kv(...[["surge", d.x], ["sway", d.y]].map(([k, x]) =>
          [k, el("span", { class: "demand" }, centreBar(x), el("b", {}, num(x, 2)))])),
        el("div", { class: "note" }, rpm.length
          ? `ESC rpm, as each ESC reports it (scale: ${num(s.peak, 0)} rpm)`
          : "no ESC rpm: the board has not announced its ESCs (none is shown as zero)"),
        // As many bars as ESCs the board reports: the hull's count is the board's to say.
        rpm.length ? el("div", { style: `display:grid;grid-template-columns:repeat(${rpm.length},1fr);gap:4px;align-items:end;height:70px` },
          ...rpm.map((r, i) => el("div", { style: "display:flex;flex-direction:column;align-items:center;gap:2px;height:100%;justify-content:flex-end" },
            el("div", { style: `width:100%;height:${(100 * Math.abs(r)) / s.peak}%;background:${r < 0 ? "var(--mb-amber)" : "var(--mb-sea)"};border-radius:2px 2px 0 0` }),
            el("span", { class: "note" }, String(i + 1))))) : null,
        kv(["demand age", `${num(ageS(s.dAt), 1)} s`, ageS(s.dAt) > FRESH_S ? "warn" : ""],
          ["ESC age", `${num(ageS(s.rAt), 1)} s`, ageS(s.rAt) > FRESH_S ? "warn" : ""],
          ["thruster pack", ageS(s.pAt) <= FRESH_S ? num(s.pack && s.pack.voltage, 2, " V") : "--"],
          ["electronics", ageS(s.sAt) <= FRESH_S ? num(s.state && s.state.battery_voltage, 2, " V") : "--"]),
      ];
    },
  },

  Link: {
    title: "Link",
    topics: ["/mongla/board_link"],
    fold(s, topic, m) { s.l = m; s.at = heard(); },
    draw(s) {
      const l = s.l;
      if (!l) return [el("div", { class: "note" }, "no /mongla/board_link")];
      // The manager sends it once a second; older, it is the line as it was.
      if (ageS(s.at) > HEALTH_STALE_S) return [el("div", { class: "stale" }, `board_link silent ${num(ageS(s.at), 0)} s: the manager is not publishing`)];
      const use = l.capacity_bytes_per_s > 0 ? l.bytes_per_s / l.capacity_bytes_per_s : NaN;
      return [
        el("span", { class: "big" }, num(100 * use, 0), el("span", { class: "unit" }, "% of the line")),
        el("div", { class: "bar" }, el("b", { style: `width:${fin(use) ? Math.min(100, 100 * use) : 0}%;${use > 0.8 ? "background:var(--mb-amber)" : ""}` })),
        kv(["bytes/s", num(l.bytes_per_s, 0)],
          ["board drops", num(l.board_drop_pct, 1, " %"), l.board_drop_pct > 1 ? "warn" : ""],
          ["board errors", num(l.board_errors, 0)],
          ["message types", String((l.rates || []).length)],
          ["named channels", String((l.named || []).length)]),
        el("div", { class: "note" }, "frames per second, by message"),
        kv(...[...(l.rates || [])].sort((a, b) => b.hz - a.hz).map((r) => [r.type, num(r.hz, 1, " Hz")])),
        el("div", { class: "note" }, "the board's named channels (-- once stale)"),
        kv(...[...(l.named || [])].sort((a, b) => a.name.localeCompare(b.name))
          .map((n) => [n.name, num(n.value, 3), n.age_s > 2 ? "stale" : ""])),
      ];
    },
  },

  // Photons to thrust (chain.py): each stage's rate and how old its data is
  // when seen. A bar per stage scales with its age; a stage with no stamp has
  // a rate and "--", and what the board does not send is listed with its ask.
  Chain: {
    title: "Chain",
    // chain.py publishes it twice a second.
    topics: ["/mongla/batighor/chain"],
    fold(s, topic, m) { try { s.c = JSON.parse(m.data); s.at = heard(); } catch { /* keep */ } },
    draw(s) {
      const c = s.c;
      if (!c) return [el("div", { class: "note" }, "no /mongla/batighor/chain: is the chain meter running?")];
      const rows = (c.order || []).map((k) => c.stages[k]);
      const worst = Math.max(1, ...rows.map((r) => (fin(r.age_ms_p95) ? r.age_ms_p95 : 0)));
      const out = [chainSvg(chainBoxes(c)), el("div", { class: "note" }, `age when seen, p50 / p95 over ${num(c.window_s, 0)} s`)];
      for (const r of rows) {
        // n > 0 with no rate is one message in the window: heard, not silent.
        const quiet = !(r.n > 0) || (fin(r.last_s) && r.last_s > 2);
        out.push(kv([r.label,
          quiet ? (r.idle ? `silent ${r.idle}` : "silent")
            : `${fin(r.hz) ? num(r.hz, 1, " Hz") : `${r.n} msg`}   ${num(r.age_ms_p50, 1)} / ${num(r.age_ms_p95, 1)} ms`,
          quiet && !r.idle ? "stale" : ""]));
        if (!quiet && fin(r.age_ms_p50)) {
          out.push(el("div", { class: "bar" }, el("b", { style: `width:${Math.min(100, (100 * r.age_ms_p50) / worst)}%` })));
        }
        if (r.ask) out.push(el("div", { class: "note" }, r.ask));
      }
      out.push(el("div", { class: "note" }, "not sent by the board"));
      out.push(kv(...(c.not_sent || []).map((n) => [n.label, n.ask, "stale"])));
      out.push(kv(["view age", `${num(ageS(s.at), 1)} s`, ageS(s.at) > 2 ? "warn" : ""]));
      return out;
    },
  },

  Health: {
    title: "Health",
    topics: ["/diagnostics"],
    fold: (s, topic, m) => foldDiagnostics(s, m),
    // Kutum's rows have their own instrument beside this one; shown here too
    // they pushed the board's rows off a small panel.
    draw: (s) => [healthRows(healthNow(s, Date.now()), "!kutum.")],
  },

  // Kutum's own rows (clock, disk, Hailo, power, temperature), on their own.
  Kutum: {
    title: "Kutum",
    topics: ["/diagnostics"],
    fold: (s, topic, m) => foldDiagnostics(s, m),
    draw: (s) => [healthRows(healthNow(s, Date.now()), "kutum.")],
  },
};

// The last message of every topic an instrument read, and when it came, for
// this app session. Lichtblick hands a panel only what arrives after it
// mounts: a latched topic (the course, the mission's text, /mongla/doing)
// arrives once, so a panel opened on a second tab, or re-laid-out, waited for
// a change that might never come. A panel that mounts is handed these first,
// at their own arrival times, so an old one reads as old.
const LAST = new Map();

function foldAt(spec, s, topic, m, at, rx) {
  HEARD = at;
  SENT = rx || null;
  try { spec.fold(s, topic, m); } finally { HEARD = 0; SENT = null; }
}

function makePanel(spec) {
  return (context) => {
    if (typeof context.setDefaultPanelTitle === "function") context.setDefaultPanelTitle(spec.title);
    const root = mount(context, spec.kind);
    // The Vitals window is a dash, not a workbench: its first start closes
    // the app's left sidebar, once per profile, so the operator can open it
    // again and it stays open.
    if (spec.kind === "vitals") setTimeout(() => {
      try {
        if (localStorage.getItem("mb.vitals.sidebar")) return;
        const b = document.querySelector('[data-testid="left-sidebar-button"]');
        if (b && /^Hide/.test(b.getAttribute("aria-label") || "")) b.click();
        localStorage.setItem("mb.vitals.sidebar", "closed");
      } catch { /* no storage: leave the sidebar as it is */ }
    }, 1500);
    // The panel's config from the layout (views.instrument(name, **config)).
    const s = { cfg: context.initialState || {} };
    let dirty = true;
    // A control built once (spec.head): a redrawn button can lose its click.
    if (spec.head) root.append(spec.head(s, () => { dirty = true; }));
    const box = el("div", { class: "body" });
    root.append(box);
    for (const topic of spec.topics) {
      const last = LAST.get(topic);
      if (last) foldAt(spec, s, topic, last.m, last.at, last.rx);
    }
    context.subscribe(spec.topics.map((topic) => ({ topic })));
    context.watch("currentFrame");
    context.onRender = (renderState, done) => {
      for (const ev of renderState.currentFrame || []) {
        const at = Date.now();
        LAST.set(ev.topic, { m: ev.message, at, rx: ev.receiveTime });
        foldAt(spec, s, ev.topic, ev.message, at, ev.receiveTime);
        dirty = true;
      }
      done();
    };
    const draw = () => {
      // A drawing that fits its panel needs the panel's size: the map's last
      // box, read before it is replaced (one redraw late on a resize).
      const was = box.querySelector("svg.map");
      if (was) { const r = was.getBoundingClientRect(); s.mapPx = { w: r.width, h: r.height }; }
      box.replaceChildren(...spec.draw(s).filter(Boolean));
    };
    // Ages move with no message, and a silent node is what an operator must see.
    const tick = setInterval(() => { dirty = true; }, 1000);
    let raf = 0;
    const loop = setInterval(() => {
      if (!dirty) return;
      dirty = false;
      cancelAnimationFrame(raf);
      raf = requestAnimationFrame(draw);
    }, REDRAW_MS);
    draw();
    return () => { clearInterval(tick); clearInterval(loop); cancelAnimationFrame(raf); };
  };
}

// -- the desktop app's layouts, kept in step with Kutum's --------------------
// The desktop app imports a layout file once and keeps its own copy in
// IndexedDB (`lichtblick-layouts`, keyed by the file it came from), so a view
// changed on Kutum never reached it. This runs once per app start, in the
// app's own page: each copy that came from one of Kutum's files and that the
// operator has not edited (`working` empty) takes Kutum's version; an edited
// one is left as the operator made it. The browser reads Kutum's copy anyway.

// Pure: which stored records to replace with which served data, and which are
// held back because the operator edited them. `served` is {name: data}.
// Exported for the test.
// Did the operator change anything the layout file says? Every value the
// baseline holds must still be in the working copy. Keys the working copy
// adds are not edits: the app writes its defaults (cameraState, publish,
// showXAxisLabels...) into a panel's config when the panel first mounts,
// choosing a tab writes the Tab panel's activeTabIdx, and a 3D panel writes
// its camera back as it renders (an angle of 200 returns as -160, the
// distance creeps). Counting any of them as an edit kept the console from
// ever taking Kutum's layout again. Where the camera looks is a view, like
// the open tab, so Kutum's camera replaces it.
function said(base, work) {
  if (Array.isArray(base) || base === null || typeof base !== "object") return JSON.stringify(base) === JSON.stringify(work);
  if (work === null || typeof work !== "object" || Array.isArray(work)) return false;
  return Object.keys(base).every((k) => said(base[k], work[k]));
}

function unedited(base, work) {
  const cfg = (d) => (d && d.configById) || {};
  // Global variables are selections made while watching (the track the
  // Detectors panel follows), like the open tab: not the layout's design.
  const { configById: bc, globalVariables: bg, ...bRest } = base || {};
  const { configById: wc, globalVariables: wg, ...wRest } = work || {};
  if (!said(bRest, wRest)) return false;
  return Object.entries(cfg(base)).every(([id, c]) => {
    if (!(id in cfg(work))) return false;
    if (id.startsWith("Tab!") && c) { const { activeTabIdx, ...rest } = c; return said(rest, cfg(work)[id]); }
    if (id.startsWith("3D!") && c) { const { cameraState, ...rest } = c; return said(rest, cfg(work)[id]); }
    return said(c, cfg(work)[id]);
  });
}

function layoutUpdates(records, served, now) {
  const update = [], edited = [];
  for (const r of records) {
    const L = r && r.layout;
    const name = L && typeof L.from === "string" ? L.from.replace(/\.json$/, "") : null;
    if (!name || !(name in served)) continue;
    if (JSON.stringify(L.baseline && L.baseline.data) === JSON.stringify(served[name])) continue;
    let data = served[name];
    if (L.working) {
      const old = L.baseline && L.baseline.data;
      if (!unedited(old, L.working.data)) { edited.push(name); continue; }
      // Kutum's layout, with what the app added kept beneath it: its
      // defaults, any setting made in a key the file never wrote, and the
      // tab that was open.
      const was = (L.working.data && L.working.data.configById) || {};
      const oldCfg = (old && old.configById) || {};
      const cfg = { ...(data.configById || {}) };
      for (const id of Object.keys(cfg)) {
        const w = was[id];
        if (!w || typeof w !== "object") continue;
        const added = Object.fromEntries(Object.entries(w).filter(([k]) => !(k in (oldCfg[id] || {}))));
        cfg[id] = { ...added, ...cfg[id] };
        if (id.startsWith("Tab!") && Number.isInteger(w.activeTabIdx) && w.activeTabIdx < (cfg[id].tabs || []).length) cfg[id].activeTabIdx = w.activeTabIdx;
      }
      const vars = L.working.data && L.working.data.globalVariables;
      data = { ...data, configById: cfg, ...(vars && data.globalVariables ? { globalVariables: { ...data.globalVariables, ...vars } } : {}) };
    }
    update.push({ ...r, layout: { ...L, baseline: { data, savedAt: now }, working: undefined } });
  }
  return { update, edited };
}

const idb = (req) => new Promise((ok, no) => { req.onsuccess = () => ok(req.result); req.onerror = () => no(req.error); });

// Batighor's own address: the page's origin in a browser (Kutum serves it),
// and in the desktop app whoever it last connected to, on 8000 there.
async function kutumBase() {
  if (typeof location !== "undefined" && /^https?:$/.test(location.protocol)) return location.origin;
  if (typeof indexedDB === "undefined") return null;
  const recents = await idb(indexedDB.open("lichtblick-recents"));
  const list = (await idb(recents.transaction("recents").objectStore("recents").getAll())).flat();
  recents.close();
  const url = list.map((x) => x && x.extra && x.extra.url).find((u) => typeof u === "string" && u.startsWith("ws"));
  return url ? `http://${new URL(url).hostname}:8000` : null;
}

// The Files panel: the missions and the deck on Kutum, read only
// (shelves.py). Built once in `head`, so a click is never lost to a redraw.
function filesPanel() {
  const list = el("div", { class: "files-list" });
  const view = el("pre", { class: "files-view" }, "pick a file");
  const title = el("div", { class: "files-title note" }, "");
  let base = null, picked = null;
  const show = async (shelf, path) => {
    picked = `${shelf}/${path}`;
    title.textContent = `${shelf}/${path}  (read only)`;
    for (const b of list.querySelectorAll("button")) b.classList.toggle("on", b.dataset.key === picked);
    try {
      const r = await fetch(`${base}/files/${encodeURIComponent(shelf)}/${path.split("/").map(encodeURIComponent).join("/")}`, { cache: "no-store" });
      view.textContent = r.ok ? await r.text() : `not shown: ${(await r.text()).trim()}`;
    } catch (e) { view.textContent = `Kutum did not answer: ${e}`; }
  };
  const load = async () => {
    base = base || await kutumBase().catch(() => null);
    if (!base) { list.replaceChildren(el("div", { class: "note" }, "no Kutum to ask: connect first")); return; }
    try {
      const shelves = await (await fetch(`${base}/files/index.json`, { cache: "no-store" })).json();
      list.replaceChildren(...Object.entries(shelves).flatMap(([shelf, files]) => [
        el("div", { class: "lbl" }, `${shelf} (${files.length})`),
        ...(files.length ? files.map((f) => el("button", { class: `file${`${shelf}/${f.path}` === picked ? " on" : ""}`,
          "data-key": `${shelf}/${f.path}`, "aria-label": f.path,
          title: `${f.path}: ${num(f.size / 1024, 1)} KB, ${new Date(f.mtime * 1000).toLocaleString()}`,
          onclick: () => show(shelf, f.path) }, f.path))
          : [el("div", { class: "note" }, "empty or not on this machine")]),
      ]));
    } catch (e) { list.replaceChildren(el("div", { class: "note" }, `Kutum did not answer: ${e}`)); }
  };
  load();
  return el("div", { class: "files" },
    el("div", { class: "files-side" }, el("button", { class: "file refresh", onclick: () => { load(); if (picked) show(...picked.split(/\/(.*)/s, 2)); } }, "refresh"), list),
    el("div", { class: "files-main" }, title, view));
}

async function syncLayouts() {
  if (typeof location === "undefined" || location.protocol !== "file:" || typeof indexedDB === "undefined") return;
  const KEY = "mongla.batighor.layoutSync";
  const kutum = await kutumBase();
  if (!kutum) return;
  const base = `${kutum}/layouts/`;
  const names = await (await fetch(base + "index.json", { cache: "no-store" })).json();
  const served = {};
  for (const n of names) served[n] = await (await fetch(`${base}${encodeURIComponent(n)}.json`, { cache: "no-store" })).json();
  const db = await idb(indexedDB.open("lichtblick-layouts"));
  const store = () => db.transaction("layouts", "readwrite").objectStore("layouts");
  const { update, edited } = layoutUpdates(await idb(store().getAll()), served, new Date().toISOString());
  for (const r of update) await idb(store().put(r));
  db.close();
  if (edited.length) console.warn(`[batighor] kept your edits to ${edited.join(", ")}; Revert in the layout's menu takes Kutum's`);
  if (!update.length) { try { sessionStorage.removeItem(KEY); } catch { /* none */ } return; }
  console.info(`[batighor] ${update.length} layout(s) taken from Kutum: ${update.map((r) => r.layout.name).join(", ")}`);
  // The app read the old copies before this ran: one reload shows the new ones.
  // Guarded, so a copy the app rewrites cannot reload it forever.
  const mark = update.map((r) => r.layout.name).join(",");
  try {
    if (sessionStorage.getItem(KEY) === mark) return;
    sessionStorage.setItem(KEY, mark);
  } catch { return; }
  location.reload();
}

module.exports = {
  healthNow, managerHeard, vitalsOf, signalBars, WATCH_CELLS, captionOf, foldCaption, fitText, NAV_WHY_CHARS,
  attitudeOf, depthRate, vitalsSvg, chainBoxes, chainSvg, classesOf,
  activate(extensionContext) {
    syncLayouts().catch((e) => console.warn(`[batighor] layout sync: ${e}`));
    for (const [name, spec] of Object.entries(INSTRUMENTS)) {
      extensionContext.registerPanel({ name, initPanel: makePanel(spec) });
    }
  },
  rungOf,
  poolMap,
  drawMap,
  statusOf,
  noteLag,
  shouldReport,
  videoWords,
  layoutUpdates,
  placeByStart,
  makePanel,
  LAST,
  INSTRUMENTS,
};
