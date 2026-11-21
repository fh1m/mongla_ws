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
const REDRAW_MS = 100;
const MISSION_ALIVE_S = 1.5;  // the runner beats at 2 Hz; silent this long is no mission (helm.MISSION_ALIVE_S)
const TRACK_S = 120;          // the navigation plan view keeps two minutes of track
const SVG = "http://www.w3.org/2000/svg";

const lockTopic = (c) => `/mongla/vision/${c}/lock`;
const seeingTopic = (c) => `/mongla/vision/${c}/seeing`;
const modelTopic = (c) => `/mongla/vision/${c}/vision_info`;
const STAGES_TOPIC = "/mongla/batighor/stages";     // overlay.py, once a second
const ageS = (t) => (t ? (Date.now() - t) / 1000 : Infinity);
// When the message being folded arrived. A fold stamps what it keeps with
// this, never with the clock, so a message replayed to a panel that mounted
// late keeps its own age (see LAST in makePanel).
let HEARD = 0;
const heard = () => HEARD || Date.now();

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
  return el("div", { class: "bar", style: "position:relative" },
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
  const here = pose && fin(pose.x) && fin(pose.y) ? pose : null;
  const pts = [...(P.outline || []).map(([x, y]) => ({ x, y })), ...(P.floor || []).map(([x, y]) => ({ x, y })),
    ...props.map((p) => ({ x: p.x_m, y: p.y_m })), ...(here ? track : []), ...(here ? fixes : []),
    ...(here ? [here] : []), ...(target ? [target] : [])].filter((p) => fin(p.x) && fin(p.y));
  if (!pts.length) return null;
  let n0 = Infinity, n1 = -Infinity, e0 = Infinity, e1 = -Infinity;
  for (const p of pts) { n0 = Math.min(n0, p.x); n1 = Math.max(n1, p.x); e0 = Math.min(e0, p.y); e1 = Math.max(e1, p.y); }
  const span = Math.max(n1 - n0, e1 - e0, 4);
  const pad = span * 0.08;
  const cn = (n0 + n1) / 2, ce = (e0 + e1) / 2;
  n0 = cn - span / 2 - pad; n1 = cn + span / 2 + pad; e0 = ce - span / 2 - pad; e1 = ce + span / 2 + pad;
  const W = e1 - e0;
  // SVG in metres: east right, north up (the y axis flipped by drawing at -x).
  const X = (p) => p.y, Y = (p) => -p.x;
  const u = W / 300;                      // one "pixel" of a 300 px wide map, in metres
  const step = W > 40 ? 5 : W > 12 ? 2 : W > 5 ? 1 : 0.5;
  const kids = [];
  for (let g = Math.ceil(e0 / step) * step; g <= e1; g += step) kids.push(svg("line", { x1: g, y1: -n1, x2: g, y2: -n0, stroke: "var(--mb-rule)", "stroke-width": u }));
  for (let g = Math.ceil(n0 / step) * step; g <= n1; g += step) kids.push(svg("line", { x1: e0, y1: -g, x2: e1, y2: -g, stroke: "var(--mb-rule)", "stroke-width": u }));
  if ((P.outline || []).length >= 3) {
    kids.push(svg("polygon", { points: P.outline.map(([x, y]) => `${y},${-x}`).join(" "), fill: "rgba(90,141,255,0.06)", stroke: "var(--mb-sea)", "stroke-width": 1.5 * u }));
  }
  for (const [x, y, d] of P.floor || []) {
    kids.push(svg("circle", { cx: y, cy: -x, r: 2 * u, fill: "var(--mb-teal)" }));
    // Left of the dot: prop names are written to the right of theirs, and a
    // sounding beside a prop read as one label (bench course, 2026-11-21).
    kids.push(svg("text", { x: y - 4 * u, y: -x + 3 * u, "font-size": 8 * u, fill: "var(--mb-teal)", "text-anchor": "end", ...HALO(u) }, `${num(d, 2)} m`));
  }
  const text = (x, y, s, fill, anchor) => svg("text", { x, y, "font-size": 9 * u, fill, "text-anchor": anchor || "start", ...HALO(u) }, s);
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

function foldMap(s, topic, m) {
  s.track = s.track || []; s.fixes = s.fixes || [];
  if (topic === "/mongla/batighor/course") { try { s.course = JSON.parse(m.data); } catch { /* keep */ } }
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
  } else if (topic === "/mongla/odom") s.odom = m;
  else if (topic === "/mongla/vision/downward/velocity") s.flow = m;
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
  const anchored = pose && !pose.placed;
  const out = [el("div", { class: "row" },
    el("b", {}, c.course || "no course"), el("span", { class: "note" }, c.from === "mission" ? "from the mission" : "from the deck"),
    !loc ? el("span", { class: "stale" }, "no localization")
      : anchored ? el("span", { class: "live" }, "anchored in the pool")
        : el("span", { class: "warn" }, `not anchored: drawn by its start (${s.start ? s.start.by : "--"})`))];
  if (c.error) { out.push(el("div", { class: "err" }, c.error)); return out; }
  const vel = [];
  if (forLocalization && pose && fin(pose.yaw)) {
    // Body-frame velocities (FRD), turned into the pool by the drawn heading.
    const turn = (vx, vy) => ({ vx: Math.cos(pose.yaw) * vx - Math.sin(pose.yaw) * vy, vy: Math.sin(pose.yaw) * vx + Math.cos(pose.yaw) * vy });
    const ot = s.odom && s.odom.twist && s.odom.twist.twist && s.odom.twist.twist.linear;
    if (ot) vel.push({ ...turn(ot.x, ot.y), ink: "var(--mb-sea)" });
    const ft = s.flow && s.flow.twist && s.flow.twist.twist && s.flow.twist.twist.linear;
    if (ft) vel.push({ ...turn(ft.x, ft.y), ink: "var(--mb-teal)" });
  }
  const map = poolMap(c, pose, s.track || [], s.fixes || [], target, { rays: forLocalization, vel });
  out.push(map || el("div", { class: "note" }, "nothing placed yet: survey the pool (course_survey --corner, --prop, --floor)"));
  const un = c.unplaced || [];
  const where = pose ? `${num(pose.x, 2)}, ${num(pose.y, 2)} m  heading ${num(fin(pose.yaw) ? (pose.yaw * 180) / Math.PI : NaN, 0)} deg` : "--";
  const lastFix = (s.fixes || []).at(-1);
  const rows = [
    ["position", anchored ? where : pose ? `${where} (by its start)` : "--", anchored ? "live" : "warn"],
    ["filter frame", loc ? `${loc.frame || "--"}  ${num(loc.x_m, 2)}, ${num(loc.y_m, 2)} m` : "--"],
    ["sigma", loc ? `${num(loc.sigma_xy_m, 2)} m  yaw ${num(loc.sigma_yaw_deg, 1)} deg` : "--"],
    ["usable", loc ? (loc.usable ? "yes" : `no: ${loc.reason || "--"}`) : "--", loc && loc.usable ? "live" : "warn"],
    ["aiding", loc ? loc.aiding || "--" : "--"],
    ["going to", target ? `${num(target.x, 2)}, ${num(target.y, 2)} m within ${num(target.tol, 2, " m")}` : "--", target ? "live" : ""],
  ];
  if (forLocalization) {
    rows.push(["arrows", "2 s ahead: filter (sea), flow (teal)"],
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
    ? el("div", { class: "health" }, ...shown.flatMap((r) => [
      el("span", { class: LEVEL[r.level] || "stale" }, r.name.replace(/^mongla: /, "")),
      el("span", { class: r.level ? LEVEL[r.level] : "", title: r.message }, r.message)]))
    : el("div", { class: "note" }, "no /diagnostics rows yet: the manager grades health once it runs");
}

function foldDiagnostics(s, m) {
  s.health = s.health || [];
  for (const st of m.status || []) {
    const i = s.health.findIndex((h) => h.name === st.name);
    const row = { name: st.name, level: typeof st.level === "number" ? st.level : Number(st.level), message: st.message };
    if (i < 0) s.health.push(row); else s.health[i] = row;
  }
}

// -- the table -----------------------------------------------------------

// The status strip's cells and the console's tab marks, from one fold. Pure:
// exported for the test. A level is "" (nothing to say), "ok", "live" (it is
// happening: armed, a mission running, a lock held), "warn" or "err".
function statusOf(s, now) {
  const age = (t) => (t ? (now - t) / 1000 : Infinity);
  const v = s.v, cells = [], tabs = {};
  const fresh = v && age(s.vAt) < 2;
  cells.push(["vehicle", !v ? "--" : v.armed ? "ARMED" : "disarmed", !fresh ? "stale" : v.armed ? "armed" : "ok"]);
  cells.push(["mode", v ? v.mode || "--" : "--", ""]);
  cells.push(["depth", v && fin(v.depth_m) ? `${num(-v.depth_m, 2)} m` : "--", ""]);
  cells.push(["heading", v ? `${num(v.yaw_deg, 0)} deg` : "--", ""]);
  cells.push(["battery", v ? `${num(v.battery_voltage, 1)} V` : "--", ""]);
  const running = age(s.aliveAt) < MISSION_ALIVE_S;
  const task = s.task && running ? ` / ${s.task.task || s.task.name || ""}` : "";
  cells.push(["mission", running ? `${s.missionName || "running"}${task}` : "none", running ? "live" : ""]);
  const doing = s.doing && s.doing.now;
  cells.push(["verb", doing ? doing.verb : "idle", doing ? "live" : ""]);
  const rungs = CAMERAS.map((c) => [c, s.locks && age(s.locks[c] && s.locks[c].at) < 2 ? rungOf(s.locks[c].m).rung : null]);
  const held = rungs.filter(([, r]) => r && r !== "lost");
  cells.push(["lock", held.length ? held.map(([c, r]) => `${c} ${r}`).join("  ") : rungs.every(([, r]) => r) ? "lost" : "--", held.length ? "live" : ""]);
  const loc = s.loc && age(s.locAt) < 2 ? s.loc : null;
  const anchored = loc && loc.anchored && loc.frame === "pool";
  cells.push(["navigation", !loc ? "--" : anchored ? `anchored  ${num(loc.sigma_xy_m, 2)} m` : loc.usable ? `${loc.aiding || "aided"}` : `unusable`, !loc ? "stale" : anchored ? "ok" : loc.usable ? "" : "warn"]);
  // ROS levels: 0 ok, 1 warn, 2 error, 3 stale. A row with no data is
  // unknown, never a fault: the bench has no barometer, not a broken one.
  const rows = s.health || [];
  const count = (lvl) => rows.filter((r) => r.level === lvl).length;
  const [errs, warns, unknown] = [count(2), count(1), count(3)];
  const said = [errs && `${errs} error${errs > 1 ? "s" : ""}`, warns && `${warns} warning${warns > 1 ? "s" : ""}`,
    unknown && `${unknown} unknown`].filter(Boolean).join("  ");
  const worst = !rows.length ? -1 : errs ? 2 : warns ? 1 : unknown ? 3 : 0;
  cells.push(["health", worst < 0 ? "--" : said || "all ok", worst < 0 ? "stale" : LEVEL[worst]]);
  const l = s.link && age(s.linkAt) < 3 ? s.link : null;
  cells.push(["link", l ? `${num(100 * l.bytes_per_s / (l.capacity_bytes_per_s || NaN), 0)} %  drops ${num(l.board_drop_pct, 1)} %` : "--", !l ? "stale" : l.board_drop_pct > 1 ? "warn" : "ok"]);
  tabs.Cockpit = v && v.armed ? "live" : "";
  tabs.Thrust = v && v.armed ? "live" : "";
  tabs.Mission = running ? "live" : "";
  tabs.Vision = held.length ? "live" : "";
  tabs.Map = tabs.Localization = !loc ? "" : anchored ? "ok" : loc.usable ? "" : "warn";
  tabs.Health = worst < 0 ? "" : LEVEL[Math.min(worst, 3)] === "ok" ? "ok" : LEVEL[Math.min(worst, 3)];
  tabs.Link = !l ? "" : l.board_drop_pct > 1 ? "warn" : "";
  return { cells, tabs };
}

const INSTRUMENTS = {
  // The strip above the console's tabs: what an operator must never have to
  // look for, on every tab, and a mark on each tab whose subject needs an eye.
  Status: {
    title: "Status",
    kind: "status",
    topics: ["/mongla/state", "/mongla/mission_alive", "/mongla/mission/task", "/mongla/doing", "/diagnostics",
      "/mongla/localization/state", "/mongla/board_link", ...CAMERAS.map(lockTopic)],
    fold(s, topic, m) {
      if (topic === "/mongla/state") { s.v = m; s.vAt = heard(); }
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
    },
    draw(s) {
      const { cells, tabs } = statusOf(s, Date.now());
      if (typeof document !== "undefined" && document.querySelectorAll) {
        for (const tab of document.querySelectorAll('[data-testid="toolbar-tab"]')) {
          const lvl = tabs[tab.getAttribute("title")] || "";
          if (tab.dataset.mb !== lvl) tab.dataset.mb = lvl;
        }
      }
      return cells.map(([k, val, lvl]) => el("div", { class: `cell ${lvl}` }, el("small", {}, k), el("b", {}, val)));
    },
  },

  Vehicle: {
    title: "Vehicle",
    topics: ["/mongla/state", "/mongla/batighor/helm_state", "/mongla/flare_order", "/mongla/mission_alive"],
    fold(s, topic, m) {
      if (topic === "/mongla/state") { s.v = m; s.at = heard(); }
      // The runner's 2 Hz beat, carrying its name.
      else if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); }
      else if (topic === "/mongla/flare_order") { try { s.flare = JSON.parse(m.data); } catch { /* keep */ } }
      else { try { s.helm = JSON.parse(m.data); } catch { /* keep */ } }
    },
    draw(s) {
      const v = s.v, h = s.helm || {};
      if (!v) return [el("div", { class: "note" }, "no /mongla/state: is the manager running?")];
      const below = fin(v.depth_m) ? -v.depth_m : NaN;
      return [
        el("div", { class: "row" },
          el("span", { class: v.armed ? "armed" : "ok" }, v.armed ? "ARMED" : "disarmed"),
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
      ];
    },
  },

  Vision: {
    title: "Vision",
    topics: [...CAMERAS.flatMap((c) => [lockTopic(c), seeingTopic(c), modelTopic(c)]), "/mongla/vision/active_camera", STAGES_TOPIC],
    fold(s, topic, m) {
      s.lock = s.lock || {}; s.lockAt = s.lockAt || {}; s.seeing = s.seeing || {}; s.model = s.model || {};
      if (topic === "/mongla/vision/active_camera") { s.active = m.data; return; }
      if (topic === STAGES_TOPIC) { try { s.stages = JSON.parse(m.data); } catch { /* keep */ } return; }
      for (const c of CAMERAS) {
        if (topic === lockTopic(c)) { s.lock[c] = m; s.lockAt[c] = heard(); }
        if (topic === seeingTopic(c)) s.seeing[c] = m.data;
        // The detector's own word for which model runs (latched), with its epoch:
        // a mission's model switch shows as the epoch moving.
        if (topic === modelTopic(c)) s.model[c] = `${m.database_location || "--"}  epoch ${m.database_version}`;
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
        // What runs on this camera; an off stage carries the switch that turns
        // it on. The overlay node counts publishers: a topic the graph lists
        // is not a stage that runs (a subscriber alone lists it too).
        const st = (s.stages || {})[c];
        if (st) {
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
        s.odom = m; s.at = heard();
        const p = m.pose && m.pose.pose && m.pose.pose.position;
        if (p) {
          s.track.push({ x: p.x, y: p.y, t: s.at });
          while (s.track.length && s.at - s.track[0].t > TRACK_S * 1000) s.track.shift();
        }
      } else if (topic === "/mongla/localization/aiding") s.aiding = m.data;
      else if (topic === "/mongla/localization/motion") s.motion = m.data;
      else if (m.point) { s.fixes.push({ x: m.point.x, y: m.point.y }); if (s.fixes.length > 20) s.fixes.shift(); }
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
          ["prop fixes", String((s.fixes || []).length)],
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
    topics: [...MAP_TOPICS, "/mongla/odom", "/mongla/vision/downward/velocity"],
    fold: foldMap,
    head: startControl,
    draw: (s) => drawMap(s, true),
  },

  // The decision layer: the mission, the task it is in and its clock, the
  // verb the manager runs now with what was asked of it, and how each recent
  // one ended (refusals too), whoever sent it.
  Mission: {
    title: "Mission",
    topics: ["/mongla/mission_alive", "/mongla/mission/task", "/mongla/doing"],
    fold(s, topic, m) {
      if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = heard(); }
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
      const task = (s.tasks || []).find((t) => t.state === "running");
      if (task) {
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
      out.push(now ? kv(["verb", now.verb, "live"], ["asked", fieldsText(now.fields) || "defaults"],
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
      } else if (topic === "/mongla/thruster_pack") s.pack = m;
      else s.state = m;
    },
    draw(s) {
      const d = s.demand || {};
      const rpm = s.rpm || [];
      return [
        el("div", { class: "note" }, "the demand sent to the board, -1 to 1 (NaN: a board primitive owns it)"),
        kv(["surge", el("span", {}, centreBar(d.x), num(d.x, 2))], ["sway", el("span", {}, centreBar(d.y), num(d.y, 2))]),
        el("div", { class: "note" }, rpm.length
          ? `ESC rpm, as each ESC reports it (scale: ${num(s.peak, 0)} rpm)`
          : "no ESC rpm: the board has not announced its ESCs (none is shown as zero)"),
        // As many bars as ESCs the board reports: the hull's count is the board's to say.
        rpm.length ? el("div", { style: `display:grid;grid-template-columns:repeat(${rpm.length},1fr);gap:4px;align-items:end;height:70px` },
          ...rpm.map((r, i) => el("div", { style: "display:flex;flex-direction:column;align-items:center;gap:2px;height:100%;justify-content:flex-end" },
            el("div", { style: `width:100%;height:${(100 * Math.abs(r)) / s.peak}%;background:${r < 0 ? "var(--mb-amber)" : "var(--mb-sea)"};border-radius:2px 2px 0 0` }),
            el("span", { class: "note" }, String(i + 1))))) : null,
        kv(["ESC age", `${num(ageS(s.rAt), 1)} s`], ["thruster pack", num(s.pack && s.pack.voltage, 2, " V")], ["electronics", num(s.state && s.state.battery_voltage, 2, " V")]),
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
      const out = [el("div", { class: "note" }, `age when seen, p50 / p95 over ${num(c.window_s, 0)} s`)];
      for (const r of rows) {
        const quiet = !(r.hz > 0) || (fin(r.last_s) && r.last_s > 2);
        out.push(kv([r.label,
          quiet ? "silent" : `${num(r.hz, 1, " Hz")}   ${num(r.age_ms_p50, 1)} / ${num(r.age_ms_p95, 1)} ms`,
          quiet ? "stale" : ""]));
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
    draw: (s) => [healthRows(s.health || [], "!kutum.")],
  },

  // Kutum's own rows (clock, disk, Hailo, power, temperature), on their own.
  Kutum: {
    title: "Kutum",
    topics: ["/diagnostics"],
    fold: (s, topic, m) => foldDiagnostics(s, m),
    draw: (s) => [healthRows(s.health || [], "kutum.")],
  },
};

// The last message of every topic an instrument read, and when it came, for
// this app session. Lichtblick hands a panel only what arrives after it
// mounts: a latched topic (the course, the mission's text, /mongla/doing)
// arrives once, so a panel opened on a second tab, or re-laid-out, waited for
// a change that might never come. A panel that mounts is handed these first,
// at their own arrival times, so an old one reads as old.
const LAST = new Map();

function foldAt(spec, s, topic, m, at) {
  HEARD = at;
  try { spec.fold(s, topic, m); } finally { HEARD = 0; }
}

function makePanel(spec) {
  return (context) => {
    if (typeof context.setDefaultPanelTitle === "function") context.setDefaultPanelTitle(spec.title);
    const root = mount(context, spec.kind);
    const s = {};
    let dirty = true;
    // A control built once (spec.head): a redrawn button can lose its click.
    if (spec.head) root.append(spec.head(s, () => { dirty = true; }));
    const box = el("div", { class: "body" });
    root.append(box);
    for (const topic of spec.topics) {
      const last = LAST.get(topic);
      if (last) foldAt(spec, s, topic, last.m, last.at);
    }
    context.subscribe(spec.topics.map((topic) => ({ topic })));
    context.watch("currentFrame");
    context.onRender = (renderState, done) => {
      for (const ev of renderState.currentFrame || []) {
        const at = Date.now();
        LAST.set(ev.topic, { m: ev.message, at });
        foldAt(spec, s, ev.topic, ev.message, at);
        dirty = true;
      }
      done();
    };
    const draw = () => box.replaceChildren(...spec.draw(s).filter(Boolean));
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
  const { configById: bc, ...bRest } = base || {}, { configById: wc, ...wRest } = work || {};
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
      data = { ...data, configById: cfg };
    }
    update.push({ ...r, layout: { ...L, baseline: { data, savedAt: now }, working: undefined } });
  }
  return { update, edited };
}

const idb = (req) => new Promise((ok, no) => { req.onsuccess = () => ok(req.result); req.onerror = () => no(req.error); });

async function syncLayouts() {
  if (typeof location === "undefined" || location.protocol !== "file:" || typeof indexedDB === "undefined") return;
  const KEY = "mongla.batighor.layoutSync";
  // Kutum is whoever the app last connected to; Batighor serves on 8000 there.
  const recents = await idb(indexedDB.open("lichtblick-recents"));
  const list = (await idb(recents.transaction("recents").objectStore("recents").getAll())).flat();
  recents.close();
  const url = list.map((x) => x && x.extra && x.extra.url).find((u) => typeof u === "string" && u.startsWith("ws"));
  if (!url) return;
  const base = `http://${new URL(url).hostname}:8000/layouts/`;
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
  layoutUpdates,
  placeByStart,
  makePanel,
  LAST,
  INSTRUMENTS,
};
