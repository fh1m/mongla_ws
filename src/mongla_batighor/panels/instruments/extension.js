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
const TRACK_S = 120;          // the navigation plan view keeps two minutes of track
const SVG = "http://www.w3.org/2000/svg";

const lockTopic = (c) => `/mongla/vision/${c}/lock`;
const seeingTopic = (c) => `/mongla/vision/${c}/seeing`;
const modelTopic = (c) => `/mongla/vision/${c}/vision_info`;
const STAGES_TOPIC = "/mongla/batighor/stages";     // overlay.py, once a second
const ageS = (t) => (t ? (Date.now() - t) / 1000 : Infinity);

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

const INSTRUMENTS = {
  Vehicle: {
    title: "Vehicle",
    topics: ["/mongla/state", "/mongla/batighor/helm_state", "/mongla/flare_order", "/mongla/mission_alive"],
    fold(s, topic, m) {
      if (topic === "/mongla/state") { s.v = m; s.at = Date.now(); }
      // The runner's 2 Hz beat, carrying its name; silent 1.5 s is no mission (helm.MISSION_ALIVE_S).
      else if (topic === "/mongla/mission_alive") { s.missionName = m.data; s.aliveAt = Date.now(); }
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
        kv(ageS(s.aliveAt) < 1.5 ? ["mission", `running ${s.missionName || ""}`, "live"] : ["mission", "none"],
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
        if (topic === lockTopic(c)) { s.lock[c] = m; s.lockAt[c] = Date.now(); }
        if (topic === seeingTopic(c)) s.seeing[c] = m.data;
        // The detector's own word for which model runs (latched), with its epoch:
        // a mission's model switch shows as the epoch moving.
        if (topic === modelTopic(c)) s.model[c] = `${m.database_location || "--"}  epoch ${m.database_version}`;
      }
    },
    draw(s) {
      const out = [];
      for (const c of CAMERAS) {
        const { rung, id, authority } = rungOf((s.lock || {})[c]);
        const at = (s.lockAt || {})[c];
        const stale = ageS(at) > 1.0;
        const live = s.active === c;
        out.push(el("div", { class: "row" },
          el("b", { class: live ? "live" : "" }, c), live && el("span", { class: "note" }, "live"),
          stale ? el("span", { class: "stale" }, at ? `lock silent ${num(ageS(at), 1)} s` : "no lock node")
            : el("span", {}, `${id || "--"}  authority ${num(authority, 2)}`)));
        out.push(el("div", { class: "ladder" }, ...RUNGS.map((r) => {
          const on = !stale && r === rung;
          return el("i", on ? { class: "on", style: `background:${RUNG_COLOUR[r]}` } : {}, r);
        })));
        out.push(el("div", { class: "bar" }, el("b", { style: `width:${stale || !fin(authority) ? 0 : 100 * Math.min(1, authority)}%;background:${RUNG_COLOUR[rung] || "var(--mb-sea)"}` })));
        const notes = [(s.model || {})[c] && `model ${s.model[c]}`, (s.seeing || {})[c] && `seeing ${s.seeing[c]}`].filter(Boolean);
        if (notes.length) out.push(el("div", { class: "note" }, notes.join("   ")));
        // What runs on this camera, and what is off with the switch that turns it on.
        // The overlay node counts publishers: a topic the graph lists is not
        // a stage that runs (a subscriber alone lists it too).
        const st = (s.stages || {})[c];
        if (st) {
          out.push(el("div", { class: "stages" }, ...st.map((x) =>
            el("i", { class: x.on ? "on" : "off", title: x.on ? x.topic : `off: ${x.switch}` }, x.stage))));
          const off = st.filter((x) => !x.on);
          if (off.length) out.push(el("div", { class: "note" }, "off: " + off.map((x) => `${x.stage} (${x.switch})`).join("  ")));
        } else out.push(el("div", { class: "note" }, "stages: no word from the overlay node (batighor:=true)"));
      }
      return out;
    },
  },

  Navigation: {
    title: "Navigation",
    topics: ["/mongla/odom", "/mongla/localization/aiding", "/mongla/localization/motion", "/mongla/localization/fix"],
    fold(s, topic, m) {
      s.track = s.track || []; s.fixes = s.fixes || [];
      if (topic === "/mongla/odom") {
        s.odom = m; s.at = Date.now();
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
        kv(["aided by", s.aiding || "--", s.aiding === "aided" ? "live" : "warn"],
          ["motion", s.motion || "--"],
          ["north, east", p ? `${num(p.x, 2)}, ${num(p.y, 2)} m` : "--"],
          ["speed", tw ? `${num(Math.hypot(tw.x, tw.y), 2)} m/s` : "--"],
          ["prop fixes", String((s.fixes || []).length)],
          ["odom age", `${num(ageS(s.at), 1)} s`, ageS(s.at) > 1 ? "warn" : ""]),
      ];
    },
  },

  Thrust: {
    title: "Thrust",
    topics: ["/mongla/demand", "/mongla/esc_rpm", "/mongla/state"],
    fold(s, topic, m) {
      if (topic === "/mongla/demand") { s.demand = m.vector; s.dAt = Date.now(); }
      else if (topic === "/mongla/esc_rpm") {
        s.rpm = Array.from(m.data || []); s.rAt = Date.now();
        s.peak = Math.max(s.peak || 1000, ...s.rpm.map((r) => Math.abs(r)));
      } else s.state = m;
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
        kv(["ESC age", `${num(ageS(s.rAt), 1)} s`], ["battery", num(s.state && s.state.battery_voltage, 2, " V")]),
      ];
    },
  },

  Link: {
    title: "Link",
    topics: ["/mongla/board_link"],
    fold(s, topic, m) { s.l = m; s.at = Date.now(); },
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

function makePanel(spec) {
  return (context) => {
    if (typeof context.setDefaultPanelTitle === "function") context.setDefaultPanelTitle(spec.title);
    const box = mount(context);
    const s = {};
    let dirty = true;
    context.subscribe(spec.topics.map((topic) => ({ topic })));
    context.watch("currentFrame");
    context.onRender = (renderState, done) => {
      for (const ev of renderState.currentFrame || []) {
        spec.fold(s, ev.topic, ev.message);
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

module.exports = {
  activate(extensionContext) {
    for (const [name, spec] of Object.entries(INSTRUMENTS)) {
      extensionContext.registerPanel({ name, initPanel: makePanel(spec) });
    }
  },
  rungOf,
  INSTRUMENTS,
};
