// The Mongla panel: the whole vehicle at a glance, as its own nodes judge it.
//
// Plain DOM, no build, zipped by serve.py like the helm panel. It reads only
// small messages: the vehicle's state, the manager's graded health board
// (/diagnostics), the link, the helm, each camera's lock and what the
// localization is aided by. No image and no detection stream comes through
// here, so opening it costs the vehicle almost nothing, and it redraws at
// most ten times a second and only when something changed.
//
// Its look is theme/batighor.css, the site's palette: red marks something
// measured and live, ocean what the machine perceives, amber caution; the
// helpers (el, num, card) are panels/common.js.

const CAMS = ["forward", "downward"];
const RUNGS = ["detection", "held", "follow", "anchor", "lost"];
const T = {
  state: "/mongla/state",
  diag: "/diagnostics",
  link: "/mongla/board_link",
  helm: "/mongla/batighor/helm_state",
  aiding: "/mongla/localization/aiding",
  motion: "/mongla/localization/motion",
  odom: "/mongla/odom",
  active: "/mongla/vision/active_camera",
};
const lockTopic = (c) => `/mongla/vision/${c}/lock`;
const seeingTopic = (c) => `/mongla/vision/${c}/seeing`;
const REDRAW_MS = 100;

const RUNG_COLOUR = { detection: "var(--mb-red)", held: "var(--mb-amber)", follow: "var(--mb-glass)", anchor: "var(--mb-violet)", lost: "var(--mb-ink-3)" };
const LEVEL = ["ok", "warn", "err", "stale"];

const ageS = (t) => (t ? ((Date.now() - t) / 1000) : Infinity);

// Exported for the test: which rung a lock message says holds the target.
function rungOf(lock) {
  const d = lock && lock.detections && lock.detections[0];
  if (!d) return { rung: "lost", id: "", authority: null };
  const r = d.results && d.results[0];
  const hyp = r && (r.hypothesis || r);
  return { rung: hyp ? String(hyp.class_id) : "lost", id: d.id || "", authority: hyp ? hyp.score : null };
}

function initPanel(context) {
  const box = mount(context, "cards");

  const s = { lock: {}, lockAt: {}, seeing: {}, health: [] };
  let dirty = true;

  const sec = (title, cls) => {
    const body = el("div", { style: "display:flex;flex-direction:column;gap:6px" });
    card(box, title, null, cls).append(body);
    return body;
  };
  const vVehicle = sec("vehicle");
  const vVision = sec("vision");
  const vNav = sec("navigation");
  const vLink = sec("link");
  const vHealth = sec("health, as the manager grades it", "wide");

  function drawVehicle() {
    const v = s.state;
    const h = s.helm || {};
    if (!v) { vVehicle.replaceChildren("no /mongla/state: is the manager running?"); return; }
    vVehicle.replaceChildren(
      el("div", { class: "row" },
        el("span", { class: v.armed ? "armed" : "safe" }, v.armed ? "ARMED" : "disarmed"),
        el("span", {}, v.mode || "--"),
        h.mission ? el("span", { class: "live" }, `mission ${h.mission}`) : null),
      el("div", { class: "row" },
        el("span", { class: "big" }, num(fin(v.depth_m) ? -v.depth_m : NaN, 2), el("span", { class: "unit" }, "m deep")),
        el("span", { class: "big" }, num(v.yaw_deg, 0), el("span", { class: "unit" }, "deg")),
        el("span", { class: "big" }, num(v.battery_voltage, 1), el("span", { class: "unit" }, "V"))),
      el("div", { class: "kv" },
        el("span", {}, "pilot"), el("span", {}, h.pilot || "nobody"),
        el("span", {}, "state age"), el("span", {}, `${num(ageS(s.stateAt), 1)} s`)),
    );
  }

  function drawVision() {
    const kids = [];
    for (const c of CAMS) {
      const { rung, id, authority } = rungOf(s.lock[c]);
      const stale = ageS(s.lockAt[c]) > 1.0;
      const live = s.active === c;
      kids.push(el("div", { class: "row" },
        el("b", { class: live ? "live" : "" }, c + (live ? "  (live)" : "")),
        stale ? el("span", { class: "stale" }, s.lockAt[c] ? `lock silent ${num(ageS(s.lockAt[c]), 1)} s` : "no lock node")
          : el("span", {}, `${id || "--"}  ${num(authority, 2)}`)));
      kids.push(el("div", { class: "ladder" }, ...RUNGS.map((r) => {
        const on = !stale && r === rung;
        return el("i", on ? { class: "on", style: `background:${RUNG_COLOUR[r]}` } : {}, r);
      })));
      if (s.seeing[c]) kids.push(el("div", { class: "stale" }, `seeing: ${s.seeing[c]}`));
    }
    vVision.replaceChildren(...kids);
  }

  function drawNav() {
    const o = s.odom;
    const p = o && o.pose && o.pose.pose && o.pose.pose.position;
    const tw = o && o.twist && o.twist.twist && o.twist.twist.linear;
    vNav.replaceChildren(el("div", { class: "kv" },
      el("span", {}, "aided by"), el("span", { class: s.aiding === "aided" ? "live" : "warn" }, s.aiding || "--"),
      el("span", {}, "motion"), el("span", {}, s.motion || "--"),
      el("span", {}, "x, y"), el("span", {}, p ? `${num(p.x, 2)}, ${num(p.y, 2)} m` : "--"),
      el("span", {}, "speed"), el("span", {}, tw ? `${num(Math.hypot(tw.x, tw.y), 2)} m/s` : "--"),
      el("span", {}, "odom age"), el("span", {}, `${num(ageS(s.odomAt), 1)} s`)));
  }

  function drawLink() {
    const l = s.link;
    if (!l) { vLink.replaceChildren("no /mongla/board_link"); return; }
    const use = l.capacity_bytes_per_s > 0 ? l.bytes_per_s / l.capacity_bytes_per_s : 0;
    vLink.replaceChildren(
      el("div", { class: "row" }, el("span", { class: "big" }, num(100 * use, 0), el("span", { class: "unit" }, "% of the line"))),
      el("div", { class: "bar" }, el("b", { style: `width:${Math.min(100, 100 * use)}%;${use > 0.8 ? "background:var(--mb-amber)" : ""}` })),
      el("div", { class: "kv" },
        el("span", {}, "bytes/s"), el("span", {}, num(l.bytes_per_s, 0)),
        el("span", {}, "board drops"), el("span", { class: l.board_drop_pct > 1 ? "warn" : "" }, `${num(l.board_drop_pct, 1)} %`),
        el("span", {}, "board errors"), el("span", {}, num(l.board_errors, 0))));
  }

  function drawHealth() {
    // Worst first, then by name: what needs a person is at the top.
    const rows = [...s.health].sort((a, b) => (b.level - a.level) || a.name.localeCompare(b.name));
    vHealth.replaceChildren(rows.length
      ? el("div", { class: "health" }, ...rows.flatMap((r) => [
        el("span", { class: LEVEL[r.level] || "stale" }, r.name.replace(/^mongla: /, "")),
        el("span", { class: r.level ? LEVEL[r.level] : "", title: r.message }, r.message)]))
      : "no /diagnostics: the manager grades health once it runs");
  }

  function draw() {
    drawVehicle(); drawVision(); drawNav(); drawLink(); drawHealth();
  }

  const topics = [...Object.values(T), ...CAMS.flatMap((c) => [lockTopic(c), seeingTopic(c)])];
  context.subscribe(topics.map((topic) => ({ topic })));
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    const now = Date.now();
    for (const ev of renderState.currentFrame || []) {
      const m = ev.message;
      dirty = true;
      switch (ev.topic) {
        case T.state: s.state = m; s.stateAt = now; break;
        case T.diag: for (const st of m.status || []) {
          const i = s.health.findIndex((h) => h.name === st.name);
          const row = { name: st.name, level: typeof st.level === "number" ? st.level : Number(st.level), message: st.message };
          if (i < 0) s.health.push(row); else s.health[i] = row;
        } break;
        case T.link: s.link = m; break;
        case T.helm: try { s.helm = JSON.parse(m.data); } catch { /* keep */ } break;
        case T.aiding: s.aiding = m.data; break;
        case T.motion: s.motion = m.data; break;
        case T.odom: s.odom = m; s.odomAt = now; break;
        case T.active: s.active = m.data; break;
        default:
          for (const c of CAMS) {
            if (ev.topic === lockTopic(c)) { s.lock[c] = m; s.lockAt[c] = now; }
            if (ev.topic === seeingTopic(c)) s.seeing[c] = m.data;
          }
      }
    }
    done();
  };
  // Ages move with no message, and a silent node is exactly what an operator
  // must see; so the clock redraws too, at the same capped rate.
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
}

module.exports = {
  activate(extensionContext) {
    extensionContext.registerPanel({ name: "Mongla", initPanel });
  },
  rungOf,
};
