/* Mongla console -- rendering.
 *
 * ONE IDEA RUNS THROUGH ALL OF IT: the render loop is decoupled from the data
 * loop. Data arrives at ~25 Hz; every drawn quantity chases its target each
 * animation frame instead of snapping to it. Repainting on arrival looks
 * stuttery no matter how fast you poll, because the eye sees the STEP, not the
 * rate. This is the same lesson as the control loop being paced by the JPEG
 * encoder: whoever consumes must not set the pace of whoever produces.
 */
const $ = id => document.getElementById(id);
const lerp = (a, b, t) => a + (b - a) * t;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
// Angles must chase the SHORT way round or the needle spins the long way
// through north -- the same wrap the heading filter handles server-side.
const lerpAng = (a, b, t) => a + (((b - a + 540) % 360) - 180) * t;

let S = {};                      // latest server snapshot
let R = {                        // rendered state, chasing S
  lat: 0, fwd: 0, bear: 0, hasB: 0, roll: 0, pitch: 0, yaw: 0,
  depth: 0, mix: new Array(8).fill(0), rpm: new Array(8).fill(0),
  link: 0, cpu: 0
};

async function poll() {
  try { S = await (await fetch('/j')).json(); } catch (e) { }
  setTimeout(poll, 40);
}
poll();

function ctx(c) {
  const dpr = window.devicePixelRatio || 1, r = c.getBoundingClientRect();
  const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, r.width, r.height);
  return [g, r.width, r.height];
}
const CSS = k => getComputedStyle(document.documentElement)
  .getPropertyValue(k).trim();

/* ---- attitude: a real artificial horizon ------------------------------- */
function drawHorizon() {
  const [g, w, h] = ctx($('horizon'));
  const cx = w / 2, cy = h / 2, R0 = Math.min(w, h) * 0.42;
  g.save(); g.beginPath(); g.arc(cx, cy, R0, 0, 7); g.clip();
  g.translate(cx, cy); g.rotate(-R.roll * Math.PI / 180);
  const pxPerDeg = R0 / 32, off = R.pitch * pxPerDeg;
  g.fillStyle = '#132030'; g.fillRect(-R0 * 2, -R0 * 2 + off, R0 * 4, R0 * 2);
  g.fillStyle = '#2a1c12'; g.fillRect(-R0 * 2, off, R0 * 4, R0 * 2);
  g.strokeStyle = CSS('--sense'); g.lineWidth = 1.4;
  g.beginPath(); g.moveTo(-R0 * 1.6, off); g.lineTo(R0 * 1.6, off); g.stroke();
  g.strokeStyle = '#4a5a6c'; g.lineWidth = 1; g.font = '8px ui-monospace';
  g.fillStyle = '#6b7c8f'; g.textAlign = 'center';
  for (let p = -30; p <= 30; p += 10) {
    if (!p) continue;
    const y = off - p * pxPerDeg, len = R0 * 0.30;
    g.beginPath(); g.moveTo(-len, y); g.lineTo(len, y); g.stroke();
    g.fillText(Math.abs(p), len + 11, y + 3);
  }
  g.restore();
  // fixed aircraft symbol
  g.strokeStyle = CSS('--cmd'); g.lineWidth = 2.4;
  g.beginPath(); g.moveTo(cx - R0 * .42, cy); g.lineTo(cx - R0 * .13, cy);
  g.moveTo(cx + R0 * .13, cy); g.lineTo(cx + R0 * .42, cy);
  g.moveTo(cx, cy - 3); g.lineTo(cx, cy + 3); g.stroke();
  g.strokeStyle = CSS('--line'); g.lineWidth = 1.4;
  g.beginPath(); g.arc(cx, cy, R0, 0, 7); g.stroke();
  // roll pointer
  g.save(); g.translate(cx, cy); g.rotate(-R.roll * Math.PI / 180);
  g.beginPath(); g.moveTo(0, -R0 + 1); g.lineTo(-5, -R0 + 10); g.lineTo(5, -R0 + 10);
  g.closePath(); g.fillStyle = CSS('--warn'); g.fill(); g.restore();
}

/* ---- heading tape ------------------------------------------------------ */
function drawHeading() {
  const [g, w, h] = ctx($('hdg'));
  const cx = w / 2, span = 90, ppd = w / span;
  g.fillStyle = '#0a0e14'; g.fillRect(0, 0, w, h);
  g.font = '9px ui-monospace'; g.textAlign = 'center';
  for (let d = -60; d <= 60; d += 5) {
    const deg = R.yaw + d, x = cx + d * ppd;
    if (x < -20 || x > w + 20) continue;
    const dd = ((Math.round(deg) % 360) + 360) % 360;
    const major = dd % 30 === 0;
    g.strokeStyle = major ? '#4a5a6c' : '#232c37'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(x, h - (major ? 15 : 8)); g.lineTo(x, h); g.stroke();
    if (major) {
      g.fillStyle = '#7b8b9c';
      const lbl = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[dd] ?? dd;
      g.fillText(lbl, x, h - 19);
    }
  }
  g.fillStyle = CSS('--sense');
  g.beginPath(); g.moveTo(cx, h - 2); g.lineTo(cx - 6, h - 12);
  g.lineTo(cx + 6, h - 12); g.closePath(); g.fill();
  g.font = '600 15px ui-monospace'; g.fillStyle = CSS('--ink');
  g.fillText(('00' + Math.round(R.yaw)).slice(-3) + '°', cx, 17);
}

/* ---- depth ladder ------------------------------------------------------ */
function drawDepth() {
  const [g, w, h] = ctx($('depth'));
  const cy = h / 2, ppm = h / 3.0;          // +-1.5 m visible
  g.fillStyle = '#0a0e14'; g.fillRect(0, 0, w, h);
  g.font = '9px ui-monospace'; g.textAlign = 'right';
  for (let d = -3; d <= 3; d += 0.25) {
    const y = cy + (d - R.depth) * ppm;
    if (y < 0 || y > h) continue;
    const major = Math.abs(d * 4) % 4 === 0;
    g.strokeStyle = major ? '#4a5a6c' : '#232c37';
    g.beginPath(); g.moveTo(w - (major ? 22 : 12), y); g.lineTo(w - 3, y); g.stroke();
    if (major) { g.fillStyle = '#7b8b9c'; g.fillText(d.toFixed(1), w - 26, y + 3); }
  }
  // the surface is a real boundary; draw it as one
  const ys = cy + (0 - R.depth) * ppm;
  if (ys > -20 && ys < h + 20) {
    const gr = g.createLinearGradient(0, ys - 14, 0, ys + 14);
    gr.addColorStop(0, 'rgba(74,168,255,0)');
    gr.addColorStop(.5, 'rgba(74,168,255,.35)');
    gr.addColorStop(1, 'rgba(74,168,255,0)');
    g.fillStyle = gr; g.fillRect(0, ys - 14, w, 28);
  }
  g.strokeStyle = CSS('--sense'); g.lineWidth = 2;
  g.beginPath(); g.moveTo(2, cy); g.lineTo(w - 3, cy); g.stroke();
}

/* ---- vehicle, top down, rotating with real heading --------------------- */
const HORIZ = [{ x: 1, y: -1, a: -45 }, { x: -1, y: -1, a: 45 },
               { x: 1, y: 1, a: 45 }, { x: -1, y: 1, a: -45 }];
function drawAUV() {
  const [g, w, h] = ctx($('auv'));
  const cx = w / 2, cy = h / 2, S1 = Math.min(w, h) * 0.30;
  // north ring: the hull rotates inside a fixed compass, so a turn is visible
  g.strokeStyle = '#151d27'; g.lineWidth = 1;
  g.beginPath(); g.arc(cx, cy, S1 * 1.72, 0, 7); g.stroke();
  g.font = '9px ui-monospace'; g.textAlign = 'center'; g.fillStyle = '#3c4756';
  [['N', 0], ['E', 90], ['S', 180], ['W', 270]].forEach(([l, d]) => {
    const a = (d - R.yaw) * Math.PI / 180;
    g.fillText(l, cx + Math.sin(a) * S1 * 1.92, cy - Math.cos(a) * S1 * 1.92 + 3);
  });

  // camera cone -- measured 63.82 deg, not a guess
  const half = (63.82 / 2) * Math.PI / 180;
  const gr = g.createLinearGradient(cx, cy - S1, cx, cy - S1 * 3.0);
  gr.addColorStop(0, 'rgba(74,168,255,.15)'); gr.addColorStop(1, 'rgba(74,168,255,0)');
  g.beginPath(); g.moveTo(cx, cy - S1 * .2);
  g.lineTo(cx + Math.sin(-half) * S1 * 3, cy - Math.cos(-half) * S1 * 3);
  g.lineTo(cx + Math.sin(half) * S1 * 3, cy - Math.cos(half) * S1 * 3);
  g.closePath(); g.fillStyle = gr; g.fill();

  g.save(); g.translate(cx, cy);
  const R1 = S1 * 0.9;
  g.beginPath();
  for (let i = 0; i < 8; i++) {
    const a = Math.PI / 8 + i * Math.PI / 4;
    const px = Math.cos(a) * R1 * .86, py = Math.sin(a) * R1;
    i ? g.lineTo(px, py) : g.moveTo(px, py);
  }
  g.closePath(); g.fillStyle = '#111922'; g.fill();
  g.strokeStyle = '#2b3644'; g.lineWidth = 1.5; g.stroke();
  g.beginPath(); g.moveTo(0, -R1 * .99); g.lineTo(-7, -R1 * .72);
  g.lineTo(7, -R1 * .72); g.closePath(); g.fillStyle = '#2b3644'; g.fill();

  HORIZ.forEach((t, i) => {
    const px = t.x * R1 * .70, py = t.y * R1 * .62, v = R.mix[i];
    g.save(); g.translate(px, py); g.rotate(t.a * Math.PI / 180);
    g.beginPath(); g.roundRect(-8, -15, 16, 30, 5);
    g.fillStyle = '#0c1219'; g.fill(); g.strokeStyle = '#2b3644'; g.stroke();
    const L = Math.abs(v) * 44, dir = v >= 0 ? -1 : 1;
    if (L > 1.5) {
      const col = v >= 0 ? CSS('--cmd') : CSS('--fault');
      g.beginPath(); g.moveTo(0, dir * 14); g.lineTo(0, dir * (14 + L));
      g.strokeStyle = col; g.lineWidth = 4.5; g.lineCap = 'round'; g.stroke();
      g.beginPath(); g.moveTo(0, dir * (14 + L + 5));
      g.lineTo(-4.5, dir * (14 + L - 2)); g.lineTo(4.5, dir * (14 + L - 2));
      g.closePath(); g.fillStyle = col; g.fill();
    }
    g.restore();
    g.fillStyle = '#3c4756'; g.font = '9px ui-monospace'; g.textAlign = 'center';
    g.fillText('M' + (i + 1), px, py + t.y * 30 + 3);
  });
  [4, 5, 6, 7].forEach((i, k) => {
    const t = HORIZ[k], px = t.x * R1 * .30, py = t.y * R1 * .28, v = R.mix[i];
    g.beginPath(); g.arc(px, py, 9, 0, 7);
    g.strokeStyle = '#2b3644'; g.lineWidth = 1.4; g.stroke();
    if (Math.abs(v) > .02) {
      g.beginPath(); g.arc(px, py, 9, -Math.PI / 2, -Math.PI / 2 + Math.abs(v) * 6.283);
      g.strokeStyle = v >= 0 ? CSS('--cmd') : CSS('--fault'); g.lineWidth = 3; g.stroke();
    }
  });

  // resultant demand. Written as a direct vector: +lat -> +x, +fwd -> -y.
  // A previous version composed atan2 with rotate() and had the axes swapped
  // and forward negated -- a port command drew an arrow pointing aft, and it
  // looked entirely plausible.
  const vx = R.lat, vy = -R.fwd, mag = Math.hypot(vx, vy);
  if (mag * S1 * 1.6 > 3) {
    const ux = vx / mag, uy = vy / mag, L = mag * S1 * 1.6;
    g.beginPath(); g.moveTo(0, 0); g.lineTo(ux * L, uy * L);
    g.strokeStyle = CSS('--cmd'); g.lineWidth = 3; g.lineCap = 'round'; g.stroke();
    g.beginPath(); g.moveTo(ux * (L + 9), uy * (L + 9));
    g.lineTo(ux * L - uy * 6, uy * L + ux * 6);
    g.lineTo(ux * L + uy * 6, uy * L - ux * 6);
    g.closePath(); g.fillStyle = CSS('--cmd'); g.fill();
  }
  if (R.hasB > .02) {
    const a = R.bear, L = S1 * 2.5;
    g.beginPath(); g.moveTo(0, 0);
    g.lineTo(Math.sin(a) * L, -Math.cos(a) * L);
    g.strokeStyle = 'rgba(74,168,255,' + (.3 + .5 * R.hasB) + ')';
    g.lineWidth = 2; g.setLineDash([7, 5]); g.stroke(); g.setLineDash([]);
    g.beginPath(); g.arc(Math.sin(a) * L, -Math.cos(a) * L, 4.5, 0, 7);
    g.fillStyle = CSS('--sense'); g.fill();
  }
  g.restore();
}

/* ---- generic trace ----------------------------------------------------- */
function trace(id, series, opts) {
  const [g, w, h] = ctx($(id));
  const pad = 4;
  let lo = opts.lo, hi = opts.hi;
  if (lo === undefined) {
    const all = series.flatMap(s => s.d).filter(v => v != null && isFinite(v));
    if (!all.length) return;
    lo = Math.min(...all); hi = Math.max(...all);
    const m = (hi - lo) * .15 || 1; lo -= m; hi += m;
  }
  const Y = v => h - pad - ((v - lo) / (hi - lo || 1)) * (h - pad * 2);
  if (opts.zero !== false && lo < 0 && hi > 0) {
    g.strokeStyle = '#1a212b'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(w, Y(0)); g.stroke();
  }
  series.forEach(s => {
    const d = s.d; if (!d || d.length < 2) return;
    const dx = w / (d.length - 1);
    g.beginPath();
    d.forEach((v, i) => {
      if (v == null || !isFinite(v)) return;
      const y = Y(v); i ? g.lineTo(i * dx, y) : g.moveTo(i * dx, y);
    });
    g.strokeStyle = s.c; g.lineWidth = s.w || 1.5; g.stroke();
  });
  g.font = '9px ui-monospace'; g.fillStyle = '#3c4756'; g.textAlign = 'left';
  g.fillText(hi.toFixed(opts.dp ?? 1), 3, 10);
  g.fillText(lo.toFixed(opts.dp ?? 1), 3, h - 3);
}

/* ---- link budget ------------------------------------------------------- */
function drawLink() {
  const [g, w, h] = ctx($('link'));
  const used = clamp(R.link, 0, 1.2), bar = h * .42;
  g.fillStyle = '#0a0e14'; g.fillRect(0, (h - bar) / 2, w, bar);
  const col = used > .85 ? CSS('--fault') : used > .6 ? CSS('--warn') : CSS('--sense');
  g.fillStyle = col; g.fillRect(0, (h - bar) / 2, w * Math.min(used, 1), bar);
  // 60/85 % marks: where headroom stops being comfortable, and where a burst
  // starts costing latency
  [.6, .85].forEach(f => {
    g.strokeStyle = '#2b3644'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(w * f, (h - bar) / 2 - 3);
    g.lineTo(w * f, (h + bar) / 2 + 3); g.stroke();
  });
  g.font = '600 11px ui-monospace'; g.fillStyle = CSS('--ink'); g.textAlign = 'left';
  g.fillText((used * 100).toFixed(0) + '% of 11520 B/s', 4, (h - bar) / 2 - 6);
}

/* ---- pipeline latency, as a stacked waterfall -------------------------- */
function drawPipe() {
  const [g, w, h] = ctx($('pipe'));
  const v = (S.vision || {}), segs = [
    ['grab', v.t_grab || 0, '#2b6cb0'], ['infer', v.t_inf || 0, CSS('--sense')],
    ['decode', v.t_dec || 0, '#7cc4ff'], ['ctrl', v.t_ctl || 0, CSS('--cmd')]];
  const total = segs.reduce((a, s) => a + s[1], 0) || 1;
  const budget = Math.max(total, 1000 / (S.loop_hz || 50));
  let x = 0; const bar = h * .40, y = (h - bar) / 2 - 4;
  segs.forEach(([n, ms, c]) => {
    const ww = (ms / budget) * w;
    g.fillStyle = c; g.fillRect(x, y, Math.max(0, ww - 1), bar);
    if (ww > 34) {
      g.font = '9px ui-monospace'; g.fillStyle = '#04070a'; g.textAlign = 'center';
      g.fillText(ms.toFixed(1), x + ww / 2, y + bar / 2 + 3);
    }
    x += ww;
  });
  g.fillStyle = '#0d131a'; g.fillRect(x, y, Math.max(0, w - x), bar);
  g.font = '9px ui-monospace'; g.fillStyle = '#5a6878'; g.textAlign = 'left';
  g.fillText(`${total.toFixed(1)} ms used of ${budget.toFixed(1)} ms budget`,
             2, y + bar + 13);
}

/* ---- thrusters: commanded outline vs measured fill --------------------- */
function drawThr() {
  const [g, w, h] = ctx($('thr'));
  const n = 8, pad = 6, bw = (w - pad * (n - 1)) / n, mid = h * .56;
  for (let i = 0; i < n; i++) {
    const x = i * (bw + pad), v = R.mix[i], rp = R.rpm[i] / 3600;
    g.strokeStyle = '#1a212b'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(x, mid); g.lineTo(x + bw, mid); g.stroke();
    const ch = Math.abs(v) * (h * .40);
    if (ch > .8) {
      g.strokeStyle = v >= 0 ? CSS('--cmd') : CSS('--fault'); g.lineWidth = 1.6;
      g.strokeRect(x + .5, v >= 0 ? mid - ch : mid, bw - 1, ch);
    }
    const mh = Math.abs(rp) * (h * .40);
    if (mh > .8) {
      g.fillStyle = rp >= 0 ? 'rgba(57,217,138,.5)' : 'rgba(255,91,110,.5)';
      g.fillRect(x + 2, rp >= 0 ? mid - mh : mid, bw - 4, mh);
    }
    g.fillStyle = '#3c4756'; g.font = '9px ui-monospace'; g.textAlign = 'center';
    g.fillText('M' + (i + 1), x + bw / 2, h - 2);
  }
}

/* ---- health matrix ----------------------------------------------------- */
const HEALTH = [
  ['LEAK', v => v == null ? ['absent', '—'] : v > .5 ? ['bad', 'WET'] : ['ok', 'DRY']],
  ['KILL', v => v == null ? ['absent', '—'] : v > .5 ? ['bad', 'KILLED'] : ['ok', 'RUN']],
  ['BARO_HEALT', v => v == null ? ['absent', '—'] :
     [['ok', 'HEALTHY'], ['warn', 'JITTER'], ['bad', 'READ FAIL'], ['bad', 'NO INIT']][v | 0]],
  ['YAW_REF', v => v == null ? ['absent', '—'] :
     v === 2 ? ['ok', 'LOCKED'] : ['warn', ['IDLE', 'SAMPLING', '', 'REF CAL',
       'REF FIELD', 'REF NOISE'][v | 0] || String(v)]],
  ['MAGACC', v => v == null ? ['absent', '—'] :
     [['bad', '0 UNREL'], ['warn', '1 LOW'], ['warn', '2 MED'], ['ok', '3 HIGH']][v | 0]],
  ['MIX_VSGN', v => v == null ? ['absent', '—'] :
     v === 4 ? ['ok', '4/4'] : ['bad', v + '/4']],
  ['MIX_VERT', v => v == null ? ['absent', '—'] :
     Math.abs(v + 1) < .01 ? ['ok', '-1.00'] : ['bad', v.toFixed(2)]],
  ['COMP_SEEN', v => v == null ? ['absent', '—'] : v > .5 ? ['ok', 'SEEN'] : ['info', 'NONE']],
  ['BARO_P2P', v => v == null ? ['absent', '—'] :
     [v > 15 ? 'bad' : v > 10 ? 'warn' : 'ok', v.toFixed(1) + ' mb']],
  ['WTEMP', v => v == null ? ['absent', '—'] : ['info', v.toFixed(1) + '°C']],
  ['GAIN', v => v == null ? ['absent', '—'] :
     [v < .99 ? 'warn' : 'ok', (v * 100).toFixed(0) + '%']],
  ['DEPTH_OUT', v => v == null ? ['absent', 'NOT RUN'] :
     [Math.abs(v) > .9 ? 'bad' : 'ok', v.toFixed(2)]],
];
function drawHealth() {
  const nm = S.board?.named || {};
  $('mat').innerHTML = HEALTH.map(([k, f]) => {
    const [cls, txt] = f(nm[k]);
    return `<div class="cell ${cls}"><div class=n>${k}</div><div class=d>${txt}</div></div>`;
  }).join('');
}

/* ---- rates ------------------------------------------------------------- */
function drawRates() {
  const r = S.board?.rates || {};
  const rows = Object.entries(r).sort((a, b) => b[1] - a[1]).slice(0, 12);
  const max = Math.max(1, ...rows.map(x => x[1]));
  $('rates').innerHTML = rows.map(([k, v]) => {
    const bad = k === 'BAD_DATA';
    return `<div class=k style="${bad ? 'color:var(--fault)' : ''}">${k}</div>
      <div class=v style="position:relative">
        <span style="position:absolute;left:0;top:2px;height:9px;
          width:${(v / max) * 60}%;background:${bad ? 'var(--fault)' : '#1d3a52'};
          border-radius:2px"></span>
        <span style="position:relative">${v.toFixed(1)}</span></div>`;
  }).join('');
}

/* ---- the frame --------------------------------------------------------- */
function txt(id, v) { const e = $(id); if (e) e.textContent = v; }
function pill(id, cls, v) {
  const e = $(id); if (!e) return;
  e.className = 'pill ' + cls; e.textContent = v;
}

function frame() {
  const k = .25, b = S.board || {}, v = S.vision || {}, hst = S.host || {};
  R.lat = lerp(R.lat, v.lat || 0, k);
  R.fwd = lerp(R.fwd, v.fwd || 0, k);
  R.roll = lerp(R.roll, b.att?.roll || 0, .18);
  R.pitch = lerp(R.pitch, b.att?.pitch || 0, .18);
  R.yaw = lerpAng(R.yaw, b.hdg_filt ?? b.att?.yaw ?? 0, .18);
  R.depth = lerp(R.depth, b.depth_filt ?? 0, .18);
  R.link = lerp(R.link, (S.link?.util || 0), .12);
  R.cpu = lerp(R.cpu, hst.cpu || 0, .1);
  for (let i = 0; i < 8; i++) {
    R.mix[i] = lerp(R.mix[i], (v.mix || [])[i] || 0, k);
    R.rpm[i] = lerp(R.rpm[i], (b.esc?.rpm || [])[i] || 0, .18);
  }
  if (v.bear != null) {
    R.bear = lerp(R.bear, v.bear * Math.PI / 180, k);
    R.hasB = lerp(R.hasB, 1, .2);
  } else R.hasB = lerp(R.hasB, 0, .12);

  pill('armed', v.armed ? 'ok' : '', v.armed ? 'ARMED' : 'DISARMED');
  pill('mode', v.armed ? 'ok' : '', b.mode || S.mode || '—');
  pill('fw', 'ok', 'fw rev ' + (S.fw_rev ?? '?'));
  pill('reboots', (b.reboots | 0) ? 'bad' : '', 'FC reboots ' + (b.reboots | 0));
  pill('uptime', '', 'up ' + ((b.boot_ms || 0) / 1000).toFixed(0) + ' s');

  txt('t_target', v.cls ? `${v.cls} ${(v.score || 0).toFixed(2)}` : '—');
  txt('t_bear', v.bear == null ? '—' : v.bear.toFixed(2));
  txt('t_bearraw', v.braw == null ? '' : 'raw ' + v.braw.toFixed(2) + '°');
  txt('t_lat', (v.lat || 0).toFixed(3));
  txt('t_loop', (S.loop_hz || 0).toFixed(1));
  txt('t_det', (v.det_hz || 0).toFixed(1));
  txt('t_snaps', (v.resets | 0));
  txt('t_depth', (b.depth_filt ?? 0).toFixed(2));
  txt('t_depthraw', b.depth_raw == null ? '' : 'raw ' + b.depth_raw.toFixed(2));
  txt('t_hdg', (b.hdg_filt ?? 0).toFixed(0));
  txt('t_roll', (b.att?.roll ?? 0).toFixed(1));
  txt('t_pitch', (b.att?.pitch ?? 0).toFixed(1));
  txt('t_yawrate', (b.att?.yawspeed ?? 0).toFixed(1));
  txt('t_press', (b.press?.abs ?? 0).toFixed(1));
  txt('t_wtemp', (b.press?.temp ?? 0).toFixed(1));
  txt('t_mag', (b.imu?.mag ?? 0).toFixed(1));
  txt('t_batt', Object.values(b.batt || {}).map(x => x.toFixed(2)).join(' / ') || '—');
  txt('t_vcc', (b.power?.vcc ?? 0).toFixed(2));
  txt('t_load', (b.sys?.load ?? 0).toFixed(0));
  txt('t_drop', (b.sys?.drop ?? 0).toFixed(2));
  txt('t_errs', (b.sys?.errors ?? 0));
  txt('t_bps', (S.link?.bps ?? 0).toFixed(0));
  txt('t_cpu', (hst.cpu ?? 0).toFixed(0));
  txt('t_ptemp', (hst.temp ?? 0).toFixed(1));
  txt('t_clock', (hst.clock_ghz ?? 0).toFixed(2));
  txt('t_mem', `${(hst.mem_used ?? 0) | 0}/${(hst.mem_total ?? 0) | 0}`);
  txt('t_heap', ((b.named?.HEAP ?? 0) / 1024).toFixed(0));
  pill('throttled', (hst.throttled && hst.throttled !== '0x0') ? 'warn' : 'ok',
       'throttled ' + (hst.throttled || '?'));
  txt('t_esc', b.esc_msgs ? (R.rpm.some(x => Math.abs(x) > 1)
      ? 'live' : 'stream live · 0 rpm (no escs)') : 'no telemetry');

  drawHorizon(); drawHeading(); drawDepth(); drawAUV(); drawThr();
  drawLink(); drawPipe(); drawHealth(); drawRates();

  const tr = b.trace || {}, vh = v.hist || [];
  trace('tr_bear', [
    { d: vh.map(p => p[0]), c: 'rgba(74,168,255,.30)', w: 1 },
    { d: vh.map(p => p[3]), c: CSS('--sense'), w: 1.8 },
    { d: vh.map(p => p[1] * 30), c: CSS('--cmd'), w: 1.8 }], { dp: 1 });
  trace('tr_depth', [
    { d: tr.depth, c: 'rgba(74,168,255,.30)', w: 1 },
    { d: tr.depth_f, c: CSS('--sense'), w: 1.8 }], { dp: 2 });
  trace('tr_hdg', [
    { d: tr.hdg, c: 'rgba(74,168,255,.30)', w: 1 },
    { d: tr.hdg_f, c: CSS('--sense'), w: 1.8 }], { dp: 0, zero: false });
  trace('tr_att', [
    { d: tr.roll, c: CSS('--cmd'), w: 1.5 },
    { d: tr.pitch, c: CSS('--warn'), w: 1.5 },
    { d: tr.gz, c: CSS('--sense'), w: 1.2 }], { dp: 1 });

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
