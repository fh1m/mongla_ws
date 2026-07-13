'use strict';
// Mongla mission console — consumes the /events SSE feed, renders both cameras,
// and POSTs control writes to the node (which mirrors the DSL SetParameters surface).

const VIDEO_HOST = location.hostname;      // same box as the console; web_video_server on :video_port
let VIDEO_PORT = 8080;
const cols = {};                            // cam name -> { el, refs, lastKey }

function videoUrl(cam) {
  return `http://${VIDEO_HOST}:${VIDEO_PORT}/stream` +
         `?topic=/duburi/vision/${cam}/image_debug&type=mjpeg&quality=80`;
}

// Escape detector-supplied strings (class/model/id tokens) before innerHTML.
// They come from our own models on a private net, but escaping is free correctness.
function esc(s) {
  return String(s).replace(/[&<>"']/g, (ch) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}

function toast(msg, isErr) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.toggle('err', !!isErr);
  t.classList.add('show');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => t.classList.remove('show'), 2200);
}

async function post(path, body) {
  try {
    const r = await fetch(path, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const j = await r.json();
    if (!j.ok) toast(j.reason || 'rejected', true);
    return j.ok;
  } catch (e) { toast('request failed: ' + e, true); return false; }
}

// ---- column construction -------------------------------------------------
function makeColumn(cam) {
  const tpl = document.getElementById('cam-tpl').content.cloneNode(true);
  const el = tpl.querySelector('.cam');
  const q = (s) => el.querySelector(s);
  const refs = {
    el, name: q('.cam-name'), model: q('.model-name'), fps: q('.fps'), fpsVal: q('.fps-val'),
    img: q('.video'), msg: q('.video-msg'), msgTxt: q('.msg-txt'),
    ohx: q('.oh-x'), ohy: q('.oh-y'),
    actBtn: q('.act-btn'), pauseBtn: q('.pause-btn'),
    conf: q('.conf'), confVal: q('.conf-val'),
    modelSel: q('.model-sel'), modelCtl: q('.model-ctl'),
    classChips: q('.class-chips'), addClass: q('.add-class'), allClass: q('.all-class'),
    counts: q('.counts-chips'), body: q('.det-body'),
  };
  refs.name.textContent = cam.toUpperCase();
  refs.img.onerror = () => { refs.img.classList.remove('on'); showMsg(refs, 'stream not available', 'bad'); };

  refs.actBtn.onclick = () => post('/api/active_camera', { camera: cam });
  refs.pauseBtn.onclick = () => post(`/api/detector/${cam}/param`,
    { name: 'paused', value: !refs.pauseBtn.classList.contains('is-paused') });
  refs.conf.onchange = () => post(`/api/detector/${cam}/param`,
    { name: 'conf', value: parseFloat(refs.conf.value) });
  refs.conf.oninput = () => { refs.confVal.textContent = (+refs.conf.value).toFixed(2); };
  refs.modelSel.onchange = () => post(`/api/detector/${cam}/param`,
    { name: 'active_model', value: refs.modelSel.value });
  refs.addClass.onclick = () => {
    const cur = cols[cam].classes || [];
    const add = prompt(`add class for ${cam} (current: ${cur.join(', ') || 'all'})`);
    if (add && add.trim()) post(`/api/detector/${cam}/param`,
      { name: 'classes', value: [...cur, add.trim()].join(',') });
  };
  refs.allClass.onclick = () => post(`/api/detector/${cam}/param`, { name: 'classes', value: '' });

  cols[cam] = { el, refs, classes: [], lastVideoKey: '' };
  document.getElementById('cams').appendChild(el);
}

function showMsg(refs, txt, kind) {
  refs.msg.classList.remove('hide', 'bad', 'warn');
  if (kind) refs.msg.classList.add(kind);
  refs.msgTxt.textContent = txt;
}

// ---- render one snapshot -------------------------------------------------
function render(snap) {
  VIDEO_PORT = snap.video_port || VIDEO_PORT;
  renderVehicle(snap.state || {});
  const active = snap.active_camera;
  for (const cam of Object.keys(snap.cameras)) {
    if (!cols[cam]) makeColumn(cam);
    renderCam(cam, snap.cameras[cam], active === cam);
  }
}

function renderVehicle(s) {
  const armed = document.querySelector('#stat-armed .v');
  if (s.armed === undefined) { armed.textContent = '--'; armed.className = 'v'; }
  else { armed.textContent = s.armed ? 'ARMED' : 'DISARMED'; armed.className = 'v ' + (s.armed ? 'armed' : 'disarmed'); }
  document.getElementById('v-mode').textContent = s.mode || '--';
  document.getElementById('v-yaw').textContent = fmt(s.yaw, '°');
  document.getElementById('v-depth').textContent = fmt(s.depth, ' m', 2);
  const batt = document.getElementById('v-batt');
  batt.textContent = fmt(s.batt, ' V', 1);
  batt.classList.toggle('lowbatt', typeof s.batt === 'number' && s.batt > 0 && s.batt < 14.4);
}

function fmt(v, unit, nd) {
  if (v === null || v === undefined) return '--';
  return (nd === undefined ? Math.round(v) : (+v).toFixed(nd)) + (unit || '');
}

function renderCam(cam, c, isLive) {
  const { refs } = cols[cam];
  refs.el.classList.toggle('live', isLive);
  refs.el.classList.toggle('absent', !c.present);
  refs.model.textContent = c.active_model || '--';
  refs.fpsVal.textContent = c.present ? Math.round(c.fps || 0) : '0';
  refs.fps.classList.toggle('slow', c.present && !c.paused && (c.fps || 0) < 6);

  // video state machine: absent -> paused -> live stream
  const key = `${c.present}|${c.paused}`;
  if (key !== cols[cam].lastVideoKey) {
    cols[cam].lastVideoKey = key;
    if (!c.present) { refs.img.classList.remove('on'); refs.img.removeAttribute('src'); showMsg(refs, 'stream not available', 'bad'); }
    else if (c.paused) { refs.img.classList.remove('on'); refs.img.removeAttribute('src'); showMsg(refs, 'paused — resume to view', 'warn'); }
    else { refs.msg.classList.add('hide'); refs.img.classList.add('on'); refs.img.src = videoUrl(cam); }
  }

  // controls reflect authoritative state
  refs.actBtn.classList.toggle('is-live', isLive);
  refs.actBtn.textContent = isLive ? 'Live cam ✓' : 'Make live cam';
  refs.pauseBtn.classList.toggle('is-paused', c.paused === true);
  refs.pauseBtn.textContent = c.paused ? 'Resume' : 'Pause';
  if (document.activeElement !== refs.conf && c.conf !== null) {
    refs.conf.value = c.conf; refs.confVal.textContent = (+c.conf).toFixed(2);
  }
  renderModelSel(refs, c);
  renderClasses(cam, c);
  renderCounts(refs, c);
  renderTable(cam, c, isLive);
}

function renderModelSel(refs, c) {
  const models = c.models || [];
  refs.modelCtl.classList.toggle('hidden', models.length < 2);
  const sig = models.join(',') + '|' + c.active_model;
  if (refs.modelSel._sig === sig) return;
  refs.modelSel._sig = sig;
  refs.modelSel.innerHTML = '';
  for (const m of models) {
    const o = document.createElement('option');
    o.value = m; o.textContent = m; o.selected = (m === c.active_model);
    refs.modelSel.appendChild(o);
  }
}

function renderClasses(cam, c) {
  cols[cam].classes = c.classes || [];
  const box = cols[cam].refs.classChips;
  const sig = (c.classes || []).join(',');
  if (box._sig === sig) return;
  box._sig = sig;
  box.innerHTML = '';
  if (!c.classes || !c.classes.length) { box.innerHTML = '<span class="muted">all</span>'; return; }
  for (const cls of c.classes) {
    const chip = document.createElement('span');
    chip.className = 'chip';
    chip.innerHTML = `${esc(cls)} <span class="x" title="remove">✕</span>`;
    chip.querySelector('.x').onclick = () => {
      const rest = c.classes.filter((x) => x !== cls);
      post(`/api/detector/${cam}/param`, { name: 'classes', value: rest.join(',') });
    };
    box.appendChild(chip);
  }
}

function renderCounts(refs, c) {
  const counts = c.counts || {};
  const expected = c.classes || [];
  const box = refs.counts;
  box.innerHTML = '';
  const names = new Set([...Object.keys(counts), ...expected]);
  if (!names.size) { box.innerHTML = '<span class="muted">—</span>'; return; }
  for (const name of names) {
    const info = counts[name];
    const chip = document.createElement('span');
    if (info) {
      chip.className = 'chip count';
      chip.innerHTML = `${esc(name)} <b>×${info.n}</b> <span class="muted">${(info.best).toFixed(2)}</span>`;
    } else {
      chip.className = 'chip count stale';
      chip.innerHTML = `${esc(name)} <span class="muted">(not seen)</span>`;
    }
    box.appendChild(chip);
  }
}

function renderTable(cam, c, isLive) {
  const body = cols[cam].refs.body;
  const dets = c.dets || [];
  const refs = cols[cam].refs;
  if (!dets.length) { body.innerHTML = '<tr class="empty"><td colspan="7">no detections</td></tr>'; refs.ohx.textContent = 'dx --'; refs.ohy.textContent = 'dy --'; refs.ohx.classList.remove('hot'); refs.ohy.classList.remove('hot'); return; }
  // strongest detection drives the offset HUD
  const top = dets[0];
  refs.ohx.textContent = 'dx ' + (top.dx ?? '--');
  refs.ohy.textContent = 'dy ' + (top.dy ?? '--');
  refs.ohx.classList.toggle('hot', typeof top.dx === 'number' && Math.abs(top.dx) > 40);
  refs.ohy.classList.toggle('hot', typeof top.dy === 'number' && Math.abs(top.dy) > 40);

  body.innerHTML = '';
  for (const d of dets) {
    const tr = document.createElement('tr');
    const confCls = d.conf >= 0.6 ? 'conf-hi' : 'conf-lo';
    tr.innerHTML =
      `<td class="cls">${esc(d.cls)}</td>` +
      `<td class="${confCls}">${d.conf.toFixed(2)}</td>` +
      `<td>${d.dx ?? '·'}</td><td>${d.dy ?? '·'}</td>` +
      `<td>${d.fill ?? '·'}</td><td>${d.vis ?? '·'}</td><td>${esc(d.id) || '·'}</td>`;
    tr.onclick = () => copySnippet(cam, d);
    body.appendChild(tr);
  }
}

// ---- DSL author helper ---------------------------------------------------
// Clicking a detection row copies a tuned mission block for that class/camera:
// switch camera, set conf, centre it (align), then the two most common next
// steps as commented options -- drive to a standoff fill (move) and a
// distance bracket. All verbs exist in the DSL; the operator uncomments+tunes.
function copySnippet(cam, d) {
  const conf = (typeof d.conf === 'number') ? d.conf.toFixed(2) : '0.50';
  const lines = [
    `duburi.use_camera('${cam}')`,
    `duburi.set_conf(${conf})                       # per-target detection threshold`,
    `duburi.vision.align('${d.cls}', lat=0, yaw=0, err=30)   # centre it (live dx=${d.dx ?? '?'} dy=${d.dy ?? '?'} px, fill=${d.fill ?? '?'}%)`,
  ];
  if (typeof d.fill === 'number') {
    lines.push(`# duburi.vision.move('${d.cls}', fwd=${Math.min(90, d.fill + 20)}, mode='area')   # drive to standoff (now ${d.fill}%)`);
  }
  lines.push(`# duburi.calc_distance('start'); duburi.move_forward(duration=2, gain=40); m = duburi.calc_distance('stop')`);
  const snippet = lines.join('\n');
  const done = () => toast(`copied ${d.cls} mission block`);
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(snippet).then(done, () => fallbackCopy(snippet, done));
  } else { fallbackCopy(snippet, done); }
}

function fallbackCopy(text, done) {
  const ta = document.createElement('textarea');
  ta.value = text; ta.style.position = 'fixed'; ta.style.opacity = '0';
  document.body.appendChild(ta); ta.select();
  try { document.execCommand('copy'); done(); } catch (e) { toast('copy failed', true); }
  document.body.removeChild(ta);
}

// ---- SSE wiring ----------------------------------------------------------
function connect() {
  const dot = document.getElementById('conn-dot');
  const txt = document.getElementById('conn-txt');
  const es = new EventSource('/events');
  es.onopen = () => { dot.className = 'dot live'; txt.textContent = 'connected'; };
  es.onmessage = (ev) => { try { render(JSON.parse(ev.data)); } catch (e) { /* skip bad frame */ } };
  es.onerror = () => { dot.className = 'dot dead'; txt.textContent = 'reconnecting…'; };
}
connect();
