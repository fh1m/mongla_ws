// The helm panel: a browser's one way to drive the vehicle (helm.py serves it).
//
// Plain DOM, no build: serve.py zips this file into a .foxe and puts
// `const VERBS = {...}` above it, read from mongla_control/commands.py and
// the Move action, and `TUNABLE` and `CAMERAS` from seat.py, so the panel
// keeps no list of its own.
//
// What it does, in the owner's rules (bead 102.25.3): a typed name, kept in
// this browser; Request / Give / Release; a heartbeat each second while this
// browser holds the helm; STOP, SURFACE and DISARM always one press for
// anyone; ARM only through two ticks; any verb with its own fields; start and
// abort a named mission. Every answer the helm gives is shown as it came.

const SERVICE = "/mongla/batighor/helm";
const STATE = "/mongla/batighor/helm_state";
const LOG = "/mongla/batighor/log";
const VEHICLE = "/mongla/state";
const NAME_KEY = "mongla.batighor.operator";
const ANYONE = ["stop", "surface", "disarm"];

const CSS = `
.bh{font:13px/1.4 system-ui,sans-serif;padding:8px;display:flex;flex-direction:column;gap:8px;height:100%;box-sizing:border-box;overflow:auto}
.bh-row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.bh button{font:inherit;padding:4px 10px;border-radius:4px;border:1px solid #888;background:transparent;color:inherit;cursor:pointer}
.bh button:disabled{opacity:.4;cursor:default}
.bh .bh-safe{font-weight:700;letter-spacing:.04em;padding:8px 16px;border-color:#d33;color:#fff;background:#b22}
.bh input,.bh select{font:inherit;padding:3px 6px;border-radius:4px;border:1px solid #888;background:transparent;color:inherit}
.bh .bh-who{font-weight:700}
.bh .bh-you{color:#3c3}
.bh .bh-msg{min-height:1.4em}
.bh .bh-bad{color:#e55}
.bh .bh-fields{display:grid;grid-template-columns:max-content 1fr;gap:4px 8px;align-items:center}
.bh .bh-log{font:12px/1.35 ui-monospace,monospace;white-space:pre;overflow:auto;flex:1;min-height:6em;border-top:1px solid #8884;padding-top:4px}
.bh dialog{color:inherit;background:Canvas;border:1px solid #888;border-radius:6px}
`;

function el(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (k === "class") e.className = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) e.append(k);
  return e;
}

function readName() {
  try { return localStorage.getItem(NAME_KEY) || ""; } catch { return ""; }
}
function keepName(n) {
  try { localStorage.setItem(NAME_KEY, n); } catch { /* private window */ }
}

function initPanel(context) {
  const root = context.panelElement;
  root.append(el("style", {}, CSS));
  const box = el("div", { class: "bh" });
  root.append(box);

  let helm = {};
  let vehicle = null;
  const lines = [];
  const canCall = typeof context.callService === "function";

  // -- who am I -------------------------------------------------------
  const name = el("input", { placeholder: "your name", size: "12" });
  name.value = readName();
  name.addEventListener("change", () => keepName(name.value.trim()));
  // Typing is not a ROS frame, so it redraws itself: without this, Request
  // stayed disabled after a name was typed until some message arrived.
  name.addEventListener("input", () => draw());
  const me = () => name.value.trim();

  const msg = el("div", { class: "bh-msg" });
  function say(text, bad) {
    msg.textContent = text;
    msg.className = "bh-msg" + (bad ? " bh-bad" : "");
  }

  async function call(op, extra) {
    if (!canCall) {
      say("this connection cannot call services", true);
      return { ok: false };
    }
    const req = Object.assign(
      { who: me(), op, cmd: "", fields_json: "{}", mission: "", confirms: [] },
      extra || {},
    );
    try {
      const res = await context.callService(SERVICE, req);
      if (op !== "heartbeat") say(res.message || (res.ok ? "ok" : "refused"), !res.ok);
      return res;
    } catch (err) {
      say(`the helm did not answer: ${err && err.message ? err.message : err}`, true);
      return { ok: false };
    }
  }

  // -- the helm -------------------------------------------------------
  const pilotLine = el("div");
  const bRequest = el("button", { onclick: () => call("request") }, "Request");
  const bGive = el("button", { onclick: () => call("give") }, "Give");
  const bRelease = el("button", { onclick: () => call("release") }, "Release");

  // -- one press for anyone -------------------------------------------
  const safe = ANYONE.map((v) =>
    el("button", { class: "bh-safe", onclick: () => call("command", { cmd: v }) }, v.toUpperCase()),
  );

  // -- arm, behind two ticks ------------------------------------------
  const tProps = el("input", { type: "checkbox" });
  const tKill = el("input", { type: "checkbox" });
  const armGo = el("button", {}, "Arm");
  const armDlg = el(
    "dialog",
    {},
    el("p", {}, "Before arming:"),
    el("label", {}, tProps, " propellers clear"),
    el("br"),
    el("label", {}, tKill, " a person on the kill switch"),
    el("div", { class: "bh-row" }, armGo, el("button", { onclick: () => armDlg.close() }, "Cancel")),
  );
  const syncArm = () => { armGo.disabled = !(tProps.checked && tKill.checked); };
  tProps.addEventListener("change", syncArm);
  tKill.addEventListener("change", syncArm);
  armGo.addEventListener("click", async () => {
    const confirms = [];
    if (tProps.checked) confirms.push("propellers_clear");
    if (tKill.checked) confirms.push("kill_switch_manned");
    armDlg.close();
    await call("command", { cmd: "arm", confirms });
  });
  const bArm = el("button", {
    onclick: () => { tProps.checked = false; tKill.checked = false; syncArm(); armDlg.showModal(); },
  }, "Arm...");

  // -- any verb, with its own fields ----------------------------------
  const verbs = Object.keys(VERBS).filter((v) => v !== "arm" && !ANYONE.includes(v)).sort();
  const pick = el("select", {}, ...verbs.map((v) => el("option", { value: v }, v)));
  const fields = el("div", { class: "bh-fields" });
  const help = el("div", { class: "bh-msg" });
  function drawFields() {
    const v = VERBS[pick.value];
    fields.replaceChildren();
    help.textContent = v.help;
    for (const [f, kind] of Object.entries(v.fields)) {
      const d = v.defaults[f];
      const input = kind === "boolean"
        ? el("input", { type: "checkbox", "data-f": f, "data-k": kind })
        : el("input", {
          "data-f": f, "data-k": kind, size: "8",
          type: kind === "float" || kind === "double" ? "number" : "text",
          step: "any", placeholder: d === undefined ? "" : String(d),
        });
      if (kind === "boolean" && d === true) input.checked = true;
      fields.append(el("label", {}, f), input);
    }
  }
  pick.addEventListener("change", drawFields);
  const bSend = el("button", {
    onclick: () => {
      // Only what the operator filled in is sent; the verb's defaults do the rest.
      const out = {};
      for (const i of fields.querySelectorAll("input")) {
        const k = i.dataset.k;
        if (k === "boolean") { if (i.checked !== (VERBS[pick.value].defaults[i.dataset.f] === true)) out[i.dataset.f] = i.checked; }
        else if (i.value !== "") out[i.dataset.f] = k === "string" ? i.value : Number(i.value);
      }
      return call("command", { cmd: pick.value, fields_json: JSON.stringify(out) });
    },
  }, "Send");

  // -- missions -------------------------------------------------------
  const mPick = el("select");
  const bStart = el("button", { onclick: () => call("mission", { mission: mPick.value }) }, "Start");
  const bAbort = el("button", { onclick: () => call("abort") }, "Abort mission");

  // -- the detectors: the live camera (pilot) and their settings ------
  // (anyone named, not during a mission). Built once: redrawing the inputs
  // each frame would wipe what an operator is typing.
  const camButtons = CAMERAS.map((c) =>
    el("button", { onclick: () => call("camera", { cmd: c }) }, c));
  const now = {};            // "camera.setting" -> the cell showing its value
  const setButtons = [];
  const tuneGrid = el("div", { class: "bh-fields" });
  for (const cam of CAMERAS) {
    for (const [s, kind] of Object.entries(TUNABLE)) {
      const key = `${cam}.${s}`;
      const input = el("input", { size: "10", type: kind === "float" ? "number" : "text", step: "any" });
      const set = el("button", {
        onclick: () => {
          if (input.value === "") return undefined;
          const value = kind === "float" ? Number(input.value) : input.value;
          return call("tune", { cmd: key, fields_json: JSON.stringify(value) });
        },
      }, "Set");
      setButtons.push(set);
      now[key] = el("span");
      tuneGrid.append(el("label", {}, key), el("div", { class: "bh-row" }, now[key], input, set));
    }
  }
  const tuneNote = el("div", { class: "bh-msg" });

  const vehLine = el("div");
  const log = el("div", { class: "bh-log" });

  box.append(
    el("div", { class: "bh-row" }, name, pilotLine),
    el("div", { class: "bh-row" }, bRequest, bGive, bRelease),
    el("div", { class: "bh-row" }, ...safe),
    vehLine,
    msg,
    el("div", { class: "bh-row" }, bArm, pick, bSend),
    help,
    fields,
    el("div", { class: "bh-row" }, "mission", mPick, bStart, bAbort),
    el("div", { class: "bh-row" }, "camera", ...camButtons),
    tuneNote,
    tuneGrid,
    log,
    armDlg,
  );
  drawFields();

  // -- drawing --------------------------------------------------------
  const num = (x, fmt) => (x === undefined || x === null || Number.isNaN(x) ? "--" : fmt(x));
  function draw() {
    const pilot = helm.pilot || "";
    const mine = pilot !== "" && pilot === me();
    pilotLine.replaceChildren(
      "helm: ",
      el("span", { class: "bh-who" + (mine ? " bh-you" : "") }, pilot ? (mine ? `${pilot} (you)` : pilot) : "free"),
      helm.asker ? `   ${helm.asker} is asking${helm.answer_by != null ? `, ${Math.max(0, helm.answer_by).toFixed(0)} s to answer` : ""}` : "",
      helm.mission ? `   mission ${helm.mission} running` : "",
    );
    const named = me() !== "";
    bRequest.disabled = !named || mine;
    bGive.disabled = !mine || !helm.asker;
    bRelease.disabled = !mine;
    bArm.disabled = bSend.disabled = bStart.disabled = !mine;
    bAbort.disabled = !helm.mission;            // like stop: anyone may end a mission
    for (const b of camButtons) {
      b.disabled = !mine;
      b.style.fontWeight = helm.camera === b.textContent ? "700" : "";
    }
    const tuning = helm.tuning || {};
    for (const [key, cell] of Object.entries(now)) {
      const [cam, s] = key.split(".");
      const v = (tuning[cam] || {})[s];
      cell.textContent = tuning[cam] ? (v === "" ? '""' : String(v)) : "--";
    }
    for (const b of setButtons) b.disabled = !named || Boolean(helm.mission);
    tuneNote.textContent = helm.mission
      ? `settings are closed while mission ${helm.mission} runs`
      : "settings: anyone named, while no mission runs";
    const ms = helm.missions || [];
    if (mPick.options.length !== ms.length) {
      mPick.replaceChildren(...ms.map((m) => el("option", { value: m }, m)));
    }
    vehLine.textContent = vehicle
      ? `vehicle ${vehicle.armed ? "ARMED" : "disarmed"}  ${vehicle.mode || "--"}  ` +
        `depth ${num(vehicle.depth_m, (x) => x.toFixed(2) + " m")}  ` +
        `heading ${num(vehicle.yaw_deg, (x) => x.toFixed(0))}  ` +
        `battery ${num(vehicle.battery_voltage, (x) => x.toFixed(1) + " V")}`
      : "vehicle --  no /mongla/state yet";
    log.textContent = lines.slice(-200).join("\n");
    log.scrollTop = log.scrollHeight;
  }

  function logLine(raw) {
    let e;
    try { e = JSON.parse(raw); } catch { lines.push(raw); return; }
    const t = new Date((e.t || 0) * 1000).toLocaleTimeString();
    const who = (e.operator || "").padStart(10);
    let text = e.event;
    if (text === undefined) {
      text = [e.op, e.cmd, e.mission].filter(Boolean).join(" ");
      text += e.ok ? "  ok" : `  REFUSED: ${e.message || ""}`;
    }
    lines.push(`${t} ${who}  ${text}`);
  }

  context.subscribe([{ topic: STATE }, { topic: LOG }, { topic: VEHICLE }]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    for (const ev of renderState.currentFrame || []) {
      if (ev.topic === STATE) {
        try { helm = JSON.parse(ev.message.data); } catch { /* keep the last */ }
      } else if (ev.topic === LOG) {
        logLine(ev.message.data);
      } else if (ev.topic === VEHICLE) {
        vehicle = ev.message;
      }
    }
    draw();
    done();
  };

  // The heartbeat: once a second while this browser holds the helm. A closed
  // tab, a dead laptop or a dropped link all end it, and the helm lets go.
  const beat = setInterval(() => {
    if (helm.pilot && helm.pilot === me()) call("heartbeat");
  }, 1000);

  if (!canCall) say("this connection cannot call services: connect through foxglove_bridge", true);
  draw();
  return () => clearInterval(beat);
}

module.exports = {
  activate(extensionContext) {
    extensionContext.registerPanel({ name: "Helm", initPanel });
  },
};
