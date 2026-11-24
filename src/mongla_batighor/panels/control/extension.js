// The Control Center: a browser's one way to drive the vehicle (helm.py serves it).
//
// Plain DOM, no build: serve.py zips this file into a .foxe and puts
// `const VERBS = {...}` above it, read from mongla_control/commands.py and
// the Move action, `TUNABLE` and `CAMERAS` from seat.py, and the shared
// helpers of panels/common.js, so the panel keeps no list of its own. Its
// look is theme/batighor.css.
//
// What it does, in the owner's rules (bead 102.25.3): a typed name, kept in
// this browser; Request / Give / Release; a heartbeat each second while this
// browser holds the helm; STOP, SURFACE and DISARM always one press for
// anyone; ARM only through two ticks; any verb with its own fields; start and
// abort a named mission. Every answer the helm gives is shown as it came.

const SERVICE = "/mongla/batighor/helm";
const STATE = "/mongla/batighor/helm_state";
// One latched message holding the last 200 lines: a bridge hands a new viewer one message, so
// the history cannot travel as a stream.
const LOG = "/mongla/batighor/log_tail";
const ROSOUT = "/rosout";
const VEHICLE = "/mongla/state";
const ANYONE = ["stop", "surface", "disarm"];
// Under each key, what it does to the vehicle: three red keys alike in a
// hurry are a guess. Shortened from each verb's help in mongla_control/
// commands.py; stop is the board's brake, which older firmware coasts.
const DOES = { stop: "brake", surface: "brake, then surface", disarm: "motors off" };

// The heartbeat belongs to this browser, not to one panel. Opening another
// view unmounts the Control Center; a beat kept in the panel stopped with it,
// the pilot lost the helm 3 s later and the helm stopped the verb they had
// sent (seen in the desktop app, 2026-11-21). Kept here it lives as long as
// the page: a closed tab, a dead laptop or a dropped link still end it. It
// calls through whichever Batighor panel is mounted (common.js callAny);
// the helm's own answer ends it.
const BEAT = { who: "", pilot: false };
function beat() {
  if (!BEAT.pilot || !BEAT.who) return;
  callAny(SERVICE, { who: BEAT.who, op: "heartbeat", cmd: "", fields_json: "{}", mission: "", confirms: [] })
    .then((res) => { if (!res || res.message !== "pilot") BEAT.pilot = false; })
    .catch(() => { /* a missed beat is the helm's to judge */ });
}
setInterval(beat, 1000);

// Two panels from one: the Control Center drives the vehicle (the keys, the
// helm, a verb, a mission, the log), and Detectors chooses the live camera
// and tunes the detectors. As one panel it needed 1042 px and showed a
// third of it, ARM and SEND below the fold (Cockpit, 2026-11-22).
// Keys: the emergency keys alone, tall, for the ScreenPad: the touchscreen
// under the operator's hands, where a key is a thumb's reach away.
const PARTS = {
  ControlCenter: ["safe", "helm", "command", "mission", "log"],
  Detectors: ["detectors"],
  Keys: ["safe"],
  // The main screen's half when the keys are on the screen under it (Vitals).
  Command: ["helm", "command", "mission", "log"],
  // The helm's own record of who asked for what, beside every node's log.
  HelmLog: ["log"],
};

const TITLE = { ControlCenter: "Control Center", Detectors: "Detectors", Keys: "Keys", Command: "Command",
  HelmLog: "Helm log" };

function initPanel(context, kind = "ControlCenter") {
  const parts = PARTS[kind];
  if (typeof context.setDefaultPanelTitle === "function") context.setDefaultPanelTitle(TITLE[kind]);
  const box = mount(context, kind === "Keys" ? "pad" : undefined);

  let helm = {};
  let vehicle = null, vehicleAt = 0;
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

  const msg = el("div", { class: "msg" });
  function say(text, bad) {
    msg.textContent = text;
    msg.className = "msg" + (bad ? " bad" : "");
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
  const pilotLine = el("div", { class: "row" });
  const bRequest = el("button", { class: "go", onclick: () => call("request") }, "Request");
  const bGive = el("button", { onclick: () => call("give") }, "Give");
  const bRelease = el("button", { onclick: () => call("release") }, "Release");

  // -- one press for anyone -------------------------------------------
  const safe = ANYONE.map((v) =>
    el("button", { class: "danger", onclick: () => call("command", { cmd: v }) },
      el("b", {}, v), el("small", {}, DOES[v])));

  // -- arm, behind two ticks ------------------------------------------
  const tProps = el("input", { type: "checkbox" });
  const tKill = el("input", { type: "checkbox" });
  const armGo = el("button", { class: "danger" }, "Arm");
  const armDlg = el(
    "dialog",
    {},
    el("p", {}, "Before arming"),
    el("div", { class: "kv" },
      tProps, el("label", {}, "propellers clear"),
      tKill, el("label", {}, "a person on the kill switch")),
    el("div", { class: "row", style: "margin-top:12px" }, armGo,
      el("button", { onclick: () => armDlg.close() }, "Cancel")),
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
  const fields = el("div", { class: "kv" });
  const help = el("div", { class: "note" });
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
    class: "go",
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
  const bStart = el("button", { class: "go", onclick: () => call("mission", { mission: mPick.value }) }, "Start");
  const bAbort = el("button", { onclick: () => call("abort") }, "Abort");

  // -- the detectors: the live camera (pilot) and their settings ------
  // (anyone named, not during a mission). Built once: redrawing the inputs
  // each frame would wipe what an operator is typing.
  const camButtons = CAMERAS.map((c) =>
    el("button", { onclick: () => call("camera", { cmd: c }) }, c));
  // What a setting left at the detector's declared default does: a bare "" or
  // 0 reads as nothing set, when each one means something (detector_setup.py).
  const UNSET = {
    active_model: ["", "model_path, as launched"],
    classes: ["", "every class the model emits"],
    nms_iou: [0, "as compiled"],
  };
  const now = {};            // "camera.setting" -> the cell showing its value
  const setButtons = [];
  const modelLists = {};     // camera -> the datalist of its registry keys
  const tuneGrid = el("div", { class: "kv" });
  for (const cam of CAMERAS) {
    for (const [s, kind] of Object.entries(TUNABLE)) {
      const key = `${cam}.${s}`;
      const input = el("input", { size: "10", type: kind === "float" && !UNSET[s] ? "number" : "text", step: "any",
        placeholder: UNSET[s] ? "value or default" : "value" });
      const row = el("div", { class: "row tune" });
      if (s === "active_model") {
        // The detector's registry keys; typing "a,b" for two models still works.
        // In the row, not the grid: a grid child shifts the odd/even label colours.
        const list = el("datalist", { id: `bh-models-${cam}` });
        input.setAttribute("list", list.id);
        modelLists[cam] = list;
        row.append(list);
      }
      const set = el("button", {
        onclick: () => {
          if (input.value === "") return undefined;
          // "default" puts a setting back to what the detector declares.
          const back = input.value.trim() === "default" && UNSET[s];
          const value = back ? UNSET[s][0] : kind === "float" ? Number(input.value) : input.value;
          return call("tune", { cmd: key, fields_json: JSON.stringify(value) });
        },
      }, "Set");
      setButtons.push(set);
      now[key] = el("span", { class: "live" });
      row.prepend(now[key]);
      row.append(input, set);
      tuneGrid.append(el("label", {}, key), row);
    }
  }
  const tuneNote = el("span");

  const vehLine = el("div", { class: "row" });
  const log = el("div", { class: "log" });

  // First and pinned: in a small window the helm above it pushed STOP below
  // the fold, and a key that needs a scroll is not one press.
  const has = (p) => parts.includes(p);
  if (has("safe")) {
    const cSafe = card(box, "anyone, one press", null, "pinned");
    cSafe.append(el("div", { class: "keys" }, ...safe), vehLine);
  }
  if (has("helm")) {
    const cHelm = card(box, "helm");
    cHelm.append(el("div", { class: "row" }, name, bRequest, bGive, bRelease), el("div", { class: "row" }, pilotLine));
  } else {
    // Settings take a name too: the same one, kept in this browser.
    const cWho = card(box, "who");
    cWho.append(el("div", { class: "row" }, name, pilotLine));
  }
  if (has("command")) {
    const cVerb = card(box, "command", "pilot only");
    cVerb.append(el("div", { class: "row" }, bArm, pick, bSend), help, fields);
  }
  if (has("mission")) {
    const cMission = card(box, "mission");
    cMission.append(el("div", { class: "row" }, mPick, bStart, bAbort));
  }
  if (has("detectors")) {
    const cDet = card(box, "detectors", tuneNote);
    cDet.append(el("div", { class: "row" }, el("span", { class: "note" }, "live camera"), ...camButtons), tuneGrid);
  }
  // Every answer the helm gives, wherever the press was made.
  box.append(msg);
  if (has("log")) {
    const cLog = card(box, "log");
    cLog.className += " fill";
    cLog.append(log);
  }
  box.append(armDlg);
  drawFields();

  // -- drawing --------------------------------------------------------
  function draw() {
    const pilot = helm.pilot || "";
    const mine = pilot !== "" && pilot === me();
    BEAT.who = me();
    BEAT.pilot = mine;
    // replaceChildren writes a null as the text "null": filter, as el() does.
    pilotLine.replaceChildren(...[
      el("span", { class: mine ? "good" : pilot ? "live" : "stale" },
        pilot ? (mine ? `${pilot} (you)` : pilot) : "helm free"),
      helm.asker && el("span", { class: "warn" },
        `${helm.asker} is asking${helm.answer_by != null ? `, ${Math.max(0, helm.answer_by).toFixed(0)} s to answer` : ""}`),
      helm.mission && el("span", { class: "live" }, `mission ${helm.mission} running`),
    ].filter(Boolean));
    const named = me() !== "";
    // The heartbeat carries this name: changed while flying, the helm would
    // stop hearing the pilot and stop their verb 3 s later. Release first.
    name.disabled = mine;
    name.title = mine ? "release the helm to change the name" : "";
    bRequest.disabled = !named || mine;
    bGive.disabled = !mine || !helm.asker;
    bRelease.disabled = !mine;
    bArm.disabled = bSend.disabled = bStart.disabled = !mine;
    bAbort.disabled = !helm.mission;            // like stop: anyone may end a mission
    for (const b of camButtons) {
      b.disabled = !mine;
      b.classList.toggle("on", helm.camera === b.textContent);
    }
    const tuning = helm.tuning || {};
    for (const [key, cell] of Object.entries(now)) {
      const [cam, s] = key.split(".");
      const v = (tuning[cam] || {})[s];
      const unset = UNSET[s] && v === UNSET[s][0];
      cell.textContent = !tuning[cam] || v === null || v === undefined ? "--" : unset ? UNSET[s][1] : v === "" ? '""' : String(v);
      cell.className = unset ? "note" : "live";
      cell.title = cell.textContent;   // the whole value, where the column cuts it
    }
    for (const [cam, list] of Object.entries(modelLists)) {
      // `models` is "key=stem,key2=stem2" or bare stems; the key is what
      // active_model takes (detector_params._parse_models_param).
      const keys = String((tuning[cam] || {}).models || "").split(",")
        .map((p) => p.split("=")[0].trim()).filter(Boolean);
      if (list.dataset.keys !== keys.join(",")) {
        list.dataset.keys = keys.join(",");
        list.replaceChildren(...keys.map((k) => el("option", { value: k })));
      }
    }
    for (const b of setButtons) b.disabled = !named || Boolean(helm.mission);
    tuneNote.textContent = helm.mission
      ? `settings closed while ${helm.mission} runs`
      : "settings: anyone named, no mission";
    const ms = helm.missions || [];
    if (mPick.options.length !== ms.length) {
      mPick.replaceChildren(...ms.map((m) => el("option", { value: m }, m)));
    }
    // The state is sent on change and at least twice a second: older, it is
    // the vehicle as it was, and says so.
    const old = vehicle && (Date.now() - vehicleAt) / 1000 > 2;
    vehLine.replaceChildren(...(vehicle
      ? [el("span", { class: old ? "warn" : vehicle.armed ? "armed" : "ok" },
          (vehicle.armed ? "ARMED" : "disarmed") + (old ? `, ${num((Date.now() - vehicleAt) / 1000, 0)} s old` : "")),
        el("span", {}, vehicle.mode || "--"),
        // Depth below the surface, as the Status strip says it (depth_m is altitude).
        el("span", {}, `${num(Number.isFinite(vehicle.depth_m) ? -vehicle.depth_m : NaN, 2, " m")} deep`),
        el("span", {}, `heading ${num(vehicle.yaw_deg, 0)}`),
        el("span", {}, `battery ${num(vehicle.battery_voltage, 1, " V")}`)]
      : [el("span", { class: "stale" }, "no /mongla/state yet")]));
    const text = lines.length ? lines.slice(-200).join("\n")
      : "nothing yet: every press at the helm is recorded here, and every node's warnings and errors arrive as they happen";
    if (log.textContent !== text) {
      const stuck = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
      log.textContent = text;
      if (stuck) log.scrollTop = log.scrollHeight;
    }
  }

  // Two records, one card, in time order: the helm's (who asked for what,
  // what it answered) and every node's warnings and errors from /rosout, the
  // board's refusals and the stack's faults that no press caused.
  const helmLog = [], nodeLog = [];
  // rcl_interfaces/Log: WARN is 30, ERROR 40, FATAL 50.
  const LEVEL_NAME = { 30: "WARNING", 40: "ERROR", 50: "FATAL" };
  function nodeLine(m) {
    const t = (m.stamp?.sec ?? 0) + (m.stamp?.nanosec ?? 0) / 1e9;
    const head = `${LEVEL_NAME[m.level] || m.level} ${m.name}`;
    return { t, key: `${head}: ${m.msg}`,
      text: `${clock(t)}  ${head}: ${m.msg}` };
  }

  function merge() {
    lines.length = 0;
    for (const e of [...helmLog, ...nodeLog].sort((a, b) => a.t - b.t)) lines.push(e.text);
  }

  context.subscribe([{ topic: STATE }, { topic: LOG }, { topic: VEHICLE }, { topic: ROSOUT }]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    for (const ev of renderState.currentFrame || []) {
      if (ev.topic === STATE) {
        try { helm = JSON.parse(ev.message.data); } catch { /* keep the last */ }
      } else if (ev.topic === LOG) {
        try {
          const fresh = JSON.parse(ev.message.data).map(logLine);
          helmLog.splice(0, helmLog.length, ...fresh);
          merge();
        } catch { /* keep the last */ }
      } else if (ev.topic === ROSOUT) {
        if (ev.message.level >= 30) {
          foldNodeLog(nodeLog, nodeLine(ev.message));
          if (nodeLog.length > 200) nodeLog.splice(0, nodeLog.length - 200);
          merge();
        }
      } else if (ev.topic === VEHICLE) {
        vehicle = ev.message; vehicleAt = Date.now();
      }
    }
    draw();
    done();
  };

  if (!canCall) say("this connection cannot call services: connect through foxglove_bridge", true);
  draw();
}

// One helm log record as a line. Exported for the test.
const clock = (t) => new Date(t * 1000).toLocaleTimeString([], { hour12: false });

function logLine(raw) {
  let e;
  try { e = JSON.parse(raw); } catch { return { t: 0, text: String(raw) }; }
  let text = e.event;
  // A verb's end carries its outcome: a failed disarm must not read like one that took.
  if (text !== undefined && e.ok === false) text += `  FAILED${e.message ? `: ${e.message}` : ""}`;
  else if (text !== undefined && e.message) text += `: ${e.message}`;
  if (text === undefined) {
    text = [e.op, e.cmd, e.mission].filter(Boolean).join(" ");
    text += e.ok ? "  ok" : `  REFUSED: ${e.message || ""}`;
  }
  const t = e.t || 0;
  return { t, text: e.operator ? `${clock(t)}  ${e.operator}: ${text}` : `${clock(t)}  ${text}` };
}

// A node that says the same thing every second (the lock's "camera looks
// degraded", Kutum 2026-11-23) filled the whole card. A repeat, digits aside,
// replaces its earlier line: the newest text, with how many times it came.
function foldNodeLog(list, e) {
  // Measurements fold (1.27, 7/8, 1043); a lone small whole number is an id
  // (thruster 3, channel 12) and stays, so two failing thrusters stay two lines.
  const key = String(e.key).replace(/\d+(?:[./]\d+)+|\d{3,}/g, "#");
  const i = list.findIndex((x) => x.key === key);
  const n = i < 0 ? 1 : list[i].n + 1;
  if (i >= 0) list.splice(i, 1);
  list.push({ ...e, key, n, text: n > 1 ? `${e.text}  (x${n})` : e.text });
}

module.exports = {
  foldNodeLog, logLine,
  activate(extensionContext) {
    for (const name of Object.keys(PARTS)) {
      extensionContext.registerPanel({ name, initPanel: (context) => initPanel(context, name) });
    }
  },
};
