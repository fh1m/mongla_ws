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
const LOG = "/mongla/batighor/log";
const VEHICLE = "/mongla/state";
const ANYONE = ["stop", "surface", "disarm"];

function initPanel(context) {
  if (typeof context.setDefaultPanelTitle === "function") context.setDefaultPanelTitle("Control Center");
  const box = mount(context);

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
    el("button", { class: "danger", onclick: () => call("command", { cmd: v }) }, v));

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
  const now = {};            // "camera.setting" -> the cell showing its value
  const setButtons = [];
  const modelLists = {};     // camera -> the datalist of its registry keys
  const tuneGrid = el("div", { class: "kv" });
  for (const cam of CAMERAS) {
    for (const [s, kind] of Object.entries(TUNABLE)) {
      const key = `${cam}.${s}`;
      const input = el("input", { size: "10", type: kind === "float" ? "number" : "text", step: "any" });
      if (s === "active_model") {
        // The detector's registry keys; typing "a,b" for two models still works.
        const list = el("datalist", { id: `bh-models-${cam}` });
        input.setAttribute("list", list.id);
        modelLists[cam] = list;
        tuneGrid.append(list);
      }
      const set = el("button", {
        onclick: () => {
          if (input.value === "") return undefined;
          const value = kind === "float" ? Number(input.value) : input.value;
          return call("tune", { cmd: key, fields_json: JSON.stringify(value) });
        },
      }, "Set");
      setButtons.push(set);
      now[key] = el("span", { class: "live" });
      tuneGrid.append(el("label", {}, key), el("div", { class: "row" }, now[key], input, set));
    }
  }
  const tuneNote = el("span");

  const vehLine = el("div", { class: "row" });
  const log = el("div", { class: "log" });

  const cHelm = card(box, "helm");
  cHelm.append(el("div", { class: "row" }, name, pilotLine),
    el("div", { class: "row" }, bRequest, bGive, bRelease));
  const cSafe = card(box, "anyone, one press");
  cSafe.append(el("div", { class: "keys" }, ...safe), vehLine, msg);
  const cVerb = card(box, "command", "pilot only");
  cVerb.append(el("div", { class: "row" }, bArm, pick, bSend), help, fields);
  const cMission = card(box, "mission");
  cMission.append(el("div", { class: "row" }, mPick, bStart, bAbort));
  const cDet = card(box, "detectors", tuneNote);
  cDet.append(el("div", { class: "row" }, el("span", { class: "note" }, "live camera"), ...camButtons), tuneGrid);
  const cLog = card(box, "log");
  cLog.style.flex = "1";
  cLog.append(log);
  box.append(armDlg);
  drawFields();

  // -- drawing --------------------------------------------------------
  function draw() {
    const pilot = helm.pilot || "";
    const mine = pilot !== "" && pilot === me();
    // replaceChildren writes a null as the text "null": filter, as el() does.
    pilotLine.replaceChildren(...[
      el("span", { class: mine ? "good" : pilot ? "live" : "stale" },
        pilot ? (mine ? `${pilot} (you)` : pilot) : "helm free"),
      helm.asker && el("span", { class: "warn" },
        `${helm.asker} is asking${helm.answer_by != null ? `, ${Math.max(0, helm.answer_by).toFixed(0)} s to answer` : ""}`),
      helm.mission && el("span", { class: "live" }, `mission ${helm.mission} running`),
    ].filter(Boolean));
    const named = me() !== "";
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
      cell.textContent = tuning[cam] ? (v === "" ? '""' : String(v)) : "--";
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
    vehLine.replaceChildren(...(vehicle
      ? [el("span", { class: vehicle.armed ? "armed" : "ok" }, vehicle.armed ? "ARMED" : "disarmed"),
        el("span", {}, vehicle.mode || "--"),
        el("span", {}, `depth ${num(vehicle.depth_m, 2, " m")}`),
        el("span", {}, `heading ${num(vehicle.yaw_deg, 0)}`),
        el("span", {}, `battery ${num(vehicle.battery_voltage, 1, " V")}`)]
      : [el("span", { class: "stale" }, "no /mongla/state yet")]));
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
    extensionContext.registerPanel({ name: "ControlCenter", initPanel });
  },
};
