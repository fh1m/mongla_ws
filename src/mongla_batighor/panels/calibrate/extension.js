// The calibrate panel: a running calibration's words and buttons (calibrate_node.py).
//
// Plain DOM, no build, zipped by serve.py like the Control Center. The picture
// and what is drawn over it (corners, the cell to fill, the worst point) are
// Lichtblick's Image panel, from the camera's own stream and the overlay; this
// panel is the rest: where the capture is, how good the fit is, and the
// buttons. Every button is a helm `calibrate` op, so the helm decides who may
// press it (any named operator, never during a mission) and logs it.

const SERVICE = "/mongla/batighor/helm";
const STATE = "/mongla/vision/calibration/state";
const STEPS = ["skip", "undo", "reset", "solve"];

function initPanel(context) {
  const box = mount(context);
  const canCall = typeof context.callService === "function";
  let st = null;

  const name = el("input", { placeholder: "your name", size: "12" });
  name.value = readName();
  name.addEventListener("input", () => draw());
  const me = () => name.value.trim();

  const msg = el("div", { class: "msg" });
  function say(text, bad) {
    msg.textContent = text;
    msg.className = bad ? "msg bad" : "msg";
  }

  async function call(op, fields) {
    if (!canCall) { say("this connection cannot call services", true); return; }
    try {
      const res = await context.callService(SERVICE, {
        who: me(), op: "calibrate", cmd: op, fields_json: JSON.stringify(fields || {}),
        mission: "", confirms: [],
      });
      say(res.message || (res.ok ? "ok" : "refused"), !res.ok);
    } catch (err) {
      say(`the helm did not answer: ${err && err.message ? err.message : err}`, true);
    }
  }

  // -- where the capture is ------------------------------------------
  const step = el("div", { class: "big", style: "font-size:16px" });
  const now = el("div");
  const fit = el("div", { class: "kv" });
  const stepButtons = STEPS.map((s) => el("button", { onclick: () => call(s) }, s));

  // -- which camera, which water, which board -------------------------
  // Built once: redrawing them each frame would wipe what is being typed.
  const applies = el("input", { size: "12", placeholder: "pi_downward" });
  const medium = el("select", {}, el("option", { value: "air" }, "air"), el("option", { value: "water" }, "water"));
  const grid = el("input", { size: "5", placeholder: "8x6" });
  const square = el("input", { size: "6", type: "number", step: "any", placeholder: "0.025" });
  const bConfigure = el("button", {
    class: "go",
    onclick: () => {
      const out = { medium: medium.value };
      if (applies.value.trim()) out.applies_to = applies.value.trim();
      if (grid.value.trim()) out.grid = grid.value.trim();
      if (square.value !== "") out.square = Number(square.value);
      return call("configure", out);
    },
  }, "Start this capture");

  const solveOut = el("pre");
  const cameras = el("div");
  const library = el("div");
  const applyButtons = [];

  card(box, "operator").append(el("div", { class: "row" }, name, msg));
  card(box, "capture").append(step, now, el("div", { class: "row" }, ...stepButtons));
  card(box, "fit").append(fit, solveOut);
  card(box, "new capture").append(el("div", { class: "row" },
    el("label", {}, "camera"), applies, medium, el("label", {}, "board"), grid,
    el("label", {}, "square m"), square, bConfigure));
  card(box, "cameras on this vehicle").append(cameras);
  card(box, "saved calibrations").append(library);

  function draw() {
    const named = me() !== "";
    for (const b of [...stepButtons, bConfigure, ...applyButtons]) b.disabled = !named || !st;
    if (!st) {
      step.textContent = "no calibration running";
      now.textContent = "ros2 launch mongla_batighor calibrate.launch.py device_path:=<camera>";
      return;
    }
    step.textContent = st.done
      ? `all ${st.n} poses captured: solve`
      : `${st.i + 1}/${st.n}  board in the ${st.want_cell}, ${st.want_tilt}`;
    now.replaceChildren(
      `${st.applies_to} in ${st.medium}, board ${st.grid} at ${st.square} m   `,
      st.cell ? `seen: ${st.cell}, ${st.band}   hold ${Math.round(100 * st.hold)} %   ` : "no board seen   ",
      el("span", { class: st.sharp != null && st.sharp > st.sharp_max ? "bad" : "" },
        `blur ${num(st.sharp, 2, " px")}`),
      st.msg ? `   ${st.msg}` : "",
      st.err ? el("span", { class: "bad" }, `   ${st.err}`) : "",
    );
    // Each cell its own element: adjacent text nodes merge into ONE grid item,
    // which ran every label and value into a single line (seen on Kutum).
    const cell = (x, cls) => (x instanceof Node ? x : el("span", cls ? { class: cls } : {}, String(x)));
    const ereClass = st.ere == null ? "" : st.ere <= st.ere_bar ? "good" : "bad";
    const pairs = [
      ["Max ERE", cell(`${num(st.ere, 2, " px")} (bar ${num(st.ere_bar, 2, " px")})`, ereClass)],
      ["after the next pose", num(st.ere_pred, 2, " px")],
      ["next pose wanted", st.suggest || "--"],
      ["fx", `${num(st.fx, 1)} +/- ${num(st.fx_sd, 1)}`],
      ["HFOV air / water", `${num(st.hfov_air, 1, " deg")} / ${num(st.hfov_water, 1, " deg")}`],
      ["boards", st.n_boards ?? "--"],
      ["solve", st.solve || "--"],
    ];
    if (st.assess_err) pairs.push(["fit", cell(st.assess_err, "bad")]);
    fit.replaceChildren(...pairs.flatMap(([k, v]) => [cell(k), cell(v)]));
    solveOut.textContent = st.solve_out || "";
    // Which camera may be trusted: "confirmed" is a calibration taken on this
    // very unit (its USB id), "unverified" one that plausibly belongs to it.
    const match = { confirmed: "good", unverified: "", null: "bad" };
    const cams = st.cameras || [];
    cameras.replaceChildren(cams.length
      ? el("table", {}, el("tr", {}, ...["camera", "device", "calibration", ""].map((h) => el("th", {}, h))),
        ...cams.map((c) => el("tr", {},
          el("td", {}, c.card || "--"), el("td", {}, c.device || "--"),
          el("td", {}, c.calibration ? c.calibration.file : "none"),
          el("td", { class: match[c.match] ?? "" }, c.match || "uncalibrated"))))
      : "no USB camera found (or still looking)");
    const rows = (st.library || []).map((c) => {
      const b = el("button", { onclick: () => call("apply", { file: c.file }) }, "apply");
      b.disabled = me() === "";
      return [el("tr", {},
        el("td", {}, c.file), el("td", {}, c.medium || ""), el("td", {}, `${c.width}x${c.height}`),
        el("td", {}, num(c.fx, 1)), el("td", {}, num(c.max_ere_px, 2)), el("td", {}, b)), b];
    });
    applyButtons.length = 0;
    applyButtons.push(...rows.map((r) => r[1]));
    library.replaceChildren(rows.length
      ? el("table", {}, el("tr", {}, ...["calibration", "medium", "size", "fx", "Max ERE", ""].map((h) => el("th", {}, h))),
        ...rows.map((r) => r[0]))
      : "no calibrations on this vehicle yet");
  }

  context.subscribe([{ topic: STATE }]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    for (const ev of renderState.currentFrame || []) {
      try { st = JSON.parse(ev.message.data); } catch { /* keep the last */ }
    }
    draw();
    done();
  };
  if (!canCall) say("this connection cannot call services: connect through foxglove_bridge", true);
  draw();
}

module.exports = {
  activate(extensionContext) {
    extensionContext.registerPanel({ name: "Calibrate", initPanel });
  },
};
