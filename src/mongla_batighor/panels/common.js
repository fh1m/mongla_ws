// What every Batighor panel shares; panels.py puts this above each one.
//
// The look is not here: it is theme/batighor.css, which serve.py links into
// the app page, so a panel only names classes (`mb`, `section`, `row`, `kv`,
// `danger`...). This file holds the few helpers the panels used to copy.

const NAME_KEY = "mongla.batighor.operator";   // one operator name per browser

function el(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k.startsWith("on") && typeof v === "function") e.addEventListener(k.slice(2), v);
    else if (k === "class") e.className = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) if (k !== null && k !== undefined && k !== false) e.append(k);
  return e;
}

function readName() {
  try { return localStorage.getItem(NAME_KEY) || ""; } catch { return ""; }
}
function keepName(n) {
  try { localStorage.setItem(NAME_KEY, n); } catch { /* a private window */ }
}

// A missing number is "--", never 0 (CLAUDE.md: absence renders `--`).
const fin = (x) => typeof x === "number" && Number.isFinite(x);
const num = (x, d, unit) => (fin(x) ? x.toFixed(d) + (unit || "") : "--");

// The panel's root, themed. `kind` adds a layout class.
function mount(context, kind) {
  const box = el("div", { class: kind ? `mb ${kind}` : "mb" });
  context.panelElement.append(box);
  return box;
}

// A titled card; returns its body. `aside` sits right of the title.
function card(parent, title, aside, cls) {
  const right = el("span");
  if (aside) right.append(aside);
  const body = el("section", cls ? { class: cls } : {}, el("h3", {}, el("span", {}, title), right));
  parent.append(body);
  body.aside = right;
  return body;
}
