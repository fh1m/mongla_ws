# Audit — bugs, design debt, wrong decisions (v0.1)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Date: 2026-08-26. Scope: `duburi_ws/sim` bringup, bridge, scenarios, web lab,
docs vs code. Severity: **P0** blocker / lie, **P1** wrong behavior operators hit,
**P2** design debt, **P3** polish.

Status **doc-fixed** = corrected in docs or CLI usage string this pass.
Status **open** = still in code; fix later.

---

## P0 — documentation / CLI lies

| ID | Finding | Evidence | Impact | Fix | Status |
|----|---------|----------|--------|-----|--------|
| A1 | `duburi_sim` `_usage` says lab on **:8088** | `scripts/duburi_sim` L99 | Operators open wrong port | Change to 28765 / `DUBURI_LAB_PORT` | **doc-fixed** (this pass) |
| A2 | Historical Electron port collision on 8088/18088 | Session history | Lab bind failures | Prefer 28765 + auto-fallback | mitigated in code |

---

## P1 — operator-visible bugs / sharp edges

| ID | Finding | Evidence | Impact | Recommended fix | Status |
|----|---------|----------|--------|-----------------|--------|
| B1 | Record stop used to SIGINT `ros2 run` parent mid-import → empty `record_dir` | `server.py` record start/stop; session logs | No zip / orphan `record_cameras` | Process group + wait `.ready` (done) | mitigated |
| B2 | Zero-frame recordings still write `meta.json` with `counts: 0` and exit 1 | `record_cameras.py` | Lab may show “ok” with empty video | Soft-fail toast + meta on `/api/record/stop` | **mitigated** (UI warns; exit 1 still) |
| B2b | MP4 played as ~1s for 10s wall take (`VideoWriter` at fixed `--fps 20`) | `record_cameras` + meta | Misleading clips | Buffer + encode at `fps_actual` | **fixed** (v0.2) |
| B3 | Dataset zip is **synchronous** on request thread | `download_dataset` builds full zip in memory | Lab UI freezes on large runs | Zip to tempfile + `FileResponse` | **mitigated** (v0.2) |
| B4 | `GET /api/props/list` returns **catalog**, not spawned instances | `props list` / `cli.py` `cmd_list` | Operators think spawn failed when list looks static | `/api/props/instances` + World UI | **mitigated** (instances API) |
| B9 | Cursor/Electron port-forward races lab bind after probe-close | `_pick_lab_port` TOCTOU | Lab fails to listen | Hold listening socket + `uvicorn fd=` | **fixed** (v0.2) |
| B5 | `duburi_sim stop` does **not** kill `lab_server` / all `record_cameras` | `_stop_all` patterns | Orphans after “stop” | Extend patterns | **fixed** (v0.2) |
| B6 | `duburi_sim lab` starts `prop_manager` **without** `-p world:=` | `duburi_sim` lab branch | Defaults to qual; wrong after course change | Pin `world:=` + `/tmp/duburi_lab_active_course.txt` | **fixed** (v0.2) |
| B7 | Turbidity UI default **0.6** then snaps to server **0.45** | `App.jsx` vs yaml | Confusing slider jump | Default UI to 0.45 | **fixed** (v0.2) |
| B8 | Vite proxy hardcodes `28765` | `frontend/vite.config.js` | Dev UI misses lab if port bumped | `DUBURI_LAB_PORT` env | **fixed** (v0.2) |

---

## P2 — design decisions (intentional but costly)

| ID | Decision | Why it hurts | Keep / change |
|----|----------|--------------|---------------|
| C1 | Course switch = full stop/start (~90s) | Operators expect hot-reload | **Keep** for v0.1; document clearly (done). Future: gz world reload R&D |
| C2 | Lab teleop on TCP 5763 vs manager 14550 | Correct split, but easy to miswire | **Keep**; document hard |
| C3 | Sibling workspace vs in-`duburi_ws` | Dual overlays confuse newcomers | **Keep** v0.1; see FUTURE_MERGE |
| C4 | Arm via subprocess planner, motion via pymavlink | Two control paths | Acceptable; unify later if needed |
| C5 | FX on side topics (`image_fx`) | Extra node; easy to record wrong feed | **Keep** (protects contract) |
| C6 | `props list` = library catalog | Name collision with “list spawned” | Change API naming (B4) |
| C7 | Static React build committed/installed under `static/` | Easy to ship stale UI vs `frontend/src` | Dev checklist: rebuild static after UI edits |
| C8 | `duburi_ws/.claude/context/sim-setup.md` legacy BlueROV path | Agents follow wrong guide | Prefer this `.context` (documented) |

---

## P3 — polish / maintainability

| ID | Finding | Recommendation | Status |
|----|---------|----------------|--------|
| D1 | Multiple static asset hashes may linger in `static/assets/` | Clean on each `npm run build` | open |
| D2 | Lab status polling chatty in logs | Sample / reduce uvicorn access log noise | open |
| D3 | No automated pytest for lab APIs | Add smoke pytest with mocked procs | open |
| D4 | `gate_transit_check` under-documented in operator path | Link from TESTING | open |
| D5 | README still long historical “What was wrong before” | Slimmed; detail stays in TROUBLESHOOTING | doc-fixed |
| D6 | Hydro / thruster_rig scripts only in worlds package | Mention in COMMAND_REFERENCE if used often | optional |

---

## Wrong decisions we should not reverse without cause

1. **Not** putting teleop RC on 14550 — would regress manager fights.  
2. **Not** remapping vision camera name to `sim_front` — missions expect `forward`.  
3. **Not** degrading `image_raw` in place — contract_check / autonomy need clean feed.  
4. **Forcing `flight_controller:=pixhawk` in stack** — required for `srot` + SITL.

---

## Security / ops notes (lab is local operator tool)

- Lab binds `0.0.0.0` by default — fine on localhost/devcontainer; do not expose WAN without auth.
- Zip endpoint serves any run id under `datasets/` — treat as trusted-operator only.
- Subprocess `ros2 run` / `pkill` from server assumes same-user workstation.

---

## Verification performed while documenting

- Contract topics and ports cross-checked against launch + `setup.py` entry points.
- Lab routes enumerated from `server.py`.
- Known session bugs (record race, dataset sort, port 8088) reconciled with current code.

---

## Suggested fix order (future coding passes)

1. B2 harden: refuse zero-frame finalize or mark `ok:false` in meta  
2. gz entity query for baked-in world props (instances today = lab-tracked)  
3. D3 API smoke tests  
4. TESTING_SUITE amenities (MCAP button, gate score overlay)  
5. Packaging: user-driven `git init` / FUTURE_MERGE  

Prior Cursor handoff: [HANDOFF.md](HANDOFF.md).
