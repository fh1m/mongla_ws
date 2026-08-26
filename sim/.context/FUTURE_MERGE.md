# Packaging: the decision, and why (record — no longer a menu)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

**Settled 2026-08-27.** The simulator was absorbed into the `duburi_ws` repo as
the nested colcon workspace **`duburi_ws/sim/`**. It is under version control and
also published standalone as
[`fh1m/duburi-sim_ws`](https://github.com/fh1m/duburi-sim_ws).
**`duburi_ws/sim/` is canonical; the standalone repo mirrors it.**

Everything below is kept as the reasoning that led there — the option tables were
the real value of this document and are still worth reading before anyone proposes
moving the tree again. What changed is only which option won.

### What was chosen

| | Chosen | Why not |
|---|---|---|
| **Nested workspace, plain files** (`sim/` + `COLCON_IGNORE`) | ✅ | — |
| Packages under `duburi_ws/src/duburi_sim_*` | ✗ | root `colcon build` / `colcon test` / `pytest src/` would pick up six Gazebo-dependent packages; autonomy must stay *identically* robust, i.e. the same test set, not a superset |
| `git subtree add` | ✗ | **was impossible** — subtree needs a source *repository* and this tree had no `.git`. It was `cp -a` + `git add sim/`. Now that a mirror repo exists the option technically reopens, and is still declined: `sim/` is already committed as plain files at a known-good bisect point, and subtree adds a pin plus a second push target for no benefit at this scale |
| Submodule | ✗ | a clone flag, a pin and a second push target, for one maintainer with one box |

### Costs accepted, named rather than hidden

- The 15 W Jetson (no WiFi) clones ~26 MB it will never build. Documented, not
  enforced: `git clone --filter=blob:none` then
  `git sparse-checkout set --no-cone '/*' '!/sim'`.
  `test_sim_contract_drift.py` skips cleanly when `sim/` is absent, so that
  checkout still passes its tests.
- Two copies exist (`duburi_ws/sim/` and the mirror repo). Drift is possible; the
  canonical-vs-mirror rule above is the only thing preventing it. Land changes in
  `duburi_ws/sim/`.

## Why sibling was chosen first (historical)

## Why sibling now

- Clear drop-in boundary vs hardware stack
- Independent colcon graph / CI timing
- Avoids forcing Gazebo/ArduPilot deps onto every `duburi_ws` checkout
- Matches “sim env + lab UI + autonomy under test” product split
- Lets the autonomy agent own the first git history when merging

## Why there was no git at first (historical)

The 2026-08-26 handoff deliberately avoided a throwaway sim remote that would
fight the eventual merge story. The cost of that choice was a 1.2 GB tree with no
history, one `rm -rf` from gone, which is why absorption became urgent rather than
optional. Resolved 2026-08-27.
When ready, either:

1. `git init` + push as Option B, then submodule into `duburi_ws`, or  
2. Copy/subtree into `duburi_ws` first (Option A) then commit there.

See [HANDOFF.md](HANDOFF.md).
## Option A — subtree inside `duburi_ws` (recommended later)

Target sketch:

```text
duburi_ws/
  sim/                    # or workspaces/duburi_sim/
    src/duburi_sim_*
    .context/
  src/duburi_manager/     # existing
  ...
```

Checklist when executing:

1. Decide overlay: single workspace vs nested `colcon` with `COLCON_IGNORE` boundaries.  
2. Preserve package names (`duburi_sim_*`) to avoid ament index churn.  
3. Update `DUBURI_WS` discovery: lab/server currently walks parents for `duburi_ws/sim` / sibling.  
4. Point `stack.launch.py` includes at same share names (unchanged if packages intact).  
5. Git: `git subtree add` or copy + history rewrite; set `.gitignore` for `datasets/`, `frontend/node_modules/`, large mp4.  
6. Docs: move `.context/` with the tree; add pointer from `duburi_ws/CLAUDE.md`.  
7. CI: job matrix — build sim packages only when `sim/**` changes.  
8. Deprecate sibling README with redirect for one release.

## Option B — dedicated public `duburi-sim` GitHub repo

- Extract current workspace as its own remote.  
- Document required sibling/or submodule pin of `duburi_ws` for stack.  
- Release tags for course worlds / contract version.  
- Still consume via `source install` overlay, not pip.

Use when external teams need sim without full autonomy history.

## Option C — git submodule under `duburi_ws/sim`

- `duburi_ws` pins sim SHA.  
- Contributors clone `--recurse-submodules`.  
- Good for versioned contract; awkward for day-to-day dual edits.

## Non-goals for merge day

- Renaming ROS topics (breaks contract)  
- Folding lab into Electron/Foxglove unless product asks  
- True Gazebo hot-swap worlds  

## Contract versioning suggestion

When merging or publishing, stamp [CONTRACT.md](CONTRACT.md) with a semver
(e.g. `sim-contract 0.1.0`) and keep `contract_check` as the machine gate.
