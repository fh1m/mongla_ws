# Future packaging: sibling → merge or separate repo

**Current (v0.2):** keep `duburi-sim_ws` as a **sibling colcon workspace**.
This directory has **no `.git`**. The `duburi_ws` agent initializes git / submodule /
subtree **when integrating** — do not `git init` here preemptively.

No package move and no new GitHub remote unless the user explicitly asks.

## Why sibling now

- Clear drop-in boundary vs hardware stack
- Independent colcon graph / CI timing
- Avoids forcing Gazebo/ArduPilot deps onto every `duburi_ws` checkout
- Matches “sim env + lab UI + autonomy under test” product split
- Lets the autonomy agent own the first git history when merging

## Why no git yet

Handoff decision (Cursor agent, 2026-08-26): avoid a throwaway sim remote that
fights the eventual merge story. Ignore rules already live in root `.gitignore`.
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
3. Update `DUBURI_WS` discovery: lab/server currently walks parents for `duburi-sim_ws` / sibling.  
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
