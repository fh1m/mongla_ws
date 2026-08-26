# Datasets and recording

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

## Output layout

Root: `<duburi_ws/sim>/datasets/` (override with `record_cameras --outdir`).

Each run:

```text
datasets/<label>_<UTC_stamp>/
  front.mp4              # if front selected
  bottom.mp4             # if bottom selected
  meta.json              # always on clean stop
  classes.txt            # if --labels
  frames/{front,bottom}/ # if --frames
  labels/{front,bottom}/ # YOLO txt if --labels
  .ready                 # present only while recording (removed on finalize)
```

## `meta.json` fields (typical)

| Field | Meaning |
|-------|---------|
| `label`, `course`, `script_id` | Provenance |
| `duration_s` | Wall-clock record length (seconds) |
| `fps_requested` | CLI `--fps` hint only (not used for encode) |
| `fps_actual` | Per-camera `{front: N}` = `count / duration_s` (clamped 1–60) |
| `frames_dumped`, `labels`, `use_fx` | Flags |
| `cameras`, `classes` | Lists |
| `utc_start` | ISO timestamp |
| `counts`, `sizes`, `encodings`, `topics` | Per-camera stats |
| `gt_start`, `gt_end` | Ground-truth pose samples |
| `trajectory` | List of GT samples during run |

Incomplete runs (SIGINT during import, crash before finalize) may lack `meta.json`;
lab dataset API **skips** those directories.

## MP4 duration = wall time

Frames are **buffered in memory**; at finalize OpenCV writes the MP4 using
`fps_actual`, so a 10 s take plays for ~10 s even when PNG+label I/O only
sustains ~3–5 writes/s. Disk PNG/labels run on a **background queue** so the
ROS callback stays light.

Verify:

```zsh
ffprobe -v error -show_entries format=duration -of default=nk=1:nw=1 datasets/<run>/front.mp4
# expect ≈ meta.duration_s (±0.2 s)
```

## CLI

```zsh
ros2 run duburi_sim_bridge record_cameras \
  --duration 20 \
  --cameras front,bottom \
  --fx --frames --labels \
  --course sauvc26_qualification \
  --label gate_approach
```

SIGINT (Ctrl-C) or lab stop finalizes: closes video writers, writes `meta.json`,
prints `wrote <abs_path>`.

## Lab UX

1. Operate → set name / cams / fx / frames / labels  
2. ● record → status `recording on <active_course>…`  
3. ■ stop + download → toast + browser download of zip  

Course for the run = lab `active_course` (World tab), not a hardcoded string.

## YOLO labels

`gt_labels.py` projects course prop poses into camera frames using GT + camera_info.
Classes include qual_gate, final_gate, flares, drums, starting_zone, etc.
Labels are **simulator GT**, not human annotations — good for bootstrap / domain-gap tests.

## Move scripts

Under `src/duburi_sim_web/scripts/`:

- `gate_approach.yaml`
- `gate_pass_through.yaml`
- `strafe_scan.yaml`
- `bottom_look.yaml`
- `drum_orbit.yaml`

Run via lab Scripts API / UI utils — drives planner motions while recording.

## Training note

Raw contract topics stay `image_raw`. Prefer `--fx` / `image_fx` when collecting
for underwater domain gap. This repo does **not** include the YOLO train pipeline
(see `duburi_ws` vision skills/docs).
