# Anchor — XFeat + LighterGlue geometric "superglue" lock (phase 1)

> **Status:** BUILT on branch `lock` (phase 1). `anchor:=false` by default until
> pool-tested. Background streamer / `sticky` / named-disk refs are **phase 2** (not built).

## Why

YOLO `align`/`move` lose the bbox when the AUV is very close to a target (the
torpedo hole fills/clips the frame) — there's no fine geometric lock to hold a
precise pose for firing. **Anchor** snaps a *reference frame* and drives the hull
to re-superimpose the live view on it (XFeat keypoints + LighterGlue → homography
→ pixel pose error), holding lat/yaw/depth steady so the hull "superglues" to the
reference for a torpedo/dropper shot — **even with no detection bbox**.

## The three verbs (DSL)

```python
duburi.vision.anchor_snap()                       # capture reference (non-blocking)
duburi.vision.anchor_align(err=20, theta=0.05, duration=30, hold=2.0,
                           fire=[1], match=20,
                           gain=25, yaw_gain=10)   # geometric lock + hold + fire
duburi.vision.anchor_clear()                       # free the reference; next snap is fresh
```

- `anchor_snap()` — anchor_node stores the **next** frame as the reference.
- `anchor_align(...)` — drives **lat from tx, yaw from theta, depth from ty** until
  the live view matches within `err` px and `theta` rad, then **actively holds**
  for `hold` s (reuses the `align(hold=)` station-keep). `fire` (int or list) fires
  those payload channels **once at first lock**, mid-hold, while the hull is glued.
  `match` = min RANSAC inliers a tick must clear to count as locked (None/0 = trust
  the node). `gain`/`lat_gain`/`yaw_gain`/`depth_gain` cap speed per axis.
- `anchor_clear()` — drop the reference.

Typical torpedo use: YOLO `align`/`move` to get close → `anchor_snap()` → `anchor_clear`
between targets. The classic flow:
```python
duburi.vision.align('hole', yaw=0, lat=0, gain=25)      # YOLO gets us roughly on
duburi.vision.anchor_snap()                              # freeze this exact view
duburi.vision.anchor_align(err=15, theta=0.04, hold=3,
                           fire=[1], gain=20, yaw_gain=10, brake=False)  # glue + fire
```

## Honest boundary — what the homography gives (and does NOT)

A **monocular** homography is a **pixel-plane** transform, not metric depth/range:
- `tx_px → lat` (Ch6), `theta_rad → yaw` (Ch4), `ty_px → depth-setpoint nudge`
  (same as `align_loop` turns `ey` into a depth nudge). **No forward/range axis** —
  homography scale is ambiguous; ArduSub holds depth, a prior YOLO `move` set the
  standoff. So "superglue / Pose2D" = the **same three axes `align_loop` already drives**.
- Anchor holds the hull *on the reference viewpoint*; it does not control how far
  away you are. Get the standoff right with `move` first, then snap.

## Architecture (mirrors the existing perception seams)

```
camera_node ─ image_raw ─→ anchor_node (XFeat+LighterGlue → homography)
                              ├─ anchor_error  (Vector3 tx,ty,theta)
                              ├─ anchor_state  (String IDLE|LOCKED|LOST)
                              ├─ anchor_conf   (Float32 RANSAC inliers)
                              └─ anchor_ref    (Image, debug reference)   ─→ HUD inset
                              services: anchor_snap / anchor_clear  (std_srvs/Trigger)

AnchorState (duburi_manager) subscribes the 3 topics, caches latest  ← mirrors VisionState
   ↑ provider
anchor_align_loop (motion_vision, rclpy-free)  drives Ch6/Ch4/depth   ← mirrors align_loop
   ↑ verb
vision_anchor_{snap,clear,align}  on /duburi/move  ← single-action surface preserved
```

- `anchor_node` mirrors `detector_node`: subscriber light, heavy match on an
  off-thread single-slot worker, model loads async (interface up immediately),
  every publish gated on `is_loaded()`/reference-set.
- The manager owns the Trigger **service clients**; snap/clear verbs call them,
  `anchor_align` runs the loop reading `AnchorState`.

## Control authority / channel ownership (verified)

`anchor_align` always drives **Ch4 (yaw from theta)**, so it **suspends the
background heading lock** for its duration (`_suspend_heading_lock`) and
re-targets it on exit — identical arbitration to `align_loop` with a yaw axis. It
shares `_freshness` (lat decays when the match is stale — low match-rate can't
blind-drive), the inertial arrival brake, and the cooperative `abort_fn`. The
action server's one-goal-at-a-time gate means a blocking `anchor_align` and a
separate `fire()` goal can't overlap — hence `fire` is wired **inside** the loop
(`on_locked`, fires once at first lock). Firing *many* across separate goals needs
the phase-2 background streamer.

## Files

- `duburi_vision/anchor/{anchor,homography,xfeat,anchor_node}.py` (one class/file).
- `duburi_manager/anchor_state.py` (`AnchorState` reader + pool/provider in the node).
- `motion_vision.anchor_align_loop`; `vision_verbs.vision_anchor_*`; `commands.py`
  3 verbs; `Move.action` `+theta_thresh +min_inliers +fire_channels`; `vision_dsl`
  3 methods; `draw_video.draw_anchor_overlay` + `display_node` subs; `setup.py`;
  `vision.launch.py anchor:=false`.

## Pool-day prerequisites

- **Pre-download XFeat weights on the Jetson** — first `torch.hub.load` downloads
  from `verlab/accelerated_features`; the pool has **no internet**. Run once on the
  Jetson with internet so the hub cache is warm; the `[ANCHOR] loaded XFeat +
  LighterGlue` canary must appear at startup (a missing cache hangs/fails the load —
  the node's interface still comes up, but `anchor_state` stays IDLE).
- **Measure match-rate** like `avg_infer`: the `[ANCHOR] match_hz` debug line.
  8 Hz on an Orin Nano is optimistic; `_freshness` handles slow rates, but a torpedo
  hold wants the highest rate you can get.
- `anchor_align` needs `ALT_HOLD` (the verb ensures it) and a snapped reference, or
  it returns `NO_CAMERA` fast.

## Phase-2 backlog (NOT built)

Background "superglue streamer" (persistent Ch6+Ch4+depth like heading_lock →
fire-many-while-glued, `vision.move(sticky=True)`); named references + disk
`save=`/`name=` / `anchor_align('NAME')` (reproducible pre-run snapshots);
snap-at-detection (`anchor_snap(target=, conf=, err=)` cropping the perfectly-aligned
moment). These multiply surface — land them once the in-memory spine proves underwater.
