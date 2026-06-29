# Anchor — XFeat + LighterGlue geometric "superglue" lock

> **Status:** BUILT on branch `lock`. `anchor:=false` by default until pool-tested.
> Phase-1.5 (this rev): the points→thrust sign bug is FIXED, theta→yaw is deadbanded,
> and disk-saved named references are in. Background streamer / `sticky` are phase 2.

## 1. Why it exists

YOLO `align`/`move` lose the bounding box when the AUV is very close to a target — the
torpedo hole fills or clips the frame, the detector flickers, and there's no fine
geometric reference to hold a precise pose for firing. **Anchor** snaps a *reference
frame* and drives the hull to re-superimpose the live camera view on it (XFeat keypoints
+ LighterGlue matching → homography → pixel pose error), holding lat/yaw/depth steady so
the hull "superglues" to the reference for a torpedo/dropper shot — **with no detection
bbox at all**. It also works on un-modelled scenery (any textured prop), which makes
**reproducible, pre-snapped missions** possible.

## 2. The three verbs

```python
duburi.vision.anchor_snap(name=None, *, target=None, conf=0.5, err=40)  # capture reference
duburi.vision.anchor_align(name=None, *, err=20, theta=0.05, duration=30,
                           hold=None, fire=None, match=None,
                           gain=30, lat_gain=, yaw_gain=, depth_gain=, brake=True)
duburi.vision.anchor_clear()                            # free the live reference
```

- **`anchor_snap()`** — anchor_node stores the **next** frame as the reference (in RAM).
  **`anchor_snap('hole')`** also saves it to `references/hole.png` (versioned in the repo)
  so it survives restarts and reloads on competition day.
  **`anchor_snap(target='hole', conf=0.6, err=40)`** — *snap-at-detection*: wait up to **3 s**
  for a `hole` detection with score≥`conf` within `err` px of centre, then snap **just that
  bbox crop** (keys the lock on the target, not moving background — ideal when YOLO loses
  the box up close). `err≤0` disables the centring gate; no qualifying detection in 3 s →
  whole-frame snap. Combine with `name=` to also save the crop (full-frame PNG + a bbox
  sidecar so the crop geometry survives reload).
- **`anchor_align()`** — locks on the last in-memory snap. **`anchor_align('hole')`**
  loads `references/hole.png` from disk first, then locks — no live snap needed.
  - `err`/`theta`: lock tolerance in px / rad. `hold`: active station-keep seconds (reuses
    the `align(hold=)` machinery). `fire` (int or `[1,2]`): payload channels fired **once**
    at first lock, mid-hold, while glued. `match`: min RANSAC inliers a tick must clear to
    count as locked (omit = trust the node). `gain`/`*_gain`: per-axis speed caps.
- **`anchor_clear()`** — drop the live reference (next snap is fresh).

Canonical torpedo flow:
```python
duburi.vision.align('hole', yaw=0, lat=0, gain=25)              # YOLO gets us roughly on
duburi.vision.anchor_snap()                                     # freeze this exact view
duburi.vision.anchor_align(err=15, theta=0.04, hold=3,
                           fire=[1], gain=20, yaw_gain=10, brake=False)   # glue + fire
```

## 3. How it works (pipeline)

```
camera_node ─ image_raw ─→ anchor_node  (process every skip_frames-th frame ~8 Hz)
   detectAndCompute(live) ─┐
   stored reference desc ──┴─→ match_lighterglue → cv2.findHomography(ref, live, RANSAC)
                              → extract_error(H) → AnchorError(tx, ty, theta, inliers)
   ├─ anchor_error (Vector3 tx,ty,theta)   ├─ anchor_state (IDLE|LOCKED|LOST by min_inliers)
   ├─ anchor_conf  (Float32 inliers)       └─ anchor_ref (Image, debug → HUD inset)
   services: anchor_snap (AnchorRef: name/load) · anchor_clear (Trigger)

AnchorState (duburi_manager)  subscribes the 3 topics, caches latest   ← mirrors VisionState
   ↑ provider (rclpy-free read)
anchor_align_loop (motion_vision)  drives Ch6 / Ch4 / depth-setpoint    ← mirrors align_loop
   ↑ verb
vision_anchor_{snap,clear,align}  on /duburi/move                       ← single-action surface
```

`anchor_node` mirrors `detector_node` (async model load so the interface is up
immediately; heavy match on an off-thread single-slot worker; every publish gated on
`is_loaded()`/reference-set). `AnchorState` mirrors `VisionState`, keeping `motion_vision`
rclpy-free. The manager owns the service clients.

## 4. The maths — points → thrust (verify this if you touch any sign)

### 4a. Homography direction (load-bearing)
`xfeat.py` computes `H = cv2.findHomography(mkpts_ref, mkpts_cur)` — **reference → live**.
Pushing the reference centre through H gives *"where the reference appears in the live
frame"*, which is a **direct analog of a YOLO bbox offset** (`+x` = right of centre). This
makes the anchor pose error have the **same sign as `align_loop`'s `ex`/`ey`**, so the
control laws are positive (no negation) **and sign-identical to the pool-verified YOLO
loop**. *Swapping the `findHomography` src/dst order inverts every axis into positive
feedback — the hull drives AWAY from the lock. This was a real bug (fixed 2026-06-28);
the arg order is not cosmetic.*

### 4b. Per-axis conversion (in `anchor_align_loop`)
| Pose error | Normalize | × gain | Clamp | → PWM | Channel | Physical |
|---|---|---|---|---|---|---|
| `tx` (px) | `/320` (nominal half-W) | `kp_lat` (60) | `±gain` | `percent_to_pwm` | **Ch6** | `tx>0`=ref right→strafe RIGHT |
| `theta` (rad) | — (deadband ±0.02) | `kp_yaw` (120) | `±gain` | `percent_to_pwm` | **Ch4** | `theta>0`→yaw right |
| `ty` (px) | `/240` (nominal half-H) | `kp_depth` (0.05) | `±max_nudge` | depth setpoint −= step | ALT_HOLD | `ty>0`=ref below→descend |

`percent_to_pwm(p) = 1500 + (p/100)·400`, clamped 1100–1900 (1500 = neutral, >1500 = Ch6
strafe-right / Ch4 yaw-right). **Negative feedback check:** AUV drifts RIGHT → forward cam
sees scene shift LEFT → reference appears left → `tx<0` → Ch6<1500 → strafe LEFT → closes
the error. (Same derivation gives descend-when-shallow and yaw-toward.) Lateral is
**freshness-decayed** (`_freshness(age_s)`) so a stale/slow match can't blind-drive; yaw
and depth are not (Ch4 is a rate ArduSub bleeds; depth is ArduSub's hold).

### 4b-crop. Crop reference keeps the SAME coordinate frame (signs unchanged)
A snap-at-detection crops the bbox, but `xfeat._describe(frame, bbox)` then **offsets the
cropped keypoints back into full-frame coords** (`kp += (x1,y1)`) and keeps `image_size` =
full frame. So the reference is "full-frame keypoints confined to the bbox region" — the
homography is single-coordinate-system and `extract_error` (with all its signs) is
**byte-identical** to a whole-frame reference. The crop only changes *which* keypoints exist
(target, not background). **Dependency:** `extract_error` pushes the frame centre, so a
near-centre crop is robust interpolation; a far-off-centre crop extrapolates → the `err`
centring gate keeps snaps near-centre. **Semantic:** a target snapped 30 px off-centre holds
at that 30-px-off snap position (anchor re-superimposes the snapped *view*), not dead-centre
— set `err` small for dead-centre. Disk crops persist via a `<name>.json` bbox sidecar.

### 4c. theta→yaw is the WEAK axis on the forward camera
`theta = atan2(H[1,0],H[0,0])` is in-plane **image roll** = camera roll about the optical
axis on a *forward* camera, **not yaw** — AUV yaw shows up as horizontal translation
(`tx`, already corrected by lateral). A wave-induced image roll would otherwise make yaw
chase a signal it cannot null and creep the hull off-aim mid-fire, so theta is
**deadbanded** (`ANCHOR_THETA_DEADBAND_RAD=0.02`, ~1.1°) — small roll → yaw neutral.
theta→yaw IS the clean primary axis for the **downward** camera (optical axis = vertical =
yaw). The sign derivation above is forward-camera; the downward camera already flips
`depth_sign`, but its lateral/yaw signs need their own bare pool check.

## 5. Control authority / racing (audited)

- **Ch4 arbitration:** `anchor_align` always drives yaw, so it **suspends the background
  heading lock** for its duration (`_suspend_heading_lock`) and re-targets it on exit —
  identical to `align_loop`-with-yaw. The retarget runs on the normal *and* abort exit
  (the loop returns an Outcome, never raises). No new arbitration.
- **No racing / stuck frames:** anchor_node match runs on a daemon thread draining a
  single-slot queue with `get(timeout=0.5)` (no busy-spin); snap is a one-shot
  `threading.Event`; the loop is deadline-bounded and checks `abort_fn` each tick;
  `AnchorState` is lock-guarded, `pose()` returns None until the first error, and a dead
  node ages out via `_freshness`/`_STALE_LIMIT_S` → LOST after grace.
- **Fire-once:** `fired`+`aligned_at` are set once and never reset, so a lock that drops
  and re-acquires mid-hold does **not** double-fire. Intended.
- **Nominal half-frame (320/240):** the gain scaling assumes ~640×480, so at other
  resolutions the effective gain shifts (re-tune `kp`/`gain`, not a correctness bug). Note
  `err_px` (lock tolerance) is raw pixels too, so in *angular* terms it's also
  resolution-dependent — 20 px is a tighter lock at 1920 than at 640. The camera default is
  640×480, so today both are exact.

## 6. Pros / cons / limitations

**Pros:** holds a precise pose with **no YOLO bbox** (the up-close torpedo case); works on
any textured scenery (no trained model); enables reproducible pre-snapped missions; fires
mid-hold while glued.

**Cons / limitations — know these before trusting a lock:**
- **Planar / rotation assumption.** A single homography is exact only for a planar scene or
  pure camera rotation. A flat torpedo board / bin is fine; a deep 3D scene with
  translation has parallax that no single H fits → noisier pose. Snap *at the standoff you
  will hold at*.
- **No metric range.** Monocular homography has scale ambiguity — there is **no forward
  axis**. A prior YOLO `move` sets the standoff; anchor holds the *viewpoint*, not distance.
- **Needs texture.** Low-texture / blank water → few keypoints. A snap on such a frame is now
  **REJECTED** at `set_reference` (`< _MIN_KP_REF=16` keypoints) so the service fails with a
  reason ("reference REJECTED -- only N keypoints; aim at a textured target / get closer")
  instead of silently storing a dud that LOSTs forever. In-flight, a frame with `< 4`
  keypoints short-circuits to no-lock **before** `match_lighterglue` — this is the guard for
  the `IndexError: max(): ... non-zero size` crash a blank frame would otherwise throw. The
  torpedo board, gate lattice, and pool-floor markings are good; open blue water is not.
- **Snap↔match drift.** Lighting, turbidity, and big viewpoint change between snap and match
  degrade matching. Re-snap if the scene has changed materially.
- **theta=roll on forward cam** (see §4c) — yaw is the weak axis there; lean on lat/depth.
- **8 Hz on an Orin Nano is optimistic.** Measure the real `match_hz` (the `[ANCHOR]`
  health line) like you measure `avg_infer`; `_freshness` covers slow rates but a fire-hold
  wants the highest rate you can get.

## 7. Installing XFeat + LighterGlue weights (pool-day prerequisite)

`torch.hub.load('verlab/accelerated_features','XFeat',...)` downloads the repo + XFeat
weights on first run; **LighterGlue's weights download lazily on the first
`match_lighterglue` call**. The pool has **no internet**, so warm BOTH caches on the Jetson
beforehand:

```bash
# On the Jetson, WITH internet, once:
python3 - <<'PY'
import torch, numpy as np
xf = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained=True, top_k=2048)
# IMPORTANT: warm on a TEXTURED frame, NOT np.zeros. A blank/low-texture image
# yields ZERO keypoints, and LighterGlue's filter_matches then reduces a
# zero-size dim -> `IndexError: max(): Expected reduction dim 1 to have
# non-zero size`. Random noise gives real keypoints so the full match path
# (and the lazy LighterGlue download) actually runs. This is the same
# condition the matcher's _MIN_KP_REF / _MIN_KP_MATCH guards reject in-flight.
rng = np.random.default_rng(0)
a = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
b = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
d0 = xf.detectAndCompute(a)[0]; d0['image_size'] = (640, 480)
d1 = xf.detectAndCompute(b)[0]; d1['image_size'] = (640, 480)
xf.match_lighterglue(d0, d1)        # <-- forces LighterGlue's lazy download too
print("warm: XFeat + LighterGlue cached")
PY
```

- Cache lives under `~/.cache/torch/hub/` (override with `TORCH_HOME`). Copy that dir to the
  Jetson if it has no internet at all.
- On `anchor:=true` startup the **`[ANCHOR] loaded XFeat + LighterGlue on cuda:0 (...)`**
  canary must appear. If the cache is cold the load hangs/fails — the node's interface still
  comes up but `anchor_state` stays IDLE and `anchor_align` returns `NO_CAMERA`.

## 8. Running it

```bash
# Single forward camera + anchor:
ros2 launch duburi_vision vision.launch.py camera:=forward anchor:=true
# Dual camera (forward+downward), both anchors:
#   (downward anchor_node STARTS, but its lateral/yaw signs are unverified -- the
#    sign derivation in §4 is forward-camera; bare-test the downward cam separately.)
ros2 launch duburi_vision vision_dual.launch.py anchor:=true
# Against dataset video:
ros2 launch duburi_vision video.launch.py fwd_video:=/path/board.mp4 anchor:=true
```
`anchor` is the canonical arg name (the `AnC` shorthand is just mnemonic). Anchor/homography
commands **require** the node running — without it they return `NO_CAMERA` (the manager
never auto-spawns nodes).

## 9. Pool test procedure (do in this order)

1. **Bare polarity test (empirical, mandatory).** Snap a reference, then hand-push the AUV
   to the RIGHT. Confirm `anchor_align` commands Ch6 **< 1500** (strafe LEFT, back toward
   the reference). If it drives further right, a mirror-mount or cv_bridge flip inverted the
   sign — stop and recheck §4a. (Unit tests assert the *convention*; this asserts the
   *physical* mounting.)
2. **Lock test.** Snap at the standoff; `anchor_align(err=20, theta=0.05, hold=5)`. The HUD
   padlock should go green (LOCKED) and the hull hold within err for the hold window. The
   reference inset now draws the **match overlay** — green dots on matched reference keypoints
   with short vectors to their live positions, and a `matches: N inliers` count (green once it
   clears `min_inliers`, amber below). No green / "no match" = the lock is not real; re-snap on
   more texture. If snap fails with **"anchor model FAILED to load: …"** the weights were not
   pre-downloaded (§7) — distinct from the transient "matcher still loading".
3. **Fire test.** `anchor_align(err=15, hold=3, fire=[1], brake=False)` → the torpedo should
   leave once, mid-hold, while glued.
4. **Named-reference reload.** `anchor_snap('hole')`, restart the stack, `anchor_align('hole')`
   → re-locks from `references/hole.png` with no live snap.
5. **Crop-snap-at-detection.** `anchor_snap(target='hole', conf=0.6, err=40)` → verify the
   HUD reference inset shows the full frame with an **amber rectangle around the cropped
   region** plus the green match overlay clustered inside it; then move too close so YOLO
   drops the bbox → confirm the anchor lock still holds geometrically.
6. **3 s fallback.** `anchor_snap(target='absent_class', conf=0.9)` with that class not in
   view → confirm `[ANCHOR] no … in 3s -- whole-frame snap` and a whole-frame reference.
7. **20 kg stiction calibration (`pass_through`).** `ros2 run duburi_planner duburi
   move_forward --duration 2 --gain N --pass_through` (bare flag), N from ~2 upward until the
   hull just moves; record per axis (forward via move_forward, lateral via move_left).
   **⚠️ ArduSub applies an RC deadzone (`RC*_DZ`, often ~30 PWM) to `RC_CHANNELS_OVERRIDE`
   *before* the motor matrix** — a 4-PWM delta (1504) is likely zeroed by ArduSub before the
   thrusters, so a sub-~30 ramp measures the **deadzone**, not the hull. For a true hull
   number set `RC*_DZ=0` for the test; otherwise read N as "min PWM that moves the hull
   *through the full ArduSub pipeline*." `pass_through` skips the reverse-kick brake so a
   sub-stiction probe isn't yanked back.

## 10. Clever XFeat+LighterGlue uses (menu — phase-2 builds)

- **Reproducible pre-run snapshots:** snap every prop (gate / hole / bin) the day before
  with internet, then `anchor_align('gate'|'hole'|'bin')` re-locks them on competition day —
  a mission that ran once becomes repeatable on the same ground.
- **Return-to-spot / waypoint memory:** named refs as visual breadcrumbs to re-find a task
  view after a detour.
- **Bbox re-acquire:** when YOLO drops the box up close, anchor holds geometrically.
- **Multi-shot torpedo from one snap** (needs the phase-2 background streamer to fire across
  separate goals while staying glued).
- **Model-free station-keep:** hold on any textured prop with no trained detector.
- **Drift self-check:** compare live vs the last snap to detect the hull has wandered.

## 11. Phase-2 backlog (NOT built)

Background "superglue streamer" (persistent Ch6+Ch4+depth like heading_lock →
fire-many-while-glued, `vision.move(sticky=True)`); descriptor-cached refs (vs PNG);
full downward-camera sign re-derivation.

> **Already built (was once backlog):** snap-at-detection
> (`anchor_snap(target=, conf=, err=)` crops the detection's bbox at the
> aligned moment — see §2 / §4b) shipped on `lock`; it is no longer pending.
