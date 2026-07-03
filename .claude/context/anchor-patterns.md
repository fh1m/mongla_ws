# Anchor (XFeat) mission patterns — how to actually use the feature lock

> The `duburi.anchor.*` namespace is the XFeat + LighterGlue homography lock. It is a
> **second, appearance-robust way to find/hold/align/fire** that does **not** depend on the
> YOLO detector staying reliable. This doc shows the three patterns it's for. Architecture:
> [`anchor-system.md`](anchor-system.md); Jetson deploy: [`xfeat-setup.md`](xfeat-setup.md);
> command surface: `duburi.anchor.snap/save/align/clear` (see `vision_dsl._AnchorDSL`).

## What the anchor gives you (and what it does NOT)

- **Gives:** a homography lock onto a *reference view* — drives **lat** (from tx), **yaw**
  (from theta), **depth** (from ty) until the live view re-superimposes on the reference,
  then holds. It keys on **gradient/structure keypoints**, so it survives lighting/colour
  shifts that break a day-specific YOLO model. It can fire mid-hold ("stick like glue").
- **Does NOT:** have a forward/range axis (a monocular homography has no metric scale) — set
  the standoff with a prior `vision.move`/`vision.align(fwd=)`. It needs **texture**: a
  reference under ~16 keypoints is rejected (aim at a textured target / get closer). It needs
  a **reference** first (`snap`/`save`/`ref=`), and the anchor node running (`anchor:=true`).

---

## Pattern 1 — Torpedo "stick like glue" (YOLO acquires, XFeat holds + fires)

YOLO finds the hole and gets you to a standoff; XFeat then **glues** the hull to the exact
hole and fires mid-hold, holding sub-pixel even as the bbox balloons/degrades point-blank.

```python
# 1. Coarse acquire + standoff with the detector (wide, robust at range).
duburi.vision.align('hole', camera='forward', lat=0, depth=0,
                    fwd=TORPEDO_STANDOFF_FILL, fwd_mode='height',
                    lock_on=True, hold=1.0)              # rough centre + park at standoff

# 2. Snap the hole crop as the anchor reference, then GLUE + fire.
if duburi.anchor.snap(source='detection', target='hole', conf=0.5, err=30):
    res = duburi.anchor.align(hold=4, fire=1, match=12)  # homography lock; torpedo mid-hold
    if not res:
        log('[torpedo] anchor never locked — fire withheld (gated on lock)')
else:
    log('[torpedo] hole too low-texture to snap — fall back to vision standoff shot')
```

Why it helps: the detector box jitters and can jump to a second opening up close; the
homography lock is continuous and keys on the hole's own texture, so the shot leaves while
genuinely superimposed. `match=` (min RANSAC inliers) gates the lock; `fire` is gated on the
lock, so a bad match never fires off-target. Keep heading on `lock_heading` (anchor drives
yaw from theta only when you let it — for the terminal shot prefer heading_lock + lat/depth).

---

## Pattern 2 — Cloudy-day / bad-detection fallback (standalone XFeat)

The operator's own plan: **train days prior, but the water on comp day looks different**
(cloudy, turbid) and YOLO scores drop. Snap the prop's reference on a good day, reload it on
the day, and navigate on XFeat alone — no reliance on the detector.

```python
# ── DECK / practice day (water clear, prop visible): capture + persist. ──
duburi.anchor.save('hole', source='detection', target='hole')   # -> references/hole.png (+bbox)
#   or from the whole view if you can't get a clean detection:
duburi.anchor.save('gate', source='frame')

# ── COMPETITION DAY (YOLO unreliable): navigate on the saved reference. ──
duburi.anchor.align(ref='hole', hold=4, fire=1)     # loads references/hole.png, locks, fires
```

`references/<name>.png` (+ a `.json` bbox sidecar) lives in the source tree, so it survives a
rebuild/restart and rides to the pool with the code. This is genuine standalone feature-nav:
XFeat is structure-based, so a reference snapped in clear water still matches in murkier water
far better than a colour/appearance model retrained for one day's look.

---

## Pattern 3 — Precise find → hold (hybrid, best of both)

YOLO is the wide-field *finder* (robust to viewpoint, finds the target from far); XFeat is the
*terminal micro-lock* (sub-pixel hold once you're close). Compose them:

```python
duburi.vision.align('gate', yaw=0, lat=0)               # detector acquires from distance
duburi.anchor.snap(source='detection', target='gate')   # freeze the view it converged on
duburi.anchor.align(hold=2)                             # homography holds it rock-steady
```

Use for a dropper hover, a station-keep in current, or any "get on it, then don't move" step.

---

## Pattern 4 — `use_feature=True` (the fallback fused into ONE verb)

Patterns 1–3 compose two verbs. `vision.align(..., use_feature=True)` folds the anchor in as
a **detection fallback inside the normal align loop**: the detector stays primary, but on any
tick where YOLO returns no box **and** the anchor is `LOCKED`, the loop steers on the
homography pose instead — so a *sustained* detector dropout (murky water, a cloud passing) is
ridden out without losing the hold. It needs a reference snapped/loaded first (it never
auto-snaps), and degrades to detection-only when there's no lock.

```python
duburi.anchor.snap(source='detection', target='hole')       # give the fallback something to hold
duburi.vision.align('hole', lat=0, depth=0, hold=4, fire=1,  # detector primary ...
                    use_feature=True)                        # ... XFeat holds the gaps
```

Semantics (v1, deliberately conservative): detection **primary** (no blend/averaging);
substitute **only** on a locked anchor; `ex`/`ey` from `tx`/`ty` normalized by the real image
size; `theta` ignored (align drives lat/yaw from horizontal, depth from vertical); range axis
untouched (holds lat/yaw/depth, not `fwd`). Off by default — existing missions are unchanged.
Use Pattern 1's explicit `anchor.align` when you want the homography to be the *sole* driver
(the terminal glue lock); use `use_feature` when you want YOLO to lead and XFeat to **cover its
gaps**.

## Pool runbook / gotchas

- Launch with `anchor:=true` (see [`xfeat-setup.md`](xfeat-setup.md)) — pre-download weights.
- **Snap references deck-side** for Pattern 2; verify each with the HUD green match lines +
  inlier count before you rely on it.
- A reference needs texture (≥16 kp) — the hole rim, board graphics, gate lettering are good;
  bare water/pipe is not. If `snap` logs `REJECTED -- only N keypoints`, get closer / aim at
  texture.
- No forward axis: set the firing standoff with the prior `vision` verb; `anchor.align` only
  centres lat/yaw/depth and holds.
- `anchor.align` returns a truthy `VisionResult` only on a real lock — branch on it and keep
  the detector path as the fallback (Pattern 1's `else`).
- Slalom: XFeat on bare PVC pipes is weak (low texture) — **not** recommended as the primary;
  if used, snap the *whole frame* (pipes + textured pool floor/background) for a short
  drift-hold, never the pipe alone. The detection + heading-lock pass remains primary.

Runnable demo of Patterns 1–3: `ros2 run duburi_planner mission demo_anchor`.

## Litmus test — fire the torpedo from a standalone XFeat lock (pool proof)

The single mission that proves the whole system, watched live on the OpenCV HUD:

```bash
# HUD + anchor node up so you SEE the lock fire:
ros2 launch duburi_vision vision.launch.py camera:=forward anchor:=true viewer:=true
ros2 run duburi_planner mission xfeat_torpedo
```

`xfeat_torpedo` (Pattern 1, focused): detection acquires the hole + standoff, XFeat snaps the
hole crop, then the **homography lock alone** drives lat/yaw/depth and fires the torpedo
mid-hold. On the HUD you should see the **bottom-right ANCHOR REF inset with green match
lines**, the **padlock turn green (LOCKED)**, inliers climb past `XFEAT_MIN_INLIERS`, and the
shot leave while LOCKED. Fire is gated on the lock — a bad/low-inlier match never fires. If the
inset shows a snapshot but few/no green lines, the reference was low-texture: get closer / aim
at the hole rim before firing. Set `XFEAT_FIRE_CHANNEL=None` for a dry lock-only rehearsal.
