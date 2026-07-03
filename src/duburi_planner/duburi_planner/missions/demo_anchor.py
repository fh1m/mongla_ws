#!/usr/bin/env python3
"""demo_anchor -- the three XFeat anchor (feature-lock) patterns, runnable.

Demonstrates `duburi.anchor.*` composed with the detector verbs. Full write-up:
`.claude/context/anchor-patterns.md`. Requires the anchor node (`anchor:=true`)
and pre-downloaded XFeat weights (`.claude/context/xfeat-setup.md`).

  Pattern 1  Torpedo "stick like glue": YOLO acquires the hole + standoff, XFeat
             homography GLUES the hull to it and fires mid-hold.
  Pattern 2  Cloudy-day fallback: reload a reference saved on a clear day and
             navigate on XFeat ALONE (no reliance on YOLO scores).
  Pattern 3  Precise find -> hold: YOLO finds from distance, XFeat holds steady.

This is a DEMO/tuning scaffold -- copy the pattern you need into a real task and
tune the constants. Every anchor verb is graceful without hardware (snap returns
False, align returns a falsy VisionResult), so the mission never hard-fails.

Run: ros2 run duburi_planner mission demo_anchor
"""

# ── tune these deck-side ────────────────────────────────────────────────────────
DEMO_DEPTH_M       = -0.5     # hold depth for the demo
TARGET_CLASS       = 'hole'   # detector class to acquire (torpedo_blood_hole model)
MODEL_NAME         = 'torpedo_blood_hole'
STANDOFF_FILL      = 25.0     # % height fill = the firing standoff (no anchor forward axis)
SNAP_CONF          = 0.5      # detection score floor for the crop snap
SNAP_ERR_PX        = 30.0     # detection must be within this px of centre to snap
GLUE_HOLD_S        = 4.0      # how long the homography lock holds (fires within it)
MIN_INLIERS        = 12.0     # RANSAC inliers a tick must clear to count as locked
FIRE_CHANNEL       = 1        # 1/2 = torpedo, 3/4 = dropper; None = no fire (dry demo)
SAVED_REF_NAME     = 'hole'   # references/<name>.png for the cloudy-day pattern

_FWD = '/duburi_detector_forward'


def run(duburi, log=None):
    _log = log or (lambda *_a, **_k: None)
    duburi.mission_reset()
    duburi.set_depth(DEMO_DEPTH_M, timeout=30)
    duburi.resume_detector('forward')
    duburi.set_model(MODEL_NAME, node=_FWD)
    duburi.set_classes(TARGET_CLASS, node=_FWD)

    _pattern_1_stick_like_glue(duburi, _log)
    _pattern_2_cloudy_day_fallback(duburi, _log)
    _pattern_3_precise_find_then_hold(duburi, _log)

    duburi.pause(1.0)
    duburi.pause_detector('forward')


# ── Pattern 1: YOLO acquires + standoff, XFeat glues + fires ────────────────────
def _pattern_1_stick_like_glue(duburi, log):
    log('[demo_anchor] P1 stick-like-glue: acquire hole + standoff (YOLO) ...')
    # Coarse acquire + park at the firing standoff with the detector (no anchor
    # forward axis, so the standoff MUST be set here before the glue lock).
    duburi.vision.align(TARGET_CLASS, camera='forward', lat=0, depth=0,
                        fwd=STANDOFF_FILL, fwd_mode='height',
                        lock_on=True, hold=1.0, duration=20)
    # Snap the hole crop as the anchor reference, then GLUE the hull to it + fire.
    if duburi.anchor.snap(source='detection', target=TARGET_CLASS,
                          conf=SNAP_CONF, err=SNAP_ERR_PX):
        res = duburi.anchor.align(hold=GLUE_HOLD_S, fire=FIRE_CHANNEL,
                                  match=MIN_INLIERS, duration=25)
        log(f'[demo_anchor] P1 glue lock -> {res}')
    else:
        log('[demo_anchor] P1 hole too low-texture to snap -- '
            'a real task would fire from the vision standoff instead')


# ── Pattern 2: reload a saved reference, navigate on XFeat alone ────────────────
def _pattern_2_cloudy_day_fallback(duburi, log):
    log('[demo_anchor] P2 cloudy-day: lock a SAVED reference on XFeat alone ...')
    # DECK step (do this on a clear day, once):
    #   duburi.anchor.save(SAVED_REF_NAME, source='detection', target=TARGET_CLASS)
    # COMP-DAY step: load references/<name>.png and lock WITHOUT the detector.
    res = duburi.anchor.align(ref=SAVED_REF_NAME, hold=2.0,
                              match=MIN_INLIERS, duration=20)
    if res:
        log(f'[demo_anchor] P2 standalone feature-nav locked -> {res}')
    else:
        log(f'[demo_anchor] P2 no references/{SAVED_REF_NAME}.png yet '
            '(save one deck-side) -- skipping')


# ── Pattern 3: YOLO finds from distance, XFeat holds steady ─────────────────────
def _pattern_3_precise_find_then_hold(duburi, log):
    log('[demo_anchor] P3 precise find -> hold ...')
    duburi.vision.align(TARGET_CLASS, camera='forward', yaw=0, lat=0, duration=15)
    if duburi.anchor.snap(source='detection', target=TARGET_CLASS,
                          conf=SNAP_CONF, err=SNAP_ERR_PX):
        res = duburi.anchor.align(hold=2.0, match=MIN_INLIERS, duration=15)
        log(f'[demo_anchor] P3 terminal hold -> {res}')
    else:
        log('[demo_anchor] P3 snap failed -- detector-only hold would apply')
    duburi.anchor.clear()
