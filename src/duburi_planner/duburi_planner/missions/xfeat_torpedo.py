#!/usr/bin/env python3
"""xfeat_torpedo -- LITMUS: fire the torpedo from a standalone XFeat lock.

The pool proof that the XFeat anchor can find/hold/fire on its own. Detection
only ACQUIRES + sets the standoff; the XFeat homography then GLUES the hull to
the hole and the torpedo leaves mid-hold while the loop is still superimposing
the live view on the snapped reference -- verified live on the OpenCV HUD (the
bottom-right ANCHOR REF inset shows green match lines + inlier count, the padlock
reads LOCKED, and the shot leaves at lock).

Run WITH the HUD + anchor node so you can SEE the lock fire:
    ros2 launch duburi_vision vision.launch.py camera:=forward anchor:=true viewer:=true
    ros2 run duburi_planner mission xfeat_torpedo

Watch the HUD: snap -> 'ANCHOR REF' inset with GREEN match lines appears; as the
hull converges the padlock turns green (LOCKED) and inliers climb past
XFEAT_MIN_INLIERS; the torpedo fires while LOCKED. If the inset shows a snapshot
but no/too-few green lines, the reference was low-texture -- get closer / aim at
the hole rim before firing (see .claude/context/anchor-patterns.md).

Pre-reqs: anchor:=true, XFeat weights pre-downloaded (.claude/context/xfeat-setup.md),
TORPEDO_DEPTH_M set in competition_config.py.

fire_channel=1 -> torpedo_1. To DRY-RUN (no launch), set XFEAT_FIRE_CHANNEL=None.
"""

from duburi_planner.missions.competition_config import (
    TORPEDO_HEADING_DEG,
    TORPEDO_DEPTH_M,
    TORPEDO_STANDOFF_FILL,
    ALIGN_ERR_PX,
    ALIGN_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/duburi_detector_forward'

# ── litmus knobs (tune deck-side) ───────────────────────────────────────────────
XFEAT_SNAP_CONF     = 0.5    # detection score floor for the crop snap
XFEAT_SNAP_ERR_PX   = 30.0   # detection must be within this px of centre to snap
XFEAT_MIN_INLIERS   = 12.0   # RANSAC inliers a tick must clear to count as LOCKED
XFEAT_HOLD_S        = 5.0    # how long the homography lock station-keeps (fires within)
XFEAT_FIRE_T        = 1.5    # seconds into the hold to fire (< XFEAT_HOLD_S)
XFEAT_FIRE_CHANNEL  = 1      # 1/2 = torpedo; set None for a dry lock-only rehearsal
XFEAT_ALIGN_GAIN    = 18     # gentle -- the homography lock on a 20 kg hull wants slow


def run(duburi, log=None):
    _log = log or (lambda *_a, **_k: None)
    duburi.mission_reset()
    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — edit competition_config.py before pool day')

    if TORPEDO_HEADING_DEG is not None:
        duburi.turn(TORPEDO_HEADING_DEG)
    duburi.set_depth(TORPEDO_DEPTH_M, timeout=30)

    duburi.resume_detector('forward')
    duburi.set_model('torpedo_blood_hole', node=_FWD)
    duburi.set_classes('hole', node=_FWD)

    # ── 1. Detection ACQUIRES the hole + parks at the firing standoff ─────────────
    #     (the anchor has no forward/range axis, so the standoff is set here first).
    _log('[xfeat_torpedo] acquiring hole + standoff (detection) ...')
    duburi.vision.align(
        'hole', camera='forward', lat=0, depth=0,
        fwd=TORPEDO_STANDOFF_FILL, fwd_mode='height',
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, lock_on=True, hold=1.0,
        duration=25, fallback=_creep_forward)

    # ── 2. Snap the hole crop as the XFeat reference (keys the lock on the hole) ──
    if not duburi.anchor.snap(source='detection', target='hole',
                              conf=XFEAT_SNAP_CONF, err=XFEAT_SNAP_ERR_PX):
        _log('[xfeat_torpedo] SNAP FAILED — hole too low-texture to lock; '
             'get closer / aim at the rim. Torpedo NOT fired.')
        duburi.pause_detector('forward')
        return

    # ── 3. STANDALONE XFeat drives the terminal lock and FIRES at lock ────────────
    #     Pure homography: lat from tx, yaw from theta, depth from ty. The torpedo
    #     leaves mid-hold while glued -- watch the HUD ANCHOR REF inset (green lines)
    #     + padlock (LOCKED). fire is gated on the lock, so a bad match never fires.
    _log('[xfeat_torpedo] XFeat lock + fire (watch the HUD ANCHOR REF inset) ...')
    res = duburi.anchor.align(
        err=15, theta=0.05, gain=XFEAT_ALIGN_GAIN,
        hold=XFEAT_HOLD_S, fire=XFEAT_FIRE_CHANNEL, match=XFEAT_MIN_INLIERS,
        duration=30)
    if res:
        _log(f'[xfeat_torpedo] LOCKED + fired from XFeat -> {res}')
    else:
        _log(f'[xfeat_torpedo] XFeat never locked ({res}) — fire withheld '
             '(gated on lock). Re-snap closer / raise the lighting.')

    duburi.anchor.clear()
    duburi.pause(2.0)
    duburi.pause_detector('forward')


def _creep_forward(duburi):
    """One short forward creep so the acquire loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
