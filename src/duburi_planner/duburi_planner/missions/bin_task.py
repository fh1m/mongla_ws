"""Bin drop task — switch to downward camera, locate bin, drop marker.

Standalone test (bin_fire_blood.pt required — logic tested with yolo11n placeholder):
    ros2 run duburi_planner mission bin_task

Called by full_mission_2026 combinator after slalom_task.

kp_forward MUST be negative when using downward camera: ey>0 means the
target is AFT of the AUV (below-frame = behind AUV), so positive ey
should drive the AUV BACKWARD, hence kp_forward=-60.0.
"""

from duburi_planner.missions.competition_config import (
    BIN_HEADING_DEG,
    BIN_DEPTH_M,
    SEARCH_FORWARD_GAIN,
    SEARCH_MAX_STEPS,
)


def run(duburi, log=None):
    if BIN_HEADING_DEG is not None:
        duburi.turn(BIN_HEADING_DEG)
    duburi.set_depth(BIN_DEPTH_M, timeout=30)

    duburi.use_camera('downward')
    duburi.resume_detector('downward')
    duburi.set_model('bin_fire_blood', node='/duburi_detector_dwn')
    duburi.set_classes('fire,blood', node='/duburi_detector_dwn')

    # ── Search: heading-locked forward creep, downward cam watching below ─────
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('fire', camera='downward', stale_after=1.0):
            break
        duburi.move_forward(duration=0.5, gain=SEARCH_FORWARD_GAIN)
    else:
        result = duburi.vision.scan(
            target='fire', camera='downward',
            step=15, dwell=1.5, speed=30, duration=60)
        if not result.success:
            if log:
                log('[bin_task] fire not found in scan — skipping task')
            duburi.pause_detector('downward')
            duburi.use_camera('forward')
            return

    # ── Align sub directly above bin ──────────────────────────────────────────
    # kp_forward negative: downward cam y-error polarity is inverted vs forward cam
    duburi.vision.home(
        target='fire', camera='downward',
        lat=True, forward=True, yaw=False, depth=False,
        kp_forward=-60.0, kp_lat=60.0,
        deadband=0.06, on_lost='hold', duration=20)

    duburi.pause(3.0)   # 3s stability confirmation before drop
    duburi.fire(3)      # dropper_1 — channel always explicit
    duburi.pause(2.0)   # confirm drop complete

    duburi.pause_detector('downward')
    duburi.use_camera('forward')
