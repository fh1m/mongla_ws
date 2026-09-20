#!/usr/bin/env python3
"""find_person_demo -- canonical vision-driven mission for desk/thruster testing.

Uses the pretrained yolov11n model (COCO 80-class, auto-downloads ~5 MB on first
run) to detect 'person'. No custom weights needed -- ideal for desk testing before
pool day. Exercises both vision verbs (align + move) and the fallback search.

To run:
    ros2 launch mongla_vision vision.launch.py camera:=laptop model:=yolov11n classes:=person
    ros2 run mongla_planner mission find_person_demo

Swap model:
    mongla.models(person='yolov11s')   # ~10 MB, more accurate
    mongla.models(person='yolov11m')   # ~40 MB, highest accuracy

The two verbs
-------------
mongla.vision.align(target, *, lat=, yaw=, depth=, err=, duration=, gain=, fallback=)
    Hold the target at a signed pixel offset on each active axis (a number
    activates the axis; 0 = centre). Returns a truthy VisionResult when
    centred; never raises on a miss.

mongla.vision.move(target, *, fwd=, mode=, maintain=, hold=, ..., fallback=)
    Drive forward until the bbox fills `fwd` % of the frame (mode =
    area/width/height). `maintain` holds a lateral pixel offset while
    driving. Never re-centres.

Live tuning (no rebuild):
  ros2 param set /mongla_manager vision.kp_yaw 80.0
  ros2 param set /mongla_manager vision.lost_grace_s 1.5

WARNING: this mission arms the vehicle.
"""

CAMERA       = 'laptop'
DIVE_DEPTH_M = -0.5

ALIGN_ERR_PX     = 50     # webcam is noisy — a loose tolerance settles reliably
ALIGN_GAIN       = 30
APPROACH_GAIN    = 35
PERSON_FWD_FILL  = 55     # drive in until the person fills 55% of frame height
ALIGN_DURATION_S = 12.0
MOVE_DURATION_S  = 15.0


def run(mongla, log):
    mongla.mission_reset()
    # Pretrained COCO weights — auto-downloads yolov11n.pt (~5 MB) on first run.
    mongla.models(person='yolov11n')
    mongla.camera = CAMERA

    mongla.arm()
    mongla.set_mode('ALT_HOLD')
    mongla.set_depth(DIVE_DEPTH_M, settle=1.0)

    target = mongla.models.person.person   # ClassRef — sets model + class automatically

    # align: yaw-centre the person; sweep to find them if not in frame.
    mongla.vision.align(
        target, camera=CAMERA, yaw=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=ALIGN_DURATION_S,
        fallback=sweep_yaw)

    # move: drive in to standoff distance by bbox height fill.
    mongla.vision.move(
        target, camera=CAMERA, fwd=PERSON_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=MOVE_DURATION_S, fallback=creep_forward)

    # Deliberate de-target then re-acquire: tests the full search → lock cycle.
    mongla.yaw_left(90.0)
    mongla.vision.align(
        target, camera=CAMERA, yaw=0, depth=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=ALIGN_DURATION_S,
        fallback=sweep_yaw)

    mongla.stop()
    mongla.set_depth(0.0)
    mongla.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    mongla.move_forward(0.6, gain=30)


def sweep_yaw(mongla, should_stop):
    """Rotate right in steps looking for the target; bail when it reappears."""
    for _ in range(6):
        mongla.yaw_right(30.0)
        if should_stop():
            return
