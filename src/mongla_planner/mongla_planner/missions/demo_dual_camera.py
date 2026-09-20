"""demo_dual_camera — gate → bin → torpedo across TWO cameras, ONE detector at a time.

A TEMPLATE showing how a mission drives both cameras in one run without relaunching
the vision stack. The rule the whole thing rests on:

  * BOTH cameras stream the whole time (two camera_nodes, independent of detection).
  * Only ONE detector INFERS at a time -- `use_camera(name)` pauses every other
    detector and resumes this one. Paused = the camera still streams, its detector
    just stops running YOLO. That is what keeps the 8 GB Jetson off the
    concurrent-inference OOM wall -- NOT unloading models.

So the sequence is forward(gate) → downward(bin) → forward(torpedo): each
`use_camera` flips which single detector is live (and flips the HUD), with a
built-in settle so the resumed detector's first frames land before we steer.

LAUNCH (paused:=true is the norm -- detectors start idle, the mission resumes one):
  ros2 launch mongla_vision vision_dual.launch.py viewer:=true paused:=true \\
      fwd_device_path:=/dev/mongla_cam_forward dwn_device_path:=/dev/mongla_cam_downward \\
      fwd_models:=gate_rescue_repair,torpedo_blood_hole \\
      fwd_classes:=gate,rescue,repair,torpedo,blood,hole \\
      dwn_models:=bin_fire_blood dwn_classes:=fire,blood

  NEVER launch paused:=false with two model-loaded detectors -- both infer from
  t=0 (before any use_camera) and that is the OOM. paused:=true + use_camera-first
  is the contract. Full rationale: .claude/context/perception/dual-camera-setup.md §4c.

Run:  ros2 run mongla_planner mission demo_dual_camera
"""

_FWD = '/mongla_detector_forward'
_DWN = '/mongla_detector_downward'


def run(mongla, log=None):
    mongla.mission_reset()
    mongla.arm()
    mongla.set_depth(-1.2, timeout=30)

    # Per-model conf: the forward detector carries TWO models; run the torpedo
    # model tight (fewer false holes on the shot) and the gate model loose. Set
    # once up front -- each override lives on its model and survives the set_model
    # switch below. (set_conf(x) with no model= would set ALL models uniformly.)
    mongla.set_conf(0.35, model='gate_rescue_repair', node=_FWD)
    mongla.set_conf(0.55, model='torpedo_blood_hole', node=_FWD)

    # ── 1. GATE (forward) ───────────────────────────────────────────────────────
    # use_camera('forward') resumes the forward detector (pausing downward) + HUD.
    mongla.use_camera('forward')
    mongla.set_model('gate_rescue_repair', node=_FWD)
    mongla.set_classes('gate', node=_FWD)
    if mongla.wait_for('gate', timeout=10):
        mongla.vision.align('gate', yaw=0, lat=0, err=40, gain=30, duration=20)
        mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    elif log:
        log('[demo] gate not seen — skipping to bin')

    # ── 2. BIN (downward) ───────────────────────────────────────────────────────
    # ONE call flips the live detector to downward (forward pauses) + the HUD.
    # Downward kwargs remap (see task_bin / downward-camera.md): lat+fwd centre
    # over the bin, depth = descent to a fill%.
    mongla.use_camera('downward')
    mongla.set_classes('fire,blood', node=_DWN)
    if mongla.wait_for('fire', timeout=10):
        if mongla.vision.align('fire', camera='downward', lat=0, fwd=0,
                               depth=35, fwd_mode='height', err=30, gain=25,
                               duration=25, max_depth_m=-2.0):
            mongla.pause(2.0)
            mongla.fire(3)          # dropper channel
    elif log:
        log('[demo] bin not seen — skipping to torpedo')

    # ── 3. TORPEDO (forward again) ──────────────────────────────────────────────
    # Back to forward: switch the SAME forward detector to its torpedo model (its
    # tight conf from above is still in effect). downward pauses automatically.
    mongla.use_camera('forward')
    mongla.set_model('torpedo_blood_hole', node=_FWD)
    mongla.set_classes('torpedo,blood,hole', node=_FWD)
    if mongla.wait_for('hole', timeout=10):
        # Standoff + centre + fire-from-lock (yaw dropped, heading_lock holds Ch4).
        mongla.vision.align('hole', lat=0, depth=0, fwd=55, fwd_mode='area',
                            err=15, gain=25, duration=30, hold=4.0,
                            fire=1, fire_t=1.5, brake=False, hold_heading=True)
    elif log:
        log('[demo] torpedo hole not seen')

    mongla.disarm()
