"""SAUVC 2026 Target Acquisition — drop the ball into the BLUE drum. 30 points.

    "There are 4 colored drums in the arena. One of the drums, chosen at random,
     will be blue in color, while the rest are red in color."  -- 60 cm diameter,
     30 cm deep. Blue 30 points, the red drum carrying the pinger 50, any other
     red 10.  https://sauvc.org/rulebook/

⛔ WE DO NOT CHASE THE 50-POINT DRUM. It is identified acoustically -- "RJE
International Pinger Model No. ULB-362B/45 kHz" -- and the rulebook gives it NO
visual marking. The `drum_red_pinger` class in sauvc_sim.yaml is separable only
because the simulator paints a yellow band on that model that no real drum has,
so a detector trained on sim imagery would confidently tag an arbitrary red drum
as the 50-pointer and drop into a 10-point one. No hydrophone is fitted. Blue is
the only drum a camera can actually tell apart, so blue is the target, and
test_sauvc_target_acquisition.py fails if any mission ever steers on the pinger.

⛔ Navigation must be completed first: "The first task, Navigation, is mandatory
and must be completed before attempting any other task." This chunk assumes the
gate is behind us -- run `sauvc_navigation` first, or a combinator that does.

Downward-camera axis remap (full table: .claude/context/downward-camera.md):
    image-X   -> Ch6 LATERAL strafe   -- align `lat`
    image-Y   -> Ch5 SURGE fore/aft   -- align `fwd`
    bbox fill -> DEPTH descent        -- align `depth` (OFF here, see config)

★ Before an armed run, VERIFY the surge sign DISARMED:
    ros2 run duburi_vision vision_thrust_check --camera downward

Run it:
    ros2 run duburi_planner mission sauvc_target_acquisition

⚠ NOT FLOWN. Guards check names, depths and budgets only.
"""

from duburi_planner.missions.competition_config import (
    SAUVC_ALIGN_GAIN,
    SAUVC_DROPPER_CHANNEL,
    SAUVC_DRUM_ALIGN_S,
    SAUVC_DRUM_CENTRE_ERR_PX,
    SAUVC_DRUM_CLASS,
    SAUVC_DRUM_DEPTH_CEILING_M,
    SAUVC_DRUM_DESCEND_FILL,
    SAUVC_DRUM_HOVER_DEPTH_M,
    SAUVC_DRUM_SETTLE_S,
    SAUVC_MAX_DEPTH_M,
    SEARCH_CREEP_S,
    SEARCH_FORWARD_GAIN,
)

_DWN = '/duburi_detector_downward'
_MODEL = 'sauvc_sim'


def run(duburi, log=None):
    duburi.mission_reset()
    acquire(duburi, log)


def acquire(duburi, log=None):
    """Hover over the blue drum and release. Assumes armed and past the gate."""
    duburi.set_depth(SAUVC_DRUM_HOVER_DEPTH_M, timeout=30)
    duburi.use_camera('downward')
    try:
        duburi.set_model(_MODEL, node=_DWN)
        duburi.set_classes(SAUVC_DRUM_CLASS, node=_DWN)
        # Depth bounds are set once per mission, not per align call. The floor is
        # the SAUVC shallow-end guard: 1.2 m of water at the pool ends, and a
        # bottom touch is -5 points per occurrence.
        duburi.set_vision_param('max_depth_m', SAUVC_MAX_DEPTH_M)
        duburi.set_vision_param('depth_ceiling', SAUVC_DRUM_DEPTH_CEILING_M)

        aligned = duburi.vision.align(
            SAUVC_DRUM_CLASS, camera='downward', lat=0, fwd=0,
            depth=(SAUVC_DRUM_DESCEND_FILL or None), fwd_mode='height',
            err=SAUVC_DRUM_CENTRE_ERR_PX, gain=SAUVC_ALIGN_GAIN,
            duration=SAUVC_DRUM_ALIGN_S, fallback=creep_forward)

        # No blind drop here, and that is deliberate -- unlike the gate, an
        # off-target release scores NOTHING rather than something, and the ball
        # is spent. Holding it costs nothing.
        if aligned:
            duburi.pause(SAUVC_DRUM_SETTLE_S)
            duburi.fire(SAUVC_DROPPER_CHANNEL)
            duburi.pause(2.0)
        elif log:
            log(f'[SAUVC] never centred over {SAUVC_DRUM_CLASS} -- ball HELD. '
                'Check the downward detector resumed and vision.surge_sign.')
    finally:
        for _p in ('max_depth_m', 'depth_ceiling'):
            try:
                duburi.set_vision_param(_p, 0.0)
            except Exception:   # noqa: BLE001 -- reset is best-effort
                pass
        duburi.use_camera('forward')


# ── Mission-authored fallback search pattern (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
