"""test -- a scratch bench mission, retargeted 2026-09-28.

⚠ IT USED TO STEER ON `rescue`, which no shipped model emits: the
`gate_rescue_repair` graph was retired because it measured +0.0 points of
separation between gate-present and gate-absent footage, against +84.2 at 0.30
for `gate_sharks` (measured-bars section 59). The weights are at
~/models/retired/.

Retargeted to `gate` rather than guarded like `robosub_gate_rescue`, because
this is a scratch file for exercising the verbs and the PROP it points at is
incidental -- the alignment behaviour it tests is the same.
"""
_FWD = '/mongla_detector_forward'

SEARCH_TIME = 5
def run(mongla, log=None):
    mongla.mission_reset()
    # mongla.countdown(5)
    mongla.lock_heading(0.0, timeout=600)   # BNO085 heading lock for full run
    mongla.set_mode('ALT_HOLD', timeout=10)
    mongla.arm()
    mongla.set_depth(-0.3, timeout=10)
    mongla.pause(2)
    mongla.resume_detector('forward')
    mongla.set_model('gate_sharks', node=_FWD)
    mongla.set_conf(0.60)
    mongla.set_classes('gate,shark,shaw_fish', node=_FWD)
    mongla.move_left(5, gain=30)
    mongla.move_forward(2, gain=30)
    mongla.vision.align(
        'gate', camera='forward', lat=0, lat_gain=40,hold=2,
        err=0, gain=20, duration=15)
    
    mongla.vision.anchor_snap(target='gate' , conf=0.6, err=10)
    mongla.vision.anchor_align(err=10, theta=0.05, duration=10,
                           hold=True, gain=30)
    mongla.vision.anchor_clear()

    
    mongla.vision.move(
        'gate', camera='forward', hold = 2, 
        fwd=45, mode='area',
        gain=30, duration=10)
    
    mongla.set_depth(-0.7, timeout=10)
    
    mongla.move_forward(4, gain=35)
    mongla.set_depth(-0.5, timeout=10)

    # mongla.style_roll(flips=5, headroom=0.2, gain=80)

    mongla.pause(1.0)
    mongla.release_heading()
    mongla.stop()
    mongla.disarm()
    mongla.pause_detector('forward')
    
