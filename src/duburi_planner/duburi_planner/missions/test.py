_FWD = '/duburi_detector_forward'

SEARCH_TIME = 5
def run(duburi, log=None):
    duburi.mission_reset()
    # duburi.countdown(5)
    duburi.lock_heading(0.0, timeout=600)   # BNO085 heading lock for full run
    duburi.set_mode('ALT_HOLD', timeout=10)
    duburi.arm()
    duburi.set_depth(-0.3, timeout=10)
    duburi.pause(2)
    duburi.resume_detector('forward')
    duburi.set_model('gate_rescue_repair', node=_FWD)
    duburi.set_conf(0.60)
    duburi.set_classes('gate,rescue,repair', node=_FWD)
    duburi.move_left(5, gain=30)
    duburi.move_forward(2, gain=30)
    duburi.vision.align(
        'rescue', camera='forward', lat=0, lat_gain=40,hold=2,
        err=0, gain=20, duration=15)
    
    duburi.vision.anchor_snap(target='rescue' , conf=0.6, err=10)
    duburi.vision.anchor_align(err=10, theta=0.05, duration=10,
                           hold=True, gain=30)
    duburi.vision.anchor_clear()

    
    duburi.vision.move(
        'rescue', camera='forward', hold = 2, 
        fwd=45, mode='area',
        gain=30, duration=10)
    
    duburi.set_depth(-0.7, timeout=10)
    
    duburi.move_forward(4, gain=35)
    duburi.set_depth(-0.5, timeout=10)

    # duburi.style_roll(flips=5, headroom=0.2, gain=80)

    duburi.pause(1.0)
    duburi.release_heading()
    duburi.stop()
    duburi.disarm()
    duburi.pause_detector('forward')
    
