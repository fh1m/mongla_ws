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
    mongla.set_model('gate_rescue_repair', node=_FWD)
    mongla.set_conf(0.60)
    mongla.set_classes('gate,rescue,repair', node=_FWD)
    mongla.move_left(5, gain=30)
    mongla.move_forward(2, gain=30)
    mongla.vision.align(
        'rescue', camera='forward', lat=0, lat_gain=40,hold=2,
        err=0, gain=20, duration=15)
    
    mongla.vision.anchor_snap(target='rescue' , conf=0.6, err=10)
    mongla.vision.anchor_align(err=10, theta=0.05, duration=10,
                           hold=True, gain=30)
    mongla.vision.anchor_clear()

    
    mongla.vision.move(
        'rescue', camera='forward', hold = 2, 
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
    
