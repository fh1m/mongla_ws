"""Named camera profiles. THIS DICT IS THE ONE THE CODE READS.

`config/cameras.yaml` is the operator-facing copy with the full rationale, and
NOTHING LOADS IT -- `get_profile` resolves against this dict. That is a trap
worth stating plainly, because it has already been walked into: switching the
Pi profiles to the low-latency source by editing only the YAML changed nothing
at all, and the launch came up on the old source with no error anywhere.

So the two files must agree, and `test_camera_profiles.py` asserts they do
rather than a docstring asking politely. If you change a profile, change both;
the test tells you when you did not.
"""


CAMERA_PROFILES = {
    # ---- Dev machine (Logitech USB RGB webcam, /dev/video4) ------------- #
    # NOTE: /dev/video0,2 on this laptop are IR (Windows Hello) cameras.
    # The Logitech C-series is on /dev/video4.  Switch with device:=N at launch.
    'laptop': {
        'source':   'webcam',
        'device':   4,
        'width':    1280,
        'height':   720,
        'fps':      30,
        'frame_id': 'laptop_cam',
    },
    'logitech': {
        'source':   'webcam',
        'device':   4,
        'width':    1280,
        'height':   720,
        'fps':      30,
        'frame_id': 'laptop_cam',
    },

    # ---- Vehicle: Blue Robotics Low-Light HD USB (Jetson Orin Nano) ----- #
    # device indices assume clean Jetson USB enumeration (no IR cameras).
    # Override at pool-day with  device:=N  if enumeration differs.
    # ── Raspberry Pi 5 + AI HAT+ (see cameras.yaml for the full rationale) ──
    # 640x360@210 is what makes the Hailo's 70-80 Hz visible: with the 30 fps
    # 'forward' profile the ROS graph published 29.2 Hz, camera-capped.
    # 210 is a property of the MICRODIA global-shutter unit on this bench, not
    # of the Pi and not of the vehicle's cameras.
    # ⛔ THESE TWO WERE SWAPPED, and the fps figures belonged to each other.
    # The global-shutter Sonix is the BOTTOM camera and the Fantech is the
    # FORWARD one; the udev rules bound the names the other way round (one of
    # them to a port that matched no device at all), so `pi_forward` opened
    # the bottom camera. Corrected in tools/udev/99-duburi-cameras.rules.
    #
    # `fps` and `fourcc` are MEASURED on the vehicle, not requested. Both were
    # previously absent or wrong, and `fourcc` could not be expressed at all:
    # the builder hardcoded MJPG, so a camera that is faster in another format
    # was unconfigurable.
    'pi_forward': {
        # The FANTECH.
        #
        # ⛔ RETRACTED 2026-09-09: "a flat 15.00 Hz ... this is the CAMERA's
        # ceiling ... nothing in software will raise it." The 15 was THIS
        # LINE, and the comment then defended it. Re-measured on the vehicle
        # by counting DISTINCT header stamps -- a topic `hz` cannot tell a
        # real frame from a republished one, which is how a self-imposed cap
        # reads as a hardware limit:
        #
        #   requested 15 -> 14.63 Hz     requested 60 -> 30.18 Hz
        #   requested 30 -> 28.03 Hz     requested 90 -> 30.18 Hz
        #
        # Identity checked against the calibration's recorded USB VID/PID and
        # serial (1d6c:0103, YGR80PU1200F23081120) so this is the Fantech
        # answering and not the Sonix -- the round-38 trap.
        #
        # End to end (camera+detector+tracker) it is worth ~2x of DETECTIONS,
        # which is what the vision loop steers on, for ~6 points of one core:
        #
        #   fps 15 -> image 14.67 Hz, detections 14.67 Hz, CPU 95.5 % idle
        #   fps 60 -> image 30.03 Hz, detections 27.11 Hz, CPU 89.3 % idle
        #   (memory 683 vs 680 MB of 3983)
        #
        # 60 rather than 30 because the request saturates at the ceiling:
        # asking 30 returns 28.03, asking 60 returns the full 30.18.
        'source':      'v4l2',
        'device_path': '/dev/duburi_cam_forward',
        'width':       640,
        'height':      360,
        'fps':         60,
        'fourcc':      'MJPG',
        'frame_id':    'forward_cam',
    },
    'pi_downward': {
        # The SONIX GLOBAL SHUTTER -- the optical-flow velocity sensor, which
        # is why it is the one that matters most here. Measured: MJPG 640x360
        # 210.17 Hz, 640x400 210.21, YUYV a flat 35.26. So MJPG is not a
        # preference, it is 6x, and the format has to be per-camera because
        # the forward unit is indifferent to it.
        #
        # A global shutter is the right sensor for flow for a reason no frame
        # rate captures: a rolling shutter skews the image while the hull
        # moves, which corrupts the displacement flow exists to measure.
        #
        # 640x400 also runs at 210 and gives 40 more rows of floor texture at
        # no cost. Kept at 360 because every calibration and bench number we
        # hold was taken there; switching is a measured upgrade, not a free
        # one (fx is unchanged at the same width, cy is not).
        'source':      'v4l2',
        'device_path': '/dev/duburi_cam_downward',
        'width':       640,
        'height':      360,
        'fps':         210,
        'fourcc':      'MJPG',
        'frame_id':    'downward_cam',
    },

    # Advertised by cameras.yaml and MISSING here, so `camera:=auto` raised
    # 'unknown profile' from the only table that is loaded. Found by
    # test_camera_profiles, not by anyone using it.
    'auto': {
        'source':   'webcam',
        'device':   0,
        'width':    640,
        'height':   480,
        'fps':      30,
        'frame_id': 'auto_cam',
    },

    'forward': {
        'source':   'webcam',
        'device':   0,
        'width':    640,
        'height':   480,
        'fps':      30,
        'frame_id': 'forward_cam',
    },
    'downward': {
        'source':   'webcam',
        'device':   2,
        'width':    640,
        'height':   480,
        'fps':      30,
        'frame_id': 'downward_cam',
    },

    # ---- Gazebo SITL: forward / down -- topics depend on the world ----- #
    # The bluerov2_gz model used today does NOT ship a camera plugin; you
    # need to add a <sensor type="camera"> + <plugin filename="ignition-..."/>
    # block to model.sdf. Until then point this at whatever Image topic is
    # available (e.g. a re-published webcam over a ros_gz bridge).
    'sim_front': {
        'source':         'ros_topic',
        'topic':          '/duburi/sim/front_camera/image_raw',
        'expected_width': 640,
        'expected_height': 480,
        'expected_fps':    30,
        'frame_id':       'front_cam',
    },
    'sim_bottom': {
        'source':         'ros_topic',
        'topic':          '/duburi/sim/bottom_camera/image_raw',
        'expected_width': 640,
        'expected_height': 480,
        'expected_fps':    30,
        'frame_id':       'bottom_cam',
    },

    # ---- Future: on-vehicle (raise NotImplementedError today) ---------- #
    'jetson_front':  {'source': 'jetson'},
    'jetson_bottom': {'source': 'jetson'},
    'blueos':        {'source': 'blueos'},
    'mavlink':       {'source': 'mavlink'},
}


def get_profile(name):
    """Return a copy of the named profile, or raise ValueError listing
    the known profiles."""
    if name not in CAMERA_PROFILES:
        known = ', '.join(sorted(CAMERA_PROFILES))
        raise ValueError(f"unknown camera profile {name!r}. known: {known}")
    return dict(CAMERA_PROFILES[name])
