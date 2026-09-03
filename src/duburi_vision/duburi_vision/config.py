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
    'pi_forward': {
        # `v4l2`, not `webcam`: a keep-up thread and a one-deep mailbox instead
        # of OpenCV's queue. Measured after a 400 ms consumer stall, against
        # the kernel's own capture timestamps: a plain read() hands you a
        # 396 ms-old frame, the standard "drain the queue" recipe 348 ms, this
        # 17 ms. The driver keeps the OLDEST frames when its buffers fill, so
        # draining empties a fossil record -- cameras/v4l2_mailbox.py.
        'source':      'v4l2',
                # `/dev/duburi_cam_*`, from tools/udev/99-duburi-cameras.rules.
        # NOT `/dev/v4l/by-path/...`: those ID_PATH values are correct --
        # udevadm reports exactly them -- but Raspberry Pi OS creates only
        # `by-id` for USB video and never `by-path`. So this named a symlink
        # the distro does not make, fell through to the integer index, and
        # BOTH profiles resolved to /dev/video0: whichever node started
        # first won and the other died EBUSY. The rule also earned itself
        # immediately -- video0 was the Sonix before a reboot and the
        # Fantech after, so the raw index had already swapped the cameras.
        'device_path': '/dev/duburi_cam_forward',
        'width':       640,
        'height':      360,
        'fps':         210,
        'frame_id':    'forward_cam',
    },
    'pi_downward': {
        'source':      'v4l2',
                'device_path': '/dev/duburi_cam_downward',
        'width':       640,
        'height':      360,
        'fps':         90,
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
