"""Named camera profiles. THIS DICT IS THE ONE THE CODE READS.

`config/cameras.yaml` is the operator-facing copy with the full rationale, and
Nothing loads it -- `get_profile` resolves against this dict. That is a trap
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

    # ---- Generic USB roles (no calibration declared) ------------------- #
    # Override with  device:=N  if enumeration differs.
    # ── Raspberry Pi 5 + AI HAT+ (see cameras.yaml for the full rationale) ──
    # 640x360@210 is what makes the Hailo's 70-80 Hz visible: with the 30 fps
    # 'forward' profile the ROS graph published 29.2 Hz, camera-capped.
    # 210 is a property of the MICRODIA global-shutter unit on this bench, not
    # of the Pi and not of the vehicle's cameras.
    # These two were swapped, and the fps figures belonged to each other.
    # The global-shutter Sonix is the BOTTOM camera and the Fantech is the
    # FORWARD one; the udev rules bound the names the other way round (one of
    # them to a port that matched no device at all), so `pi_forward` opened
    # the bottom camera. Corrected in kutum/system/udev/99-mongla-cameras.rules.
    #
    # `fps` and `fourcc` are MEASURED on the vehicle, not requested. Both were
    # previously absent or wrong, and `fourcc` could not be expressed at all:
    # the builder hardcoded MJPG, so a camera that is faster in another format
    # was unconfigurable.
    'pi_forward': {
        # The FANTECH, a rolling-shutter webcam.
        #
        # It delivers 15.0 fps. On Kutum on 2026-11-10, raw `v4l2-ctl
        # --stream-mmap` (no OpenCV) gave 14.97-15.06 fps at MJPG 640x360,
        # requested at 30 or 60, in auto exposure and in manual 5, 30 and
        # 100 ms. The driver accepts the request and reports 30/1 back, so
        # the reported rate is not evidence. The exposure controls are
        # accepted and read back, but they are inert: mean brightness was
        # 124.5-124.8 at every setting.
        #
        # A 2026-09-09 run measured 30.18 Hz here; that does not reproduce
        # (measured-bars "Forward camera rate"). Asking for 60 stays,
        # because the request costs nothing when the camera ignores it and
        # takes whatever a better unit offers. A camera that must deliver
        # more is the global-shutter profile below, and in the end a
        # board-triggered sensor (upstream pr-w).
        'source':      'v4l2',
        'device_path': '/dev/mongla_cam_forward',
        'width':       640,
        'height':      360,
        'fps':         60,
        'fourcc':      'MJPG',
        'frame_id':    'forward_cam',
    },
    'pi_forward_global_shutter': {
        # The Sonix global-shutter unit, on the FORWARD port, until the
        # board-triggered sensor arrives (upstream pr-w). Selected with
        # `fwd_profile:=pi_forward_global_shutter`; the Fantech stays the
        # default. udev names a Sonix on the forward port
        # /dev/mongla_cam_forward (99-mongla-cameras.rules), and it runs at
        # the downward profile's measured MJPG 640x360 @ 210.
        #
        # 14x the Fantech's frames, and no rolling-shutter skew while the
        # hull turns. Its lens is the one in pi_downward_1280x720.json, which
        # names this profile too; with only one Sonix on the vehicle, the
        # bottom camera is absent while it sits forward.
        'source':      'v4l2',
        'device_path': '/dev/mongla_cam_forward',
        'width':       640,
        'height':      360,
        'fps':         210,
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
        'device_path': '/dev/mongla_cam_downward',
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
}


def get_profile(name):
    """Return a copy of the named profile, or raise ValueError listing
    the known profiles."""
    if name not in CAMERA_PROFILES:
        known = ', '.join(sorted(CAMERA_PROFILES))
        raise ValueError(f"unknown camera profile {name!r}. known: {known}")
    return dict(CAMERA_PROFILES[name])
