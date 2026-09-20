"""WebcamCamera — local USB / built-in webcam via cv2.VideoCapture.

Robust to:
  * device index OR device path ("/dev/video0") — both work
  * driver returning False on the very first read (cold start)
  * resolution requests the driver silently downgrades — we read back
    what the driver actually gave us so CameraInfo never lies
  * `cap.read()` blocking briefly (no threading; the camera_node owns the
    timer and we honor the timer cadence by returning whatever's ready)
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import subprocess

import numpy as np

from .camera import Camera, FrameMeta


def blur_capped_exposure(exposure_us, max_blur_px, f_px, max_rate_rad_s):
    """Clamp a requested shutter to what the expected rotation allows.

    Blur from rotation is `f_px * omega * t_exp`, so the longest shutter that
    keeps it under `max_blur_px` is `max_blur_px / (f_px * omega)`. Returned
    in units of 0.1 ms, which is what V4L2's `exposure_time_absolute` wants.

    Module-level and pure ON PURPOSE: the arithmetic is the part that is easy
    to get silently wrong, and a test must be able to drive THIS function
    rather than a restatement of it. A first version of the test reimplemented
    the rule and consequently passed while the shipped cap was broken --
    exactly the defect that survived ten green tests in round 33.

    It only ever TIGHTENS. A cap that could also lengthen the shutter would
    introduce blur nobody asked for, under a name that promises the opposite.
    """
    exp = int(exposure_us)
    if max_blur_px > 0 and f_px > 0 and max_rate_rad_s > 0:
        cap = int(1e4 * max_blur_px / (f_px * max_rate_rad_s))
        if cap < exp:
            exp = max(1, cap)
    return exp


class WebcamCamera(Camera):
    source_kind = 'webcam'

    def __init__(self, device=0, width=640, height=480, fps=30,
                 frame_id='laptop_cam', name='laptop', logger=None,
                 fourcc='MJPG', exposure_us=0, brightness=None,
                 max_blur_px=0.0, f_px=0.0, max_rate_rad_s=0.0):
        self.name      = str(name)
        self._device   = device
        self._frame_id = str(frame_id)
        self._req      = (int(width), int(height), int(fps))
        self._log      = logger

        self._cap         = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"webcam: cv2.VideoCapture({device!r}) failed to open. "
                f"On the Jetson the USB cams do NOT sit at the profile's default int "
                f"index -- pass the PORT-STABLE by-path symlink instead: "
                f"device_path:=/dev/mongla_cam_<forward|downward> (or a raw "
                f"/dev/v4l/by-path/...). Otherwise check /dev/video* perms / index.")

        # Set the format before resolution/fps -- V4L2 locks format first.
        #
        # MJPG is the default because most USB webcams otherwise deliver YUYV
        # at ~1-2 fps on USB 2.0. But it is a PARAMETER, not a constant, and
        # the reason is measured: FORMAT IS PER-CAMERA. On this vehicle the
        # Sonix global shutter does 210.17 Hz in MJPG against a flat 35.26 in
        # YUYV -- 6x -- while the Fantech returns exactly 15.00 Hz in both.
        # Hardcoding MJPG here made a faster-in-YUYV camera unconfigurable,
        # and the profile could not say otherwise because this argument did
        # not exist.
        self._cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*str(fourcc)))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)        # avoid stale frames
        self._set_exposure(exposure_us, brightness, max_blur_px, f_px,
                           max_rate_rad_s)

        self._actual_w   = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)  or width)
        self._actual_h   = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or height)
        self._actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS)        or fps)
        fourcc_int = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join(chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4))

        if self._log:
            self._log.info(
                f'[CAM  ] webcam {device!r} opened: requested {width}x{height}@{fps} '
                f'{fourcc} -> got {self._actual_w}x{self._actual_h}@'
                f'{self._actual_fps:.1f} fourcc={fourcc_str}')
            if fourcc_str.strip() and fourcc_str != str(fourcc):
                self._log.warn(
                    f'[CAM  ] {device!r} ignored fourcc={fourcc} and gave '
                    f'{fourcc_str}. The driver fell back; frame rate below is '
                    f'the fallback format\'s, not the one requested.')
            # A camera that quietly delivers a fraction of what was asked is the
            # most expensive kind of silent underperformance here, because
            # nothing downstream looks broken -- the pipeline is simply slower
            # than its own accelerator. Measured: the Hailo backend runs at
            # 70-80 Hz while the ROS graph published 29.2 Hz, purely because the
            # profile asked for 30 fps. This line is the difference between
            # noticing that in a log and finding it with a stopwatch.
            if self._actual_fps < 0.6 * float(fps):
                self._log.warn(
                    f'[CAM  ] {device!r} is delivering {self._actual_fps:.1f} fps '
                    f'against {fps} requested. The mode may not exist at this '
                    f'resolution/fourcc -- check `v4l2-ctl -d <dev> '
                    f'--list-formats-ext`. Perception is capped here, not at '
                    f'the detector.')

        self._idx          = 0
        self._last_ok      = 0.0
        self._consec_fail  = 0

    def _set_exposure(self, exposure_us, brightness, max_blur_px, f_px,
                      max_rate_rad_s):
        """Pin the shutter, because AUTO CHOOSES BLUR.

        ⛔ MEASURED ON THIS VEHICLE. Nothing here touched exposure, so both
        cameras ran on whatever the driver picked. On the forward Fantech
        that was Aperture Priority with `exposure_time_absolute = 2000` -- a
        **200 ms shutter**. Frames came out at mean 26.6 with every hand
        movement smeared; manual exp 50 (5 ms) with brightness 150 gave mean
        135.0 at 6.5 % clipping. Same room, same lens, 40x shorter shutter.

        A long shutter is not merely dim-looking: blur from rotation is
        `f_px * omega * t_exp`, so at f = 514 and 0.64 rad/s a 200 ms
        exposure smears **66 px**. No detector survives that, and the failure
        presents as "the detector is bad" -- which is how it was reported.

        `max_blur_px` inverts that relation into a CAP, which is the
        motion-blur-aware half of Han et al. (IEEE/ASME T-Mech 2023): pick an
        exposure for image quality, then clamp it to what the expected motion
        allows. They estimate motion from optical flow; this takes the
        vehicle's expected `max_rate_rad_s`, because a static bound needs no
        feedback loop and so cannot lag the motion it is bounding.

        ⚠ GAIN IS DELIBERATELY NOT OFFERED, and that is measured: on the
        Fantech, gain 20 / 50 / 100 give identical frames. It is inert on
        this unit, so exposing it would be a knob that does nothing -- this
        package already has four of those on its record.

        v4l2-ctl rather than cv2 properties, AFTER the format is set, because
        the ordering is load-bearing: cv2 resets the controls when it
        configures the stream, and `exposure_time_absolute` is silently
        ignored while auto exposure is engaged.
        """
        if not exposure_us:
            return                                    # 0 = leave on auto
        exp = blur_capped_exposure(exposure_us, max_blur_px, f_px,
                                   max_rate_rad_s)
        dev = (self._device if isinstance(self._device, str)
               else f'/dev/video{self._device}')
        applied = []
        for key, val in (('auto_exposure', 1),
                         ('exposure_time_absolute', exp),
                         ('brightness', brightness)):
            if val is None:
                continue
            r = subprocess.run(['v4l2-ctl', '-d', dev, '-c', f'{key}={val}'],
                               capture_output=True)
            applied.append(f'{key}={val}'
                           + ('' if r.returncode == 0 else ' FAILED'))
        if self._log:
            extra = (f'  (blur cap {max_blur_px:.1f}px at '
                     f'{max_rate_rad_s:.2f}rad/s -> {exp})'
                     if exp != int(exposure_us) else '')
            self._log.info(f'[CAM  ] {self.name}: exposure pinned -- '
                           + ', '.join(applied) + extra)

    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        ok, frame = self._cap.read()
        meta = FrameMeta(
            frame_index=self._idx,
            width=self._actual_w,
            height=self._actual_h,
        )
        if not ok or frame is None:
            self._consec_fail += 1
            meta.fresh = False
            return None, meta

        self._consec_fail = 0
        self._last_ok     = meta.stamp_monotonic
        self._idx        += 1
        meta.fresh        = True
        return frame, meta

    def is_healthy(self) -> bool:
        return (self._cap.isOpened()
                and self._consec_fail < 30
                and (time.monotonic() - self._last_ok) < 2.0)

    def info(self) -> dict:
        return {
            'name':        self.name,
            'source_kind': self.source_kind,
            'width':       self._actual_w,
            'height':      self._actual_h,
            'fps':         self._actual_fps,
            'frame_id':    self._frame_id,
            'device':      self._device,
        }

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
