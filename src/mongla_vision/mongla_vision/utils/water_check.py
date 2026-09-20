#!/usr/bin/env python3
"""Should CLAHE be on in THIS water? Decide from the pool, not from memory.

CLAHE is not universally good. Measured on real competition footage:

                        gate approach      torpedo on bin
    tracker + conf .10      53.6 %             79.5 %
    ...+ CLAHE              95.4 %  (+42)      15.3 %  (-64)

The clips separate on frame statistics, and saturation splits them harder
than blur:

                  blur (lapvar)   contrast   saturation
    gate                  315        27.7        159.9
    bin/torpedo          1241        36.1         27.9

CLAHE helps blurry, low-contrast, SATURATED (green/murky) water and hurts
sharp, desaturated water. Point the camera at the pool, run this, and it says
which side of the line you are on -- rather than discovering it mid-mission.

    ros2 run mongla_vision water_check                 # live camera topic
    python3 tools/water_check.py --video clip.mkv      # a recording
"""
import argparse
import statistics as st
import sys

import cv2
import numpy as np

# From the two measured clips. Deliberately a WIDE band in the middle where
# the honest answer is "measure it both ways" -- a threshold that pretends to
# separate two samples into a universal rule would be the overfit this file
# exists to warn about.


# The statistics and the rule live in `mongla_vision.underwater`, imported by
# the dataset survey and the preprocessing decision as well. Three copies of
# "how blurry is this frame" is how two of them come to disagree.
from mongla_vision.underwater import analyse_frames, recommend   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', default='')
    ap.add_argument('--camera', default='forward')
    ap.add_argument('--frames', type=int, default=200)
    a = ap.parse_args()

    frames = []
    if a.video:
        cap = cv2.VideoCapture(a.video)
        for _ in range(a.frames):
            ok, f = cap.read()
            if not ok:
                break
            frames.append(f)
        cap.release()
        src = a.video
    else:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        from mongla_vision import qos
        rclpy.init()
        node = Node('mongla_water_check')
        br = CvBridge()
        topic = f'/mongla/vision/{a.camera}/image_raw'
        node.create_subscription(
            Image, topic,
            lambda m: frames.append(br.imgmsg_to_cv2(m, 'bgr8')), qos.IMAGE)
        import time
        end = time.monotonic() + 20.0
        while rclpy.ok() and len(frames) < a.frames and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        src = topic

    if len(frames) < 10:
        print(f'\n  only {len(frames)} frames from {src} -- cannot judge\n')
        return 1

    st = analyse_frames(frames)
    blur, bright, contrast, sat = (st.sharpness, st.brightness,
                                   st.contrast, st.saturation)
    call, why = recommend(st)
    print(f'\n  {src}  ({len(frames)} frames)\n')
    print(f'    blur (Laplacian var) {blur:8.1f}   '
          f'(gate 315 -> CLAHE helped | bin 1241 -> CLAHE hurt)')
    print(f'    saturation           {sat:8.1f}   '
          f'(gate 160 -> CLAHE helped | bin  28 -> CLAHE hurt)')
    print(f'    contrast             {contrast:8.1f}')
    print(f'    colour cast (B-R)    {st.cast:+8.1f}   '
          f'(green/murky water is strongly positive)')
    print(f'    brightness           {bright:8.1f}')
    print(f'\n    -> {call}: {why}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
