#!/usr/bin/env python3
"""Which water is this? Characterise the pool before trusting a model in it.

⛔ THIS TOOL USED TO ANSWER "SHOULD CLAHE BE ON?" AND IT WAS BROKEN FOR BOTH
REASONS AT ONCE. The answer itself was wrong: the CLAHE rule was fitted to two
clips, and re-measured on raw detection rate across 17 configurations -- 4
props, 3 venues, a 39x sharpness range -- preprocessing was NEVER positive, and
on the gate it took detection from 30.4 % to 1.2 %. So `underwater.recommend()`
was deleted, deliberately and with the reasoning written where it used to live.

This file kept importing it. `ros2 run mongla_vision water_check` therefore died
with `ImportError: cannot import name 'recommend'` before parsing an argument --
a console entry point in `setup.py`, dead on arrival, and nothing noticed
because no test imports an entry point. `test_every_entry_point_imports.py` now
does.

WHAT IT REPORTS, WHICH IS A FACT ABOUT THE POOL AND REPRODUCED ACROSS ALL THREE
VENUES:

                  blur (lapvar)   contrast   saturation   cast (B-R)
    gate                  315        27.7        159.9
    bin / torpedo        1241        36.1         27.9
    Mirpur                 33         ----       148        +92

That is which optical regime you are in -- which decides whether a model
trained on one of them is being asked about the other, and which venue a
held-out split is actually holding out. What to DO about the water is not
something this tool claims to know: the models were trained on UNPROCESSED
underwater frames, so any preprocessing that makes an image look better to a
person moves it away from the distribution the detector learned.

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


# The statistics live in `mongla_vision.underwater`, imported by the footage
# inventory as well. Three copies of "how blurry is this frame" is how two of
# them come to disagree. There is no `recommend` to import -- see the header.
from mongla_vision.underwater import analyse_frames               # noqa: E402


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
    print(f'\n  {src}  ({len(frames)} frames)\n')
    print(f'    blur (Laplacian var) {blur:8.1f}   '
          f'(gate 315 | bin 1241 | Mirpur 33)')
    print(f'    saturation           {sat:8.1f}   '
          f'(gate 160 | bin   28 | Mirpur 148)')
    print(f'    contrast             {contrast:8.1f}')
    print(f'    colour cast (B-R)    {st.cast:+8.1f}   '
          f'(green/murky water is strongly positive)')
    print(f'    brightness           {bright:8.1f}')
    # ⚠ NO VERDICT LINE, AND NO "NEAREST REGIME" EITHER. The first rewrite of
    # this function scored the frame against the three reference rows with a
    # log-ratio distance -- which is a new metric fitted to three points, the
    # exact move that produced `recommend()`. Each line above already prints
    # the references beside the measurement; the reader can see where they sit
    # without this file inventing a number to say it.
    print('\n    preprocessing is OFF on this vehicle and measured never to '
          'help -- see mongla_vision/underwater.py\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
