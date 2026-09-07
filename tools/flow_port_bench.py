#!/usr/bin/env python3
"""Drive the flow node's WATER path with synthetic motion through REAL optics.

⛔ WHAT THIS DOES AND DOES NOT PROVE. Every velocity number in
`measured-bars.md` §13 -- the 30 cm slides, the 0.02 cm synthetic control --
was taken with `medium='air'` and PREDATES the refractive rectifier. The
rectified path has been verified to come up on the vehicle and its maths is
verified against a true pinhole and the n=1 identity, but it had never
produced a measured velocity of any kind. This closes that, as far as a dry
room allows.

It proves: the node's inverse (undistort -> rectify -> planar fit -> scale by
h/f_ref) recovers a known camera translation from a real, textured frame
taken through the real lens, across the whole field -- including the 7.1 px
of centre-to-corner non-uniformity and the 12.8 px of cross-axis motion a
30 cm slide induces at h = 0.70 m. Those are far above the 2 px
forward-backward gate, so a wrong model cannot hide in them.

It does NOT prove the physical port obeys Snell's law with a thin-window
approximation. Port thickness, a non-planar window and any tilt are absent
from both sides of this test. **That is water-only** and stays owed.

THE FORWARD MODEL IS INDEPENDENT OF THE THING UNDER TEST, deliberately. It
is written here as ray trig on ground-plane geometry -- pick the ray, refract
it into water, intersect the floor, translate the floor point, re-project --
and never imports `RefractiveRectifier`, which works in normalised-coordinate
scale factors. Two implementations of one physics, meeting only at the answer.

And there is a NEGATIVE CONTROL, because without one this measures whether
the rig can talk to itself. Arm `pinhole` warps with n = 1 and runs the node
in air: any error there is the harness -- the warp, the feed, the
accumulation -- not the refraction model. Only if that arm is clean does the
water arm mean anything.

Usage (on the vehicle, downward camera, lit scene):
    python3 tools/flow_port_bench.py --steps 30 --step-cm 1.0 --height 0.70
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np


def build_map(w, h_px, fx, fy, cx, cy, n, height_m, dx_m, dy_m):
    """Pixel remap for a camera translation of (dx, dy) metres at `height_m`.

    Returns `map_x, map_y` for cv2.remap, i.e. for each pixel of the OUTPUT
    (post-move) frame, where to sample the INPUT (pre-move) frame.

    Straight geometry, no shortcuts:
      pixel -> normalised -> ray angle in AIR (this is what the lens sees)
      -> Snell into WATER -> ground point at `height_m`
      -> translate the ground point by -(dx, dy)   [camera moved +(dx,dy)]
      -> back through Snell -> back to a pixel.

    n = 1.0 makes this the plain pinhole case, which is the negative control.
    """
    ys, xs = np.mgrid[0:h_px, 0:w].astype(np.float64)
    xn = (xs - cx) / fx
    yn = (ys - cy) / fy
    rn = np.hypot(xn, yn)
    ang = np.arctan2(yn, xn)

    ta = np.arctan(rn)                                  # ray angle in air
    sin_tw = np.clip(np.sin(ta) / n, -1.0, 1.0)
    tw = np.arcsin(sin_tw)                              # refracted into water
    r_ground = height_m * np.tan(tw)

    X = r_ground * np.cos(ang) + dx_m                   # sample the PRE-move
    Y = r_ground * np.sin(ang) + dy_m                   # scene point
    r2 = np.hypot(X, Y)
    ang2 = np.arctan2(Y, X)

    tw2 = np.arctan2(r2, height_m)
    sin_ta2 = np.clip(np.sin(tw2) * n, -1.0, 1.0)
    ta2 = np.arcsin(sin_ta2)
    rn2 = np.tan(ta2)

    map_x = (rn2 * np.cos(ang2) * fx + cx).astype(np.float32)
    map_y = (rn2 * np.sin(ang2) * fy + cy).astype(np.float32)
    return map_x, map_y


def grab(device, w, h, warm=12):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f'cannot open {device}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    frame = None
    for _ in range(warm):
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        sys.exit('no frame')
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if g.shape != (h, w):
        g = cv2.resize(g, (w, h))
    return g


def run_arm(label, gray, cal, medium, rectify, n_warp, steps, step_m, height):
    """One arm: warp with `n_warp`, measure with the node in `medium`."""
    import rclpy
    from rclpy.parameter import Parameter
    from duburi_vision.distance.flow_node import FlowVelocityNode

    raw = json.load(open(cal))
    s = gray.shape[1] / raw['image_width']
    K = np.array(raw['camera_matrix'], float)
    fx, fy = K[0, 0] * s, K[1, 1] * s
    cx, cy = K[0, 2] * s, K[1, 2] * s

    overrides = [
        Parameter('calibration', value=cal),
        Parameter('medium', value=medium),
        Parameter('refractive_rectify', value=rectify),
        Parameter('pool_depth_m', value=float(height)),
        Parameter('estimate_time_offset', value=False),
    ]
    # The node takes no parameter_overrides kwarg, so inject them the way
    # test_flow_node does: patch Node.__init__ for the one construction. That
    # keeps this driving the SHIPPED constructor rather than a variant of it.
    import rclpy.node
    orig_init = rclpy.node.Node.__init__

    def patched(self, name, **kw):
        kw['parameter_overrides'] = list(kw.get('parameter_overrides', [])) \
            + overrides
        orig_init(self, name, **kw)

    rclpy.node.Node.__init__ = patched
    try:
        node = FlowVelocityNode()
    finally:
        rclpy.node.Node.__init__ = orig_init
    try:
        # Height comes from pool_depth - depth; depth 0 puts the floor at
        # `height` below, which is what the warp assumed.
        node._depth_m = 0.0
        refusals = []
        orig = node._refuse
        node._refuse = lambda why: (refusals.append(why), orig(why))[1]

        node._process(gray, 0.0, 0)
        for i in range(1, steps + 1):
            mx, my = build_map(gray.shape[1], gray.shape[0], fx, fy, cx, cy,
                               n_warp, height, i * step_m, 0.0)
            warped = cv2.remap(gray, mx, my, cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REFLECT)
            node._process(warped, i * 0.05, i)

        truth = steps * step_m
        got = node._acc.distance_m
        print(f'  {label:26s} truth {truth*100:6.2f} cm   '
              f'got {got*100:7.2f} cm   {100*got/truth:6.1f} %   '
              f'({node._n_ok} used / {node._n_refused} refused)')
        if refusals:
            from collections import Counter
            for why, k in Counter(refusals).most_common(2):
                print(f'      refused x{k}: {why}')
        return 100 * got / truth
    finally:
        node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/duburi_cam_downward')
    ap.add_argument('--calibration', default=os.path.expanduser(
        '~/duburi_ws/src/duburi_vision/config/calibration/'
        'pi_downward_1280x720.json'))
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height-px', type=int, default=360)
    ap.add_argument('--height', type=float, default=0.70)
    ap.add_argument('--steps', type=int, default=30)
    ap.add_argument('--step-cm', type=float, default=1.0)
    a = ap.parse_args()

    import rclpy
    rclpy.init()
    try:
        gray = grab(a.device, a.width, a.height_px)
        print(f'frame {gray.shape} mean={gray.mean():.1f} sd={gray.std():.1f}')
        if gray.std() < 8:
            print('  ⚠ almost no texture -- this measures the dark room, '
                  'not the model')
        step = a.step_cm / 100.0
        print(f'\n{a.steps} steps x {a.step_cm} cm at h = {a.height} m\n')

        print('NEGATIVE CONTROL -- pinhole warp (n=1), node in air.')
        print('  Any error here is the HARNESS, not refraction.')
        ctrl = run_arm('pinhole / air', gray, a.calibration, 'air', False,
                       1.0, a.steps, step, a.height)

        print('\nEXPERIMENT -- flat-port warp (n=1.333), node in water.')
        rec = run_arm('port / water RECTIFIED', gray, a.calibration, 'water',
                      True, 1.333, a.steps, step, a.height)
        raw = run_arm('port / water single-f', gray, a.calibration, 'water',
                      False, 1.333, a.steps, step, a.height)

        print('\n--- verdict ---')
        if abs(ctrl - 100) > 3:
            print(f'  CONTROL FAILED at {ctrl:.1f} %. The harness is wrong; '
                  f'the water arms below say nothing.')
        else:
            print(f'  control clean ({ctrl:.1f} %), so the water arms mean '
                  f'something.')
            print(f'  rectified {rec:.1f} %  vs  single-f {raw:.1f} %  '
                  f'-- the single-f arm carries the error the rectifier '
                  f'removes.')
        print('\n  NOT PROVEN HERE: that the physical port obeys this model. '
              'Port\n  thickness and tilt are in neither side of this test. '
              'Water only.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
