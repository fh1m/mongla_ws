#!/usr/bin/env python3
"""vision_check -- standalone health probe for the perception pipeline.

Run this BEFORE any vision_* mission so a missing camera or stalled
detector_node can't pretend everything's fine until thrust time.

What it tests (all topic-based, no model load):
  * /duburi/vision/<cam>/image_raw       -- frames per second
  * /duburi/vision/<cam>/camera_info     -- presence + image dims
  * /duburi/vision/<cam>/detections      -- frequency + class labels seen

Exits 0 when all three streams meet the requested thresholds, 1 otherwise.

Examples
--------
ros2 run duburi_vision vision_check
ros2 run duburi_vision vision_check --camera laptop --duration 5
ros2 run duburi_vision vision_check --camera sim_front --require-class person
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


def main(argv=None):
    parser = argparse.ArgumentParser(description='Vision pipeline smoke test.')
    parser.add_argument('--camera',         default='laptop',
                        help='camera namespace under /duburi/vision/ (default: laptop)')
    parser.add_argument('--duration',       type=float, default=5.0,
                        help='measurement window seconds (default: 5)')
    parser.add_argument('--min-image-hz',   type=float, default=10.0,
                        help='minimum image_raw rate (Hz) required (default: 10)')
    parser.add_argument('--min-det-hz',     type=float, default=2.0,
                        help='minimum detections rate (Hz) required (default: 2)')
    parser.add_argument('--require-class',  default='',
                        help='if set, fail unless this class label was seen at least once')
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = Node('vision_check')
    ns   = f'/duburi/vision/{args.camera}'

    state = {
        'image_count':  0,
        'det_count':    0,
        'classes':      Counter(),
        'image_size':   (0, 0),
        'info_seen':    False,
    }
    qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

    def _on_image(_msg):
        state['image_count'] += 1

    def _on_info(msg):
        state['info_seen']  = True
        state['image_size'] = (int(msg.width), int(msg.height))

    def _on_det(msg):
        state['det_count'] += 1
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            if hasattr(hyp, 'hypothesis'):
                cid = str(hyp.hypothesis.class_id)
            else:
                cid = str(getattr(hyp, 'id', ''))
            if cid:
                state['classes'][cid] += 1

    node.create_subscription(Image,            f'{ns}/image_raw',   _on_image, qos)
    node.create_subscription(CameraInfo,       f'{ns}/camera_info', _on_info,  qos)
    node.create_subscription(Detection2DArray, f'{ns}/detections',  _on_det,   qos)

    print(f'[VCHK] watching {ns} for {args.duration:.1f}s ...')
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    elapsed = max(args.duration, 1e-3)
    image_hz = state['image_count'] / elapsed
    det_hz   = state['det_count']   / elapsed

    rows = [
        ('camera_info present',         state['info_seen']),
        (f'image_raw   >= {args.min_image_hz:.1f} Hz',
         image_hz >= args.min_image_hz),
        (f'detections  >= {args.min_det_hz:.1f} Hz',
         det_hz >= args.min_det_hz),
    ]
    if args.require_class:
        rows.append(
            (f'class {args.require_class!r} seen',
             state['classes'].get(args.require_class, 0) > 0))

    width = max(len(label) for label, _ in rows) + 2
    print()
    print(f'  {"check":<{width}} {"value":>10}   pass')
    print('  ' + '-' * (width + 22))
    print(f'  {"image_size":<{width}} {state["image_size"]!s:>10}')
    print(f'  {"image_raw rate":<{width}} {image_hz:>9.2f}Hz')
    print(f'  {"detections rate":<{width}} {det_hz:>9.2f}Hz')
    if state['classes']:
        top = ', '.join(f'{c}:{n}' for c, n in state['classes'].most_common(5))
        print(f'  {"classes seen":<{width}} {top}')
    else:
        print(f'  {"classes seen":<{width}} (none)')
    print()

    all_ok = True
    for label, ok in rows:
        marker = 'PASS' if ok else 'FAIL'
        print(f'  [{marker}] {label}')
        all_ok = all_ok and ok

    print()
    print(f'[VCHK] result: {"PASS" if all_ok else "FAIL"}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
