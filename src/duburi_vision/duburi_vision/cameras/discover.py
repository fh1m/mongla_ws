#!/usr/bin/env python3
"""discover_cameras() -- probe connected USB cameras.

Used by camera_node when discover_on_start:=true to log a device table,
and by display_node to know which cameras are physically available.
"""
import os

import cv2


def discover_cameras(max_probe: int = 8) -> list[dict]:
    """Return [{index, path, name}] for all openable USB cameras.

    Resolves /dev/v4l/by-id/ symlinks (stable names) first, then
    probes cv2.VideoCapture(i) for indices 0..max_probe-1 as fallback.
    Only indices where VideoCapture.isOpened() returns True are included.
    """
    found: dict[int, dict] = {}

    by_id = '/dev/v4l/by-id'
    if os.path.isdir(by_id):
        for name in sorted(os.listdir(by_id)):
            try:
                real = os.path.realpath(os.path.join(by_id, name))
                idx = int(real.replace('/dev/video', ''))
            except (ValueError, OSError):
                continue
            if idx not in found:
                found[idx] = {
                    'index': idx,
                    'path': os.path.join(by_id, name),
                    'name': name,
                }

    for i in range(max_probe):
        if i in found:
            continue
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            found[i] = {'index': i, 'path': f'/dev/video{i}', 'name': f'video{i}'}
        cap.release()

    return sorted(found.values(), key=lambda d: d['index'])
