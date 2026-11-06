#!/usr/bin/env python3
"""The vision loops' tick rate and log throttle.

50 Hz, not the ~54 Hz the detector achieves: the loop should not be faster
than its own feedback, and running exactly AT the perception rate means every
tick either has a new frame or does not, with no margin -- the freshness decay
then chatters on frame jitter rather than on real loss.

The link can carry it. Measured load: telemetry 19 %, +13 % for ATTITUDE at
50 Hz; MANUAL_CONTROL at 50 Hz adds ~1500 B/s = 13 % of the 11520 B/s link.
The vision loop is nearly the only writer: the board holds heading and depth
itself, so nothing else streams.
"""

VISION_LOOP_HZ = 50.0
LOG_THROTTLE_S = 0.5    # seconds between motion-loop log heartbeats
