"""When was this actually SEEN? One implementation, every consumer.

THE DEFECT THIS EXISTS TO STOP, which this codebase has now shipped four
times: a message carries the instant its data was captured, and the code that
needs that instant reads a local clock instead.

    camera_node      header.stamp = now() at PUBLISH        (round 30)
    vision_state     _latest_stamp = monotonic() on ARRIVAL (round 32)
    srot_replay      a little-endian timestamp round-trip    (round 27)
    lock_node        _det_t = monotonic() on ARRIVAL         (this one)

Every instance returned a plausible number, none raised, and each one made a
freshness gate measure something other than freshness. That is the signature:
**a clock read at the wrong place does not fail, it flatters.** The age comes
out too small, which is always the direction that makes the vehicle act with
more confidence than it has earned.

TWO CLOCK DOMAINS, AND MIXING THEM IS THE SECOND HALF OF THE BUG.
`header.stamp` is WALL time -- it has to be, because it crosses hosts, and the
monotonic clock is meaningless off the machine that read it. Every freshness
gate in the stack is MONOTONIC, because wall time can step. Both offsets are
read at the same instant here so the difference between them is the conversion
and nothing else.

FAIL SAFE, NOT FAIL SILENT. A stamp that is absent, in the future, or absurdly
old means a publisher we do not understand -- `use_sim_time`, an NTP step, a
node still stamping `now()`. Arrival time is then the HONEST answer, and the
caller is told so it can say it once. The alternative is handing a control loop
a negative age, which every gate in the stack reads as maximally fresh.
"""
from __future__ import annotations

import time
from typing import Tuple

# Bounds on a stamp interpreted as wall time. Outside these we do not trust it.
SKEW_TOL_S = 0.5    # tolerated clock lead of the source host over this one
MAX_AGE_S = 5.0     # older than this is a stopped clock, not a slow frame


def capture_monotonic(header, *, skew_tol_s: float = SKEW_TOL_S,
                      max_age_s: float = MAX_AGE_S) -> Tuple[float, str]:
    """`(instant on THIS host's monotonic clock, '' | why it fell back)`.

    The reason string is returned rather than logged because this module has no
    logger and should not acquire one: the same function runs inside a node, a
    bag replay and a bench, and each wants to report differently. An empty
    string means the stamp was used.
    """
    now_mono = time.monotonic()
    try:
        stamp_wall = float(header.stamp.sec) + header.stamp.nanosec * 1e-9
    except AttributeError:
        # Covers BOTH a message with no header and a header with no stamp --
        # `None.stamp` and `header.stamp` missing raise the same thing, and a
        # callback that dies here takes the whole subscription with it.
        return now_mono, 'no header.stamp'
    if stamp_wall <= 0.0:
        return now_mono, 'header.stamp is zero (nothing stamped it)'
    age = time.time() - stamp_wall
    if not (-skew_tol_s <= age <= max_age_s):
        return now_mono, (f'stamp is {age:+.3f}s from wall clock, outside '
                          f'[{-skew_tol_s:+.1f}, {max_age_s:.1f}]s')
    # Clamp: a stamp a hair in the future (sub-millisecond disagreement between
    # two hosts' `time.time()`) must not come back as a NEGATIVE age.
    return now_mono - max(age, 0.0), ''
