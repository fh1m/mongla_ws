"""One QoS table for the vision topics, imported by every end of every link.

WHY THIS FILE EXISTS
--------------------
A RELIABLE subscriber against a BEST_EFFORT publisher is INCOMPATIBLE. rclpy
does not raise. It logs one warning at construction and then delivers nothing,
for ever -- a node that starts clean, reports healthy, and receives silence.

That trap has now been hit twice on the same topic, and the second time the
lesson was already written down. When `image_raw` became a BEST_EFFORT mailbox
(round 30), five subscribers were updated to match and one was not:
`mongla_manager.vision_state`, the control host. Its frame counter therefore
sat at 0, and `preflight.wait_vision_state_ready` gated on that counter -- so
every mission's first vision verb burned the full 10 s preflight timeout and
blamed a "slow pipeline".

A comment in five files is not a mechanism; a shared constant is. Import from
here rather than constructing a `QoSProfile` inline, and the two ends of a
link cannot disagree about a topic they both name.

THE POLICIES, AND THE REASONING BEHIND EACH
-------------------------------------------
``IMAGE`` -- BEST_EFFORT, KEEP_LAST **depth 1**.
    A frame stream is a MAILBOX, not a queue: only the newest frame has any
    value, and the driver's own queue was measured holding the OLDEST frames
    (396 ms of staleness) precisely because something treated it as a backlog.
    Note `qos_profile_sensor_data` is NOT this -- it is BEST_EFFORT but depth
    **5**, i.e. still a five-deep queue. RELIABLE here asks the middleware to
    retransmit 691 kB frames for a consumer that discards all but the newest,
    buying latency and CPU for nothing.

``CAMERA_INFO`` -- RELIABLE, TRANSIENT_LOCAL, depth 1.
    ~1 kB of calibration that changes never, and a subscriber that joins late
    must still get it. Without it `CameraInfo.k` is empty and every
    pixel->bearing conversion silently falls back to a guessed FOV.

``DETECTIONS`` -- RELIABLE, depth 10.
    Small messages, and the readiness/coast bookkeeping downstream counts
    them. Kept RELIABLE deliberately: it is the existing behaviour of the
    control path and changing it is a measurement, not a tidy-up.

``DEBUG_IMAGE`` -- BEST_EFFORT, depth 1.
    An operator overlay. Same mailbox argument as ``IMAGE``, and it must never
    be allowed to apply back-pressure to the pipeline that produces it.

``LATCHED`` -- RELIABLE, TRANSIENT_LOCAL, depth 1.
    For announcements a late joiner must still see (`active_camera`,
    `classes_filter`).

COMPATIBILITY
-------------
`is_compatible(pub, sub)` encodes the rule the middleware applies, so a test
can assert a pairing rather than a human re-deriving it: the publisher must
offer at least what the subscriber requests, on reliability and on durability.
"""
from rclpy.qos import (QoSProfile, HistoryPolicy, ReliabilityPolicy,
                       DurabilityPolicy)

__all__ = ['IMAGE', 'CAMERA_INFO', 'DETECTIONS', 'DEBUG_IMAGE', 'LATCHED',
           'is_compatible']


IMAGE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)

DEBUG_IMAGE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)

CAMERA_INFO = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

DETECTIONS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


# Weakest-to-strongest. A publisher satisfies a subscriber when it offers a
# policy at least as strong as the one requested.
_RELIABILITY_RANK = {ReliabilityPolicy.BEST_EFFORT: 0,
                     ReliabilityPolicy.RELIABLE:    1}
_DURABILITY_RANK  = {DurabilityPolicy.VOLATILE:        0,
                     DurabilityPolicy.TRANSIENT_LOCAL: 1}


def is_compatible(pub: QoSProfile, sub: QoSProfile) -> bool:
    """True when a subscriber with `sub` will receive from a publisher `pub`.

    Depth plays no part -- it is a local queue size, not a contract, and a
    mismatch there costs latency rather than delivery.
    """
    try:
        return (_RELIABILITY_RANK[pub.reliability]
                >= _RELIABILITY_RANK[sub.reliability]
                and _DURABILITY_RANK[pub.durability]
                >= _DURABILITY_RANK[sub.durability])
    except KeyError:
        # SYSTEM_DEFAULT / UNKNOWN resolve in the middleware, not here. Say so
        # rather than answering True and being believed.
        raise ValueError(
            f'cannot compare non-concrete QoS: pub={pub.reliability}/'
            f'{pub.durability} sub={sub.reliability}/{sub.durability}')
