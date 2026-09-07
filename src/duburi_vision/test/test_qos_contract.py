"""Every end of every vision link, checked against the rule the middleware uses.

WHY THIS FILE EXISTS
--------------------
A RELIABLE subscriber against a BEST_EFFORT publisher receives NOTHING. rclpy
logs one warning at construction and is silent afterwards, so the symptom is a
node that starts clean, reports healthy, and never gets a message.

It has happened twice on the same topic. When `image_raw` became a BEST_EFFORT
mailbox, five subscribers were updated and one was missed -- `vision_state`,
the CONTROL HOST -- whose frame counter then sat at 0 for ever. And
`preflight.wait_vision_state_ready` gated on exactly that counter, so every
mission's first vision verb burned its full 10 s timeout and reported "did not
pass within 10s", which reads as a slow pipeline rather than a QoS mismatch.

The comment warning about this was already present, in FIVE files. So the
mechanism is not another comment: it is `duburi_vision.qos`, one table both
ends import, plus these tests.

Two layers, deliberately:
  * the PAIRINGS, computed -- so a future policy change is checked, not eyeballed
  * a SOURCE SCAN -- so a new file that constructs its own QoSProfile for one of
    these topics is caught rather than quietly bypassing the table
"""
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision import qos                                    # noqa: E402

SRC = Path(__file__).resolve().parents[2]          # .../src


# --------------------------------------------------------------------------- #
#  The compatibility rule itself
# --------------------------------------------------------------------------- #
def test_best_effort_publisher_cannot_feed_a_reliable_subscriber():
    """THE bug, stated as an assertion. If this ever returns True, every other
    test in this file is meaningless."""
    assert not qos.is_compatible(qos.IMAGE, qos.DETECTIONS)


def test_a_reliable_publisher_does_feed_a_best_effort_subscriber():
    """The other direction is fine -- the publisher offers more than asked."""
    assert qos.is_compatible(qos.DETECTIONS, qos.IMAGE)


def test_a_volatile_publisher_cannot_feed_a_transient_local_subscriber():
    """Durability is the second axis, and it is the one that silently cost the
    console its class chips."""
    assert not qos.is_compatible(qos.DETECTIONS, qos.LATCHED)
    assert qos.is_compatible(qos.LATCHED, qos.DETECTIONS)


def test_an_unresolved_policy_raises_rather_than_answering_true():
    """SYSTEM_DEFAULT resolves in the middleware, not here. Answering `True`
    for something we cannot evaluate is how a checker becomes a rubber stamp."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    bad = QoSProfile(depth=1, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
    with pytest.raises(ValueError):
        qos.is_compatible(bad, qos.IMAGE)


# --------------------------------------------------------------------------- #
#  The links we actually run
# --------------------------------------------------------------------------- #
#   topic                 publisher          subscribers
_LINKS = [
    ('image_raw',      qos.IMAGE,       [qos.IMAGE]),
    ('image_debug',    qos.DEBUG_IMAGE, [qos.DEBUG_IMAGE, qos.IMAGE]),
    ('camera_info',    qos.CAMERA_INFO, [qos.CAMERA_INFO, qos.DETECTIONS, qos.IMAGE]),
    ('detections',     qos.DETECTIONS,  [qos.DETECTIONS, qos.IMAGE]),
    ('tracks',         qos.DETECTIONS,  [qos.DETECTIONS]),
    ('classes_filter', qos.LATCHED,     [qos.LATCHED, qos.DETECTIONS]),
]


@pytest.mark.parametrize('topic,pub,subs', _LINKS, ids=[l[0] for l in _LINKS])
def test_every_declared_link_delivers(topic, pub, subs):
    for sub in subs:
        assert qos.is_compatible(pub, sub), (
            f'{topic}: publisher {pub.reliability}/{pub.durability} cannot '
            f'feed subscriber {sub.reliability}/{sub.durability}')


def test_image_is_a_mailbox_not_a_queue():
    """Depth 1 is load-bearing, not a default. A full V4L2 queue was measured
    holding the OLDEST frames -- 396 ms of staleness -- because something
    treated a frame stream as a backlog. `qos_profile_sensor_data` is depth 5
    and is NOT a substitute."""
    from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
    assert qos.IMAGE.depth == 1
    assert qos.IMAGE.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_profile_sensor_data.depth != qos.IMAGE.depth


def test_calibration_is_latched():
    """A subscriber that joins after the camera must still receive K, or every
    pixel->bearing conversion silently falls back to a guessed FOV."""
    from rclpy.qos import DurabilityPolicy
    assert qos.CAMERA_INFO.durability == DurabilityPolicy.TRANSIENT_LOCAL


# --------------------------------------------------------------------------- #
#  The source scan -- catches a new file that bypasses the table
# --------------------------------------------------------------------------- #
_GUARDED = ('image_raw', 'image_debug', 'camera_info', 'detections',
            'tracks', 'classes_filter')

# Files allowed to hand-roll a profile for a guarded topic, each with a reason.
_EXEMPT = {
    # sim/replay source: a dropped frame there is a LOST SAMPLE, not a stale
    # one, so depth 5 is deliberate. Still BEST_EFFORT, so still compatible.
    'cameras/ros_topic.py',
}


def _guarded_calls(path: Path):
    """Yield (lineno, text) for create_publisher/create_subscription calls that
    name a guarded topic. Calls are matched across line breaks."""
    text = path.read_text()
    for m in re.finditer(r'create_(?:publisher|subscription)\s*\((.*?)\)\s*$',
                         text, re.S | re.M):
        call = m.group(1)
        if not any(f'/{t}' in call or f"'{t}'" in call for t in _GUARDED):
            continue
        yield text[:m.start()].count('\n') + 1, ' '.join(call.split())


def test_no_file_hand_rolls_a_qos_for_a_guarded_topic():
    offenders = []
    for path in sorted(SRC.rglob('*.py')):
        if '/test/' in str(path) or '/build/' in str(path):
            continue
        rel = str(path)
        if any(rel.endswith(e) for e in _EXEMPT):
            continue
        for lineno, call in _guarded_calls(path):
            if 'QoSProfile(' in call or 'qos_profile_sensor_data' in call:
                offenders.append(
                    f'{path.relative_to(SRC)}:{lineno}  {call[:110]}')
    assert not offenders, (
        'these construct their own QoS for a topic duburi_vision.qos owns; '
        'import the shared profile instead:\n  ' + '\n  '.join(offenders))


def test_the_control_host_does_not_subscribe_to_raw_images():
    """`vision_state` is the CONTROL host. It steers on detections and has no
    use for pixels; subscribing costs a full-frame deserialisation per message
    on the machine that owes a 50 Hz loop. It also used to be the one broken
    QoS pairing in the tree."""
    vs = SRC / 'duburi_manager' / 'duburi_manager' / 'vision_state.py'
    # Match the CALL, not the word -- the file explains this fix in a comment,
    # and a substring search on 'image_raw' fires on the explanation.
    raw = [f'{n}: {c}' for n, c in _guarded_calls(vs) if 'image_raw' in c]
    assert not raw, 'vision_state subscribes to image_raw again:\n  ' + \
        '\n  '.join(raw)


def test_the_scan_would_notice_a_new_offender(tmp_path):
    """The guard the guard needs. A source scan that matches nothing passes for
    free, which is exactly how the `_srot_drive` grep stayed green through the
    change it existed to catch."""
    bad = tmp_path / 'offender.py'
    bad.write_text(
        "node.create_subscription(\n"
        "    Image, f'{ns}/image_raw', cb,\n"
        "    QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))\n")
    found = [c for _, c in _guarded_calls(bad) if 'QoSProfile(' in c]
    assert found, 'the scan does not detect a hand-rolled QoS on a guarded topic'
