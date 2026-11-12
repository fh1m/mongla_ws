"""A published topic nobody reads, and a subscription nobody feeds.

The third sweep, and the last one that was not enforced. Three of the four
reachability sweeps already fail the build: modules
(`test_no_capability_is_built_and_unreachable`), functions
(`test_no_function_is_built_and_uncalled`) and shipped constants
(`test_shipped_constants_match_the_ledger`). Topics were a report nobody had to
read, and reports get read once.

This cannot be a zero-tolerance check, and pretending otherwise would make it
useless. A published topic with no in-graph subscriber is often a DIAGNOSTIC —
the CLI, Foxglove, a bag and an operator are all real consumers the scanner
cannot see. A subscription with no in-graph publisher is usually fed by the
BOARD or a driver. So the bar is not "zero", it is **"no new ones"**: the
known set is listed here with the consumer or producer named, and anything
outside it fails.

That is exactly how the last real finding surfaced. `/target_pose_fused`
sat on the published-and-unread list three separate times and was written up
as "genuinely unread" in a tool comment — until someone checked and found
`MonglaMission._wait_fused_pose` subscribing to it through a VARIABLE, which
the regex could not see. The sweep was wrong, the list was stale, and nothing
forced the recheck.
"""
from __future__ import annotations

import pathlib
import re
import subprocess
import sys

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_TOOL = _ROOT / 'tools' / 'ci' / 'topic_wiring_sweep.py'

# Published with no in-graph subscriber -> the consumer that DOES read it.
PUBLISHED_UNREAD = {
    '/anchor_decomposition':
        'a DIAGNOSTIC. The anchor\'s rotation and scale, published beside the '
        'lock so a consumer can fuse them; nothing is forced to. Read on a '
        'scorecard and in Foxglove.',
    '/mongla/esc_rpm':
        'operator telemetry. 958/958 frames read exactly 0 with nothing '
        'attached, so it is watched by a human rather than acted on.',
    '/flow_quality':
        'a DIAGNOSTIC. The gate it describes is applied inside flow_node, which '
        'publishes NO velocity at quality 0, so the filter never needs it. Read '
        'by tools/bench/flow_launch_check.py and tools/bench/flow_derot_ab.py.',
    '/mongla/board_link':
        'operator telemetry: what the link carries and every named channel '
        'with its age, for a human in Batighor\'s board_link layout. The '
        'manager grades the link itself on /diagnostics; nothing acts on this.',
    '/diagnostics':
        'the ROS standard health topic, for a human. The health board on it '
        'is read in Foxglove\'s Diagnostics panel and by rqt_robot_monitor, '
        'and recorded in the run bag; nothing in the loop acts on it.',
    '/annotations':
        'what the vision stack sees, for a human: mongla_batighor/overlay.py '
        'publishes it and Lichtblick\'s image panel draws it over the '
        'camera\'s JPEG (layouts/*.json name it; test_layouts.py holds that). '
        'Built only while someone watches.',
    '/flow_annotations':
        'the flow node\'s tracked point pairs, for a human: overlay.py turns '
        '`flow_tracks` into them and Batighor\'s `flow` layout draws them '
        'over the downward camera. Built only while someone watches.',
    '/undistorted/compressed':
        'a running calibration\'s preview with its current fit applied, for '
        'a human: Batighor\'s `calibrate` layout shows it beside the raw '
        'picture, so straight edges show whether the fit is right. Built '
        'only while someone watches.',
    '/mongla/localization/aiding':
        'a DIAGNOSTIC of which aiding sources the filter accepted. Read in a '
        'bag after a run, never in the loop.',
}

# Subscribed with no in-graph publisher -> what actually feeds it.
SUBSCRIBED_UNFED = {
    '/target_pose':
        'published by `pnp_node`, whose topic name is built with a variant '
        'suffix the scanner cannot resolve. Real, and launched.',
}


def _sweep():
    r = subprocess.run([sys.executable, str(_TOOL)],
                       capture_output=True, text=True, cwd=_ROOT)
    assert r.returncode == 0, r.stdout + r.stderr
    pub, sub, section = set(), set(), None
    for line in r.stdout.splitlines():
        if 'PUBLISHED' in line:
            section = pub
        elif 'SUBSCRIBED' in line:
            section = sub
        elif section is not None:
            m = re.match(r'\s+(/\S+)', line)
            if m:
                section.add(m.group(1))
    return pub, sub


def test_the_sweep_runs():
    """A guard whose tool moved silently stops guarding."""
    assert _TOOL.is_file()
    pub, sub = _sweep()
    assert pub or sub, 'the sweep found nothing at all -- did its regex break?'


def test_no_new_published_and_unread_topic():
    """The enforced half. A new row here means a capability publishes into
    the void -- the repo's oldest defect, one level up from the module sweep."""
    pub, _ = _sweep()
    new = pub - set(PUBLISHED_UNREAD)
    assert not new, (
        'topic(s) published with no in-graph reader and no entry here:\n  '
        + '\n  '.join(sorted(new)) +
        '\nName the consumer. If it is a diagnostic, add it above WITH the '
        'human or tool that reads it. If nothing reads it, it is not wired.')


def test_no_new_subscription_without_a_publisher():
    _, sub = _sweep()
    new = sub - set(SUBSCRIBED_UNFED)
    assert not new, (
        'topic(s) subscribed with no in-graph publisher and no entry here:\n  '
        + '\n  '.join(sorted(new)) +
        '\nName the producer -- the board, a driver, or a node whose topic '
        'string the scanner cannot resolve.')


def test_the_lists_shrink_rather_than_rot():
    """And the other direction. An entry here that the sweep no longer
    reports has been WIRED -- delete the row. Leaving it lets the next real
    one hide behind a stale exemption, which is how `/target_pose_fused`
    survived three passes."""
    pub, sub = _sweep()
    stale_pub = set(PUBLISHED_UNREAD) - pub
    stale_sub = set(SUBSCRIBED_UNFED) - sub
    assert not (stale_pub | stale_sub), (
        'these are no longer reported by the sweep -- they got wired, so '
        'remove them from this file:\n  '
        + '\n  '.join(sorted(stale_pub | stale_sub)))


def test_every_entry_names_its_consumer():
    """A bare exemption is an exemption nobody can challenge."""
    for name, why in {**PUBLISHED_UNREAD, **SUBSCRIBED_UNFED}.items():
        assert len(why) > 30, f'{name} has no stated consumer/producer'
