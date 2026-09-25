#!/usr/bin/env python3
"""Topics that are PUBLISHED and nobody subscribes -- and vice versa.

⛔ THE GAP THIS CLOSES. `orphan_sweep.py` finds modules nothing imports. It
cannot see the other half of the same defect: a node that publishes something
no node reads, or -- worse -- a node that never subscribes to something it
needs. Both pieces work perfectly alone, every test passes, and the capability
does not exist.

⭐ THE CASE THAT MOTIVATED IT. `lock_node` had odometry and floor height but
never subscribed `/mongla/state`, so vision did not know the hull's HEADING.
`visibility.WorldTarget` -- object permanence, the thing that turns "target
left the frame" from a perception failure into a navigation problem -- was
unreachable for want of one number the board already published. No import was
missing. No test failed. Nothing in the module graph showed it.

⚠ WHAT THIS TOOL CANNOT DO, said plainly so its output is not over-read:

  * a topic with no subscriber is not automatically a defect. `/mongla/state`
    is consumed by the CLI, by Foxglove, and by an operator watching a
    terminal -- consumers a source scan cannot see. Diagnostics SHOULD be
    published without an in-graph reader.
  * remapping is invisible here. Launch files rename topics, so a name that
    looks unmatched may be wired under another.
  * it reads string literals, so a topic built by f-string interpolation is
    matched on its literal suffix only.
  * ⛔ a topic passed as a VARIABLE is invisible. `pnp_node` does
    `topic = f'{ns}/target_pose' + ...` then `create_publisher(T, topic, 10)`,
    so `/target_pose` is reported as having no publisher although two nodes
    depend on it and it is produced. Resolving that needs real dataflow
    analysis, not a regex, and pretending otherwise would make the tool lie in
    the confident direction.

So this is a QUESTION GENERATOR, not a verdict. It narrows "what in this
system is not talking to what" from thousands of lines to a page someone can
actually read -- which is all that was needed to find the yaw gap.

    python3 tools/topic_wiring_sweep.py
    python3 tools/topic_wiring_sweep.py --pub-only
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
from collections import defaultdict

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'

# `create_publisher(Type, 'topic', qos)` / `create_subscription(Type, 'topic', cb, qos)`
_PUB = re.compile(r'create_publisher\(\s*[\w.]+\s*,\s*([\'"f][^,]*?)\s*,', re.S)
_SUB = re.compile(r'create_subscription\(\s*[\w.]+\s*,\s*([\'"f][^,]*?)\s*,', re.S)

# ⛔ AN EXPLICIT LIST, NOT "any _suffix". `pnp_node` publishes
# `f'{ns}/target_pose' + (f'_{variant}' if variant else '')`, so the scanner
# sees `/target_pose` while the node may publish `/target_pose_near`. A
# permissive "anything after an underscore is a variant" rule ALSO swallowed
# `/target_pose_fused` -- a genuinely unread topic -- and hid the only real
# finding in the sweep. Widen this only for a variant that actually exists.
_VARIANTS = ('_near',)


def literal(expr: str) -> str:
    """The stable part of a topic expression.

    An f-string like f'{ns}/detections' has no fixed prefix but a real suffix,
    so match on the suffix; a plain literal matches whole.
    """
    e = expr.strip()
    if e.startswith(('f"', "f'")):
        inner = e[2:-1]
        tail = inner.rsplit('}', 1)[-1]
        return tail or inner
    return e.strip('\'"')


def scan():
    pubs, subs = defaultdict(set), defaultdict(set)
    for p in SRC.rglob('*.py'):
        if '/test' in str(p):
            continue
        try:
            txt = p.read_text()
        except (OSError, UnicodeDecodeError):
            continue
        who = str(p.relative_to(SRC))
        for m in _PUB.finditer(txt):
            pubs[literal(m.group(1))].add(who)
        for m in _SUB.finditer(txt):
            subs[literal(m.group(1))].add(who)
    return pubs, subs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pub-only', action='store_true')
    ap.add_argument('--sub-only', action='store_true')
    a = ap.parse_args()

    pubs, subs = scan()
    sub_keys = list(subs)
    pub_keys = list(pubs)

    def matched(topic, others):
        """A topic is matched if either name is a suffix of the other, OR one
        extends the other with a `_variant` suffix.

        ⚠ THE VARIANT RULE IS NOT COSMETIC. `pnp_node` builds its topic as
        `f'{ns}/target_pose' + (f'_{variant}' if variant else '')`, so the
        scanner records `/target_pose` while the node may publish
        `/target_pose_near`. Without this the tool reported `/target_pose` as
        having NO PUBLISHER -- a false alarm on a topic two nodes depend on,
        and a false alarm is how a sweep like this gets ignored.
        """
        for o in others:
            if o.endswith(topic) or topic.endswith(o):
                return True
            for a, b in ((o, topic), (topic, o)):
                if a.startswith(b) and a[len(b):] in _VARIANTS:
                    return True
        return False

    orphan_pub = sorted(t for t in pubs if not matched(t, sub_keys))
    orphan_sub = sorted(t for t in subs if not matched(t, pub_keys))

    if not a.sub_only:
        print(f'PUBLISHED, no in-graph subscriber ({len(orphan_pub)}):')
        for t in orphan_pub:
            print(f'   {t:<44} {", ".join(sorted(pubs[t]))[:70]}')
    if not a.pub_only:
        print(f'\nSUBSCRIBED, no in-graph publisher ({len(orphan_sub)}):')
        for t in orphan_sub:
            print(f'   {t:<44} {", ".join(sorted(subs[t]))[:70]}')

    print('\n⚠ Neither list is a defect list. A published topic with no '
          'in-graph reader is often a DIAGNOSTIC (the CLI, Foxglove and an '
          'operator are all real consumers this cannot see), and a subscribed '
          'topic with no in-graph publisher is usually one the BOARD or a '
          'driver provides. Read it as a question list: for each row, name '
          'the consumer or the producer out loud. The yaw gap was found '
          'exactly that way.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
