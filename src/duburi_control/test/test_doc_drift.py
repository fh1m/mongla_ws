"""Docs that state a SHIPPED CONSTANT must not disagree with the source.

⛔ WHY THIS EXISTS. `srot-integration.md` asserted "`FW_BEHAVIOUR_REV_REQUIRED`
stays at 2, deliberately", with a paragraph of reasoning, while the code had
said **10** since round 26. That is the dangerous kind of stale: it names a
SAFETY FLOOR, and its reasoning ("rev 3's changes are additive") is exactly
what stopped being true -- rev 10 INVERTS YAW and rev 13 requires ARMED. A
reader trusting the doc believes the host will fly a board it will in fact
refuse, and believes a rev-2 board is safe when it would take our yaw
commands with the wrong sign.

`test_bars.py` guards `measured-bars.md` this way and has caught real drift.
This is the same idea pointed at the srot docs, where the constants are
safety interlocks rather than tuning values.

READ AS TEXT, NEVER IMPORTED. In a worktree `import duburi_control` resolves
to the MAIN workspace's `install/` tree -- a different commit -- which once
reported a constant's value from code this branch does not contain.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
CTX = ROOT.parent / '.claude' / 'context'
PROTO = ROOT / 'duburi_control' / 'duburi_control' / 'fc' / 'srot_protocol.py'


def _const(name):
    m = re.search(rf'^{name}\s*=\s*([0-9]+)', PROTO.read_text(), re.M)
    assert m, f'{name} not found in srot_protocol.py'
    return int(m.group(1))


def _docs():
    # `**/*.md` already includes the top level, so globbing both double-counts
    # every finding -- which made this file's own first failure print each
    # line twice and look like two separate problems.
    return sorted(set(CTX.glob('**/*.md')))


def _live_lines(doc):
    """(lineno, text) for lines that are ASSERTIONS, not preserved history.

    Skips blockquotes (a correction quoting what it corrects) and
    strikethrough. ⛔ STRIKETHROUGH IS A BLOCK, NOT A LINE: `~~` opens on one
    line and closes paragraphs later, so a per-line `'~~' in line` test
    exempts only the first line and then flags the rest -- which is exactly
    what this guard did to a correction written twenty minutes earlier.
    """
    out = []
    struck = False
    for i, line in enumerate(doc.read_text().splitlines(), 1):
        n = line.count('~~')
        if struck:
            if n:
                struck = False    # the block closes here
            continue
        if n:
            # opens and closes on the same line -> inline, skip the line;
            # odd count -> a block opens here and stays open.
            struck = (n % 2 == 1)
            continue
        if line.lstrip().startswith('>'):
            continue
        out.append((i, line))
    return out


@pytest.mark.parametrize('name', ['FW_BEHAVIOUR_REV_REQUIRED',
                                  'FW_BEHAVIOUR_REV'])
def test_no_doc_asserts_a_stale_value_for_a_safety_constant(name):
    """A doc may MENTION the constant; it may not state a different value.

    Matches `NAME` followed by "is|stays at|= <n>", which is how a doc makes
    the claim. A struck-through line (`~~...~~`) is a deliberately preserved
    wrong statement and is exempt -- see the correction in
    `srot-integration.md`.
    """
    live = _const(name)
    pat = re.compile(rf'`?{name}`?\s*(?:is|stays at|=|:)\s*\*?\*?(\d+)')
    bad = []
    for doc in _docs():
        for i, line in _live_lines(doc):
            m = pat.search(line)
            if m and int(m.group(1)) != live:
                # Bondor's own constant is a THIRD repo's value and is
                # legitimately different -- the doc says so by naming it.
                if 'Bondor' in line or 'protocol.ts' in line:
                    continue
                bad.append(f'{doc.name}:{i} says {m.group(1)}, source says '
                           f'{live}\n      {line.strip()}')
    assert not bad, (
        f'{name} drift between docs and srot_protocol.py:\n  '
        + '\n  '.join(bad))


def test_the_required_rev_is_at_least_the_yaw_inversion_rev():
    """Rev 10 inverted yaw. A floor below it accepts a board that turns the
    WRONG WAY on every command, which no amount of host-side care fixes."""
    assert _const('FW_BEHAVIOUR_REV_REQUIRED') >= 10


def test_the_fov_blocker_is_not_still_asserted_as_open():
    """The FOV was the named critical path for the whole vision split.

    A doc still calling it a blocker defers work that is unblocked -- which
    is a cost, not just an inaccuracy.
    """
    cal = (ROOT / 'duburi_vision' / 'config' / 'calibration')
    assert list(cal.glob('*.json')), 'no calibrations ship; the claim is TRUE'
    bad = []
    for doc in _docs():
        for i, line in _live_lines(doc):
            low = line.lower()
            if ('fov' in low and
                    ('do not exist' in low or 'does not exist' in low
                     or 'blocked on fov' in low or 'unmeasured' in low)):
                bad.append(f'{doc.name}:{i}  {line.strip()}')
    assert not bad, (
        'the FOV is measured and shipping, but a doc still calls it open:\n  '
        + '\n  '.join(bad))
