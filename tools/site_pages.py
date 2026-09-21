#!/usr/bin/env python3
"""The two long-form pages, previewed on the front page from their own text.

`docs/the-shift.md` and `docs/capability-map.md` were reachable only from one
row of a table near the bottom. They are the two documents that actually answer
"why is it built this way" and "what is proven", so they get a card each -- and
the cards are GENERATED from the documents, not written beside them:

  * the capability card counts every table row of each state (green/purple/
    yellow/red) in capability-map.md, so the tally on the front page and the
    tally that page computes for itself come from the same rows;
  * the shift card reads the board's timetable table and reports what runs on
    each core, at the rate the document states.

Writes the block into docs/index.html between `pages:begin` / `pages:end`.

    python3 tools/site_pages.py
    python3 tools/site_pages.py --check    # exit 1 if the page has drifted
"""
from __future__ import annotations

import argparse
import html
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
STATES = [('🟢', 'WATER', 'water', 'verified in water'),
          ('🟣', 'BENCH', 'bench', 'verified on the bench or on real footage'),
          ('🟡', 'BUILT', 'built', 'built and tested, never flown'),
          ('🔴', 'BLOCKED', 'blocked', 'waiting on hardware or a firmware merge')]


def capability_tally() -> tuple[list[tuple], int]:
    """Count the state cell of every table row, by the same rule the page's own
    script uses -- one rule, two readers, so the two tallies cannot disagree."""
    text = (DOCS / 'capability-map.md').read_text(encoding='utf-8')
    rows = [ln for ln in text.splitlines() if ln.startswith('|')]
    out = []
    for glyph, word, cls, blurb in STATES:
        n = sum(1 for ln in rows if glyph in ln.rsplit('|', 2)[-2] and word in ln)
        out.append((word, cls, blurb, n))
    return out, sum(n for *_, n in out)


def board_timetable() -> list[tuple[str, str, str]]:
    """(core, task, rate) from the timetable table in the-shift.md."""
    text = (DOCS / 'the-shift.md').read_text(encoding='utf-8')
    block = text[text.index('| Core | Task | Rate'):]
    block = block[:block.index('\n\n')]
    out = []
    for ln in block.splitlines()[2:]:
        cells = [c.strip().strip('*') for c in ln.strip().strip('|').split('|')]
        if len(cells) >= 3:
            out.append((cells[0], cells[1], cells[2]))
    return out


def render() -> str:
    e = html.escape
    tally, total = capability_tally()
    table = board_timetable()
    fast = [r for r in table if r[2].replace('*', '') == '500 Hz']
    slow = [r for r in table if r not in fast]

    bars = ''.join(
        f'<li class="t--{cls}"><b>{n}</b><span class="silk">{word.lower()}</span>'
        f'<i style="--w:{n / total * 100:.1f}%"></i><em>{e(blurb)}</em></li>'
        for word, cls, blurb, n in tally)
    core1 = ''.join(f'<li><span>{e(t)}</span><b>500<u>Hz</u></b></li>' for _, t, _ in fast)
    core0 = ''.join(
        f'<li><span>{e(t)}</span><b>{e(r.replace("*", "").replace(" Hz", ""))}<u>Hz</u></b></li>'
        for _, t, r in slow)
    return (
        '  <div class="pages">\n'
        '   <a class="pages__card" href="the-shift.html">\n'
        '    <span class="silk">Long read <i>//</i> the architecture from first principles</span>\n'
        '    <h3>The Shift</h3>\n'
        '    <p>Why the reflexes live on a 500 Hz board and the thinking on a Pi, written for '
        'someone who has never seen a flight controller. The whole argument is one table — the '
        'board’s own timetable, two cores that never share a desk.</p>\n'
        f'    <div class="pages__cores">'
        f'<div><span class="silk">core 1 — nothing else may interrupt it</span><ul>{core1}</ul></div>'
        f'<div><span class="silk">core 0 — everything a human might wait on</span><ul>{core0}</ul></div>'
        f'</div>\n'
        '    <span class="pages__go">Read it <i>→</i></span>\n'
        '   </a>\n'
        '   <a class="pages__card" href="capability-map.html">\n'
        '    <span class="silk">Long read <i>//</i> every claim with its evidence</span>\n'
        '    <h3>Capability map</h3>\n'
        f'    <p>{total} capabilities, each carrying the file or the measurement behind it and an '
        'honest state. The bar below is counted out of that page’s own table rows by '
        '<code>tools/site_pages.py</code> — including the state whose count is still zero.</p>\n'
        f'    <ul class="pages__tally">{bars}</ul>\n'
        '    <span class="pages__go">Read it <i>→</i></span>\n'
        '   </a>\n'
        '  </div>')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    args = ap.parse_args()
    rc = splice('pages', render(), args.check)
    if rc == 0 and not args.check:
        tally, total = capability_tally()
        print(f'{total} capabilities: ' + ', '.join(f'{n} {w.lower()}' for w, _, _, n in tally))
    return rc


if __name__ == '__main__':
    sys.exit(main())
