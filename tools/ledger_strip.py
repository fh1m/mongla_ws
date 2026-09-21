#!/usr/bin/env python3
"""The measurement ledger as a core sample -- counted, not characterised.

`.claude/context/measured-bars.md` is appended to in the order things were
measured. This tool reads it and draws one column per entry (`## ` heading),
left to right in that order:

  height  the number of table rows in the entry -- how many numbers it holds
  colour  what the entry's OWN HEADING says happened:
            taken back  -- RETRACT / REFUTED / WITHDRAWN / REJECTED / WRONG /
                           DELETED / CONTAMINATED
            said no     -- the idea was measured and the answer was to leave it
                           off (leave it off / OFF wins / not chosen /
                           UNSUPPORTED / ADOPT NOTHING)
            stands      -- everything else

The rule is deliberately crude and printed on the page: it counts what each
entry says about itself, so it can undercount (an entry that retracts something
without saying so in its title is counted as standing) but it cannot invent a
retraction. That is the right direction to be wrong in.

It also writes the lede sentence that carries the counts, because the previous
hand-written one said "more than half" and the file never supported it.

    python3 tools/ledger_strip.py            # rewrite the figure in place
    python3 tools/ledger_strip.py --check    # exit 1 if the page has drifted
"""
from __future__ import annotations

import argparse
import html
import re
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

LEDGER = Path(__file__).resolve().parents[1] / '.claude' / 'context' / 'measured-bars.md'
TAKEN_BACK = re.compile(r'RETRACT|REFUTED|WITHDRAWN|REJECTED|WRONG|DELETED|CONTAMINATED')
SAID_NO = re.compile(r'leave it off|OFF wins|not chosen|UNSUPPORTED|ADOPT NOTHING', re.I)
RULE_ROW = re.compile(r'^\|[\s:|-]+\|?\s*$')          # a markdown table's |---| line


@dataclass(frozen=True)
class Entry:
    title: str
    rows: int
    kind: str      # 'back' | 'no' | 'ok'


def read_ledger(text: str) -> list[Entry]:
    entries, title, rows = [], None, 0
    for line in text.split('\n') + ['## ']:
        if line.startswith('## '):
            if title is not None:
                kind = ('back' if TAKEN_BACK.search(title)
                        else 'no' if SAID_NO.search(title) else 'ok')
                entries.append(Entry(title, rows, kind))
            title, rows = line[3:].strip(), 0
        elif title is not None and line.startswith('|') and not RULE_ROW.match(line):
            rows += 1
    return entries


def _plain(title: str) -> str:
    """Heading text for a tooltip: markdown backticks and the ⛔ marker dropped."""
    return re.sub(r'[`⛔]', '', title).strip()


def render(entries: list[Entry]) -> str:
    e = html.escape
    tallest = max(x.rows for x in entries)
    back = [x for x in entries if x.kind == 'back']
    said_no = [x for x in entries if x.kind == 'no']
    rows = sum(x.rows for x in entries)
    cols = ''.join(
        f'<li class="core--{x.kind}" style="--h:{max(x.rows / tallest, 0.04):.3f}" '
        f'title="{e(_plain(x.title))} · {x.rows} rows"></li>'
        for x in entries)
    lede = (
        f'  <p class="lede col quiet">The measurement ledger holds {len(entries)} entries and '
        f'{rows} rows of numbers. {len(back)} of those entries say, in their own title, that '
        f'they took an earlier result back; {len(said_no)} more measured an idea and the answer '
        f'was to leave it off. They are kept where anyone can read them, because a project that '
        f'quietly deletes its wrong answers is a project you cannot check. The full ledger is '
        f'<a href="https://github.com/fh1m/mongla_ws/blob/main/.claude/context/measured-bars.md">'
        f'here</a>.</p>')
    fig = (
        f'  <figure class="core" aria-label="{len(entries)} ledger entries in the order they '
        f'were written; {len(back)} taken back, {len(said_no)} said no">\n'
        f'   <ol class="core__cols">{cols}</ol>\n'
        f'   <div class="core__axis"><span>first entry</span><span>newest</span></div>\n'
        f'   <figcaption><span class="core__k core--back">taken back · {len(back)}</span>'
        f'<span class="core__k core--no">measured, answer was no · {len(said_no)}</span>'
        f'<span class="core__k core--ok">stands · '
        f'{len(entries) - len(back) - len(said_no)}</span><br>'
        f'One column per entry, in the order the file was written; height is how many numbers '
        f'it holds. The colour is what each entry says about <em>itself</em> in its title, '
        f'counted by <code>tools/ledger_strip.py</code> &mdash; so it can miss a retraction '
        f'that is not in a title, but it cannot invent one.</figcaption>\n'
        f'  </figure>')
    return lede + '\n' + fig


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    args = ap.parse_args()
    entries = read_ledger(LEDGER.read_text(encoding='utf-8'))
    rc = splice('ledger', render(entries), args.check)
    if rc == 0:
        kinds = {k: sum(1 for x in entries if x.kind == k) for k in ('back', 'no', 'ok')}
        print(f'{len(entries)} entries, {sum(x.rows for x in entries)} rows, {kinds}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
