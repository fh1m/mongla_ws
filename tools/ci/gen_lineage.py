#!/usr/bin/env python3
"""Write docs/evidence/lineage.md from docs/evidence/water.json (#87).

    python3 tools/ci/gen_lineage.py           # rewrite the page
    python3 tools/ci/gen_lineage.py --check   # exit 1 if the page is stale

The registry is the one place a water claim lives; this page is its readable
view, and `test_water_claims_have_a_scope.py` fails if the two disagree.

It also writes the two front-page statements of the current platform's water
status, the README footnote and the site's "In water" gauge, from the same
registry. They were hand-written and checked against it; now the first
`CURRENT_WATER` event added to `water.json` changes both, and nothing else
can.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / 'docs/evidence/water.json'
PAGE = ROOT / 'docs/evidence/lineage.md'
README = ROOT / 'README.md'
SITE = ROOT / 'docs/index.html'

# How the front pages name the current platform. One copy, here.
PLATFORM = ('The current platform, this stack on the SROT board and Kutum in '
            'the Mongla hull,')


def status(reg: dict) -> dict:
    """The current platform's water status: its dive count and one sentence."""
    cur = reg['current_platform']
    dives = [e for e in reg['events']
             if e['platform'] == cur and e['status'] == 'CURRENT_WATER']
    if not dives:
        return {'dives': 0, 'sentence': f'{PLATFORM} has never been in water'}
    last = max(dives, key=lambda e: e['date'])
    n = len(dives)
    return {'dives': n, 'sentence': (
        f'{PLATFORM} has been in water: {n} recorded '
        f'{"dive" if n == 1 else "dives"}, the latest on {last["date"]} '
        f'({last["id"]})')}


def front_pages(reg: dict) -> dict:
    """{path: text} for the README and the site, with the status written in.

    Raises if either page no longer holds the sentence this rewrites: a
    statement that has been reworded by hand is no longer derived.
    """
    st = status(reg)
    n = st['dives']
    edits = {
        README: (r'<sub>The current platform,[^<]*?; its predecessors have \(',
                 f'<sub>{st["sentence"]}; its predecessors have ('),
        SITE: (r'<span class="silk">In water</span><b>\d+ dives?</b><span>'
               r'The current platform,[^<]*?\. Its predecessors',
               f'<span class="silk">In water</span><b>{n} '
               f'{"dive" if n == 1 else "dives"}</b><span>{st["sentence"]}. '
               f'Its predecessors'),
    }
    out = {}
    for path, (pattern, to) in edits.items():
        new, hits = re.subn(pattern, lambda m: to, path.read_text(encoding='utf-8'))
        if hits != 1:
            raise SystemExit(
                f'{path.relative_to(ROOT)}: expected the water-status sentence '
                f'once, found it {hits} times ({pattern!r})')
        out[path] = new
    return out


def _evidence(items) -> str:
    out = []
    for e in items:
        ref = e['ref']
        if e['kind'] == 'path':
            depth = '../../' if not ref.startswith('docs/') else '../'
            link = f'[`{ref}`]({depth}{ref[5:] if ref.startswith("docs/") else ref})'
        elif e['kind'] == 'commit':
            link = f'commit `{ref}`'
        elif e['kind'] == 'upstream':
            repo, n = ref.split('#')
            link = (f'[{ref}](https://github.com/RakibulIslam1/{repo}/issues/{n})')
        else:
            link = f'{ref} (off-repository)'
        out.append(f'{link}: {e["note"]}')
    return '<br>'.join(out)


def render(reg: dict) -> str:
    cur = reg['current_platform']
    lines = [
        '# Water lineage',
        '',
        '*Generated from [`water.json`](water.json) by `tools/ci/gen_lineage.py`; edit the '
        'registry, not this page.*',
        '',
        reg['about'],
        '',
        '**Key point:** the vehicles before this one were in water; this platform has not '
        'been. A status belongs to the platform and code it was earned on. Code here that '
        'descends from a water-tested predecessor carries its own state in '
        '[`capability-map.md`](../capability-map.md).',
        '',
        '## The platforms',
        '',
        '| platform | what it is |',
        '|---|---|',
    ]
    for name, what in reg['platforms'].items():
        mark = ' (current)' if name == cur else ''
        lines.append(f'| `{name}`{mark} | {what} |')
    lines += ['', '## Every time one was in water', '',
              '| id | when | platform | status | what was observed | what ran | '
              'code identity | evidence | still active |',
              '|---|---|---|---|---|---|---|---|---|']
    for ev in reg['events']:
        lines.append('| ' + ' | '.join([
            f'`{ev["id"]}`', ev['date'], f'`{ev["platform"]}`', f'**{ev["status"]}**',
            ev['observed'] + (f' ({ev["place"]})' if ev['place'] else ''),
            ev['code_in_water'] or '--', ev['code_identity'],
            _evidence(ev['evidence']), ev['active_today'],
        ]) + ' |')
    lines += ['', '## The statuses', '', '| status | meaning |', '|---|---|']
    lines += [f'| **{k}** | {v} |' for k, v in reg['statuses'].items()]
    lines.append('')
    return '\n'.join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args(argv)
    reg = json.loads(REGISTRY.read_text())
    pages = {PAGE: render(reg), **front_pages(reg)}
    if args.check:
        stale = [p for p, text in pages.items()
                 if not p.is_file() or p.read_text(encoding='utf-8') != text]
        for p in stale:
            print(f'{p.relative_to(ROOT)} is stale: run tools/ci/gen_lineage.py')
        return int(bool(stale))
    for p, text in pages.items():
        if not p.is_file() or p.read_text(encoding='utf-8') != text:
            p.write_text(text, encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
