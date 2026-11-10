#!/usr/bin/env python3
"""Write docs/evidence/lineage.md from docs/evidence/water.json (#87).

    python3 tools/gen_lineage.py           # rewrite the page
    python3 tools/gen_lineage.py --check   # exit 1 if the page is stale

The registry is the one place a water claim lives; this page is its readable
view, and `test_water_claims_have_a_scope.py` fails if the two disagree.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / 'docs/evidence/water.json'
PAGE = ROOT / 'docs/evidence/lineage.md'


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
        '*Generated from [`water.json`](water.json) by `tools/gen_lineage.py`; edit the '
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
    text = render(json.loads(REGISTRY.read_text()))
    if args.check:
        stale = not PAGE.is_file() or PAGE.read_text() != text
        if stale:
            print(f'{PAGE.relative_to(ROOT)} is stale: run tools/gen_lineage.py')
        return int(stale)
    PAGE.write_text(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
