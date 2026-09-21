#!/usr/bin/env python3
"""The competition record, read out of RoboNation's own published score sheets.

Placements were typed by hand in eight places and disagreed with the author's
own log. This tool makes the official "Scores Master" sheets the one source:

  * `--fetch` downloads each year's published master sheet (CSV export of the
    public Google Sheet), pulls the team's column round by round, and caches it
    in docs/data/robosub.json -- so the page can be checked offline;
  * without `--fetch` it renders the cached record into docs/index.html between
    the `record:begin` / `record:end` markers.

Both results belong to the university programme's vehicles, which flew the old
stack. Nothing on the SROT board has been in water, and the figure says so.

    python3 tools/robosub_record.py --fetch   # re-read the official sheets
    python3 tools/robosub_record.py           # rewrite the figure
    python3 tools/robosub_record.py --check   # exit 1 if the page has drifted
"""
from __future__ import annotations

import argparse
import csv
import html
import io
import json
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / 'docs' / 'data' / 'robosub.json'
TEAM = 'BRAC U (Bangladesh)'
MASTER_GID, RANKINGS_GID = '1115176301', '362427049'
SHEETS = {
    2025: '2PACX-1vTXxXI-kv_FB-9PbUNexgPf2k1Lm8jAugNN8Tb4Z0YoCMsdRyDcnY_SyaPkbVhoJCy2HjYw2ulb67dE',
    2026: '2PACX-1vRFIz4fUxtK1p7pD0GRf8mYJGdG5wNQq_L8wrNF_xvTL37KEJT3oGB66BJb31gaiFGohmNzaSKBBYOA',
}
ROUNDS = ('Semi-Final 1', 'Semi-Final 2', 'Third Chance', 'Final')
SKIP = ('Actual dry weight', 'Rank', 'Website Rank', 'TDR Rank', 'Video Rank',
        'Presentation Rank', 'Assessment Rank', 'Best Round Total', 'Best Round Rank')


def pub_url(key: str, gid: str | None = None) -> str:
    base = f'https://docs.google.com/spreadsheets/d/e/{key}/pub'
    return f'{base}?gid={gid}&single=true&output=csv' if gid else f'{base}html'


def _num(s: str) -> float | None:
    try:
        return float(s.replace(',', '').strip())
    except ValueError:
        return None


def parse_master(text: str) -> dict:
    """The team's column, round by round, exactly as the sheet scores it."""
    rows = list(csv.reader(io.StringIO(text)))
    header = rows[1]
    col = header.index(TEAM)
    rec: dict = {'teams': sum(1 for c in header[3:] if c.strip()), 'rounds': {}}
    tasks: list = []
    for r in rows[2:]:
        label = ' '.join(r[1].split()) if len(r) > 1 else ''
        val = r[col].strip() if col < len(r) else ''
        if not label:
            continue
        if label == 'Sub-Total Design Doc':
            rec['design'] = {'score': _num(val), 'max': 860}
        elif label == 'Rank' and 'design' in rec and 'rank' not in rec['design']:
            rec['design']['rank'] = int(_num(val))
        elif label.startswith('Actual dry weight'):
            tasks = []                       # a new round starts at the weigh-in
        elif label in (f'{n} Total' for n in ROUNDS):
            rec['rounds'][label[:-6]] = {'score': _num(val), 'tasks': tasks}
            tasks = []
        elif label in (f'{n} Rank' for n in ROUNDS):
            rank = _num(val)
            rec['rounds'][label[:-5]]['rank'] = int(rank) if rank else None
        elif label == 'OVERALL TOTAL':
            rec['overall'] = {'score': _num(val)}
        elif label == 'OVERALL RANK':
            rec['overall']['rank'] = int(_num(val))
        elif 'design' in rec and label not in SKIP and _num(val) is not None:
            tasks.append({'task': label, 'max': r[2].strip(), 'points': _num(val)})
    return rec


def parse_wildcard(text: str) -> int | None:
    """The finals wild-card eligibility score printed at the top of the rankings sheet."""
    for c in next(csv.reader(io.StringIO(text))):
        if 'Wild Card Eligibility' in c:
            return int(_num(c.split(':')[1].replace('points', '')))
    return None


def fetch() -> dict:
    out = {'team': TEAM, 'source': 'RoboNation, RoboSub "Scores Master" (published sheets)',
           'years': {}}
    for year, key in SHEETS.items():
        def get(gid: str) -> str:
            return urllib.request.urlopen(pub_url(key, gid), timeout=60).read().decode()
        rec = parse_master(get(MASTER_GID))
        rec['wildcard'] = parse_wildcard(get(RANKINGS_GID))
        rec['url'] = pub_url(key)
        out['years'][str(year)] = rec
    return out


def render(data: dict) -> str:
    e = html.escape
    years = data['years']
    # one axis for both cards, long enough to hold every round AND every wild-card line
    top = max([r['score'] or 0 for y in years.values() for r in y['rounds'].values()]
              + [y['wildcard'] for y in years.values()])
    cards = []
    for year, y in sorted(years.items()):
        bars = ''.join(
            f'<li><span class="silk">{e(name)}</span>'
            f'<i style="--w:{(r["score"] or 0) / top * 100:.1f}%"></i>'
            f'<b>{r["score"]:,.0f}</b><em>{"#" + str(r["rank"]) if r.get("rank") else "—"}</em></li>'
            for name, r in y['rounds'].items() if name != 'Final')
        wc = y['wildcard']
        cards.append(
            f'<article class="score"><header><time>{year}</time>'
            f'<b class="place">{y["overall"]["rank"]}<sup>th</sup></b>'
            f'<span class="silk">of {y["teams"]} teams · {y["overall"]["score"]:,.0f} points</span>'
            f'</header><ol class="score__rounds" style="--wc:{wc / top * 100:.1f}%">{bars}</ol>'
            f'<p class="score__foot">finals wild card at <b>{wc:,}</b> · design documentation '
            f'{y["design"]["score"]:.0f}/860, #{y["design"]["rank"]}</p></article>')
    ys = sorted(years)
    a, b = years[ys[0]]['overall']['score'], years[ys[-1]]['overall']['score']
    return (
        f'  <figure class="record" aria-label="RoboSub placements, read from the official score sheets">\n'
        f'   <div class="record__cards">{"".join(cards)}</div>\n'
        f'   <figcaption>Read round by round out of RoboNation\'s published score sheets by '
        f'<code>tools/robosub_record.py</code>, never typed in. Best round {a:,.0f} → {b:,.0f} '
        f'points ({(b - a) / a * 100:+.0f}%). The red tick is that year\'s published finals '
        f'wild-card threshold; seven teams made the finals, and clearing the line on a third-chance '
        f'run, as 2025 did, was not enough to be one of them. Both vehicles were the university programme\'s, on the old '
        f'stack; nothing on the SROT board has been in water.</figcaption>\n'
        f'  </figure>')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--fetch', action='store_true', help='re-read the official sheets')
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    args = ap.parse_args()
    if args.fetch:
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        CACHE.write_text(json.dumps(fetch(), indent=1) + '\n')
    data = json.loads(CACHE.read_text())
    rc = splice('record', render(data), args.check)
    if rc == 0 and not args.check:
        for year, y in sorted(data['years'].items()):
            print(f'{year}: #{y["overall"]["rank"]} of {y["teams"]}, '
                  f'{y["overall"]["score"]:,.0f} points')
    return rc


if __name__ == '__main__':
    sys.exit(main())
