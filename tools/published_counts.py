#!/usr/bin/env python3
"""The test counts the README and the site publish, from the last full run.

Until 2026-11-06 the counts were typed by hand: the README badge, its run
command, its table, its diagram, two tiles on the site, and the README's lines
of code, test files and chapter counts. Every change left them a little more
wrong; the package sizes were off by up to 1 740 lines. Now there is one file:

    docs/stats.json      written by every FULL test run (the root conftest.py)

and this tool copies it into the places that show it. A full run is the whole
of `testpaths` -- all six suites, no `-k`, no `-m`, nothing stopped early --
so a partial run never overwrites the published numbers with a smaller one.

    python3 tools/published_counts.py           # re-copy docs/stats.json into the pages
    python3 tools/published_counts.py --code    # also re-measure the code (no test run)
    python3 tools/published_counts.py --check   # exit 1 if a page disagrees (the guard)

`test/test_published_test_counts_are_the_last_run.py` runs `--check`, so a hand
edit to any published count fails the suite. Standalone: no ROS, no pytest.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATS = ROOT / 'docs' / 'stats.json'
README = ROOT / 'README.md'
SITE = ROOT / 'docs' / 'index.html'

# Display order and the one-line summary of what each suite holds.
SUITES = (
    ('mongla_control', 'the verbs, the wire, the refusals'),
    ('mongla_vision', 'detection, the lock ladder, optics, flow'),
    ('mongla_manager', 'the node, bring-up gates, the docs contract'),
    ('mongla_planner', 'the CLI, the DSL, the missions'),
    ('mongla_localization', 'the filter, retrodiction, course priors'),
    ('test/', 'nothing unreachable, the retired stack stays retired'),
)
SUITE_NAMES = tuple(name for name, _ in SUITES)


def suite_of(path: str) -> str | None:
    """`src/mongla_vision/test/x.py` -> 'mongla_vision'; `test/x.py` -> 'test/'."""
    parts = Path(path).parts
    if parts[:1] == ('test',):
        return 'test/'
    if len(parts) > 2 and parts[0] == 'src' and parts[2] == 'test':
        return parts[1]
    return None


def num(n: int) -> str:
    """4018 -> '4 018', the house style for counts (a plain space)."""
    return f'{n:,}'.replace(',', ' ')


def _table(s: dict) -> str:
    label = {'test/': 'test/ (repo-wide)'}
    rows = [f'{label.get(name, name):<20}{num(s["suites"][name]["passed"]):>6} passed        {what}'
            for name, what in SUITES]
    tail = f'{num(s["passed"])} passed, {s["failed"]} failed'
    return '```\n' + '\n'.join(rows) + '\n' + ' ' * 21 + '─' * 13 + '\n' + ' ' * 21 + tail + '\n```'


def _badge(s: dict) -> str:
    if s['failed']:
        label, colour = f'{num(s["passed"])} passing, {s["failed"]} failing', 'red'
    else:
        label, colour = f'{num(s["passed"])} passing', 'brightgreen'
    return f'tests-{label.replace(" ", "%20").replace(",", "%2C")}-{colour}'


PACKAGES = ('mongla_vision', 'mongla_control', 'mongla_planner', 'mongla_manager',
            'mongla_localization')
_PARTS = ('zero', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten')


def code_stats() -> dict:
    """Lines of Python and test files per package, and the book's size, from the tree."""
    pkgs = {}
    for name in PACKAGES:
        src = ROOT / 'src' / name
        pkgs[name] = {
            'loc': sum(len(p.read_text(encoding='utf-8').splitlines())
                       for p in sorted((src / name).rglob('*.py'))),
            'test_files': len(list((src / 'test').glob('test_*.py'))),
        }
    book = (ROOT / '.claude' / 'context' / 'README.md').read_text(encoding='utf-8')
    return {'packages': pkgs,
            'test_files': sum(p['test_files'] for p in pkgs.values()),
            'chapters': len(re.findall(r'^\d+\. \[', book, flags=re.M)),
            'parts': len(re.findall(r'^## Part ', book, flags=re.M))}


def _failing(s: dict) -> str:
    return 'none failing' if not s['failed'] else f'{s["failed"]} failing'


def edits(s: dict) -> dict[Path, list[tuple[str, str]]]:
    """For each page: (pattern, replacement) pairs. Each pattern must match once."""
    n = len(SUITES)
    code = s['code']
    rows = [(rf'(\| \[`{name}`\]\([^)]*\) \| [^|\n]* \| )[\d ]+( \| )\d+( \|)',
             rf'\g<1>{num(c["loc"])}\g<2>{c["test_files"]}\g<3>')
            for name, c in code['packages'].items()]
    return {
        README: rows + [
            (r'\*\*[\d ]+ test files against the source\.\*\*',
             f'**{num(code["test_files"])} test files against the source.**'),
            (r'— \d+ chapters in \w+ parts,',
             f'— {code["chapters"]} chapters in {_PARTS[code["parts"]]} parts,'),
            (r'(img\.shields\.io/badge/)tests-[^)]*?-(?:brightgreen|red)\)',
             rf'\g<1>{_badge(s)})'),
            (r'# the tests \(\d+ suites, [\d ]+ of them,',
             f'# the tests ({n} suites, {num(s["collected"])} of them,'),
            (r'(<!-- stats:table:begin -->\n).*?(\n<!-- stats:table:end -->)',
             rf'\g<1>{_table(s)}\g<2>'),
            (r'UNIT\["\d+ suites · [\d ]+ tests"\]',
             f'UNIT["{n} suites · {num(s["passed"])} tests"]'),
        ],
        SITE: [
            (r'(<span class="silk">Tests</span><b>)[\d ]+(</b><span>passing, )[^<]*(</span>)',
             rf'\g<1>{num(s["passed"])}\g<2>{_failing(s)}\g<3>'),
            (r'(Suites</span><b>)[\d ]+(</b><span>passing across )\w+ suites; \d+ failed(</span>)',
             rf'\g<1>{num(s["passed"])}\g<2>{_words(n)} suites; {s["failed"]} failed\g<3>'),
        ],
    }


def _words(n: int) -> str:
    return {6: 'six', 7: 'seven', 5: 'five'}.get(n, str(n))


def apply(s: dict, check: bool) -> int:
    """Rewrite the pages from `s`; with `check`, write nothing, return 1 on drift."""
    drift = 0
    for page, pairs in edits(s).items():
        text = page.read_text(encoding='utf-8')
        new = text
        for pat, rep in pairs:
            new, hits = re.subn(pat, rep, new, flags=re.S)
            if hits != 1:
                print(f'{page.name}: expected one match of {pat!r}, found {hits}',
                      file=sys.stderr)
                return 2
        if new != text:
            if check:
                print(f'{page.name}: published test counts differ from docs/stats.json '
                      f'-- run python3 tools/published_counts.py', file=sys.stderr)
                drift = 1
            else:
                page.write_text(new, encoding='utf-8')
    return drift


def load() -> dict:
    return json.loads(STATS.read_text(encoding='utf-8'))


def record(counts: dict[str, dict[str, int]], collected: int) -> dict:
    """Write docs/stats.json from per-suite outcome counts; splice the pages."""
    # An error (a broken fixture) is published as a failure: both mean red.
    counts = {name: {**c, 'failed': c.get('failed', 0) + c.pop('error', 0)}
              for name, c in counts.items()}
    totals = {k: sum(c.get(k, 0) for c in counts.values())
              for k in ('passed', 'failed', 'skipped', 'xfailed', 'xpassed')}
    s = {'collected': collected, **totals,
         'suites': {name: counts.get(name, {}) for name in SUITE_NAMES},
         'code': code_stats()}
    STATS.write_text(json.dumps(s, indent=2) + '\n', encoding='utf-8')
    apply(s, check=False)
    return s


def main(argv: list[str]) -> int:
    s = load()
    if '--code' in argv:
        s['code'] = code_stats()
        STATS.write_text(json.dumps(s, indent=2) + '\n', encoding='utf-8')
    return apply(s, check='--check' in argv)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
