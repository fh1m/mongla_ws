"""Replace a generated block in docs/index.html between two HTML comment markers.

Shared by the tools that write figures into the site (`wire_frame.py`,
`ledger_strip.py`), so the splice-and-check logic exists once. Standalone and
dependency-free: `tools/` runs without a ROS environment.
"""
from __future__ import annotations

import sys
from pathlib import Path

PAGE = Path(__file__).resolve().parents[1] / 'docs' / 'index.html'


def splice(name: str, block: str, check: bool) -> int:
    """Put `block` between `<!-- {name}:begin -->` and `<!-- {name}:end -->`.

    With `check`, write nothing and return 1 if the page differs from what the
    tool would write -- the drift guard the docs contract test runs."""
    begin, end = f'<!-- {name}:begin -->', f'<!-- {name}:end -->'
    page = PAGE.read_text(encoding='utf-8')
    if page.count(begin) != 1 or page.count(end) != 1:
        print(f'expected exactly one {begin} / {end} pair in {PAGE}', file=sys.stderr)
        return 2
    head, rest = page.split(begin, 1)
    _, tail = rest.split(end, 1)
    new = f'{head}{begin}\n{block}\n  {end}{tail}'
    if check:
        if new != page:
            print(f'{name} figure has drifted -- re-run its tool', file=sys.stderr)
            return 1
        return 0
    PAGE.write_text(new, encoding='utf-8')
    return 0
