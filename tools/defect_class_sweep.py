#!/usr/bin/env python3
"""Sweep the whole stack for the defect CLASSES this audit has actually found."""
import ast, pathlib, re, sys, collections

# "Absence is the signal" -- the project's own recurring defect (B05, B10, B25).
ABSENCE = re.compile(r'\bor\s+0\.0\b|\bor\s+0\b(?!\s*[.\w])|getattr\([^)]+,\s*0(\.0)?\)')
findings = collections.defaultdict(list)

for f in sorted(pathlib.Path('src').rglob('*.py')):
    p = str(f)
    if '/test' in p or '__pycache__' in p:
        continue
    try:
        src = f.read_text(encoding='utf-8'); tree = ast.parse(src)
    except SyntaxError:
        continue
    lines = src.splitlines()

    # 1) absence coerced to zero
    for i, ln in enumerate(lines, 1):
        code = ln.split('#', 1)[0]
        if ABSENCE.search(code):
            findings['absence-as-zero'].append((p, i, ln.strip()[:74]))

    for n in ast.walk(tree):
        # 2) bare/blind except with no annotation and no logging
        if isinstance(n, ast.ExceptHandler):
            seg = ast.get_source_segment(src, n) or ''
            blind = n.type is None or (isinstance(n.type, ast.Name) and n.type.id == 'Exception')
            silent = not re.search(r'log|warn|error|print|raise|noqa', seg, re.I)
            if blind and silent:
                findings['silent-blind-except'].append((p, n.lineno, seg.splitlines()[0][:74]))
        # 3) unbounded while (JSF-AV: every loop needs a bound)
        if isinstance(n, ast.While):
            seg = ast.get_source_segment(src, n) or ''
            if not re.search(r'time\.|monotonic|deadline|timeout|break|range|len\(|idx|<|>', seg):
                findings['unbounded-while'].append((p, n.lineno, seg.splitlines()[0][:74]))

for k in sorted(findings):
    v = findings[k]
    print(f'=== {k}: {len(v)} ===')
    for p, ln, txt in v[:14]:
        print(f'   {p.replace("src/","")}:{ln}  {txt}')
    if len(v) > 14: print(f'   ... +{len(v)-14} more')
    print()
