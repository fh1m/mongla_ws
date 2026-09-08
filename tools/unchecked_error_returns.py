#!/usr/bin/env python3
"""Power-of-10 Rule 7 for this stack: is every error return actually CHECKED?

Holzmann, "The Power of 10: Rules for Developing Safety-Critical Code" (IEEE
Computer, 2006), rule 7: *the return value of non-void functions must be checked
by each calling function.* J02 on this register is exactly that violation --
`SurfaceState` calls `set_depth`, swallows the failure, and returns SUCCEED.

Finds functions that report failure -- either `return False, ...` /
`return (False, ...)`, or `MoveResult(...)` -- and then finds call sites that
DISCARD the result: a bare `obj.method(...)` as its own statement (ast.Expr).
"""
import ast, pathlib, collections, sys

# 1) which functions report failure in their return value?
reporters = {}          # name -> (file, lineno, kind)
for f in sorted(pathlib.Path('src').rglob('*.py')):
    p = str(f)
    if '/test' in p or '__pycache__' in p: continue
    try: tree = ast.parse(f.read_text(encoding='utf-8'))
    except SyntaxError: continue
    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)): continue
        kind = None
        for n in ast.walk(fn):
            if not isinstance(n, ast.Return) or n.value is None: continue
            v = n.value
            if isinstance(v, ast.Tuple) and v.elts and \
               isinstance(v.elts[0], ast.Constant) and v.elts[0].value is False:
                kind = '(False, reason)'
            elif isinstance(v, ast.Call) and isinstance(v.func, ast.Name) \
                 and v.func.id == 'MoveResult':
                kind = kind or 'MoveResult'
        if kind: reporters[fn.name] = (p, fn.lineno, kind)

# 2) call sites that throw the result away
discarded = collections.defaultdict(list)
for f in sorted(pathlib.Path('src').rglob('*.py')):
    p = str(f)
    if '/test' in p or '__pycache__' in p: continue
    try: src = f.read_text(encoding='utf-8'); tree = ast.parse(src)
    except SyntaxError: continue
    lines = src.splitlines()
    for n in ast.walk(tree):
        if not isinstance(n, ast.Expr) or not isinstance(n.value, ast.Call):
            continue                      # only a bare call statement discards
        fnc = n.value.func
        name = fnc.attr if isinstance(fnc, ast.Attribute) else \
               (fnc.id if isinstance(fnc, ast.Name) else None)
        if name in reporters:
            discarded[name].append((p, n.lineno, lines[n.lineno-1].strip()[:70]))

print(f'functions that report failure via their return value: {len(reporters)}')
print(f'of those, DISCARDED at some call site: {len(discarded)}\n')
for name in sorted(discarded):
    src_p, src_ln, kind = reporters[name]
    print(f'{name}()  [{kind}]  defined {src_p.replace("src/","")}:{src_ln}')
    for p, ln, txt in discarded[name]:
        print(f'     DISCARDED {p.replace("src/","")}:{ln}   {txt}')
