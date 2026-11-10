#!/usr/bin/env python3
"""Find compound (non-atomic) operations on state shared across threads.

The GIL never made these safe, and PEP 703's per-object locks do not either:
read-modify-write (`self.x += 1`) and check-then-act are two bytecode sequences
with a window between them. Only single dict/list ops are atomic.

Reports classes that (a) start a thread and (b) mutate self-state with a compound
operation, flagging those with no visible lock.
"""
import ast, pathlib, collections

hits = collections.defaultdict(list)
for f in sorted(pathlib.Path('src').rglob('*.py')):
    p = str(f)
    if '/test' in p or '__pycache__' in p: continue
    try: src = f.read_text(encoding='utf-8'); tree = ast.parse(src)
    except SyntaxError: continue
    for cls in ast.walk(tree):
        if not isinstance(cls, ast.ClassDef): continue
        seg = ast.get_source_segment(src, cls) or ''
        if 'threading.Thread' not in seg and 'ThreadPool' not in seg:
            continue
        has_lock = 'Lock()' in seg or 'RLock()' in seg
        for n in ast.walk(cls):
            # read-modify-write on self.<attr>
            if isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Attribute) \
               and isinstance(n.target.value, ast.Name) and n.target.value.id == 'self':
                line = src.splitlines()[n.lineno-1].strip()
                inside_with = 'with self._' in seg[:seg.find(line)][-400:] if line in seg else False
                hits[p].append((n.lineno, cls.name, f'self.{n.target.attr} {type(n.op).__name__}=',
                                has_lock, line[:56]))
print(f'threaded classes with compound self-state mutation: {sum(len(v) for v in hits.values())} site(s)\n')
for p in sorted(hits):
    print(p.replace('src/',''))
    for ln, cls, what, lk, line in sorted(set(hits[p])):
        print(f'   :{ln:5d} {cls:22s} {"lock-in-class" if lk else "NO LOCK IN CLASS"}  {line}')


# --------------------------------------------------------------------------- #
#  B23 / B40 / B44 -- unguarded bare Thread targets                            #
# --------------------------------------------------------------------------- #
# Three bugs of one shape: a `threading.Thread(target=...)` whose body does I/O
# with no exception handler. Each died silently and presented as something else
# (a dead camera, a dead cable, a missing log). Enumerated here so the fourth is
# not found by accident.

def thread_targets_without_a_handler():
    import ast as _ast, pathlib as _pl
    out = []
    for f in sorted(_pl.Path('src').rglob('*.py')):
        p = str(f)
        if '/test' in p or '__pycache__' in p:
            continue
        try:
            src = f.read_text(encoding='utf-8'); tree = _ast.parse(src)
        except SyntaxError:
            continue
        targets = set()
        for n in _ast.walk(tree):
            if isinstance(n, _ast.Call) and 'Thread' in _ast.dump(n.func)[:80]:
                for kw in n.keywords:
                    if kw.arg == 'target':
                        t = kw.value
                        targets.add(t.attr if isinstance(t, _ast.Attribute)
                                    else getattr(t, 'id', ''))
        for fn in _ast.walk(tree):
            if isinstance(fn, _ast.FunctionDef) and fn.name in targets:
                guarded = any(isinstance(x, _ast.ExceptHandler) for x in _ast.walk(fn))
                if not guarded:
                    out.append((p.replace('src/', ''), fn.lineno, fn.name))
    return out


if __name__ == '__main__':
    bad = thread_targets_without_a_handler()
    print(f'\n=== bare Thread targets with NO exception handler: {len(bad)} ===')
    for p, ln, name in bad:
        print(f'   {p}:{ln}  {name}()')
    if not bad:
        print('   (none -- B23/B40/B44 class is clear)')
