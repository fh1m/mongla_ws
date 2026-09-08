#!/usr/bin/env python3
"""Which duburi_control functions can actually RUN on the srot backend?

⛔ USE THIS INSTEAD OF GREP WHEN CLASSIFYING A BUG AS "not on the srot path".

Hand-tracing call sites is how B18 was mis-classified and how B30 -- a live,
default-on AttributeError in the vision arrival brake -- stayed hidden. The
specific failure was a `grep ... | head` that truncated its own evidence at ten
lines, and a habit of reasoning at MODULE granularity ("motion_writers is
imported by motion_vision, so it is live") when the question is SYMBOL
granularity ("is thrust_loop actually called?"). Module granularity gave a false
positive on B12 and a false negative on B18 in the same pass.

This walks CALLS, not imports, seeded from the real srot entry verbs: the facade
verbs that are neither collapsed onto the board (MOVE_VERBS) nor refused
(UNSUPPORTED_VERBS), plus the two ported vision verbs.

IT OVER-APPROXIMATES ON PURPOSE. It walks both sides of an `if is_srot(...)`
branch, so a "REACHABLE" verdict means "check this by hand"; a "not reachable"
verdict is the strong one. That is the safe direction for a question whose wrong
answer is "we shipped a live bug".

Run from the workspace root:  python3 tools/srot_reachability.py
"""
import ast, pathlib, sys, collections

ROOT = pathlib.Path('src/duburi_control/duburi_control')
mods = {}                      # modname -> ast.Module
for f in ROOT.rglob('*.py'):
    rel = f.relative_to(ROOT).with_suffix('')
    mods['.'.join(rel.parts)] = (ast.parse(f.read_text(encoding='utf-8')), f)

funcs = {}                     # qualname -> (node, modname)
methods_of = collections.defaultdict(set)   # classname -> set(methodname)
imports = collections.defaultdict(dict)     # modname -> {localname: targetmod}

for m, (tree, _) in mods.items():
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            if isinstance(node, ast.ImportFrom) and node.module:
                tgt = node.module.lstrip('.')
                for a in node.names:
                    imports[m][a.asname or a.name] = f'{tgt}.{a.name}' if tgt else a.name
            else:
                for a in node.names:
                    imports[m][a.asname or a.name.split('.')[-1]] = a.name
        elif isinstance(node, ast.FunctionDef):
            funcs[f'{m}.{node.name}'] = (node, m)
        elif isinstance(node, ast.ClassDef):
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef):
                    funcs[f'{node.name}.{sub.name}'] = (sub, m)
                    methods_of[node.name].add(sub.name)

# Duburi's facade surface includes its mixins (VisionVerbs).
FACADE = {c for c in methods_of if c in ('Duburi', 'VisionVerbs')}

def resolve(call, modname):
    """Best-effort: which qualnames could this Call reach?"""
    out = set()
    fn = call.func
    if isinstance(fn, ast.Attribute):
        # self.foo(...) -> a facade method
        if isinstance(fn.value, ast.Name) and fn.value.id == 'self':
            for c in FACADE:
                if fn.attr in methods_of[c]:
                    out.add(f'{c}.{fn.attr}')
        # mod.foo(...) where mod was imported
        elif isinstance(fn.value, ast.Name):
            tgt = imports[modname].get(fn.value.id)
            if tgt and f'{tgt}.{fn.attr}' in funcs:
                out.add(f'{tgt}.{fn.attr}')
            elif f'{fn.value.id}.{fn.attr}' in funcs:
                out.add(f'{fn.value.id}.{fn.attr}')
    elif isinstance(fn, ast.Name):
        tgt = imports[modname].get(fn.id)
        if tgt and tgt in funcs:
            out.add(tgt)
        elif f'{modname}.{fn.id}' in funcs:
            out.add(f'{modname}.{fn.id}')
        else:                                  # class instantiation
            for c in methods_of:
                if fn.id == c:
                    out.add(f'{c}.__init__')
    return out

# --- srot entry points ------------------------------------------------------
ENTRIES = [f'Duburi.{v}' for v in (
    'arm','calc_distance','calibrate_depth','disarm','dvl_connect','fire',
    'head','mission_reset','set_mode','unlock_heading')]
ENTRIES += ['VisionVerbs.vision_align', 'VisionVerbs.vision_move']
ENTRIES = [e for e in ENTRIES if e in funcs]

seen, queue = set(), list(ENTRIES)
while queue:
    q = queue.pop()
    if q in seen or q not in funcs:
        continue
    seen.add(q)
    node, modname = funcs[q]
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call):
            for t in resolve(sub, modname):
                if t not in seen:
                    queue.append(t)

print(f'entry points: {len(ENTRIES)}')
print(f'reachable functions on srot: {len(seen)}\n')
mods_hit = sorted({funcs[q][1] for q in seen})
print('MODULES with at least one reachable function:')
for m in mods_hit:
    print('  ', m)
print('\nMODULES with NONE reachable:')
for m in sorted(mods):
    if m not in mods_hit:
        print('  ', m)

# --- the open duburi_control bugs, by defect SITE ---------------------------
SITES = {
 'B06 heading-lock timeout leaves Ch4': ('heading_lock', None),
 'B07 arc() fabricated heading':        ('motion_forward', 'arc'),
 'B08 prime_alt_hold ungated':          ('motion_depth', 'prime_alt_hold'),
 'B12 heading fabricated due-north':    ('motion_writers', 'thrust_loop'),
 'B13 motion_yaw derivative no dt':     ('motion_yaw', None),
 'B15 LOCK_* deadbands unenforced':     ('heading_lock', None),
 'B17 HeadingLock.stop() unstarted':    ('heading_lock', 'stop'),
 'B18 percent_to_pwm truncation':       ('pixhawk', 'percent_to_pwm'),
 'J01 HeadingLock.suspend nesting':     ('heading_lock', 'suspend'),
}
print('\n--- OPEN duburi_control bugs: is the defect site reachable on srot? ---')
for name, (mod, fname) in SITES.items():
    if fname:
        hits = [q for q in seen if funcs[q][1] == mod and q.endswith('.' + fname)]
    else:
        hits = [q for q in seen if funcs[q][1] == mod]
    verdict = 'REACHABLE  <-- ' + ', '.join(sorted(hits)) if hits else 'not reachable'
    print(f'  {name:38s} {verdict}')

# --- why is percent_to_pwm reachable? print the path ------------------------
print('\n--- path to Pixhawk.percent_to_pwm ---')
parent = {}
seen2, queue2 = set(), list(ENTRIES)
while queue2:
    q = queue2.pop(0)
    if q in seen2 or q not in funcs: continue
    seen2.add(q)
    node, modname = funcs[q]
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call):
            for t in resolve(sub, modname):
                if t not in seen2 and t not in parent:
                    parent[t] = q
                    queue2.append(t)
tgt = 'Pixhawk.percent_to_pwm'
chain = []
while tgt:
    chain.append(tgt); tgt = parent.get(tgt)
print('   ' + '\n     <- '.join(chain))
