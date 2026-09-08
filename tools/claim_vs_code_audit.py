#!/usr/bin/env python3
"""Find functions whose DOCSTRING promises something the BODY does not do.

Every defect this audit has found by reading is the same shape:

  B11   docstring "BNO healthy AND DVL streaming" -> body returns only the BNO
  B16   comment "scale defensively"               -> body never scales
  B26   stub invented an attribute the real class lacked
  B33   worked example inverted vs its own field list
  B25   a reporter that could never report

So: extract each function's docstring, extract the identifiers its body actually
touches, and flag the promises that have no counterpart.

Two rules, both deliberately narrow to keep the hit list readable:

  CONJUNCTION  the docstring joins two backticked/`snake_case` names with
               "and"/"AND", and the body references only one of them.
  CLAIM VERB   the docstring says the function clamps/scales/guards/validates/
               verifies/rejects/refuses/normalises, and no operation in the body
               plausibly corresponds.

Hits are CANDIDATES, not verdicts -- read each one.
"""
import ast, pathlib, re, sys, collections

CLAIM = {
    'clamp':     (r'\bclamp',      ('min','max','clamp','constrain','clip')),
    'scale':     (r'\bscal(e|es|ing)\b', ('*','/','scale','ratio')),
    'normalise': (r'\bnormali[sz]', ('%','normali','fmod','wrap')),
    'reject':    (r'\breject|\brefus', ('raise','return','False','None','abort')),
    'verify':    (r'\bverif(y|ies)',   ('assert','==','!=','check','verify','if')),
}

def body_text(fn, src):
    seg = ast.get_source_segment(src, fn) or ''
    doc = ast.get_docstring(fn) or ''
    return seg.replace(doc, '') if doc else seg

def names_in(fn):
    out = set()
    for n in ast.walk(fn):
        if isinstance(n, ast.Name): out.add(n.id)
        elif isinstance(n, ast.Attribute): out.add(n.attr)
    return out

hits = collections.defaultdict(list)
roots = sys.argv[1:] or ['src']
for root in roots:
    for f in sorted(pathlib.Path(root).rglob('*.py')):
        p = str(f)
        if '/test' in p or '__pycache__' in p or '/sim/' in p:
            continue
        try:
            src = f.read_text(encoding='utf-8'); tree = ast.parse(src)
        except SyntaxError:
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            doc = ast.get_docstring(fn)
            if not doc:
                continue
            bt = body_text(fn, src); nm = names_in(fn)

            # RULE 1 -- conjunction of two identifier-ish names
            for a, b in re.findall(r'`(\w+)`\s+(?:and|AND)\s+`(\w+)`', doc):
                if a == b: continue
                ha = a in nm or a in bt
                hb = b in nm or b in bt
                if ha != hb:
                    miss = b if ha else a
                    hits[p].append((fn.lineno, fn.name, f'doc joins `{a}` AND `{b}` but body never touches `{miss}`'))

            # RULE 2 -- claim verb with no corresponding operation
            for label, (pat, ops) in CLAIM.items():
                if re.search(pat, doc, re.I) and not any(o in bt for o in ops):
                    hits[p].append((fn.lineno, fn.name, f'doc claims to {label.upper()} but body shows no such operation'))

n = sum(len(v) for v in hits.values())
print(f'candidate claim-vs-code mismatches: {n}\n')
for p in sorted(hits):
    print(p)
    for ln, name, why in sorted(set(hits[p])):
        print(f'   :{ln:5d} {name:34s} {why}')
