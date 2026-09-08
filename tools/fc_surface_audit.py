#!/usr/bin/env python3
"""Every method called on a flight-controller object, vs what each backend implements.

⛔ THIS EXISTS BECAUSE B30 WAS FOUND BY ACCIDENT.

B30 was `writers.lateral` -- a LAMBDA closing over `pixhawk.send_rc_override`,
stored in a dataclass field, invoked through that field. `SrotFC` implements no
such method, so the vision arrival brake (on by default) raised AttributeError on
the vehicle. Every scan we had looked for the literal pattern `self.pixhawk.<attr>`
in the facade, and this call was three indirections away from that.

So this does not grep for a name shape. It finds every function that RECEIVES an
fc-like object -- by parameter name, by `self.<fcattr>`, or by local alias -- and
records every attribute LOADED off it. Then it diffs against the real `dir()` of
each backend class, so the answer comes from the classes rather than from a list
someone maintains.

READ THE OUTPUT AS A WORKLIST, NOT A VERDICT. An attribute here is only a bug if
the site is actually reachable on that backend and not inside an `is_srot` guard;
pair it with `tools/srot_reachability.py`. It over-reports on purpose -- the
failure that costs us is the one it stays silent about.

Run from the workspace root:  python3 tools/fc_surface_audit.py
"""
import ast, pathlib, sys, collections

sys.path.insert(0, 'src/duburi_control')
sys.path.insert(0, 'src/duburi_manager')

FC_PARAM_NAMES = {'pixhawk', 'fc', 'flight_controller', 'master_fc'}
FC_SELF_ATTRS  = {'pixhawk', 'fc', '_pixhawk', '_fc'}

class FCUse(ast.NodeVisitor):
    """Collect (attr, lineno) for every attribute read off an fc-like value."""
    def __init__(self):
        self.hits = []          # (attr, lineno, how)
        self._fc_locals = set() # local names currently bound to an fc

    def _is_fc(self, node):
        if isinstance(node, ast.Name):
            return node.id in FC_PARAM_NAMES or node.id in self._fc_locals
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
           and node.value.id == 'self':
            return node.attr in FC_SELF_ATTRS
        return False

    def visit_Assign(self, node):
        # track `x = self.pixhawk` / `x = fc`
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
           and self._is_fc(node.value):
            self._fc_locals.add(node.targets[0].id)
        self.generic_visit(node)

    def visit_Attribute(self, node):
        # LOAD only. `self.fc.allow_saturated_depth_arm = ...` is a STORE -- it
        # creates the attribute, it does not require it to pre-exist. Counting
        # stores reported two false positives on the first run.
        if isinstance(node.ctx, ast.Load) and self._is_fc(node.value):
            how = 'self.' + node.value.attr if isinstance(node.value, ast.Attribute) \
                  else node.value.id
            self.hits.append((node.attr, node.lineno, how))
        self.generic_visit(node)

def scan(root):
    per_file = collections.defaultdict(list)
    for f in sorted(pathlib.Path(root).rglob('*.py')):
        if '/test' in str(f) or '__pycache__' in str(f):
            continue
        try:
            tree = ast.parse(f.read_text(encoding='utf-8'))
        except SyntaxError:
            continue
        v = FCUse(); v.visit(tree)
        for attr, ln, how in v.hits:
            per_file[str(f)].append((attr, ln, how))
    return per_file

# --- the real backend surfaces, from the classes themselves -----------------
from duburi_control.fc.srot_fc import SrotFC
from duburi_control.fc.base import FlightController
try:
    from duburi_control.fc.pixhawk_fc import PixhawkFC
    PIX = set(dir(PixhawkFC))
except Exception as e:
    PIX = None
    print(f'(PixhawkFC unavailable: {e})', file=sys.stderr)

SROT = set(dir(SrotFC))
ABC  = set(dir(FlightController))

roots = ['src/duburi_control/duburi_control', 'src/duburi_manager/duburi_manager',
         'src/duburi_planner/duburi_planner']
missing = collections.defaultdict(list)
allattrs = collections.Counter()
for r in roots:
    for fname, hits in scan(r).items():
        for attr, ln, how in hits:
            allattrs[attr] += 1
            if attr.startswith('__'):
                continue
            if attr not in SROT and attr not in ABC:
                missing[attr].append(f'{fname}:{ln} (via {how})')

print(f'distinct attributes called on an fc-like object: {len(allattrs)}')
print(f'NOT on SrotFC and NOT on the ABC: {len(missing)}\n')
for attr in sorted(missing):
    onpix = (PIX is not None and attr in PIX)
    print(f'  {attr:26s} {"(exists on PixhawkFC)" if onpix else "(on NEITHER backend!)"}')
    for site in missing[attr]:
        print(f'      {site}')
