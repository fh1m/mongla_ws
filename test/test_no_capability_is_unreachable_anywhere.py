"""Every capability module in EVERY package is reached, or says why not.

⛔ THE GAP THIS CLOSES. `mongla_vision/test/test_no_capability_is_built_and_
unreachable.py` is this repo's guard against its own oldest and most expensive
defect -- code that is correct, tested, and not on the vehicle. It works. It also
scans exactly ONE of the seven packages:

    for p in sorted((_PKG / 'mongla_vision').rglob('*.py')):

Its `_imported_by()` already searches the whole tree for importers, so only the
module side was scoped -- and it is scoped because the file lives in that
package's test directory, which is a natural place to write `_PKG`. Six packages
have never been checked at all.

Applying that file's own logic to the rest found four unreachable modules, 875
lines, and one of them is the allocator for the hull now being built:

    mongla_control/geometric_allocation.py        336   the 5-thruster hull's allocator
    mongla_control/hydrodynamics.py               294   the mass band hull_geometry.yaml cites
    mongla_control/nav_filter.py                  138
    mongla_manager/estimator/thrust_model.py      107

⚠ AND THE FIRST COUNT WAS 114, WHICH WAS WRONG. The sweep that produced it used
`(^|[^\\w.])<stem>` to find imports -- excluding a leading dot, which rejects
`from .srot_fc import` and `from mongla_control.fc.srot_fc import`, i.e. nearly
every real import in the tree. Reported as a finding it would have been absurd and
the guard would have been ignored. Look at the value, not the reasoning.

⛔ SCOPE, SO THE TWO FILES CANNOT DISAGREE. This file owns the six non-vision
packages. `mongla_vision` stays with its own guard, which holds the vision
register. Nothing is listed twice, and `test_the_vision_guard_still_exists` fails
if that file is moved or renamed so the split cannot silently become a hole.

⚠ WHY MATCHING BY STEM IS NOT ENOUGH, once more than one package is in scope.
Four basenames are ambiguous across packages -- `factory.py` appears FOUR times
(`mongla_control/fc`, `mongla_sensors`, `mongla_vision`, `mongla_vision/detection`),
and `base.py`, `health.py`, `draw.py` twice each. The vision guard matches on stem,
so `mongla_vision/anchor/health.py` is currently satisfied by ANY import of a
module called `health` -- including `mongla_manager`'s unrelated one, which is
imported twice. It happens to be genuinely wired today, so the assertion is right
by luck rather than by construction. Extending stem matching to the whole tree
would turn that luck into routine false passes, and a guard that passes wrongly is
worse than no guard.

So this file matches the module's real DOTTED PATH, absolute and relative, using
the importer's own position in the package for relative forms.
"""
import pathlib
import re

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[1]
_SRC = _ROOT / 'src'

# This file's scope. mongla_vision is deliberately excluded -- see the docstring.
_OWNED = ('mongla_control', 'mongla_localization', 'mongla_manager',
          'mongla_planner', 'mongla_sensors')
_VISION_GUARD = _SRC / 'mongla_vision' / 'test' / \
    'test_no_capability_is_built_and_unreachable.py'


def _entry_points() -> set:
    """`console_scripts` are invoked by NAME, so no import points at them."""
    out = set()
    for sp in _SRC.rglob('setup.py'):
        try:
            txt = sp.read_text()
        except OSError:
            continue
        out |= {m for m in re.findall(r'=\s*([a-z_0-9.]+):main', txt)}
    return out


# ⚠ THE FULL DOTTED MODULE, not its package. Taking rsplit('.', 1)[0] gives
# `mongla_manager` for every entry point, which exempts NOTHING and then flags
# bringup_check, course_survey, flare_order_send and srot_autotune -- all four of
# which are documented `ros2 run` commands. A guard whose first run reports four
# false positives is a guard that gets switched off.
_ENTRY_POINT_MODULES = set(_entry_points())


def _dotted(path: pathlib.Path) -> str:
    """`src/mongla_control/mongla_control/fc/srot_fc.py` -> `mongla_control.fc.srot_fc`.

    The outer directory is the colcon package; the inner one is the Python
    package, and they share a name. The importable path starts at the inner one.
    """
    parts = path.relative_to(_SRC).with_suffix('').parts
    return '.'.join(parts[1:])


# module path -> the node/module that must import it
WIRED: dict = {}

# ⚠ NOT wired, ON PURPOSE, each with the consumer it needs. A row here is a debt
# with a name, not a silent gap. Same contract as the vision register.
DEFERRED = {
    'mongla_control/mongla_control/geometric_allocation.py': (
        '⭐ THE BIGGEST ONE, and it is the allocator for the hull actually being '
        'built: measured moment arms, roll refused rather than rounded, a dead '
        'thruster expressible. Nothing that flies imports it -- only its own test '
        'and tools/control_bench, which are off the mission path. The live path is '
        '`allocation.py`, an 8x6 matrix of +-1 that describes the COMPETITION '
        'vehicle, and motion_vision imports that one at runtime for its '
        'anti-windup. Waits on the board reporting FRAME_CLASS on the wire (now in '
        'the upstream allocator patch, unsent) so srot_fc can read which '
        'allocation is live and hand motion_vision the matching one. Until then '
        'wiring it would mean GUESSING which hull the board is flying, which is '
        'the whole defect it exists to fix.'),
    'mongla_control/mongla_control/hydrodynamics.py': (
        'computes the added-mass coefficients and the 8.0-13.6 kg mass band that '
        '`hull_geometry.yaml` cites, from the hull shape alone. No control '
        'consumer: every number it produces is an ENVELOPE, and the thing control '
        'needs is NET BUOYANCY, which is a difference of two large numbers that a '
        '2 % shape error swamps. Waits on the float test (half an hour, sealed '
        'hull, find the ballast that hovers), after which the measured value is '
        'what gets shipped and this stays a cross-check rather than a source.'),
    'mongla_control/mongla_control/nav_filter.py': (
        'a complementary/heading filter superseded by mongla_localization: the '
        'RIEKF in `inekf.py` plus `heading_anchor.py` own fusion now, and '
        '`yaw_source=mavlink_ahrs` reads the board directly. Kept only because no '
        'one has confirmed the DVL-era callers are all gone. ⚠ DELETE IT or name '
        'its consumer -- 138 lines of superseded estimator reads as current and '
        'gets copied, which is exactly how draw_strip.py became a problem.'),
    'mongla_manager/mongla_manager/estimator/thrust_model.py': (
        'maps commanded thrust to expected RPM so a demand can be checked against '
        'what the thrusters actually did. Blocked on ROADMAP G2: 958/958 ESC '
        'frames read exactly 0 with nothing attached, so there is no RPM to model '
        'against. Waits on thrusters fitted AND upstream #10, which is what puts '
        'ESC presence on the wire -- without presence a zero RPM and an absent '
        'thruster are the same reading.'),
}


def _modules():
    """Capability modules in the packages this file owns.

    ⚠ Three exemptions, each a real reachability path an import grep cannot see.
    Getting these wrong makes the test cry wolf, and a guard that cries wolf gets
    deleted rather than obeyed:
      * ROS nodes are LAUNCHED, not imported;
      * `console_scripts` entry points are invoked by name;
      * `missions/` is loaded dynamically by `missions/__init__.py`.
    """
    for pkg in _OWNED:
        inner = _SRC / pkg / pkg
        if not inner.is_dir():
            continue
        for p in sorted(inner.rglob('*.py')):
            rel = str(p.relative_to(_SRC))
            if '/test' in rel or p.name == '__init__.py':
                continue
            if '/missions/' in rel or '/launch/' in rel:
                continue
            txt = p.read_text()
            if 'def main(' in txt and 'rclpy' in txt:
                continue                                  # a node
            if _dotted(p) in _ENTRY_POINT_MODULES:
                continue                                  # invoked by name
            yield rel, p, txt


def _import_patterns(dotted: str, pkg_parts: tuple) -> list:
    """Regexes that match a real import of exactly this module.

    Absolute:  `import a.b.c` · `from a.b.c import X` · `from a.b import c`
    Relative:  `from .c import X` · `from . import c` · `from ..b.c import X`
    -- the relative forms are resolved against the IMPORTER's package, so a
    same-named module in another package cannot satisfy this one.
    """
    head, _, leaf = dotted.rpartition('.')
    pats = [
        rf'^\s*import\s+{re.escape(dotted)}\b',
        rf'^\s*from\s+{re.escape(dotted)}\s+import\b',
    ]
    if head:
        pats.append(rf'^\s*from\s+{re.escape(head)}\s+import\s+[^\n]*\b{re.escape(leaf)}\b')
    return pats


def _imported_by(rel: str) -> list:
    """Files that really IMPORT this module -- not ones that MENTION it.

    A substring grep is useless: 'approach', 'health' and 'factory' appear in
    prose all over this tree and read as wired when they are not.
    """
    target = _SRC / rel
    dotted = _dotted(target)
    parts = dotted.split('.')
    leaf = parts[-1]
    abs_pats = _import_patterns(dotted, tuple(parts[:-1]))

    hits = []
    for p in _SRC.rglob('*.py'):
        srel = str(p.relative_to(_SRC))
        if '/test' in srel or p.resolve() == target.resolve():
            continue
        try:
            txt = p.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        if any(re.search(pat, txt, re.M) for pat in abs_pats):
            hits.append(srel)
            continue
        # Relative imports, resolved against THIS importer's own package path.
        imp_parts = _dotted(p).split('.')[:-1]          # the importer's package
        for m in re.finditer(r'^\s*from\s+(\.+)([\w.]*)\s+import\s+([^\n#]+)',
                             txt, re.M):
            dots, mid, names = m.group(1), m.group(2), m.group(3)
            up = len(dots) - 1
            base = imp_parts[:len(imp_parts) - up] if up else list(imp_parts)
            cand = base + ([s for s in mid.split('.') if s] if mid else [])
            # `from .pkg.mod import X` -> cand ends at the module
            if cand == parts:
                hits.append(srel)
                break
            # `from .pkg import mod` -> the module is one of the imported names
            imported = [n.strip().split(' as ')[0].strip() for n in names.split(',')]
            if cand == parts[:-1] and leaf in imported:
                hits.append(srel)
                break
    return hits


def test_the_vision_guard_still_exists():
    """This file owns six packages and delegates mongla_vision. If that file is
    moved or renamed, the split becomes a hole and nothing else would notice."""
    assert _VISION_GUARD.is_file(), (
        f'{_VISION_GUARD.relative_to(_ROOT)} is gone. mongla_vision is then '
        f'unguarded: either add it to _OWNED here or restore that file.')


def test_the_two_registers_do_not_overlap():
    """One module, one register, or they drift and each blames the other."""
    vision = _VISION_GUARD.read_text()
    for rel in list(WIRED) + list(DEFERRED):
        assert f"'{rel}'" not in vision, f'{rel} is registered in BOTH guards'


@pytest.mark.parametrize('rel,consumer', sorted(WIRED.items()))
def test_a_wired_capability_really_is_imported(rel, consumer):
    hits = _imported_by(rel)
    assert hits, (
        f'{rel} is listed as wired into {consumer} but NOTHING imports it. '
        f"That is this repo's oldest defect: the code is correct, the tests "
        f'pass, and the capability does not exist on the vehicle.')


@pytest.mark.parametrize('rel,reason', sorted(DEFERRED.items()))
def test_a_deferred_capability_has_a_named_consumer(rel, reason):
    """The reason must name what it waits for, not merely exist."""
    assert (_SRC / rel).is_file(), f'{rel} is gone; drop its row'
    assert len(reason) > 60, f'{rel}: the reason is too thin to act on'


def test_a_deferred_capability_that_got_wired_is_promoted():
    """Housekeeping with teeth: once something IS imported it must move to WIRED,
    or the register slowly becomes fiction."""
    for rel in DEFERRED:
        hits = _imported_by(rel)
        assert not hits, (
            f'{rel} is now imported by {hits} -- move it from DEFERRED to WIRED '
            f'so the register keeps meaning something')


def test_every_capability_module_is_in_the_register():
    """The point of the whole file: a new module cannot be added to any of these
    five packages without deciding whether anything runs it."""
    known = set(WIRED) | set(DEFERRED)
    missing = []
    for rel, path, txt in _modules():
        if rel in known:
            continue
        if not _imported_by(rel):
            missing.append((rel, len(txt.splitlines())))
    assert not missing, (
        'unreachable and unregistered:\n' +
        '\n'.join(f'  {n:5d} lines  {r}' for r, n in sorted(missing, key=lambda x: -x[1])) +
        '\n\nEach one is either imported by something that runs, or gets a row in '
        'DEFERRED naming the consumer it waits for. There is no third state.')


def test_the_matcher_is_package_aware_not_stem_aware():
    """⛔ THE REGRESSION GUARD FOR THIS FILE'S OWN MATCHER.

    `factory.py` exists four times in this tree and `base.py` twice. A stem
    matcher reports every one of them wired as soon as ANY `factory` is imported,
    which is how a guard starts passing for the wrong reason. Assert that two
    same-named modules in different packages resolve to different importer sets.
    """
    a = 'mongla_control/mongla_control/fc/base.py'
    b = 'mongla_sensors/mongla_sensors/sources/base.py'
    assert (_SRC / a).is_file() and (_SRC / b).is_file(), 'fixtures moved'
    ha, hb = set(_imported_by(a)), set(_imported_by(b))
    assert ha and hb, f'both should be wired: fc/base={ha}, sources/base={hb}'
    assert ha != hb, (
        'the matcher cannot tell two same-named modules apart, so every '
        'ambiguous basename in the tree would report as wired'
    )
