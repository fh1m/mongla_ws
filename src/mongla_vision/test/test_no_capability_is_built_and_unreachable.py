"""Every capability module is either REACHED by a node, or says why not.

⛔ THIS REPO'S OLDEST DEFECT, and the count is now in double figures:

  * the lock ladder was built, measured, given an entry point -- and appeared
    in no launch file;
  * `vision.lock_s` was held at 0, so the ladder published and nothing read it;
  * the checkpoint bank stored a position that nothing ever passed, so
    `locate(near=...)` and every loop closure were unreachable;
  * `device_path` was read, logged, and ignored;
  * loop closure was wired into `vision_pi.launch.py` while `bringup`
    includes `vision.launch.py`;
  * and in ONE session, five more: time_to_contact, approach, scale_check,
    confidence_calibration and anchor/health were each written, tested, and
    imported by nothing.

Every one looked finished. Tests passed, the code was correct, and the
capability did not exist on the vehicle. That is what makes this failure mode
expensive: it is invisible to every check except this one.

So each module below is either imported by something that runs, or carries an
explicit reason with the consumer it is waiting for. Adding a module without
deciding which forces the decision at the time it is written -- which is the
only moment anyone knows the answer.
"""
import pathlib
import re

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1]
_SRC = _PKG.parents[0]


def _entry_points() -> set:
    """`console_scripts` are invoked by NAME, so no import ever points at
    them. Read from setup.py rather than listed here, or the two drift."""
    out = set()
    for sp in _SRC.rglob('setup.py'):
        try:
            txt = sp.read_text()
        except OSError:
            continue
        out |= {m.rsplit('.', 1)[-1]
                for m in re.findall(r'=\s*([a-z_0-9.]+):main', txt)}
    return out


_ENTRY_POINTS = _entry_points()

# module path -> the node or module that must import it
WIRED = {
    'mongla_vision/mongla_vision/anchor/loop_closure.py': 'lock_node.py',
    'mongla_vision/mongla_vision/anchor/health.py': 'lock_node.py',
    'mongla_vision/mongla_vision/anchor/bank.py': 'lock_node.py',
    'mongla_vision/mongla_vision/tracking/lock_state.py': 'lock_node.py',
    # Promoted 2026-09-25: wired ADVISORY -- lock_node measures and reports
    # the field-of-view margin but scales no demand. The demand-scaling half
    # is a control change and waits on a pool day; see ladder-register.md.
    'mongla_vision/mongla_vision/tracking/visibility.py': 'lock_node.py',
}

# ⚠ NOT wired, ON PURPOSE, each with the consumer it needs. A row here is a
# debt with a name, not a silent gap.
DEFERRED = {
    'mongla_vision/mongla_vision/tracking/cascade.py': (
        'the association cascade that makes appearance affordable -- it runs '
        'the expensive stage only on detections motion could not explain, and '
        'carries the ego-motion stage no published tracker has, because ours '
        'is MEASURED rather than fitted from image content. Waits on '
        'tracker_node, which currently calls its tracker library directly; '
        'inserting a cascade is a change to the association hot path and its '
        'rate must be measured on the vehicle first.'),
    'mongla_vision/mongla_vision/tracking/reid.py': (
        'the 179-switch fix, and the one piece of the BoT-SORT / McByte++ '
        'recipe an embedded budget can normally not afford -- a dedicated '
        'Re-ID network costs 15-25 ms/frame, while XFeat is ALREADY loaded '
        'for the anchor rung at 701 FPS on the Hailo. Waits on tracker_node '
        'supplying crop descriptors at track birth and death, which is a '
        'change to the tracker hot path and needs its rate measured on the '
        'vehicle first.'),
    'mongla_vision/mongla_vision/tracking/presence.py': (
        'the rung BELOW detection -- track-before-detect, which postpones the '
        'threshold and accumulates sub-threshold evidence across frames. ⛔ IT '
        'MAY NEVER BE WIRED, and the reason is B-59: on real water the band '
        'below the bar is not faint signal, it is confident open-water FALSE '
        'POSITIVES. The gate model fires on 90.8 % of gate-free frames at the '
        'shipped 0.15, with half-frame boxes over empty turquoise at '
        '0.44-0.56, and separation between gate-present and gate-absent '
        'footage is +0.0 points at 0.15 and 0.30. Accumulating that would '
        'manufacture a confident track out of hallucination. Its constants '
        '(GATE_PX 40, MIN_RUN 3) come from a PERSON IN AIR, where the control '
        'is 0.090; the one water control measured 0.360, which would need '
        'MIN_RUN 6 to stay under 1 % false chains -- longer than the median '
        'gap. Precedent: continuity.py, whose person-in-air constants were '
        'also not shipped. Revisit only after B-59 is fixed, the bar is '
        're-derived WITH negative clips, and the constants are re-swept on '
        'water.'),
    'mongla_vision/mongla_vision/continuity.py': (
        'NOW EXERCISED by tools/continuity_from_bag.py, which produced the '
        'measurement it was written for (section 50): 338 real gaps, p50 '
        '116 ms, p90 1.12 s, max 7.64 s, and one rung -- the 0.20 s freshness '
        'zero -- outrun by 35 % of them. It stays DEFERRED because no NODE '
        'imports it: it is an offline analyser, and the ladder constants it '
        'implies are NOT shipped, since the recording is a person in air and '
        'gap durations there belong to the subject, not to underwater '
        'detection. Promote it when a water bag exists and the constants are '
        're-derived from that.'),
    'mongla_vision/mongla_vision/draw_strip.py': (
        'NOT DEAD after all -- corrected 2026-09-25. Its old note said '
        '"superseded UI ... delete it or revive it deliberately", and the '
        'orphan sweep agreed because it scanned only src/. '
        '`tools/render_hud_stills.py` and `tools/render_real_stills.py` both '
        'import `render_ui_strip` to generate the site HUD imagery, so this '
        'is live code with an out-of-tree consumer. It stays DEFERRED rather '
        'than WIRED because no NODE imports it -- the register tracks the '
        'vehicle, and a tools-only consumer is a real but different kind of '
        'reachability. ⚠ Nearly deleted on the strength of a sweep that could '
        'not see tools/; the sweep now scans it.'),
    'mongla_vision/mongla_vision/time_to_contact.py': (
        'needs a consumer in the approach/standoff control path, which does '
        'not exist yet -- adding one now would be a control change justified '
        'by no measurement. Blocked behind an in-water standoff run.'),
    'mongla_vision/mongla_vision/approach.py': (
        'the BAND is measured (section 23) but the controller that would act '
        'on back-off does not exist; vision_verbs closes range only. Wiring '
        'it means a new verb, which is a mission-surface change.'),
    'mongla_vision/mongla_vision/flow/scale_check.py': (
        'both heights exist but the barometer reports "not initialised" on '
        'this hull, so the comparison has one side. Wire it the first time a '
        'working baro and the tile grating run together.'),
    'mongla_vision/mongla_vision/detection/confidence_calibration.py': (
        'ships OFF by design: its gain is DECLARED, not measured, and it '
        'needs a labelled per-venue set to calibrate. Wiring it before that '
        'would put an unmeasured constant in the detection path.'),
}


def _modules():
    """Capability modules only.

    ⚠ Three exemptions, each a real reachability path that an import grep
    cannot see -- getting these wrong makes the test cry wolf and it will be
    deleted rather than obeyed:
      * ROS nodes are LAUNCHED, not imported;
      * `console_scripts` entry points are invoked by name;
      * `missions/` is loaded dynamically by `missions/__init__.py`.
    """
    for p in sorted((_PKG / 'mongla_vision').rglob('*.py')):
        rel = str(p.relative_to(_PKG.parent))
        if '/test' in rel or rel.endswith('__init__.py'):
            continue
        if '/missions/' in rel or '/launch/' in rel:
            continue
        txt = p.read_text()
        if 'def main(' in txt and 'rclpy' in txt:
            continue                      # a node
        if p.stem in _ENTRY_POINTS:
            continue                      # invoked by name, never imported
        yield rel, p, txt


def _imported_by(name: str) -> list:
    """Files that actually IMPORT this module -- not ones that MENTION it.

    ⚠ A plain substring grep is useless here: "approach" and "health" appear
    in prose all over this codebase and both read as wired when they were
    not. Only a real import statement counts.
    """
    stem = pathlib.Path(name).stem
    hits = []
    for p in _SRC.rglob('*.py'):
        rel = str(p.relative_to(_SRC))
        if '/test' in rel or p.name == pathlib.Path(name).name:
            continue
        try:
            txt = p.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        if re.search(rf'^\s*(from\s+\S*\b{stem}\b\s+import|'
                     rf'from\s+\S+\s+import\s+[^\n]*\b{stem}\b|'
                     rf'import\s+\S*\b{stem}\b)', txt, re.M):
            hits.append(rel)
    return hits


@pytest.mark.parametrize('rel,consumer', sorted(WIRED.items()))
def test_a_wired_capability_really_is_imported(rel, consumer):
    hits = _imported_by(rel)
    assert hits, (
        f'{rel} is listed as wired into {consumer} but NOTHING imports it. '
        f'That is this repo\'s oldest defect: the code is correct, the tests '
        f'pass, and the capability does not exist on the vehicle.')


@pytest.mark.parametrize('rel,reason', sorted(DEFERRED.items()))
def test_a_deferred_capability_has_a_named_consumer(rel, reason):
    """The reason must name what it is waiting for, not merely exist."""
    assert (_SRC / rel).is_file(), f'{rel} is gone; drop its row'
    assert len(reason) > 60, f'{rel}: the reason is too thin to act on'


def test_a_deferred_capability_that_got_wired_is_promoted():
    """Housekeeping with teeth: once something IS imported, it must move to
    WIRED, or the register slowly becomes fiction.

    ⛔ AN IMPORT FROM ANOTHER DEFERRED MODULE IS NOT REACHABILITY, and reading
    it as one would invert this test's purpose. `presence.py` imports
    `visibility.py` for its arming interlock; both are deferred, so nothing on
    the vehicle reaches either, and promoting visibility on that evidence
    would put a module in WIRED that no running code can touch -- the exact
    fiction this file exists to prevent. Only an importer that is itself
    reachable counts.
    """
    for rel in DEFERRED:
        hits = [h for h in _imported_by(rel) if h not in DEFERRED]
        assert not hits, (
            f'{rel} is now imported by {hits} -- move it from DEFERRED to '
            f'WIRED so the register keeps meaning something')


def test_every_capability_module_is_in_the_register():
    """The point of the whole file: a new module cannot be added without
    deciding whether anything runs it."""
    known = set(WIRED) | set(DEFERRED)
    missing = []
    for rel, path, txt in _modules():
        if rel in known:
            continue
        if len(txt.splitlines()) < 80:
            continue                      # small helpers, not capabilities
        if _imported_by(rel):
            continue
        missing.append(rel)
    assert not missing, (
        "capability modules that nothing imports, in neither register:\n  "
        + "\n  ".join(missing)
        + "\nDecide NOW: wire it, or add a DEFERRED row naming the consumer "
          "it waits for. Deciding later means never -- that is how ten of "
          "these accumulated.")
