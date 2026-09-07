"""Pin the CARRIED WORK LEDGER's machine-checkable claims against the tree.

⛔ WHY. The ledger in `.claude/plans/` is what decides what each round works on.
On 2026-09-07 an audit found **five of its seven "open engineering" items were
already DONE** and still listed as open — `system-harmony.md` was recorded as
"never written" when it had been written a round earlier and is 177 lines long;
the uplink's `coasted` bit and frozen `target_num` map were both shipped; the
return leg had been measured on the vehicle with a null baseline. The round that
read it picked "the next item" and walked straight into finished work.

A stale ledger is worse than no ledger: it is confidently wrong about the one
thing it exists to answer.

So the claims that CAN be checked against the tree are checked here. This cannot
verify "we ran the experiment" — but it can verify every claim of the form "file
X does not exist yet", "symbol Y is referenced nowhere", or "branches differ",
which is where the rot actually happened.

⚠ These tests fail when the LEDGER is stale, not when the code is broken. The
fix is to update the ledger.
"""
import re
import subprocess
from pathlib import Path

import pytest

_WS = Path(__file__).resolve().parents[3]
_PLANS = _WS.parent.parent / '.claude' / 'plans'


def _ledger() -> str:
    """The most recently modified plan file — the live ledger."""
    cands = sorted(_PLANS.glob('*.md'), key=lambda p: p.stat().st_mtime, reverse=True)
    if not cands:
        pytest.skip('no plan file beside this workspace')
    return cands[0].read_text(errors='ignore')


def _section() -> str:
    t = _ledger()
    i = t.find('### Open engineering, not blocked')
    if i < 0:
        pytest.skip('ledger has no "Open engineering" section')
    j = t.find('### Upstream', i)
    return t[i:j if j > 0 else len(t)]


# --------------------------------------------------------------------------- #
#  Claims of the form "this artifact does not exist yet"
# --------------------------------------------------------------------------- #
def test_ledger_does_not_call_system_harmony_unwritten():
    """It was listed as "never written" while being 177 lines on disk."""
    sec = _section()
    if 'system-harmony' not in sec:
        pytest.skip('ledger no longer mentions system-harmony.md')
    exists = (_WS / '.claude' / 'context' / 'system-harmony.md').is_file()
    claims_unwritten = bool(re.search(
        r'system-harmony[^|\n]*\|[^|\n]*\b(never written|not written|OPEN)\b',
        sec, re.I))
    assert not (exists and claims_unwritten), (
        'system-harmony.md EXISTS but the ledger still calls it unwritten/open')


def test_ledger_does_not_claim_DEPTH_OUT_ARM_LIMIT_still_needs_deleting():
    """r27 item 8 asked for its deletion; it is already gone from src/."""
    sec = _section()
    hits = subprocess.run(['grep', '-rl', 'DEPTH_OUT_ARM_LIMIT', str(_WS / 'src')],
                          capture_output=True, text=True).stdout.split()
    if hits:
        return                      # still present -> the ledger may fairly list it
    assert 'DEPTH_OUT_ARM_LIMIT' not in sec or 'DONE' in sec, (
        'DEPTH_OUT_ARM_LIMIT is already absent from src/ but the ledger still '
        'lists deleting it as outstanding')


# --------------------------------------------------------------------------- #
#  Claims about the uplink, which shipped while the ledger said otherwise
# --------------------------------------------------------------------------- #
def test_uplink_really_does_send_coasted_and_gap_age():
    """The ledger called this "the one with a latent cross-repo bug" long after
    the caller began passing both fields.

    ⛔ THIS TEST'S FIRST VERSION WAS THE `_srot_drive` DEFECT AGAIN. It asserted
    `'coasted=' in body`, and `coasted=False` contains `coasted=` -- so hardcoding
    the field back to a literal passed cleanly. Verified: that injection was the
    one of four this file did NOT catch.

    It now parses the call and requires each field to be DERIVED from `sample`.
    A literal is exactly the regression that matters: sending a constant `False`
    and `0.0` is precisely the state the upstream spec work was about, and it is
    indistinguishable from a live value on the wire.

    Reads the SOURCE, never an import -- in a worktree an import resolves to the
    main workspace's stale `install/` tree.
    """
    import ast
    node = (_WS / 'src' / 'duburi_manager' / 'duburi_manager' / 'auv_manager_node.py')
    tree = ast.parse(node.read_text(), str(node))
    fn = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == '_vision_uplink_tick'), None)
    assert fn is not None, '_vision_uplink_tick is gone -- the ledger entry needs rewriting'

    call = next((c for c in ast.walk(fn)
                 if isinstance(c, ast.Call)
                 and getattr(c.func, 'attr', None) == 'send_landing_target'), None)
    assert call is not None, 'the uplink tick no longer calls send_landing_target'
    kw = {k.arg: k.value for k in call.keywords}

    for field in ('coasted', 'gap_age_s'):
        assert field in kw, f'uplink no longer passes {field}'
        val = kw[field]
        assert not isinstance(val, ast.Constant), (
            f'{field} is hardcoded to {val.value!r}. A constant here is '
            'indistinguishable from a live value on the wire, and sending a '
            'permanent 0.0 is the exact defect the upstream spec work was about.')
        names = {n.id for n in ast.walk(val) if isinstance(n, ast.Name)}
        assert 'sample' in names, (
            f'{field} does not derive from `sample`; it reads {sorted(names)}')

    assert 'target_num' in kw, 'uplink no longer passes target_num'
    tn = kw['target_num']
    assert not isinstance(tn, ast.Constant), (
        'target_num is hardcoded -- that was the original defect: this tick sent '
        'a literal 0 while tools/srot_uplink_check.py sent d.class_id, so neither '
        'was a wire contract')
    assert any(getattr(f, 'id', None) == 'srot_uplink_class_num'
               for f in ast.walk(tn) if isinstance(f, ast.Name)), (
        "target_num must come from the FROZEN class map -- the detector's own "
        'class_id means something different for every model loaded')


def test_ledger_does_not_still_call_the_uplink_open():
    sec = _section()
    if 'coasted' not in sec:
        pytest.skip('ledger no longer tracks the uplink item')
    row = next((ln for ln in sec.splitlines() if 'coasted' in ln), '')
    assert 'DONE' in row, f'uplink ships coasted/gap_age but the ledger row reads: {row.strip()}'


# --------------------------------------------------------------------------- #
#  Claims about branch state
# --------------------------------------------------------------------------- #
def test_the_motion_guard_blocker_is_recorded_as_gone():
    """The `main` merge entry named a blocker -- srot missing main's DVL runaway
    guards. Those files are identical now, so the entry must not still cite it
    as the reason the merge is pending."""
    diff = subprocess.run(
        ['git', '-C', str(_WS), 'diff', '--stat', 'main', 'srot', '--',
         'src/duburi_control/duburi_control/motion_forward.py',
         'src/duburi_control/duburi_control/motion_lateral.py'],
        capture_output=True, text=True)
    if diff.returncode != 0:
        pytest.skip('git diff unavailable (shallow clone or missing branch)')
    if diff.stdout.strip():
        return                      # they still differ -> the old wording is fair
    sec = _section()
    row = next((ln for ln in sec.splitlines() if 'merge' in ln.lower()), '')
    if not row:
        pytest.skip('ledger no longer tracks the main merge')
    assert 'GONE' in row or 'EMPTY' in row, (
        'the motion files are identical between main and srot, but the ledger '
        f'still presents that difference as the blocker: {row.strip()}')


# --------------------------------------------------------------------------- #
#  The audit itself must stay visible
# --------------------------------------------------------------------------- #
def test_the_section_records_that_it_was_audited():
    """Without the audit note the next reader has no reason to distrust it --
    which is exactly the state that cost a round."""
    sec = _section()
    assert 'AUDITED' in sec, (
        'the ledger section lost its audit note; a reader cannot tell whether '
        'these states were verified against the tree or merely carried forward')
