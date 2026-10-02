"""The ladder climbs rungs, not only falls down them.

⛔ THE ONE-WAY RATCHET THIS FIXES. `_follower.reset()` was called on an
accepted DETECTION and nowhere else. So when LK's points died mid-gap --
exactly what the survival and forward-backward gates exist to detect -- the
fast ~50 Hz rung stayed dead until a detection came back, **even though the
anchor was still finding the target every 222 ms**. Every rung could fall and
none could climb.

⭐ The anchor's box is a legitimate seed, and a better one than the follower's
own: it is fitted frame-to-REFERENCE against a stored patch, so unlike a
carried box it has not drifted. `goodFeaturesToTrack` inside it finds the same
corners LK wanted.

⚠ THE TWO REFUSALS ARE THE DESIGN, not caution:
  * a LIVE follower is never reseeded -- that would replace a frame-fresh box
    with one up to 222 ms old;
  * a weak fit is never used -- restarting the fast rung on a bad box hands it
    FULL authority for the rest of the gap, which is the failure the acting
    bar exists to prevent arriving by another door.

The bar is `_ANCHOR_RESEED_INLIERS` = 120, deliberately above the anchor's own
100-inlier trust bar: a box good enough to REPORT is not automatically good
enough to RESTART a rung on.
"""
from __future__ import annotations

import numpy as np

from mongla_vision.tracking.follower import Follower
import sys as _sys_ct
import pathlib as _pl_ct
_sys_ct.path.insert(0, str(_pl_ct.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_of, code_of_file  # noqa: E402  (issue #22)


def _textured(h=240, w=320, seed=0):
    """A frame with real corners, so `goodFeaturesToTrack` has something to
    find -- a flat gradient yields no features and would make every reseed
    'fail' for the wrong reason."""
    rng = np.random.default_rng(seed)
    img = (rng.random((h, w)) * 255).astype(np.uint8)
    for y in range(0, h, 16):
        img[y:y + 3, :] = 255
    for x in range(0, w, 16):
        img[:, x:x + 3] = 0
    return img


def test_a_dead_follower_can_be_reseeded_from_a_box():
    """The mechanism the heal depends on: `reset` revives a follower that has
    dropped, returning a positive point count."""
    f = Follower()
    assert not f.active, 'a fresh follower is not active'
    n = f.reset(_textured(), (80.0, 60.0, 200.0, 180.0))
    assert n > 0, 'reset found no features in a textured box'
    assert f.active


def test_the_revived_follower_actually_tracks():
    """⭐ A reseed that produces points but cannot step would be a false heal:
    the ladder would believe the fast rung was back."""
    f = Follower()
    g0 = _textured(seed=1)
    assert f.reset(g0, (80.0, 60.0, 200.0, 180.0)) > 0
    r = f.step(g0)                      # same frame: zero motion
    assert r.ok, 'a revived follower could not carry its box'
    assert r.points >= 8


def test_a_featureless_box_refuses_rather_than_pretending():
    """⛔ Reseeding into flat water must FAIL, not return a follower that
    reports a box it cannot track. `reset` returns 0 and stays inactive."""
    f = Follower()
    flat = np.full((240, 320), 128, np.uint8)
    assert f.reset(flat, (80.0, 60.0, 200.0, 180.0)) == 0
    assert not f.active


def test_the_reseed_bar_is_stricter_than_the_report_bar():
    """⚠ THE BAR IS THE POINT. A box good enough to report (100 inliers) is
    not automatically good enough to restart the fast rung on, because a
    reseeded follower carries full authority for the rest of the gap."""
    from mongla_vision.lock_node import _ANCHOR_RESEED_INLIERS

    assert _ANCHOR_RESEED_INLIERS > 100, \
        'the reseed bar must exceed the anchor trust bar'
    assert _ANCHOR_RESEED_INLIERS <= 200, \
        'an absurdly high bar would mean the heal never fires'


def test_the_heal_is_wired_into_the_ladder_loop():
    """⛔ THE §9 CHECK, INLINE. The mechanism above is worthless if the ladder
    never calls it -- which is the defect this repo keeps finding. Assert the
    call site exists, guarded by both refusals."""
    from pathlib import Path

    import ast

    src_path = (Path(__file__).resolve().parents[1] / 'mongla_vision'
                / 'lock_node.py')
    tree = ast.parse(src_path.read_text())

    # ⛔ AST, NOT A SUBSTRING. A substring check passed when the call was
    # neutered to `reset(gray, ab) if False else 0` -- the text was still
    # there and the test still agreed. Find a real `reset(gray, ab)` CALL,
    # then require its enclosing `if` to mention both guards.
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        cond = ast.dump(node.test)
        body = ast.dump(ast.Module(body=node.body, type_ignores=[]))
        if ("attr='reset'" in body and "id='gray'" in body
                and "id='ab'" in body):
            found.append((cond, body))
    assert found, 'no reset(gray, ab) call inside any if-statement'
    # Several `if`s enclose the call (the anchor branch, the pose branch, the
    # guard itself). The INNERMOST is the guard -- pick the shortest body.
    cond, body = min(found, key=lambda cb: len(cb[1]))
    assert "attr='active'" in cond, \
        'the reseed is not guarded on the follower being dead'
    assert 'UnaryOp' in cond and 'Not' in cond, \
        'the guard does not NEGATE active -- a live follower would be reseeded'
    assert '_ANCHOR_RESEED_INLIERS' in cond, \
        'the reseed is not gated on fit quality'
    # and the call must not be short-circuited by a constant
    assert 'IfExp' not in body, \
        'the reset call is behind a conditional expression -- is it neutered?'


def test_a_reseed_is_counted_so_a_pool_day_can_see_it():
    """A heal that fires invisibly cannot be trusted or debugged: the operator
    needs to know the fast rung came back rather than inferring it."""
    from pathlib import Path

    src = code_of_file(Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py')
    assert 'self._reseeds' in src
    assert 'reseeded from the' in src, 'the first reseed must be logged'
