"""Re-identification must fix identity switches WITHOUT welding two objects.

The measurement behind this: 179 identity switches in a 253 s recording of one
person in one room. ByteTrack associates on motion, so a target that leaves and
returns cannot be matched to the track it came from.

⛔ THE FAILURE MODES ARE NOT SYMMETRIC, so neither are these tests. A missed
re-identification costs one extra id. A WRONG one welds two objects into one
identity and every consumer downstream believes the target teleported. Most of
what follows is therefore about refusing.
"""
import numpy as np
import pytest

from mongla_vision.tracking.reid import (IDENTITY_INLIERS, MATCHED, REFUSED,
                                         TrackReid)


def desc(n=64, seed=0):
    rng = np.random.default_rng(seed)
    d = rng.normal(size=(n, 64)).astype(np.float32)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def kp(n=64):
    return np.zeros((n, 2), np.float32)


def matcher_from(table):
    """A stub matcher: descriptor-id -> inliers. Keeps the tests about the
    POLICY, not about XFeat -- which has its own measurements."""
    def _m(ka, da, kb, db):
        return table.get((id(da), id(db)), 0)
    return _m


def test_a_returning_target_reuses_its_old_id():
    old_d, new_d = desc(seed=1), desc(seed=2)
    r = TrackReid(matcher_from({(id(old_d), id(new_d)): 120}))
    r.retire(7, 'person', 0.0, kp(), old_d)
    out = r.identify(kp(), new_d, label='person', now=1.0)
    assert out.state == MATCHED and out.track_id == 7


def test_a_weak_match_is_refused():
    """15 is the TRACKING bar and sits inside the far distribution. Asserting
    identity needs more evidence than following a box."""
    old_d, new_d = desc(seed=1), desc(seed=2)
    r = TrackReid(matcher_from({(id(old_d), id(new_d)): IDENTITY_INLIERS - 1}))
    r.retire(7, 'person', 0.0, kp(), old_d)
    assert r.identify(kp(), new_d, label='person', now=1.0).state == REFUSED


def test_an_ambiguous_match_is_refused_even_when_strong():
    """⛔ THE UNRECOVERABLE CASE. Two identities that both match well is
    exactly where a wrong merge happens, and an absolute bar cannot see it."""
    a, b, new = desc(seed=1), desc(seed=2), desc(seed=3)
    r = TrackReid(matcher_from({(id(a), id(new)): 100,
                                (id(b), id(new)): 90}))
    r.retire(1, 'person', 0.0, kp(), a)
    r.retire(2, 'person', 0.0, kp(), b)
    out = r.identify(kp(), new, label='person', now=1.0)
    assert out.state == REFUSED and 'ambiguous' in out.reason


def test_a_clear_winner_over_a_runner_up_is_accepted():
    a, b, new = desc(seed=1), desc(seed=2), desc(seed=3)
    r = TrackReid(matcher_from({(id(a), id(new)): 200,
                                (id(b), id(new)): 40}))
    r.retire(1, 'person', 0.0, kp(), a)
    r.retire(2, 'person', 0.0, kp(), b)
    out = r.identify(kp(), new, label='person', now=1.0)
    assert out.state == MATCHED and out.track_id == 1


def test_a_different_class_is_never_matched():
    """Geometry alone cannot tell a torpedo from a gate in one venue -- the
    detector's label is the cheap guard against that."""
    old_d, new_d = desc(seed=1), desc(seed=2)
    r = TrackReid(matcher_from({(id(old_d), id(new_d)): 500}))
    r.retire(7, 'gate', 0.0, kp(), old_d)
    assert r.identify(kp(), new_d, label='torpedo', now=1.0).state == REFUSED


def test_a_stale_identity_is_not_revived():
    old_d, new_d = desc(seed=1), desc(seed=2)
    r = TrackReid(matcher_from({(id(old_d), id(new_d)): 500}), max_age_s=10.0)
    r.retire(7, 'person', 0.0, kp(), old_d)
    assert r.identify(kp(), new_d, label='person', now=5.0).state == MATCHED
    assert r.identify(kp(), new_d, label='person', now=99.0).state == REFUSED


def test_an_empty_descriptor_set_is_not_stored():
    """It can never match, and it would occupy a slot a usable identity needs."""
    r = TrackReid(matcher_from({}))
    assert not r.retire(7, 'person', 0.0, kp(0), desc(2))
    assert r.size == 0


def test_capacity_drops_the_OLDEST_not_the_weakest():
    """A recent identity is the one a returning target is most likely to be."""
    r = TrackReid(matcher_from({}), capacity=3)
    for i in range(5):
        r.retire(i, 'person', float(i), kp(), desc(seed=i))
    assert r.size == 3
    ids = {x.track_id for x in r._retired}
    assert ids == {2, 3, 4}, f'kept {ids}, expected the three newest'


def test_forget_removes_an_identity():
    r = TrackReid(matcher_from({}))
    r.retire(7, 'person', 0.0, kp(), desc())
    r.forget(7)
    assert r.size == 0


def test_retiring_the_same_id_twice_replaces_it():
    r = TrackReid(matcher_from({}))
    r.retire(7, 'person', 0.0, kp(), desc(seed=1))
    r.retire(7, 'person', 5.0, kp(), desc(seed=2))
    assert r.size == 1


def test_every_refusal_is_counted_and_named():
    """A re-identifier that silently never fires is indistinguishable from one
    that is switched off -- the defect this repo has produced most often."""
    r = TrackReid(matcher_from({}))
    r.identify(kp(), desc(), label='person', now=0.0)
    r.retire(1, 'person', 0.0, kp(), desc(seed=1))
    r.identify(kp(), desc(seed=9), label='person', now=1.0)
    assert sum(r.refusals.values()) == 2
    assert all(k.strip() for k in r.refusals)


def test_nothing_retired_is_a_refusal_not_a_crash():
    r = TrackReid(matcher_from({}))
    out = r.identify(kp(), desc(), label='person', now=0.0)
    assert out.state == REFUSED and out.track_id is None


def test_the_bar_is_the_measured_identity_bar():
    """Not a new constant: section 25's bar, measured on ROI-cropped
    references -- the same configuration a track crop produces."""
    assert IDENTITY_INLIERS == 40
