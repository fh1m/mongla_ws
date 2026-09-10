"""Two classes in frame must stay two classes after a round trip.

`array_to_detections` hard-coded `class_id=0` with the note "integer class_id
not preserved". Measured consequence, not a cosmetic loss: the trackers'
`_name_from(cid, dets)` resolves a class id back to a name by finding the first
detection carrying it, so with `blood` and `fire` both in frame -- the bin
task's two classes -- every track resolved to `'blood'`. The trackers also saw
a single class for everything, so class-aware association could not keep two
different objects apart.
"""
import pytest
from std_msgs.msg import Header

from duburi_vision.detection.detector import Detection
from duburi_vision.detection.messages import (array_to_detections, class_index,
                                              detections_to_array)


@pytest.fixture(autouse=True)
def _fresh_registry():
    """`_CLASS_INDEX` is module-global and never cleared.

    Without this, `test_it_is_a_registry_and_not_a_hash` leaves 20 entries in
    it and every later test sees a registry seeded by whatever ran first --
    the same leakage shape as the loader's `_ERRORS`/`_REJECTED` lists. The
    assertions here survive it, because first-seen order is stable within a
    run, but making the index depend on test ORDER is how a real defect gets
    masked later.
    """
    from duburi_vision.detection import messages
    messages._CLASS_INDEX.clear()
    yield
    messages._CLASS_INDEX.clear()


def _round_trip(dets):
    return array_to_detections(detections_to_array(dets, Header()))


def _two():
    return [Detection(class_id=3, class_name='blood', score=0.9,
                      xyxy=(10.0, 10.0, 50.0, 50.0)),
            Detection(class_id=7, class_name='fire', score=0.8,
                      xyxy=(80.0, 80.0, 120.0, 120.0))]


def test_two_classes_do_not_collapse_into_one():
    ids = [d.class_id for d in _round_trip(_two())]
    assert len(set(ids)) == 2, (
        f'both classes came back as {ids} -- the trackers see one class for '
        f'everything and cannot keep two objects apart')


@pytest.mark.parametrize('tracker_mod', ['roboflow_tracker', 'bytetrack'])
def test_a_track_resolves_to_its_OWN_class_name(tracker_mod):
    """Both tracker backends map an id back to a name the same way."""
    import importlib
    mod = importlib.import_module(f'duburi_vision.tracking.{tracker_mod}')
    name_from = getattr(mod, '_name_from', None) or mod._class_name_from_id
    back = _round_trip(_two())
    got = {d.class_name: name_from(d.class_id, back) for d in back}
    assert got == {'blood': 'blood', 'fire': 'fire'}, got


def test_the_label_is_the_identity_that_survives_a_model_switch():
    """The same label gets the same index however it arrives.

    A detector's integer index belongs to whichever model minted it -- `gate`
    is 0 in one model and something else in the next -- so the label is the
    only identity that survives a switch. Two detections of one class must
    therefore agree, whatever integers they carried in.
    """
    dets = [Detection(class_id=3, class_name='gate', score=0.9,
                      xyxy=(0.0, 0.0, 10.0, 10.0)),
            Detection(class_id=99, class_name='gate', score=0.7,
                      xyxy=(20.0, 20.0, 30.0, 30.0))]
    ids = {d.class_id for d in _round_trip(dets)}
    assert len(ids) == 1, f'one label produced {ids}'


def test_the_index_is_stable_across_calls():
    """A track outlives a frame, so an id that changed per message would break
    association exactly when a target is briefly occluded."""
    first = [d.class_id for d in _round_trip(_two())]
    for _ in range(5):
        assert [d.class_id for d in _round_trip(_two())] == first


def test_it_is_a_registry_and_not_a_hash():
    """A hash is stable across processes and can COLLIDE, and a collision here
    silently merges two classes -- the failure being fixed, reintroduced by a
    different route. Indices stay small and dense instead."""
    got = [class_index(f'class_{i}') for i in range(20)]
    assert len(set(got)) == 20
    assert max(got) < 200, (
        f'indices reached {max(got)} -- that is a hash, not a registry')
