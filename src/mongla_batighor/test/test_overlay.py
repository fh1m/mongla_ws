"""The browser draws the boxes where the detector put them, each layer as itself.

`overlay.annotate` is the whole of the overlay node's logic; these build the
messages the vision stack publishes and read back what Lichtblick would draw.
"""
from builtin_interfaces.msg import Time
from foxglove_msgs.msg import PointsAnnotation
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from mongla_batighor.overlay import (CANDIDATE, FEATURE, LOCK, RUNG, _color, annotate,
                                     calibrate_annotate, fresh)


def _det(cx, cy, w, h, label='gate', score=0.8, track=''):
    d = Detection2D()
    c = d.bbox.center
    (c.position if hasattr(c, 'position') else c).x = float(cx)
    (c.position if hasattr(c, 'position') else c).y = float(cy)
    d.bbox.size_x, d.bbox.size_y = float(w), float(h)
    hyp = ObjectHypothesisWithPose()
    target = getattr(hyp, 'hypothesis', hyp)
    target.class_id, target.score = label, score
    d.results.append(hyp)
    d.id = track
    return d


def _arr(*dets):
    a = Detection2DArray()
    a.detections.extend(dets)
    return a


STAMP = Time(sec=12, nanosec=5)


def test_a_box_lands_on_the_detectors_pixels():
    out = annotate(STAMP, _arr(_det(320, 180, 100, 60)))
    (box,) = out.points
    assert box.type == PointsAnnotation.LINE_LOOP
    assert [(p.x, p.y) for p in box.points] == [(270, 150), (370, 150), (370, 210), (270, 210)]
    assert out.timestamp == STAMP and box.timestamp == STAMP
    assert out.texts[0].text == 'gate 0.80'


def test_every_layer_is_drawn_and_the_lock_stands_out():
    out = annotate(STAMP, _arr(_det(100, 100, 20, 20), _det(300, 100, 20, 20)),
                   tracks=_arr(_det(100, 100, 20, 20, track='7')),
                   lock=_arr(_det(300, 100, 20, 20, label='shark')))
    assert len(out.points) == 4
    lock = out.points[-1]
    assert lock.thickness == LOCK[1] and lock.thickness > max(p.thickness for p in out.points[:-1])
    texts = {t.text: t.position for t in out.texts}
    assert '#7' in texts and 'LOCK shark' in texts
    # The lock is drawn on a box the detector also labels: the two labels
    # must not share a place, or one hides the other (seen on Kutum).
    places = [(round(p.x), round(p.y)) for p in texts.values()]
    assert len(places) == len(set(places)), places


def test_a_missing_layer_is_simply_absent():
    out = annotate(STAMP, _arr())
    assert not out.points and not out.texts
    out = annotate(STAMP, None, lock=_arr(_det(10, 10, 4, 4, label='')))
    assert len(out.points) == 1 and out.texts[0].text == 'LOCK'


def test_flow_pairs_are_drawn_from_anchor_to_now_on_the_frame_they_were_found_in():
    from mongla_interfaces.msg import FlowTracks

    from mongla_batighor.overlay import flow_annotate
    m = FlowTracks()
    m.header.stamp = STAMP
    m.anchor_xy = [10.0, 20.0, 30.0, 40.0]
    m.now_xy = [12.0, 21.0, 33.0, 41.0]
    out = flow_annotate(m)
    segs, dots = out.points
    assert segs.type == PointsAnnotation.LINE_LIST
    assert [(p.x, p.y) for p in segs.points] == [(10, 20), (12, 21), (30, 40), (33, 41)]
    assert [(p.x, p.y) for p in dots.points] == [(12, 21), (33, 41)]
    assert out.timestamp == STAMP and segs.timestamp == STAMP
    assert out.texts[0].text == '2 tracks'


def test_no_pairs_draw_nothing():
    from mongla_interfaces.msg import FlowTracks

    from mongla_batighor.overlay import flow_annotate
    assert not flow_annotate(FlowTracks()).points


def _cal_state(**kw):
    s = {'i': 2, 'n': 20, 'done': False, 'want_cell': 'top left', 'want_tilt': 'flat',
         'hold': 0.5, 'msg': 'tilt the board', 'err': None, 'corners': None,
         'target': None, 'worst_px': None}
    s.update(kw)
    return s


def test_a_calibration_draws_the_cell_to_fill_and_the_corners_found():
    a = calibrate_annotate(_cal_state(corners=[[10.0, 20.0], [30.0, 40.0]],
                                      target=[0.0, 0.0, 320.0, 240.0]), Time(sec=7))
    kinds = {p.type: p for p in a.points}
    assert [(q.x, q.y) for q in kinds[PointsAnnotation.POINTS].points] == [(10, 20), (30, 40)]
    box = kinds[PointsAnnotation.LINE_LOOP].points
    assert [(q.x, q.y) for q in box] == [(0, 0), (320, 0), (320, 240), (0, 240)]
    assert '3/20' in a.texts[0].text and 'top left' in a.texts[0].text
    assert a.texts[1].text == 'tilt the board'
    assert a.timestamp.sec == 7


def test_the_worst_point_is_marked_where_the_fit_put_it():
    a = calibrate_annotate(_cal_state(worst_px=[100.0, 50.0]), Time())
    (cross,) = [p for p in a.points if p.type == PointsAnnotation.LINE_LIST]
    xs = sorted({q.x for q in cross.points})
    ys = sorted({q.y for q in cross.points})
    assert (xs[0] + xs[-1]) / 2 == 100.0 and (ys[0] + ys[-1]) / 2 == 50.0


def test_nothing_found_draws_only_the_instruction():
    a = calibrate_annotate(_cal_state(msg=None), Time())
    assert not a.points and len(a.texts) == 1
    assert calibrate_annotate(_cal_state(done=True), Time()).texts[0].text.startswith('all poses')


def test_the_lock_says_what_it_holds_which_rung_holds_it_and_how_hard():
    """The lock box's class_id is the rung, its id the class observed, its
    score the authority (vision/lock_publish.py); each rung has its colour."""
    for rung, rgba in RUNG.items():
        out = annotate(STAMP, None, lock=_arr(_det(50, 50, 20, 20, label=rung,
                                                    score=0.82, track='gate')))
        (box,) = out.points
        assert box.outline_color == _color(rgba), rung
        assert out.texts[0].text == f'LOCK gate {rung} 0.82'
    assert len({RUNG[r] for r in RUNG}) == len(RUNG)


def test_candidates_are_faint_and_beneath_the_detections():
    out = annotate(STAMP, _arr(_det(100, 100, 20, 20)),
                   candidates=_arr(_det(200, 100, 20, 20, score=0.1)))
    assert out.points[0].outline_color == _color(CANDIDATE[0])
    assert out.points[0].thickness < out.points[1].thickness
    assert len(out.texts) == 1                   # a candidate carries no label


def test_a_contour_is_drawn_as_the_outline_it_carries():
    from mongla_interfaces.msg import TargetContours
    c = TargetContours(class_name=['gate', 'bin'], offset=[0, 3],
                       points=[0, 0, 10, 0, 10, 10, 20, 20, 30, 20, 30, 30, 20, 30])
    rings = [[(p.x, p.y) for p in a.points] for a in annotate(STAMP, contours=c).points]
    assert rings == [[(0, 0), (10, 0), (10, 10)],
                     [(20, 20), (30, 20), (30, 30), (20, 30)]]


def test_the_anchors_features_land_on_the_cameras_pixels_and_go_when_stale():
    from mongla_interfaces.msg import TargetCorrespondences
    m = TargetCorrespondences(image_width=320, image_height=180,
                              image_points=[10.0, 20.0, 160.0, 90.0])
    m.header.stamp = STAMP
    (pts,) = annotate(STAMP, features=m, size=(640, 360)).points
    assert pts.type == PointsAnnotation.POINTS and pts.thickness == FEATURE[1]
    assert [(p.x, p.y) for p in pts.points] == [(20, 40), (320, 180)]
    assert fresh(m, Time(sec=12, nanosec=400_000_005), 0.5)
    assert not fresh(m, Time(sec=13), 0.5) and not fresh(None, STAMP, 0.5)


def test_nothing_is_subscribed_while_nobody_watches():
    """Unwatched, the overlay costs the vehicle nothing (and keeps no
    publisher working for nobody); a new viewer gets layers within a poll,
    and never what an earlier viewer's layers held."""
    from mongla_batighor.overlay import Watched

    class Pub:
        n = 0

        def get_subscription_count(self):
            return self.n

    class Node:
        def __init__(self):
            self.live = set()

        def create_subscription(self, *spec):
            self.live.add(spec[1])
            return spec[1]

        def destroy_subscription(self, s):
            self.live.discard(s)

    node, pub, dropped = Node(), Pub(), []
    w = Watched(node, pub, [(None, 'a', None, 1), (None, 'b', None, 1)],
                on_drop=lambda: dropped.append(1))
    w.poll()
    assert node.live == set()
    pub.n = 1
    w.poll()
    w.poll()
    assert node.live == {'a', 'b'}
    pub.n = 0
    w.poll()
    assert node.live == set() and dropped == [1]
