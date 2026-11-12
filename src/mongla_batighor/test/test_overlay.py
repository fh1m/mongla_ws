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
    # Two detections and the lock; the track is named on the box it carries.
    assert len(out.points) == 3
    lock = out.points[-1]
    assert lock.thickness == LOCK[1] and lock.thickness > max(p.thickness for p in out.points[:-1])
    texts = {t.text: t.position for t in out.texts}
    assert 'gate 0.80 #7' in texts and 'LOCK shark' in texts
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
    assert out.texts[0].text == 'flow 2 points'


def test_the_flow_count_sits_above_the_stats_clear_of_box_labels():
    """A box at the frame's top edge has its label pinned there; the flow
    count at the top left covered it (seen on the bench, 2026-11-12)."""
    from mongla_interfaces.msg import FlowTracks

    from mongla_batighor.overlay import HUD_LINES, LABEL_PX, annotate, flow_annotate, hud
    m = FlowTracks()
    m.anchor_xy, m.now_xy = [10.0, 20.0], [12.0, 21.0]
    lines = hud('downward', 'm', (640, 480), [], None)
    assert len(lines) == HUD_LINES
    stats = annotate(STAMP, None, None, None, None, None, None, (640, 480), lines)
    top_stat = min(t.position.y for t in stats.texts)
    y = flow_annotate(m, (640, 480)).texts[0].position.y
    assert top_stat - (LABEL_PX + 6) <= y < top_stat and y > 240


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


def test_candidates_are_faint_beneath_the_detections_and_say_what_they_are():
    out = annotate(STAMP, _arr(_det(100, 100, 20, 20)),
                   candidates=_arr(_det(200, 100, 20, 20, label='shark', score=0.1)))
    weak, found = out.points
    assert weak.outline_color.a < found.outline_color.a
    assert weak.thickness == CANDIDATE[1] < found.thickness
    assert 'shark weak 0.10' in [t.text for t in out.texts]


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


def test_a_detection_is_not_drawn_again_as_a_candidate():
    """`candidates` carries the detections too; drawn beneath them, each box showed twice."""
    out = annotate(STAMP, _arr(_det(100, 100, 20, 20)),
                   candidates=_arr(_det(100, 100, 20, 20, score=0.8),
                                   _det(200, 100, 20, 20, score=0.1)))
    assert len(out.points) == 2
    assert [p.points[0].x for p in out.points] == [190, 90]


def test_only_the_frame_being_drawn_lends_its_tracks():
    from mongla_batighor.overlay import same_frame
    t = _arr(_det(100, 100, 20, 20, track='7'))
    t.header.stamp = STAMP
    assert same_frame(t, Time(sec=12, nanosec=5))
    assert not same_frame(t, Time(sec=12, nanosec=33_000_005))
    assert not same_frame(None, STAMP)


def test_nothing_the_tracker_holds_is_left_off_and_each_says_what_it_is():
    """A coasting track or one on a box below the floor is drawn, and named so."""
    out = annotate(STAMP, _arr(_det(100, 100, 20, 20)),
                   tracks=_arr(_det(102, 100, 20, 20, track='7'),
                               _det(200, 200, 20, 20, label='shark', track='8'),
                               _det(300, 300, 20, 20, track='9')),
                   candidates=_arr(_det(200, 200, 20, 20, label='shark', score=0.2)))
    texts = [t.text for t in out.texts]
    assert 'gate 0.80 #7' in texts
    assert '#8 shark weak 0.20' in texts
    assert '#9 gate coasting' in texts
    assert len(out.points) == 3


def test_each_class_keeps_one_colour_and_two_classes_differ():
    from mongla_batighor.overlay import class_colour
    out = annotate(STAMP, _arr(_det(10, 10, 4, 4, label='gate'), _det(50, 50, 4, 4, label='shark'),
                               _det(90, 90, 4, 4, label='gate')))
    a, b, c = (p.outline_color for p in out.points)
    assert a == c and a != b
    assert class_colour('gate') == class_colour('gate')


def test_the_picture_says_how_fast_and_how_late_its_boxes_are():
    from mongla_batighor.overlay import hud
    lines = hud('forward', 'gate_sharks', (640, 480), [0.0, 0.05, 0.10, 0.15], 0.068)
    assert lines == ['forward  gate_sharks  640x480', 'detector 20 Hz  68 ms behind the frame']
    assert hud('downward', None, None, [], None) == [
        'downward  no model named  --', 'detector -- Hz  -- ms behind the frame']
    out = annotate(STAMP, _arr(), hud=lines)
    assert [t.text for t in out.texts] == lines


def test_the_stats_sit_at_the_bottom_once_the_camera_has_said_its_size():
    """At the top they covered the label of a box near the top edge (Kutum, 2026-11-12)."""
    out = annotate(STAMP, _arr(), size=(640, 480), hud=['a', 'b'])
    ys = [t.position.y for t in out.texts]
    assert ys[-1] == 472 and ys[0] < ys[1]
