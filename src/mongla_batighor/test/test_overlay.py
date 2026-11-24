"""The browser draws the boxes where the detector put them, each layer as itself.

`overlay.annotate` is the whole of the overlay node's logic; these build the
messages the vision stack publishes and read back what Lichtblick would draw.
"""
import pytest
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


def _drawn(points):
    """Point sets that draw something: the frame's stamp mark has no points."""
    return [p for p in points if p.points]


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


def _rects(out):
    """Each label's box, as the placer measures it: baseline-left position."""
    from mongla_batighor.overlay import CHAR_EM
    return [(t.position.x, t.position.y - t.font_size,
             t.position.x + len(t.text) * t.font_size * CHAR_EM, t.position.y)
            for t in out.texts]


def test_no_label_covers_another_and_none_leaves_the_picture():
    # The cockpit shot: a gate at the right edge with a shark box on its top
    # corner, the shark's label drawn over the gate's and off the picture.
    size = (640, 480)
    out = annotate(STAMP, _arr(_det(560, 250, 130, 210, track='13'),
                               _det(625, 180, 28, 60, label='shark', score=0.72, track='25')),
                   tracks=_arr(_det(560, 250, 130, 210, track='13'),
                               _det(625, 180, 28, 60, label='shark', track='25')),
                   size=size)
    rects = _rects(out)
    assert len(rects) == 2
    for i, a in enumerate(rects):
        assert 0 <= a[0] and a[2] <= size[0] + 1e-6 and 0 <= a[1] and a[3] <= size[1], a
        for b in rects[i + 1:]:
            apart = a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1]
            assert apart, (a, b)


def test_a_missing_layer_is_simply_absent():
    out = annotate(STAMP, _arr())
    assert not _drawn(out.points) and not out.texts
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


def test_the_flow_count_sits_on_the_bottom_line_clear_of_box_labels():
    """A box at the frame's top edge has its label pinned there; the flow
    count at the top left covered it (seen on the bench, 2026-11-12). The
    bottom line is free now that the camera's state is a caption."""
    from mongla_interfaces.msg import FlowTracks

    from mongla_batighor.overlay import flow_annotate
    m = FlowTracks()
    m.anchor_xy, m.now_xy = [10.0, 20.0], [12.0, 21.0]
    assert flow_annotate(m, (640, 480)).texts[0].position.y == 472


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


def test_the_caption_says_how_fast_and_how_late_its_boxes_are():
    from mongla_batighor.overlay import caption, hud
    lines = hud('forward', 'gate_sharks', (640, 480), [0.0, 0.05, 0.10, 0.15], 0.068)
    assert lines == ['forward  gate_sharks  640x480', 'detector 20 Hz  68 ms behind the frame']
    assert hud('downward', None, None, [], None) == [
        'downward  model unknown  --', 'detector -- Hz  -- ms behind the frame']
    c = caption('forward', 'gate_sharks', (640, 480), [0.0, 0.05, 0.10, 0.15], 0.068)
    assert (c['state'], c['rate_hz'], c['late_ms'], c['size'], c['alarm']) == \
        ('running', 20.0, 68, [640, 480], False)
    assert c['lines'] == lines
    # Absence is said, never a zero: no rate from fewer than two answers.
    assert caption('downward', None, None, [], None)['rate_hz'] is None


def test_the_picture_carries_no_caption():
    """The camera's state was drawn bottom left in the picture, where it
    covered the floor a downward box sits on (owner, 2026-11-24). It is the
    caption under the picture now: nothing in the picture but what is seen."""
    out = annotate(STAMP, _arr(), size=(640, 480))
    assert not out.texts and not _drawn(out.points)


def test_the_floor_read_is_drawn_as_the_grid_and_the_lane_it_says():
    """A FloorRead becomes its grout lines (both families, every line across
    the frame), the lane centreline, and what each says in words; a read that
    found nothing draws no lines and says so."""
    from foxglove_msgs.msg import PointsAnnotation
    from mongla_batighor.overlay import floor_annotate
    from mongla_interfaces.msg import FloorRead
    m = FloorRead()
    m.width, m.height = 640, 480
    m.tiles, m.period_px = True, 64.0
    m.grout_normal_deg, m.grout_offset_px = [0.0, 90.0], [10.0, 20.0]
    m.tile_height_m = 1.25
    m.lane, m.lane_direction_deg, m.lane_offset_px = True, 0.0, 30.0
    a = floor_annotate(m)
    grid = [p for p in a.points if p.type == PointsAnnotation.LINE_LIST]
    (lane,) = [p for p in a.points if p.type == PointsAnnotation.LINE_STRIP]
    # Vertical lines at x = 10, 74, ... 586 (10) and horizontal at y = 20 ... 468 (8).
    assert len(grid[0].points) == 2 * (10 + 8)
    assert {round(p.y) for p in lane.points} == {270}       # centre 240, +30 along +y
    words = ' '.join(t.text for t in a.texts)
    assert 'tiles 64 px' in words and '1.25 m above' in words and 'lane 0 deg' in words
    empty = floor_annotate(FloorRead(width=640, height=480))
    assert not empty.points and 'none read' in empty.texts[0].text
    # Each line in the colour of what it names: no tiles beside a lane read
    # once drew "tiles: none read" in the lane's amber.
    from mongla_batighor.overlay import AMBER, INK_2
    lane_only = floor_annotate(FloorRead(width=640, height=480, lane=True,
                                         lane_direction_deg=0.0, lane_offset_px=0.0))
    tiles_line, lane_line = lane_only.texts
    assert 'none read' in tiles_line.text and tiles_line.text_color == _color(INK_2)
    assert lane_line.text_color == _color(AMBER)


def test_the_demand_arrow_points_where_the_board_is_told_to_push():
    """FRD: a pure sway right turns the arrow +90 deg about z (down), a pure
    reverse 180; its length is the demand's share of full force."""
    import math
    from builtin_interfaces.msg import Time
    from mongla_batighor.overlay import DEMAND_ARROW_M, demand_scene
    for (x, y), yaw in (((1.0, 0.0), 0.0), ((0.0, 0.5), 90.0), ((-0.8, 0.0), 180.0)):
        e = demand_scene(x, y, Time(), 'mongla').entities[0]
        assert e.frame_id == 'mongla'
        (a,) = e.arrows
        q = a.pose.orientation
        assert math.degrees(2 * math.atan2(q.z, q.w)) == pytest.approx(yaw)
        assert a.shaft_length + a.head_length == pytest.approx(DEMAND_ARROW_M * math.hypot(x, y))


def test_an_unknown_demand_is_said_and_never_drawn_as_none():
    from builtin_interfaces.msg import Time
    from mongla_batighor.overlay import demand_scene
    unknown = demand_scene(float('nan'), 0.2, Time(), 'mongla').entities[0]
    none = demand_scene(0.0, 0.0, Time(), 'mongla').entities[0]
    assert not unknown.arrows and unknown.texts[0].text == 'demand unknown'
    assert not none.arrows and none.texts[0].text == 'demand none'


def test_two_boxes_on_one_thing_each_name_the_track_that_follows_it():
    """Two overlapping boxes on one bin, each overlapping both tracks, each
    said "#783 #785" (seen on the bench, Bins view): neither named the track
    following it. The pairing is one to one, best overlap first."""
    out = annotate(STAMP, _arr(_det(100, 100, 40, 40, label='fire', score=0.51),
                               _det(104, 104, 40, 40, label='fire', score=0.50)),
                   tracks=_arr(_det(105, 105, 40, 40, label='fire', track='785'),
                               _det(100, 100, 40, 40, label='fire', track='783')))
    texts = sorted(t.text for t in out.texts)
    assert texts == ['fire 0.50 #785', 'fire 0.51 #783'], texts
    # A second track over the same box, which the first holds, is not coasting:
    # it is left off rather than drawn grey on a detection.
    out = annotate(STAMP, _arr(_det(100, 100, 40, 40, label='fire')),
                   tracks=_arr(_det(100, 100, 40, 40, label='fire', track='1'),
                               _det(103, 103, 40, 40, label='fire', track='2')))
    assert sorted(t.text for t in out.texts) == ['fire 0.80 #1']


def test_a_silent_detector_still_lets_the_picture_show_and_says_why():
    """The Vision view shows a picture only with its annotations. A detector
    paused until a mission asks sends nothing, so the picture stayed blank
    ("Waiting for synchronized annotations") and a paused detector looked
    like a dead camera (seen on the bench, 2026-11-22). Silent past
    SILENT_S, every frame is marked by the overlay itself, saying so."""
    from mongla_batighor.overlay import SILENT_S, caption, hud, silence
    assert silence([], 10.0, 10.0 + SILENT_S / 2) is None       # just started
    assert silence([], 10.0, 13.0) == pytest.approx(3.0)        # never answered
    assert silence([5.0, 20.0], 0.0, 20.0 + SILENT_S / 2) is None  # answering
    assert silence([5.0, 20.0], 0.0, 24.5) == pytest.approx(4.5)
    lines = hud('forward', 'gate_sharks', (640, 480), [], None, silent_s=4.6, paused=True)
    assert lines[1] == 'detector paused 5 s: it runs when a mission asks'
    c = caption('forward', 'gate_sharks', (640, 480), [], None, silent_s=4.6, paused=True)
    assert (c['state'], c['silent_s'], c['alarm'], c['rate_hz']) == ('paused', 4.6, False, None)
    # The frame is still marked, so a synchronised picture keeps showing:
    # Lichtblick 1.30.0 files a synchronised annotation under each shape's own
    # stamp and never reads the message's, so the mark is one shape on the
    # frame's stamp that draws nothing (Chrome, 2026-11-25: an empty message
    # left the Vision view "Waiting for synchronized annotations").
    out = annotate(STAMP, size=(640, 480))
    assert not out.texts and not out.circles
    (mark,) = out.points
    assert mark.timestamp == STAMP and not mark.points and mark.type == PointsAnnotation.POINTS


def test_every_frame_carries_its_stamp_on_a_shape_even_with_nothing_found():
    """A detector that ran and found nothing is the same blank as a paused
    one, unless the frame's stamp rides on a shape: every frame drawn has at
    least one, each on the frame's own stamp."""
    out = annotate(STAMP, Detection2DArray(), size=(640, 480))
    stamps = [a.timestamp for a in [*out.points, *out.texts, *out.circles]]
    assert stamps and all(s == STAMP for s in stamps)


def test_a_detector_silent_while_not_paused_is_an_alarm_not_a_pause():
    """Marking a silent frame kept the picture, but it also made a detector
    that died mid-mission look like a paused one, where the blank picture had
    been the warning. The overlay knows the detector's `paused`; silent and
    not paused (or no detector at all) is said in capitals, in red."""
    from mongla_batighor.overlay import caption, hud
    stopped = hud('forward', 'm', (640, 480), [], None, silent_s=3.2, paused=False)
    assert stopped[1] == 'DETECTOR SILENT 3 s, NOT paused: stopped or stalled'
    unknown = hud('forward', 'm', (640, 480), [], None, silent_s=3.2, paused=None)
    assert unknown[1] == 'NO DETECTOR publishes forward detections (3 s)'
    assert caption('forward', 'm', (640, 480), [], None, 3.2, paused=False)['alarm']
    assert caption('forward', 'm', (640, 480), [], None, 3.2, paused=None)['state'] == 'none'
    assert caption('forward', 'm', (640, 480), [], None, 3.2)['state'] == 'unread'
    assert not caption('forward', 'm', (640, 480), [], None, 3.2, paused=True)['alarm']

def test_the_track_keeps_where_the_vehicle_moved_and_forgets_the_old():
    from mongla_batighor.overlay import TRACK_S, TRACK_STEP_M, Track, track_scene
    tr = Track()
    assert tr.note(0.0, 0, 0, 0, 'odom_ned')
    assert not tr.note(0.1, TRACK_STEP_M / 2, 0, 0, 'odom_ned')      # at rest: nothing added
    assert tr.note(1.0, 1.0, 0, 0.5, 'odom_ned')
    (e,) = track_scene(tr, STAMP).entities
    assert e.frame_id == 'odom_ned' and [(p.x, p.z) for p in e.lines[0].points] == [(0, 0), (1, 0.5)]
    tr.note(TRACK_S + 0.5, 2.0, 0, 0, 'odom_ned')                    # the first point aged out
    assert [p[1] for p in tr.points] == [1.0, 2.0]
    tr.note(TRACK_S + 1, 0, 0, 0, 'pool')                            # a new frame starts again
    assert tr.points == [(TRACK_S + 1, 0, 0, 0)]
    assert not track_scene(tr, STAMP).entities[0].lines             # one point is no line
