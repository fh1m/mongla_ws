"""Outlines on the wire, and more than one model on one frame.

Two claims are under test and both are silent when wrong. A contour that is
not offset back into frame coordinates is a valid polygon in the wrong place;
and two models merged into one array mint their class ids from two private
tables, so `gate`(0) and `person`(0) become one class with no error anywhere.
"""
import numpy as np
import pytest

from mongla_vision.detection.contours import (
    ANGLE_UNKNOWN, box_contour, contour_for, flatten, mask_contour,
    obb_angle_deg, polygon_area_px,
)
from mongla_vision.detection.detector import Detection
from mongla_vision.detector_node import _split_active


class TestSplitActive:
    def test_a_bare_name_is_unchanged(self):
        assert _split_active('gate_rescue_repair') == ('gate_rescue_repair', [])

    def test_a_csv_gives_a_primary_and_the_rest(self):
        assert _split_active('a, b ,c') == ('a', ['b', 'c'])

    def test_empty_and_whitespace_mean_nothing_selected(self):
        assert _split_active('') == ('', [])
        assert _split_active('  ,, ') == ('', [])


class TestBoxContour:
    def test_a_box_is_four_corners_in_order(self):
        pts = box_contour((10, 20, 30, 50))
        assert pts.shape == (4, 2)
        assert pts[0].tolist() == [10, 20]
        assert pts[2].tolist() == [30, 50]

    def test_its_area_is_the_box_area(self):
        assert polygon_area_px(box_contour((0, 0, 4, 5))) == pytest.approx(20.0)


class TestMaskContour:
    def _mask(self):
        m = np.zeros((40, 30), np.uint8)
        m[5:35, 4:26] = 1
        return m

    def test_the_contour_is_offset_into_frame_coordinates(self):
        """The whole failure mode: a correct polygon in the wrong place."""
        pts = mask_contour(self._mask(), 100.0, 200.0)
        assert pts is not None
        assert pts[:, 0].min() == pytest.approx(104.0, abs=1.0)
        assert pts[:, 1].min() == pytest.approx(205.0, abs=1.0)

    def test_simplification_keeps_the_area(self):
        pts = mask_contour(self._mask(), 0.0, 0.0)
        assert len(pts) <= 12, len(pts)          # a rectangle needs four
        # `findContours` traces PIXEL CENTRES, so a solid W x H block of
        # pixels encloses (W-1) x (H-1) of area. Comparing against W*H and
        # widening the tolerance until it passes would hide a real
        # simplification loss, so compare against what the tracer actually
        # bounds and keep the tolerance tight.
        assert polygon_area_px(pts) == pytest.approx(29 * 21, rel=0.02)

    def test_speckle_is_refused_rather_than_published(self):
        m = np.zeros((40, 30), np.uint8)
        m[1, 1] = 1
        assert mask_contour(m, 0.0, 0.0) is None

    def test_the_largest_blob_wins_over_a_speck(self):
        m = self._mask()
        m[0, 0] = 1
        pts = mask_contour(m, 0.0, 0.0)
        assert polygon_area_px(pts) > 500


class TestObbAngle:
    def test_an_axis_aligned_rectangle_is_zero_or_ninety(self):
        assert obb_angle_deg(box_contour((0, 0, 10, 4))) in (0, 90)

    def test_a_rotated_rectangle_reports_its_rotation(self):
        th = np.radians(30.0)
        r = np.array([[np.cos(th), -np.sin(th)],
                      [np.sin(th), np.cos(th)]], np.float32)
        base = box_contour((-20, -5, 20, 5))
        pts = (base @ r.T).astype(np.float32) + 100.0
        ang = obb_angle_deg(pts)
        assert min(abs(ang - 30), abs(ang - 150)) <= 2, ang

    def test_too_few_points_says_unknown_not_zero(self):
        assert obb_angle_deg(np.zeros((2, 2), np.float32)) == ANGLE_UNKNOWN


class TestContourFor:
    def test_a_box_only_detection_says_its_angle_is_unknown(self):
        d = Detection(class_id=0, class_name='gate', score=0.9,
                      xyxy=(1, 2, 11, 12))
        pts, ang = contour_for(d)
        assert len(pts) == 4
        assert ang == ANGLE_UNKNOWN, 'a box has no orientation; 0 is a claim'

    def test_a_masked_detection_uses_the_mask(self):
        m = np.zeros((20, 30), np.uint8)
        m[2:18, 3:27] = 1
        d = Detection(class_id=0, class_name='gate', score=0.9,
                      xyxy=(50, 60, 80, 80), mask=m)
        pts, ang = contour_for(d)
        assert ang != ANGLE_UNKNOWN
        assert pts[:, 0].min() >= 50 and pts[:, 1].min() >= 60


class TestFlatten:
    def test_offset_has_one_more_entry_than_contours(self):
        """The last contour needs no special case -- that is what the extra
        entry buys, and a consumer that assumes equal lengths loses it."""
        a = box_contour((0, 0, 2, 2))
        b = box_contour((5, 5, 9, 9))
        offset, points = flatten([a, b], width=100, height=100)
        assert len(offset) == 3
        assert offset == [0, 4, 8]
        assert len(points) == 16

    def test_points_are_clamped_into_the_frame(self):
        """uint16 on the wire: a negative value wraps to 65535, not an error."""
        offset, points = flatten([box_contour((-10, -10, 500, 500))],
                                 width=64, height=48)
        xs, ys = points[0::2], points[1::2]
        assert min(xs) >= 0 and max(xs) <= 63
        assert min(ys) >= 0 and max(ys) <= 47

    def test_no_contours_still_gives_the_leading_zero(self):
        assert flatten([], width=10, height=10) == ([0], [])


class TestContourMessage:
    def test_every_detection_gets_exactly_one_contour(self):
        """Box and mask alike -- that is what makes it one consumer."""
        from std_msgs.msg import Header
        from mongla_vision.detection.messages import detections_to_contours
        m = np.zeros((10, 10), np.uint8)
        m[1:9, 1:9] = 1
        dets = [
            Detection(class_id=0, class_name='gate', score=0.8, xyxy=(0, 0, 10, 10)),
            Detection(class_id=1, class_name='bin', score=0.7,
                      xyxy=(20, 20, 30, 30), mask=m),
        ]
        msg = detections_to_contours(dets, Header(), camera='forward',
                                     width=640, height=480)
        assert list(msg.class_name) == ['gate', 'bin']
        assert len(msg.offset) == 3
        assert msg.offset[-1] == len(msg.points) // 2
        assert msg.angle_deg[0] == ANGLE_UNKNOWN     # the box
        assert msg.angle_deg[1] != ANGLE_UNKNOWN     # the mask
        assert msg.area_px[0] == pytest.approx(100, abs=2)


class TestMergedClassIds:
    """Two models numbering their own classes from zero is the silent one."""

    @pytest.fixture(autouse=True)
    def _clean(self):
        from mongla_vision.detection import messages
        messages._CLASS_INDEX.clear()
        yield
        messages._CLASS_INDEX.clear()

    def test_two_models_zeroth_classes_stop_colliding(self):
        from mongla_vision.detection.messages import remint_class_ids
        merged = [
            Detection(class_id=0, class_name='gate', score=0.9, xyxy=(0, 0, 1, 1)),
            Detection(class_id=0, class_name='person', score=0.8, xyxy=(2, 2, 3, 3)),
        ]
        assert merged[0].class_id == merged[1].class_id, 'the defect being fixed'
        remint_class_ids(merged)
        assert merged[0].class_id != merged[1].class_id

    def test_the_same_label_from_two_models_stays_one_class(self):
        from mongla_vision.detection.messages import remint_class_ids
        merged = remint_class_ids([
            Detection(class_id=3, class_name='gate', score=0.9, xyxy=(0, 0, 1, 1)),
            Detection(class_id=7, class_name='gate', score=0.8, xyxy=(2, 2, 3, 3)),
        ])
        assert merged[0].class_id == merged[1].class_id

    def test_the_id_agrees_with_what_the_wire_mints(self):
        """A subscriber re-derives ids from labels; the two must not disagree."""
        from mongla_vision.detection.messages import class_index, remint_class_ids
        d = remint_class_ids([Detection(class_id=99, class_name='bin',
                                        score=0.5, xyxy=(0, 0, 1, 1))])[0]
        assert d.class_id == class_index('bin')


class TestASecondaryModelCannotTearDownThePrimary:
    """The failure counter escalates to REBUILDING the primary detector.

    A second model running beside it must not be able to reach that counter:
    the primary succeeded, and tearing down a healthy model because a different
    one misbehaved is a mission lost to the wrong cause. Found by the recovery
    suite when the merge was inside the same `try` -- 0 failures became 5.
    """

    def _node(self, monkeypatch, extra_infer):
        import queue as q
        import threading
        from types import SimpleNamespace
        import mongla_vision.detector_node as D

        n = D.DetectorNode.__new__(D.DetectorNode)
        logs = []
        n._log = SimpleNamespace(info=logs.append, warn=logs.append,
                                 warning=logs.append, error=logs.append,
                                 fatal=logs.append, debug=logs.append)
        n.get_logger = lambda: n._log
        n._infer_fails = 0
        n._want = threading.Event()
        n._infer_q = q.Queue()
        n._bridge = n._pre = n._crop = None
        n._publish_dbg = False
        n._pub_contours = None
        n._det = SimpleNamespace(infer=lambda _f: [])
        n._build_kwargs = None
        n._extra = [SimpleNamespace(infer=extra_infer)]
        n._extra_names = ['seg']
        n.get_parameter = lambda _k: SimpleNamespace(value=False)
        n._rebuild_detector = lambda: None
        monkeypatch.setattr(
            D, 'rclpy', SimpleNamespace(ok=lambda: n._infer_q.qsize() > 0))
        return n, D

    def test_anything_the_merge_raises_leaves_the_counter_at_zero(self, monkeypatch):
        """The merge must sit OUTSIDE the primary's failure `try`.

        ⛔ An earlier version of this test made the secondary detector raise,
        and the injection that moved the merge back inside the `try` still
        passed -- because `_merge_extra` catches a detector exception itself,
        so that test could never distinguish the two placements. It checked the
        inner guard and called it the outer one. Raising from the merge as a
        whole is what actually separates them.
        """
        n, D = self._node(monkeypatch, lambda _f: [])

        def explode(_frame, _dets):
            raise RuntimeError('merge exploded')
        n._merge_extra = explode

        for _ in range(4):
            n._infer_q.put(D._DirectFrame(frame=object(), header=None))
        try:
            n._infer_loop()
        except Exception:
            pass
        assert n._infer_fails == 0, (
            'a secondary-model fault reached the counter that rebuilds the '
            'PRIMARY detector')

    def test_a_node_with_no_extras_attribute_still_infers(self, monkeypatch):
        """A harness that bypasses both init paths has no `_extra`; an
        AttributeError there would surface as an inference failure."""
        n, _D = self._node(monkeypatch, lambda _f: [])
        del n._extra
        assert n._merge_extra(object(), ['a']) == ['a']


def test_the_STARTUP_path_splits_the_csv_too(monkeypatch):
    """⛔ A knob wired to nothing, caught by reading rather than by a test.

    The live parameter handler split `active_model` on commas from the first
    draft; the STARTUP path did not, because the patch that was supposed to
    add it aborted before writing the file. Everything still looked right --
    the DSL sent a CSV, the launch argument existed and was forwarded, the
    param handler understood it -- and a launch that asked for two models
    would resolve the whole string `'a,b'` as one key, fail to find it, and
    fall back to ONE model with an error nobody reads at boot.

    So this asserts the two paths use the SAME splitter, which is the only
    property that keeps a launch argument and a mid-mission switch meaning the
    same thing.
    """
    import inspect
    import mongla_vision.detector_node as D
    src = inspect.getsource(D.DetectorNode.__init__)
    assert '_split_active(active_model)' in src, (
        'startup does not split active_model; a CSV from the launch would '
        'resolve as a single key and silently load one model')
    assert 'self._set_extra(extra_names)' in src, (
        'startup splits the CSV and then never uses the secondary names')
