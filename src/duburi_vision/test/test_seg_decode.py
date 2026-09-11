"""The raw-seg-head decode, and specifically the two claims that make it fast.

Both optimisations in `seg_decode` are claims of EXACTNESS, not of "close
enough": thresholding quantised bytes is asserted to select the same cells as
thresholding dequantised floats, and the mask threshold is asserted to select
the same pixels with the sigmoid and the dequantisation removed. A wrong
threshold here does not raise -- it silently shifts the operating point, or
grows/shrinks every mask -- so each claim is tested against the naive path it
replaces rather than against a remembered number.
"""
import numpy as np
import pytest

from duburi_vision.detection.seg_decode import (
    MASK_DIM, Quant, REG_MAX, ScaleLayout, SegLayout,
    candidates, decode, dfl_boxes, masks_for, nms,
)

UNIT = Quant(zp=0.0, scale=1.0 / 255.0)        # the class heads, as measured
LOGIT = Quant(zp=97.0, scale=0.043)            # a plausible logit head


def _naive_float_keep(q, u8, conf):
    """What every published decode does: dequantise, sigmoid, then compare."""
    v = (u8.astype(np.float32) - q.zp) * q.scale
    if not q.is_unit_probability:
        v = 1.0 / (1.0 + np.exp(-v))
    return np.nonzero(v >= conf)[0]


class TestQuantisedThresholdIsExact:
    def test_unit_probability_head_is_recognised(self):
        assert UNIT.is_unit_probability
        assert not LOGIT.is_unit_probability

    @pytest.mark.parametrize('q', [UNIT, LOGIT])
    @pytest.mark.parametrize('conf', [0.02, 0.05, 0.10, 0.15, 0.25, 0.4, 0.5, 0.75, 0.9])
    def test_byte_threshold_picks_exactly_the_float_cells(self, q, conf):
        every = np.arange(256, dtype=np.uint8)
        floor = q.score_floor_u8(conf)
        fast = np.nonzero(every >= floor)[0]
        slow = _naive_float_keep(q, every, conf)
        assert np.array_equal(fast, slow), (
            f'conf={conf} floor={floor}: byte threshold kept {fast.size} of 256, '
            f'the float path kept {slow.size}')

    def test_the_floor_is_the_tightest_one_that_works(self):
        # Off by one in the safe direction would still pass the test above on
        # most values; it would quietly raise the operating point. The floor
        # must be the SMALLEST byte that qualifies.
        for conf in (0.10, 0.25, 0.5):
            floor = UNIT.score_floor_u8(conf)
            assert UNIT.dequant_score(np.uint8(floor)) >= conf - 1e-9
            assert floor == 0 or UNIT.dequant_score(np.uint8(floor - 1)) < conf


class TestCandidates:
    def _cls(self):
        c = np.zeros((4, 4, 6), np.uint8)
        c[1, 2, 3] = 200            # strong, class 3
        c[0, 0, 5] = 200            # strong, class 5
        c[3, 3, 1] = 10             # below any sane floor
        return c

    def test_finds_the_cells_and_reports_the_winning_class(self):
        cells, cids, score = candidates(self._cls(), UNIT.score_floor_u8(0.25), None)
        got = dict(zip(cells.tolist(), cids.tolist()))
        assert got == {1 * 4 + 2: 3, 0 * 4 + 0: 5}
        assert set(score.tolist()) == {200}

    def test_gating_drops_other_classes_and_keeps_the_id_right(self):
        allow = np.array([3], dtype=np.int64)
        cells, cids, _ = candidates(self._cls(), UNIT.score_floor_u8(0.25), allow)
        # The id must be the MODEL's class id, not the index into the gate --
        # returning the local index relabels every detection.
        assert cells.tolist() == [1 * 4 + 2]
        assert cids.tolist() == [3]

    def test_nothing_above_the_floor_returns_empty_not_none(self):
        cells, cids, score = candidates(self._cls(), 255, None)
        assert cells.size == cids.size == score.size == 0


class TestDflBoxes:
    def test_a_one_hot_head_reproduces_the_distance_it_encodes(self):
        # All the softmax mass on bin k gives a distance of k * stride. Build a
        # head whose dequantised value is large on one bin per side.
        q = Quant(zp=0.0, scale=1.0)
        grid, stride = 8, 8
        box = np.zeros((grid, grid, 4 * REG_MAX), np.uint8)
        bins = (2, 3, 4, 5)                       # l, t, r, b
        cell = 3 * grid + 5                       # row 3, col 5
        for side, b in enumerate(bins):
            box[3, 5, side * REG_MAX + b] = 60    # 60 >> the zeros: ~one-hot
        out = dfl_boxes(box, np.array([cell]), grid_w=grid, stride=stride, quant=q)
        ax, ay = (5 + 0.5) * stride, (3 + 0.5) * stride
        want = [ax - bins[0] * stride, ay - bins[1] * stride,
                ax + bins[2] * stride, ay + bins[3] * stride]
        assert np.allclose(out[0], want, atol=0.05), (out[0], want)

    def test_the_side_and_bin_axes_are_not_swapped(self):
        # One bin of ONE side set. The other three sides are flat, so their
        # softmax is uniform and their distance is the mean bin, 7.5 cells.
        # A transposed (16, 4) read would spread that single peak across all
        # four sides instead and produce a different, still plausible, box.
        q = Quant(zp=0.0, scale=1.0)
        stride = 8
        box = np.zeros((4, 4, 4 * REG_MAX), np.uint8)
        box[0, 0, 0 * REG_MAX + 7] = 60          # side 0 (left), bin 7
        out = dfl_boxes(box, np.array([0]), grid_w=4, stride=stride, quant=q)
        ax = ay = 0.5 * stride
        flat = 7.5 * stride
        assert out[0] == pytest.approx(
            [ax - 7 * stride, ay - flat, ax + flat, ay + flat], abs=0.1)


class TestNms:
    def test_suppresses_an_overlapping_box_of_the_same_class(self):
        b = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], np.float32)
        keep = nms(b, np.array([0.9, 0.8], np.float32), np.array([0, 0]), 0.45, 10)
        assert keep.tolist() == [0]

    def test_keeps_the_same_overlap_when_the_classes_differ(self):
        b = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], np.float32)
        keep = nms(b, np.array([0.9, 0.8], np.float32), np.array([0, 1]), 0.45, 10)
        assert sorted(keep.tolist()) == [0, 1]

    def test_max_det_cuts_the_weakest(self):
        b = np.array([[0, 0, 1, 1], [20, 20, 21, 21], [40, 40, 41, 41]], np.float32)
        keep = nms(b, np.array([0.1, 0.9, 0.5], np.float32),
                   np.array([0, 0, 0]), 0.45, 2)
        assert keep.tolist() == [1, 2]


class TestMasksMatchTheNaivePath:
    def _fixture(self):
        """A SMOOTH prototype field, because the claim is about a boundary.

        White noise has a threshold crossing at almost every pixel, so any
        comparison of two thresholdings of it measures the noise rather than
        the arithmetic. A real prototype plane is smooth and its mask has a
        boundary of finite length -- reproduced here with a low-frequency
        field.
        """
        q = Quant(zp=128.0, scale=0.031)
        yy, xx = np.mgrid[0:160, 0:160].astype(np.float32)
        proto = np.zeros((160, 160, MASK_DIM), np.float32)
        rng = np.random.default_rng(7)
        for k in range(MASK_DIM):
            fx, fy, ph = rng.uniform(0.01, 0.05, 2).tolist() + [rng.uniform(0, 6.3)]
            proto[:, :, k] = 128 + 110 * np.sin(fx * xx + fy * yy + ph)
        proto = np.clip(proto, 0, 255).astype(np.uint8)
        coeff = rng.normal(0, 0.4, (3, MASK_DIM)).astype(np.float32)
        # ⛔ TWO FIXTURE DEFECTS WERE FOUND HERE BY INJECTION, IN OPPOSITE
        # DIRECTIONS, AND THE SECOND WAS CAUSED BY FIXING THE FIRST.
        #
        # White noise first: it crosses the threshold at nearly every pixel, so
        # comparing two thresholdings of it measures noise, not arithmetic.
        # Then a zero-mean prototype with a zero-SUM coefficient vector: that
        # makes `zp * sum(c)` identically zero, and an injected defect that
        # deleted the zero-point term entirely still passed all 34 tests.
        #
        # So the zero point must sit INSIDE the field's range (hence zp=128
        # against a field centred on 128) and the coefficients must keep a
        # non-zero sum, or the term under test is multiplied away.
        assert abs(float(coeff.sum(axis=1).min())) > 0.1
        return q, proto, coeff

    def test_folded_threshold_is_EXACT_before_any_resampling(self):
        """The claim itself: no sigmoid, no dequantisation, same pixels.

        Compared at prototype resolution, where no interpolation stands
        between the two paths, so the assertion is equality and not a
        tolerance.
        """
        q, proto, coeff = self._fixture()
        flat = proto.reshape(-1, MASK_DIM).astype(np.float32)

        fast = (flat @ coeff[0]) >= q.zp * coeff[0].sum()

        deq = (flat - q.zp) * q.scale
        slow = (1.0 / (1.0 + np.exp(-(deq @ coeff[0])))) >= 0.5

        assert np.array_equal(fast, slow)
        # And the field must actually contain both outcomes, or "equal" is
        # trivially true of two all-False arrays.
        assert 0.05 < fast.mean() < 0.95, fast.mean()

    def test_whole_mask_matches_the_naive_path_on_a_real_boundary(self):
        q, proto, coeff = self._fixture()
        boxes = np.array([[10, 20, 300, 420], [0, 0, 639, 639], [500, 500, 620, 610]],
                         np.float32)
        fast = masks_for(coeff, proto, q, boxes, size=640)

        # The naive path, written out in full: dequantise the whole plane,
        # sigmoid it, resize, threshold at 0.5.
        import cv2
        flat = ((proto.astype(np.float32) - q.zp) * q.scale).reshape(-1, MASK_DIM)
        prob = 1.0 / (1.0 + np.exp(-(flat @ coeff.T))).reshape(160, 160, 3)
        for n, (x1, y1, x2, y2) in enumerate(boxes):
            px1, py1 = int(np.floor(x1 / 4)), int(np.floor(y1 / 4))
            px2, py2 = int(np.ceil(x2 / 4)), int(np.ceil(y2 / 4))
            patch = prob[py1:py2, px1:px2, n]
            bw, bh = max(1, int(round(x2 - x1))), max(1, int(round(y2 - y1)))
            ref = (cv2.resize(patch, (bw, bh),
                              interpolation=cv2.INTER_LINEAR) >= 0.5).astype(np.uint8)
            assert fast[n].shape == ref.shape
            disagree = int(np.abs(fast[n].astype(int) - ref.astype(int)).sum())
            # Not zero: the fast path interpolates the SCORE and the naive one
            # interpolates the probability, and sigmoid is not linear, so the
            # two can differ on boundary pixels that straddle the threshold.
            # Anything beyond a thin boundary is a real disagreement.
            assert disagree <= 0.01 * ref.size, (
                f'mask {n}: {disagree} of {ref.size} pixels differ')

    def test_no_detections_gives_no_masks(self):
        q = Quant(zp=0.0, scale=1.0)
        proto = np.zeros((160, 160, MASK_DIM), np.uint8)
        assert masks_for(np.zeros((0, MASK_DIM), np.float32), proto, q,
                         np.zeros((0, 4), np.float32), size=640) == []


def _layout():
    return SegLayout(size=640, num_classes=4,
                     scales=tuple(ScaleLayout(stride=s, box=Quant(0.0, 1.0),
                                              cls=UNIT, coeff=Quant(0.0, 1.0))
                                  for s in (8, 16, 32)),
                     proto=Quant(0.0, 1.0))


def _blank_heads():
    heads = []
    for grid in (80, 40, 20):
        heads.append((np.zeros((grid, grid, 4 * REG_MAX), np.uint8),
                      np.zeros((grid, grid, 4), np.uint8),
                      np.zeros((grid, grid, MASK_DIM), np.uint8)))
    return heads


class TestDecodeEndToEnd:
    def test_a_planted_peak_becomes_one_detection_at_that_cell(self):
        heads = _blank_heads()
        box, cls, _ = heads[1]                       # stride 16, 40x40 grid
        cls[10, 20, 2] = 230
        for side, b in enumerate((1, 1, 1, 1)):
            box[10, 20, side * REG_MAX + b] = 60
        xyxy, scores, cids, masks = decode(
            _layout(), heads, np.zeros((160, 160, MASK_DIM), np.uint8),
            conf=0.25, want_masks=False)
        assert cids.tolist() == [2]
        assert scores[0] == pytest.approx(230 / 255, abs=1e-6)
        cx = (xyxy[0, 0] + xyxy[0, 2]) / 2
        cy = (xyxy[0, 1] + xyxy[0, 3]) / 2
        assert cx == pytest.approx((20 + 0.5) * 16, abs=1.0)
        assert cy == pytest.approx((10 + 0.5) * 16, abs=1.0)
        assert masks == []

    def test_gating_to_another_class_returns_nothing(self):
        heads = _blank_heads()
        heads[1][1][10, 20, 2] = 230
        xyxy, scores, cids, _ = decode(_layout(), heads, None, conf=0.25,
                                       allow_ids=[0, 1], want_masks=False)
        assert scores.size == 0 and xyxy.shape == (0, 4)

    def test_an_empty_head_returns_empty_arrays_not_none(self):
        xyxy, scores, cids, masks = decode(_layout(), _blank_heads(), None,
                                           conf=0.25, want_masks=False)
        assert xyxy.shape == (0, 4) and scores.size == 0 and masks == []
