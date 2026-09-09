"""Timing: the term that decides whether de-rotation helps or hurts.

De-rotation subtracts `f·ω·Δt`, so a timing error leaves uncorrected flow
PROPORTIONAL to ω -- measured on this rig, 5 ms costs 1.6 px at 0.64 rad/s.
Qin & Shen put the tolerance for an uncompensated offset at 6 ms.

Measured on the vehicle, ATTITUDE at 50 Hz: the BOARD's interval is 20.00 ms
with sd 0.00, and the HOST's arrival interval is 20.00 ms with sd 6.67 and
p2p 35.12. All of the jitter is transport, and we were inheriting all of it by
stamping on arrival.
"""
import math

import numpy as np
import pytest

from duburi_vision.flow.flow_timing import (
    ClockMap, TimeOffset, exposure_offset_s, interval_midpoint,
)


# --------------------------------------------------------------------------- #
#  ClockMap -- board clock to host clock, with no TIMESYNC available
# --------------------------------------------------------------------------- #
class TestClockMap:
    """The srot board does not implement MAVLink TIMESYNC -- 0 of 12 requests
    answered, measured. So the only signal is one-way pairs of (board time,
    host arrival) with a variable, unknown, strictly POSITIVE transport delay
    in between."""

    def _feed(self, cm, skew=1.0002, offset=12.345, n=600, hz=50.0,
              delay_mean=0.003, delay_sd=0.004, seed=0):
        rng = np.random.default_rng(seed)
        for i in range(n):
            b = i / hz
            cm.add(b, b * skew + offset + abs(rng.normal(delay_mean, delay_sd)))

    def test_it_recovers_offset_and_skew_from_one_way_data(self):
        cm = ClockMap(window_s=30.0)
        self._feed(cm)
        assert cm.fit()
        assert cm.skew_ppm == pytest.approx(200.0, abs=25.0)
        err = [abs(cm.to_host(i / 50.0) - (i / 50.0 * 1.0002 + 12.345))
               for i in range(600)]
        assert max(err) < 0.001, f'{max(err)*1000:.2f} ms'

    def test_it_beats_a_MEAN_fit_by_the_mean_delay(self):
        """The whole reason for the lower envelope. A least-squares fit
        through the middle of the cloud estimates offset PLUS mean delay; the
        minimum-delay samples are the ones nearest the truth."""
        cm = ClockMap(window_s=30.0)
        self._feed(cm, delay_mean=0.010, delay_sd=0.004)
        assert cm.fit()
        bias = cm.to_host(5.0) - (5.0 * 1.0002 + 12.345)
        assert abs(bias) < 0.004, f'envelope fit is {bias*1000:+.2f} ms out'

    def test_a_short_window_REFUSES_rather_than_guessing_skew(self):
        """Over a short span a slope error and an offset error are the same
        thing. Reporting a skew from 0.5 s of data is reporting noise."""
        cm = ClockMap(window_s=30.0, min_pairs=10)
        self._feed(cm, n=20)
        assert not cm.fit()
        assert not cm.ready

    def test_too_few_pairs_refuses(self):
        cm = ClockMap(min_pairs=40)
        self._feed(cm, n=10)
        assert not cm.fit()

    def test_it_ignores_non_finite_input(self):
        cm = ClockMap()
        cm.add(float('nan'), 1.0)
        cm.add(1.0, float('inf'))
        assert cm.n_pairs == 0


# --------------------------------------------------------------------------- #
#  The three DETERMINISTIC terms, each larger than the estimated residual
# --------------------------------------------------------------------------- #
class TestExposureOffset:
    """The standard image timestamp is MID-exposure: the average position of
    everything in frame is where it was when half the exposure had elapsed.
    The literature states the camera-IMU offset as fixed delays PLUS HALF
    EXPOSURE TIME."""

    def test_half_the_exposure_in_seconds(self):
        # V4L2 reports in 100 us units; 157 is the measured auto value here.
        assert exposure_offset_s(157) == pytest.approx(0.00785, abs=1e-6)

    def test_our_measured_exposure_exceeds_the_whole_error_budget(self):
        """7.85 ms against Qin & Shen's 6 ms tolerance -- this single term is
        larger than the offset most papers estimate, and it is not constant
        because the camera is on AUTO exposure."""
        assert exposure_offset_s(157) > 0.006

    def test_absent_or_nonsense_exposure_is_zero_not_a_guess(self):
        for bad in (None, 0, -5, float('nan'), 'x'):
            assert exposure_offset_s(bad) == 0.0


class TestIntervalMidpoint:
    """The BIGGEST timing error in this pipeline. Flow gives displacement over
    an interval, so velocity is the AVERAGE across it and belongs at the
    midpoint -- PX4 defines its flow delay exactly that way, 'to the middle of
    the optical flow integration interval'."""

    def test_the_midpoint_is_the_midpoint(self):
        assert interval_midpoint(10.0, 10.5) == pytest.approx(10.25)

    def test_an_adaptive_baseline_makes_this_worth_375_ms(self):
        """Our baseline stretches to max_baseline_s (0.75 s) when the hull is
        slow, and stamping at the END is then wrong by half of that -- two
        orders of magnitude past the 6 ms tolerance, and WORST exactly during
        station-keeping, the regime the sensor exists for."""
        err = 0.75 - interval_midpoint(0.0, 0.75)
        assert err == pytest.approx(0.375)
        assert err > 0.006 * 50


# --------------------------------------------------------------------------- #
#  TimeOffset -- trace correlation + parabolic sub-sample peak
# --------------------------------------------------------------------------- #
class TestTimeOffset:
    """Measured against injected offsets: mean |error| 0.218 ms, max 0.376 ms
    over -100..+150 ms. The 2025 review puts plain cross-correlation at 360 ms
    of error for large delays and trace-correlation-plus-quadratic at <0.5 ms;
    this is the latter."""

    def _est(self, true_td, seed=1, noise=0.02, secs=12.0, img_hz=25.0,
             gyro_hz=50.0, freq=0.7):
        rng = np.random.default_rng(seed)
        est = TimeOffset(max_lag_s=0.20)

        def w(t):
            return (1.2 * math.sin(2 * math.pi * freq * t)
                    + 0.4 * math.sin(2 * math.pi * freq * 2.7 * t))

        for i in range(int(secs * gyro_hz)):
            t = i / gyro_hz
            est.add_gyro_yaw(t, w(t) + rng.normal(0, noise))
        for i in range(int(secs * img_hz)):
            t = i / img_hz
            # The image is stamped LATE by true_td: an image stamped t shows
            # the world at t - true_td.
            est.add_image_yaw(t, w(t - true_td) + rng.normal(0, noise * 2))
        return est

    @pytest.mark.parametrize('true_td', (0.0, 0.005, 0.015, 0.030, -0.020,
                                         0.080, 0.150, -0.100))
    def test_it_recovers_an_injected_offset_to_under_a_millisecond(self, true_td):
        est = self._est(true_td)
        td = est.estimate()
        assert td is not None
        assert abs(td - true_td) < 0.001, f'{(td-true_td)*1000:+.2f} ms'

    def test_THE_SIGN(self):
        """A sign error here steers de-rotation the WRONG WAY and doubles the
        residual instead of removing it -- while returning a plausible number.
        Positive td must mean the IMAGE is late."""
        est = self._est(0.040)
        td = est.estimate()
        assert td > 0.03, f'sign inverted: {td*1000:+.1f} ms for a LATE image'

    def test_a_still_rig_REFUSES(self):
        """Two flat signals correlate wherever the noise says, and the
        parabola will interpolate a confident answer out of it."""
        rng = np.random.default_rng(3)
        est = TimeOffset()
        for i in range(600):
            est.add_gyro_yaw(i / 50.0, rng.normal(0, 0.004))
        for i in range(300):
            est.add_image_yaw(i / 25.0, rng.normal(0, 0.004))
        assert est.estimate() is None
        assert est.quality == 0.0

    def test_a_CONSTANT_rotation_refuses(self):
        """Li & Mourikis: constant-velocity motion is degenerate -- td is not
        identifiable from a steady turn however long you watch it."""
        rng = np.random.default_rng(4)
        est = TimeOffset()
        for i in range(600):
            est.add_gyro_yaw(i / 50.0, 0.8 + rng.normal(0, 0.004))
        for i in range(300):
            est.add_image_yaw(i / 25.0, 0.8 + rng.normal(0, 0.004))
        assert est.estimate() is None

    def test_too_little_data_refuses(self):
        est = TimeOffset()
        for i in range(5):
            est.add_gyro_yaw(i / 50.0, math.sin(i))
        assert est.estimate() is None

    def test_a_peak_at_the_search_EDGE_refuses(self):
        """An offset outside the window would otherwise be reported as the
        window's own edge -- reporting the search bound as a measurement."""
        est = self._est(0.180)          # beyond max_lag_s=0.05 below
        est._max_lag_s = 0.05
        assert est.estimate() is None

    def test_quality_separates_a_sharp_peak_from_a_broad_one(self):
        """A high correlation with a flat top means the LAG is poorly
        determined even though the signals agree. Peak height alone would call
        that excellent."""
        sharp = self._est(0.080, freq=1.4)
        sharp.estimate()
        broad = self._est(0.080, freq=0.15)
        broad.estimate()
        assert sharp.quality > broad.quality
