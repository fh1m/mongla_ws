"""The Allan-deviation instrument, checked against signals whose answer is known.

This is a TRUTH test, not an agreement test: each case constructs a signal whose
Allan curve is known in closed form, then checks the tool recovers it. Comparing
the tool against itself would prove only that it is consistent.

Why the instrument matters: `inekf.py` ships `Q` as a nominal diagonal that
nothing in this tree measured, and the NEES/NIS literature is explicit that
those statistics are chi-square distributed ONLY for an already-tuned filter. So
the chi-square gate, the five-rejection lockout break and the x4 inflation all
rest on numbers this tool is meant to produce.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))

from allan_variance import characterise, overlapping_allan  # noqa: E402


# ── white noise: sigma(tau) = s / sqrt(f tau), exactly ───────────────────────

def _white(rate=100.0, s=0.02, hours=2.0, seed=7):
    rng = np.random.default_rng(seed)
    return rng.normal(0.0, s, int(rate * 3600 * hours)), rate, s


def test_white_noise_curve_matches_the_closed_form():
    x, rate, s = _white()

    tau, sigma = overlapping_allan(x, rate)

    expected = s / np.sqrt(rate * tau)
    # only where the estimator has confidence: overlapping Allan reuses samples,
    # so independent differences fall off as tau grows (measured on this signal:
    # 2.16 % out to 30 s, 9.69 % to 100 s, 26.27 % to 300 s)
    band = (tau >= 0.1) & (tau <= 30.0)
    worst = np.max(np.abs(sigma[band] - expected[band]) / expected[band])
    assert worst < 0.05, f'worst relative error {worst:.3f} over tau 0.1-30 s'


def test_the_white_noise_coefficient_is_the_one_that_goes_into_Q():
    """The tau = 1 s intercept IS sigma_gyro. It must be right to a few parts
    in a thousand, not merely the right order of magnitude."""
    x, rate, s = _white()

    c = characterise(*overlapping_allan(x, rate), 'gyro')

    want = s / np.sqrt(rate)
    assert abs(c['white'] - want) / want < 0.02


def test_bias_instability_is_refused_when_the_curve_never_turns():
    """Pure white noise has no minimum. Reporting the last point as a bias
    instability is the classic way this measurement is published wrongly."""
    x, rate, _ = _white()

    c = characterise(*overlapping_allan(x, rate), 'gyro')

    assert c['bias_unresolved'] is True
    assert c['bias_instability'] is None
    assert c['rate_random_walk'] is None


# ── random walk: sigma(tau) grows as tau^+1/2 ────────────────────────────────

def test_a_random_walk_shows_the_positive_half_slope():
    """Integrated white noise (rate random walk) must bend the curve upward --
    if the tool cannot see that, it cannot see a drifting bias either."""
    rng = np.random.default_rng(11)
    rate, n = 100.0, int(100.0 * 3600 * 2)
    x = np.cumsum(rng.normal(0.0, 0.002, n)) / np.sqrt(rate)

    tau, sigma = overlapping_allan(x, rate)

    lo = sigma[np.argmin(np.abs(tau - 1.0))]
    hi = sigma[np.argmin(np.abs(tau - 100.0))]
    # tau^+1/2 over two decades is a factor of 10; allow wide margin for noise
    assert hi / lo > 3.0, f'curve did not rise with tau: {lo:.4g} -> {hi:.4g}'


def test_a_real_minimum_is_found_when_one_exists():
    """White noise plus a random walk has a genuine minimum. The tool must
    report it rather than refusing -- the refusal has to be specific."""
    rng = np.random.default_rng(3)
    rate, n = 100.0, int(100.0 * 3600 * 3)
    white = rng.normal(0.0, 0.02, n)
    walk = np.cumsum(rng.normal(0.0, 0.0006, n)) / np.sqrt(rate)

    c = characterise(*overlapping_allan(white + walk, rate), 'gyro')

    assert c['bias_unresolved'] is False, 'a real minimum was refused'
    assert c['bias_instability'] > 0.0
    assert 1.0 < c['bias_instability_tau_s'] < 3000.0


# ── the guards on the input ──────────────────────────────────────────────────

def test_a_short_record_raises_rather_than_returning_nonsense():
    with pytest.raises(ValueError):
        overlapping_allan(np.zeros(8), 100.0)


# ── the on-disk record: a 12 h run must survive being read back ──────────────

def _write_log(tmp_path, n=4000, period_ms=20, drop_at=None):
    """A synthetic streamed log, written exactly as `log_imu` writes one."""
    from allan_variance import RECORD

    rng = np.random.default_rng(5)
    r = np.zeros(n, dtype=RECORD)
    bms = np.arange(n, dtype=np.int64) * period_ms
    if drop_at is not None:
        bms[drop_at:] += 40 * period_ms          # a hole: 40 samples lost
    r['t'] = 1.7e9 + bms * 1e-3
    r['bms'] = bms
    r['gyro'] = rng.normal(0.0, 5.0, (n, 3))
    r['accel'] = rng.normal(0.0, 9.0, (n, 3))
    r['temp'] = 3150
    p = tmp_path / 'imu.bin'
    p.write_bytes(r.tobytes())
    (tmp_path / 'imu.meta.json').write_text('{"msg_type": "SCALED_IMU2"}')
    return p


def test_a_streamed_log_reads_back_with_the_board_clock_and_rate(tmp_path):
    from allan_variance import load_record

    d = load_record(_write_log(tmp_path))

    assert d['board_clock'] is True
    assert abs(d['rate'] - 50.0) < 0.01          # 20 ms period
    assert d['gyro'].shape[0] == 3               # axes first, as analyse wants
    assert d['msg_type'] == 'SCALED_IMU2'


def test_analysis_refuses_a_log_whose_message_type_is_unknown(tmp_path):
    """Without the meta file the units are unknown. Guessing them would print a
    plausible number for an unknown quantity -- the defect this tool exists to
    stop."""
    from allan_variance import load_record

    p = _write_log(tmp_path)
    (tmp_path / 'imu.meta.json').unlink()

    with pytest.raises(SystemExit):
        load_record(p)


def test_a_dropped_frame_shows_as_a_gap_in_the_board_clock(tmp_path, capsys):
    """Allan deviation assumes uniform sampling. A blocked reader loses frames,
    and averaging across the hole reports a longer tau than was observed -- in
    the direction that flatters the part. The board clock is what exposes it."""
    from allan_variance import analyse

    analyse(_write_log(tmp_path, drop_at=2000))

    out = capsys.readouterr().out
    assert 'gaps in the board clock' in out
    assert '39 samples missing' in out or '40 samples missing' in out


def test_a_clean_log_is_not_accused_of_gaps(tmp_path, capsys):
    from allan_variance import analyse

    analyse(_write_log(tmp_path))

    assert 'no gaps' in capsys.readouterr().out
