#!/usr/bin/env python3
"""Allan deviation of the board's IMU -- the measurement `Q` has never had.

⛔ WHY THIS EXISTS. `inekf.py` ships process noise as a NOMINAL DIAGONAL:

    sigma_gyro 0.01, sigma_accel 0.1, bias 1e-4 / 1e-3, and a position block of
    EXACTLY ZERO

Nothing in this tree derives those from the part we actually fly. That would be
merely untidy, except for what the estimator does with them: every measurement
goes through a chi-square gate, and after five consecutive rejections the gate
is bypassed once and P is inflated x4.

    The NEES/NIS literature is explicit: those statistics are chi-square
    distributed ONLY FOR AN ALREADY-TUNED FILTER.

So today a rejection cannot distinguish "this measurement is an outlier" from
"this filter is mistuned", and the lockout break is built on top of that. This
run is a PRECONDITION for the gate meaning anything -- not a refinement.

WHAT IT PRODUCES, per axis:

    angle/velocity random walk   the tau^-1/2 slope -- white noise. This is the
                                 number sigma_gyro / sigma_accel should BE.
    bias instability             the flat minimum -- the floor no averaging
                                 beats. Bounds how long dead reckoning is worth
                                 anything.
    rate random walk             the tau^+1/2 slope -- the slow wander a bias
                                 state has to track.

    python3 tools/allan_variance.py --log --port /dev/ttyACM0 --hours 12
    python3 tools/allan_variance.py --analyse imu_allan_<stamp>.npz
    python3 tools/allan_variance.py --selftest      # truth test, no hardware

⚠ THE OVERCLAIM THIS TOOL REFUSES TO MAKE. Bias instability is only readable if
the curve has actually TURNED -- if the minimum sits at the longest tau, the log
was too short and that "minimum" is just where the data ran out. That is the
classic way this measurement is reported wrongly, and `characterise()` returns
UNRESOLVED rather than printing the last point.
"""
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

# A log shorter than this cannot resolve a bias-instability minimum for a
# consumer MEMS part, whose 1/f corner typically sits in the 10-1000 s decade.
MIN_USEFUL_HOURS = 2.0
MAX_TAU_FRACTION = 0.4          # never average over more than 40 % of the record
# A long unattended run that only writes at the end is one power blip away from
# losing everything. Checkpoint, so the worst case costs five minutes, not a
# night -- the same lesson that cost two research sweeps when agents held their
# findings in memory and then hit a limit.
CHECKPOINT_S = 300.0


# ───────────────────────────────────────────────────────────── the maths ──

def overlapping_allan(x: np.ndarray, rate_hz: float,
                      taus: np.ndarray | None = None):
    """Overlapping Allan deviation of a RATE signal sampled at `rate_hz`.

    `x` is the instantaneous measurement (rad/s, or m/s^2), not an integrated
    angle. Returns (tau_s, sigma) with sigma in the same units as `x`.

        theta_k      = cumulative sum of x / rate            (angle / velocity)
        sigma^2(tau) = sum_k (theta_{k+2m} - 2 theta_{k+m} + theta_k)^2
                       / (2 tau^2 (N - 2m))

    Overlapping rather than non-overlapping: it reuses samples for far better
    confidence at long tau. The cost is correlated points, which matters for
    error bars, not for the estimate.
    """
    x = np.asarray(x, dtype=float)
    n = x.size
    if n < 16:
        raise ValueError(f'need at least 16 samples, got {n}')
    dt = 1.0 / float(rate_hz)

    theta = np.concatenate(([0.0], np.cumsum(x) * dt))      # N+1 points

    if taus is None:
        m_max = int(n * MAX_TAU_FRACTION / 2)
        if m_max < 1:
            raise ValueError('record too short for any averaging factor')
        m = np.unique(np.floor(np.logspace(0, np.log10(m_max), 120)).astype(int))
    else:
        m = np.unique(np.maximum(1, np.round(np.asarray(taus) * rate_hz)).astype(int))
        m = m[m <= int(n * MAX_TAU_FRACTION / 2)]

    tau, sigma = [], []
    for mi in m:
        if n - 2 * mi < 1:
            continue
        d = theta[2 * mi:] - 2.0 * theta[mi:-mi] + theta[:-2 * mi]
        t = mi * dt
        var = np.sum(d * d) / (2.0 * t * t * d.size)
        tau.append(t)
        sigma.append(np.sqrt(var))
    return np.asarray(tau), np.asarray(sigma)


def _fit_slope(tau, sigma, slope, lo, hi):
    """Fit sigma = c * tau**slope over [lo, hi] with the slope FIXED.

    Returns None on an empty or too-thin window rather than fitting air.
    """
    m = (tau >= lo) & (tau <= hi)
    if m.sum() < 3:
        return None
    return float(np.exp(np.mean(np.log(sigma[m]) - slope * np.log(tau[m]))))


def characterise(tau: np.ndarray, sigma: np.ndarray, kind: str) -> dict:
    """Pull the three coefficients off an Allan curve, refusing to guess."""
    out: dict = {'tau_min_s': float(tau[0]), 'tau_max_s': float(tau[-1])}

    # white noise: the tau^-1/2 region, conventionally read at tau = 1 s
    c = _fit_slope(tau, sigma, -0.5, tau[0], max(1.0, tau[0] * 10))
    out['white'] = c                  # (rad/s)/sqrt(Hz) or (m/s^2)/sqrt(Hz)
    if c is not None:
        if kind == 'gyro':
            out['arw_deg_per_sqrt_hr'] = c * np.degrees(1.0) * 60.0
        else:
            out['vrw_m_per_s_per_sqrt_hr'] = c * 60.0

    # bias instability: the flat minimum, scaled by the standard 0.664 factor
    i = int(np.argmin(sigma))
    at_edge = (i == 0) or (i == sigma.size - 1)
    out['bias_instability'] = None if at_edge else float(sigma[i] / 0.664)
    out['bias_instability_tau_s'] = None if at_edge else float(tau[i])
    out['bias_unresolved'] = bool(at_edge)

    # rate random walk: the tau^+1/2 region beyond the minimum, if one exists
    out['rate_random_walk'] = (None if at_edge
                               else _fit_slope(tau, sigma, 0.5, tau[i] * 2, tau[-1]))
    return out


# ──────────────────────────────────────────────────────────── the logger ──

def _save(out_path: Path, ts, bts, gx, gy, gz, ax, ay, az, temp, mtype: str) -> None:
    """Write the record so far. Atomic: write a temp file and rename, so a kill
    mid-write cannot leave a truncated npz where a good one used to be."""
    t = np.asarray(ts)
    b = np.asarray(bts, dtype=float)
    if t.size < 2:
        return
    board_clock = bool(np.any(b > 0)) and float(np.median(np.diff(b))) > 0
    rate = (1000.0 / float(np.median(np.diff(b))) if board_clock
            else 1.0 / float(np.median(np.diff(t))))
    # numpy appends '.npz' to any name that lacks it, so the temp file must
    # already end in .npz or savez writes somewhere we did not name.
    tmp = out_path.with_name(out_path.stem + '.part.npz')
    np.savez_compressed(
        tmp, t=t, t_board_ms=b, board_clock=board_clock, rate_hz=rate,
        msg_type=mtype,
        gyro=np.vstack([gx, gy, gz]).astype(float),
        accel=np.vstack([ax, ay, az]).astype(float),
        temp_cdegc=np.asarray(temp, dtype=float),
        taken=datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'))
    tmp.replace(out_path)


def log_imu(port: str, hours: float, out_path: Path) -> Path:
    """Passively record the board's IMU.

    Reads only. The vehicle must be STILL and undisturbed for the whole run --
    a door slam is a rate-random-walk artefact that no analysis can remove.
    """
    from pymavlink import mavutil

    seconds = hours * 3600.0
    print(f'logging {hours:.2f} h from {port} -> {out_path.name}', flush=True)
    m = mavutil.mavlink_connection(port, baud=115200)

    t_start = time.time()
    t_end = t_start + seconds
    ts, bts, gx, gy, gz, ax, ay, az, temp = [], [], [], [], [], [], [], [], []
    last_report = t_start
    msg = None
    while time.time() < t_end:
        msg = m.recv_match(type=['SCALED_IMU2', 'RAW_IMU', 'SCALED_IMU'],
                           blocking=True, timeout=5.0)
        if msg is None:
            continue
        d = msg.to_dict()
        ts.append(time.time())
        # ⚠ THE BOARD'S OWN CLOCK, NOT ARRIVAL TIME. Allan deviation assumes
        # uniform sampling, and host arrival jitter on this link was measured at
        # sd 4.741 ms -- which at a 20 ms period is a quarter of the interval and
        # would land entirely in the short-tau region we are trying to read.
        # `time_boot_ms` is stamped where the sample was taken.
        bts.append(d.get('time_boot_ms', 0))
        gx.append(d['xgyro']); gy.append(d['ygyro']); gz.append(d['zgyro'])
        ax.append(d['xacc']);  ay.append(d['yacc']);  az.append(d['zacc'])
        temp.append(d.get('temperature', 0))     # cdegC; 0 when unpopulated
        if time.time() - last_report > CHECKPOINT_S:
            _save(out_path, ts, bts, gx, gy, gz, ax, ay, az, temp, msg.get_type())
            print(f'  {(time.time() - t_start) / 3600.0:.2f} h, {len(ts)} samples, '
                  f'checkpointed', flush=True)
            last_report = time.time()

    ts = np.asarray(ts)
    bts = np.asarray(bts, dtype=float)
    if ts.size < 1000:
        raise SystemExit(f'only {ts.size} samples -- is the board streaming IMU?')

    # Prefer the board clock. Fall back to arrival time only if the board does
    # not stamp, and SAY SO -- a silent fallback would quietly invalidate the
    # short-tau end of every curve.
    board_clock = bool(np.any(bts > 0)) and float(np.median(np.diff(bts))) > 0
    if board_clock:
        rate = 1000.0 / float(np.median(np.diff(bts)))
        jitter = float(np.std(np.diff(ts))) * 1e3
        print(f'using the board clock: {rate:.2f} Hz '
              f'(host arrival jitter was sd {jitter:.2f} ms, excluded)')
    else:
        rate = 1.0 / float(np.median(np.diff(ts)))
        print(f'⚠ board does not stamp time_boot_ms -- falling back to ARRIVAL '
              f'time at {rate:.2f} Hz. Short-tau results carry link jitter.')
    _save(out_path, ts, bts, gx, gy, gz, ax, ay, az, temp, msg.get_type())
    print(f'wrote {out_path}  {ts.size} samples at {rate:.1f} Hz')
    return out_path


# ────────────────────────────────────────────────────────── the analysis ──

def analyse(path: Path) -> int:
    d = np.load(path)   # no allow_pickle: numeric arrays + plain strings only
    rate = float(d['rate_hz'])
    t = d['t']
    hours = (t[-1] - t[0]) / 3600.0
    mtype = str(d['msg_type'])

    # SCALED_IMU*: gyro mrad/s -> rad/s, accel mG -> m/s^2
    gyro = d['gyro'] * 1e-3
    accel = d['accel'] * 9.80665e-3

    clock = 'board clock' if bool(d['board_clock']) else 'HOST ARRIVAL TIME (jittered)'
    print(f'\n{path.name}: {t.size} samples, {rate:.1f} Hz, {hours:.2f} h, {mtype}, {clock}')
    if 'temp_cdegc' in d.files:
        tc = np.asarray(d['temp_cdegc'], dtype=float)
        if np.any(tc != 0):
            span = (tc.max() - tc.min()) / 100.0
            print(f'IMU temperature {tc.min()/100:.1f} to {tc.max()/100:.1f} C '
                  f'(span {span:.2f} C)')
            if span > 2.0:
                print('⚠ that span is large enough to move bias. Any rate-random-walk '
                      'read off this curve may be thermal, not intrinsic.')
        else:
            print('IMU temperature field is not populated by this firmware')
    if hours < MIN_USEFUL_HOURS:
        print(f'⚠ {hours:.2f} h is under the {MIN_USEFUL_HOURS} h floor -- the '
              f'bias-instability minimum will very likely be unresolved.')

    for name, data, kind in (('gyro', gyro, 'gyro'), ('accel', accel, 'accel')):
        print(f'\n── {name} ──')
        for i, axis in enumerate('xyz'):
            tau, sigma = overlapping_allan(data[i], rate)
            c = characterise(tau, sigma, kind)
            if kind == 'gyro':
                w = c.get('arw_deg_per_sqrt_hr')
                wtxt = f'ARW {w:.4f} deg/sqrt(h)' if w else 'ARW unresolved'
            else:
                w = c.get('vrw_m_per_s_per_sqrt_hr')
                wtxt = f'VRW {w:.5f} (m/s)/sqrt(h)' if w else 'VRW unresolved'
            if c['bias_unresolved']:
                btxt = 'bias instability UNRESOLVED (minimum at the edge -- log longer)'
            else:
                btxt = (f"bias instability {c['bias_instability']:.3e} "
                        f"at tau={c['bias_instability_tau_s']:.1f} s")
            print(f'  {axis}: {wtxt};  {btxt}')
            if c['bias_instability_tau_s'] and c['bias_instability_tau_s'] > 0.01 * hours * 3600:
                print(f'       ⚠ that minimum sits past 1 % of the record; '
                      f'overlapping Allan has few independent samples there')

    print('\nWhat to do with these:')
    print('  * the white-noise number IS sigma_gyro / sigma_accel in inekf.py')
    print('  * bias instability bounds how long dead reckoning is worth anything')
    print('  * until these exist, the chi-square gate cannot separate an outlier')
    print('    from a mistuned filter -- see .claude/context/sota/localization.md')
    return 0


# ─────────────────────────────────────────────────────────── truth test ──

def selftest() -> int:
    """Construct a signal whose Allan curve is KNOWN and check we recover it.

    White noise of standard deviation s, sampled at rate f, has
        sigma_allan(tau) = s / sqrt(f * tau)
    so the tau = 1 s intercept is exactly s / sqrt(f). Nothing is fitted to our
    own output: the answer is fixed before the measurement runs.
    """
    rng = np.random.default_rng(7)
    rate, s = 100.0, 0.02                     # 100 Hz, 0.02 rad/s white noise
    n = int(rate * 3600 * 2)                  # two hours
    x = rng.normal(0.0, s, n)

    tau, sigma = overlapping_allan(x, rate)
    expected = s / np.sqrt(rate * tau)
    # Judge where the estimator HAS confidence. Overlapping Allan reuses
    # samples, so the number of independent differences falls as tau grows:
    # measured on this very signal, worst error is 2.16 % out to 30 s, 9.69 %
    # to 100 s and 26.27 % to 300 s. Asserting on the tail would be asserting
    # on sampling noise.
    band = (tau >= 0.1) & (tau <= 30.0)
    worst = float(np.max(np.abs(sigma[band] - expected[band]) / expected[band]))

    c = characterise(tau, sigma, 'gyro')
    intercept, want = c['white'], s / np.sqrt(rate)

    print(f'white-noise truth test: worst relative error over tau 0.1-30 s = '
          f'{worst * 100:.2f} %  (bound 5 %)')
    print(f'  tau=1 s intercept  measured {intercept:.6f}  expected {want:.6f}  '
          f'({abs(intercept - want) / want * 100:.2f} %)')

    ok = worst < 0.05 and abs(intercept - want) / want < 0.02
    # the refusal must fire: pure white noise never turns, so bias instability
    # MUST come back unresolved rather than as the last point
    if not c['bias_unresolved']:
        print('  ⛔ bias instability was reported for a pure white-noise signal, '
              'which has no minimum -- the refusal is broken')
        ok = False
    else:
        print('  bias instability correctly reported UNRESOLVED for white noise')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--log', action='store_true', help='record from the board')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--hours', type=float, default=12.0)
    ap.add_argument('--out', help='where to write (default: CWD)')
    ap.add_argument('--analyse', metavar='NPZ')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if a.log:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        # CWD, not the source tree: this tool is copied to the vehicle and run
        # from /tmp, where parents[1] is '/' and the write is refused.
        out = Path(a.out) if a.out else Path.cwd() / f'imu_allan_{stamp}.npz'
        log_imu(a.port, a.hours, out)
        return analyse(out)
    if a.analyse:
        return analyse(Path(a.analyse))
    ap.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
