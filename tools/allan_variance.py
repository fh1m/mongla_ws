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

    python3 tools/allan_variance.py --log --port /dev/ttyUSB0 --hours 12
    python3 tools/allan_variance.py --analyse imu_allan_<stamp>.bin
    python3 tools/allan_variance.py --selftest      # truth test, no hardware

⚠ THE OVERCLAIM THIS TOOL REFUSES TO MAKE. Bias instability is only readable if
the curve has actually TURNED -- if the minimum sits at the longest tau, the log
was too short and that "minimum" is just where the data ran out. That is the
classic way this measurement is reported wrongly, and `characterise()` returns
UNRESOLVED rather than printing the last point.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


# A log shorter than this cannot resolve a bias-instability minimum for a
# consumer MEMS part, whose 1/f corner typically sits in the 10-1000 s decade.
MIN_USEFUL_HOURS = 2.0
MAX_TAU_FRACTION = 0.4          # never average over more than 40 % of the record
# A long unattended run that only writes at the end is one power blip away from
# losing everything. Flush, so the worst case costs five minutes, not a night.
CHECKPOINT_S = 300.0
# How long without an IMU frame before the logger says so. A stream that dies
# at hour 1 must not be discovered at hour 12.
STALE_S = 30.0

# ─────────────────────────────────────────────────── the on-disk record ──

# One sample, 38 bytes, written the moment it arrives. The logger holds NO
# history, and that is the point: 12 h at 50 Hz is 2.16 M samples, which as
# Python lists costs ~700 MB of boxed objects on a board that has 3 983 MB and
# also has to run the detector. Streaming makes memory flat and the per-sample
# cost constant.
#
# It also deletes a subtler defect. The previous design re-serialised the WHOLE
# record every five minutes, inside the read loop -- so the checkpoint meant to
# protect the run would, by hour 11, have blocked the reader long enough to
# overflow the tty buffer and drop frames. Dropped frames are not a lost sample:
# they are a GAP, and overlapping Allan deviation assumes uniform sampling.
# The guard would have quietly corrupted the measurement it was guarding.
RECORD = np.dtype([('t', '<f8'), ('bms', '<u4'),
                   ('gyro', '<f4', (3,)), ('accel', '<f4', (3,)),
                   ('temp', '<i2')])


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


def log_imu(port: str, hours: float, out_path: Path) -> Path:
    """Passively record the board's IMU, one sample at a time, straight to disk.

    Reads only. The vehicle must be STILL and undisturbed for the whole run --
    a door slam is a rate-random-walk artefact that no analysis can remove.
    """
    from pymavlink import mavutil

    out_path = out_path.with_suffix('.bin')
    meta_path = out_path.with_suffix('.meta.json')
    seconds = hours * 3600.0
    print(f'logging {hours:.2f} h from {port} -> {out_path.name}', flush=True)
    m = mavutil.mavlink_connection(port, baud=115200)

    rec = np.zeros(1, dtype=RECORD)
    t_start = time.time()
    t_end = t_start + seconds
    last_flush = last_msg = t_start
    n, mtype, complained = 0, None, False

    with open(out_path, 'wb') as f:
        while time.time() < t_end:
            now = time.time()
            # ⚠ BOTH CHECKS LIVE ABOVE THE RECEIVE, on purpose. Put them below
            # the `msg is None: continue` and a stream that dies at hour 1 never
            # flushes and never complains again -- you find out at hour 12.
            if now - last_flush > CHECKPOINT_S:
                f.flush()
                os.fsync(f.fileno())
                print(f'  {(now - t_start) / 3600.0:.2f} h, {n} samples, flushed',
                      flush=True)
                last_flush = now
            if now - last_msg > STALE_S and not complained:
                print(f'⚠ no IMU frame for {now - last_msg:.0f} s -- the board may '
                      f'have reset or the port re-enumerated. Still listening.',
                      flush=True)
                complained = True

            msg = m.recv_match(type=['SCALED_IMU2', 'RAW_IMU', 'SCALED_IMU'],
                               blocking=True, timeout=5.0)
            if msg is None:
                continue
            d = msg.to_dict()
            if mtype is None:
                mtype = msg.get_type()
                # Written on the FIRST sample, not at the end: a run killed at
                # hour 11 must still be readable. The sample count is not stored
                # here -- it is the file size divided by the record size, so
                # there is only ever one truth about how long the run was.
                meta_path.write_text(json.dumps({
                    'msg_type': mtype, 'port': port,
                    'record_dtype': [[k, str(RECORD[k].base), list(RECORD[k].shape)]
                                     for k in RECORD.names],
                    'started': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
                }, indent=2))
            last_msg, complained = time.time(), False

            rec['t'] = last_msg
            rec['bms'] = d.get('time_boot_ms', 0)
            rec['gyro'] = (d['xgyro'], d['ygyro'], d['zgyro'])
            rec['accel'] = (d['xacc'], d['yacc'], d['zacc'])
            rec['temp'] = d.get('temperature', 0)      # cdegC; 0 when unpopulated
            f.write(rec.tobytes())
            n += 1

    if n < 1000:
        raise SystemExit(f'only {n} samples -- is the board streaming IMU?')
    print(f'wrote {out_path}  {n} samples')
    return out_path


def load_record(path: Path) -> dict:
    """Read a streamed log back. Refuses rather than guessing the scaling."""
    r = np.fromfile(path, dtype=RECORD)
    if r.size < 2:
        raise SystemExit(f'{path} holds {r.size} samples')
    meta_path = path.with_suffix('.meta.json')
    if not meta_path.exists():
        # The unit scaling depends on which message this was. Inventing one
        # would produce a plausible number for an unknown quantity, which is
        # exactly the defect this whole tool exists to stop.
        raise SystemExit(f'{meta_path.name} is missing -- the message type, and '
                         f'so the units, are unknown. Refusing to analyse.')
    meta = json.loads(meta_path.read_text())

    bms = r['bms'].astype(float)
    # Prefer the board clock. Fall back to arrival time only if the board does
    # not stamp, and SAY SO -- a silent fallback quietly invalidates the
    # short-tau end of every curve.
    board_clock = bool(np.any(bms > 0)) and float(np.median(np.diff(bms))) > 0
    if board_clock:
        rate = 1000.0 / float(np.median(np.diff(bms)))
    else:
        rate = 1.0 / float(np.median(np.diff(r['t'])))
    return {'t': r['t'], 'bms': bms, 'rate': rate, 'board_clock': board_clock,
            'gyro': r['gyro'].T.astype(float), 'accel': r['accel'].T.astype(float),
            'temp': r['temp'].astype(float), 'msg_type': meta['msg_type']}


# ────────────────────────────────────────────────────────── the analysis ──

def analyse(path: Path) -> int:
    d = load_record(path)
    rate, t = d['rate'], d['t']
    hours = (t[-1] - t[0]) / 3600.0

    # SCALED_IMU*: gyro mrad/s -> rad/s, accel mG -> m/s^2
    gyro = d['gyro'] * 1e-3
    accel = d['accel'] * 9.80665e-3

    clock = 'board clock' if d['board_clock'] else 'HOST ARRIVAL TIME (jittered)'
    print(f"\n{path.name}: {t.size} samples, {rate:.1f} Hz, {hours:.2f} h, "
          f"{d['msg_type']}, {clock}")
    if d['board_clock']:
        jitter = float(np.std(np.diff(t))) * 1e3
        print(f'host arrival jitter was sd {jitter:.2f} ms, excluded')

    # ⚠ THE GAP CHECK. Allan deviation assumes UNIFORM sampling. A dropped frame
    # is not a lost sample, it is a hole, and averaging across it reports a
    # longer tau than was actually observed -- silently, and in the direction
    # that flatters the part. The board's own clock is what makes this visible:
    # a hole shows as a step in time_boot_ms that arrival time cannot reveal.
    step = np.diff(d['bms'])
    nominal = float(np.median(step))
    holes = int(np.sum(step > 3 * nominal))
    if holes:
        lost = float(np.sum(step[step > 3 * nominal]) / nominal) - holes
        print(f'⚠ {holes} gaps in the board clock ({lost:.0f} samples missing, '
              f'{100.0 * lost / (t.size + lost):.3f} % of the record; '
              f'worst {step.max() / nominal:.1f} x the {nominal:.1f} ms period)')
        if lost / (t.size + lost) > 0.001:
            print('  that is enough to bias the long-tau end. Find what blocked '
                  'the reader before trusting the bias-instability number.')
    else:
        print(f'no gaps: every interval within 3x the {nominal:.1f} ms period')

    tc = d['temp']
    if np.any(tc != 0):
        span = (tc.max() - tc.min()) / 100.0
        print(f'IMU temperature {tc.min() / 100:.1f} to {tc.max() / 100:.1f} C '
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
    # ⚠ THE BOARD PRESENTS TWO SERIAL DEVICES, and only one speaks MAVLink:
    #   /dev/ttyUSB0  CH340 (1a86:7523)  -- the ESP32. MAVLink. THIS one.
    #   /dev/ttyACM0  Pico 2 (2e8a:000f) -- the RP2350's ESC debug console, which
    #                 emits plain text ("M1[cmd=0 out=0 rpm=0 ...]"). pymavlink
    #                 decodes that as an endless run of BAD_DATA, so the port
    #                 looks busy and alive while yielding no IMU at all.
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--hours', type=float, default=12.0)
    ap.add_argument('--out', help='where to write (default: CWD)')
    ap.add_argument('--analyse', metavar='BIN')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if a.log:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        # CWD, not the source tree: this tool is copied to the vehicle and run
        # from /tmp, where parents[1] is '/' and the write is refused.
        out = Path(a.out) if a.out else Path.cwd() / f'imu_allan_{stamp}.bin'
        # analyse what log_imu ACTUALLY wrote -- it normalises the suffix, and
        # analysing the name we asked for rather than the name it returned is
        # how a tool ends up reading a file that is not the one it just made.
        return analyse(log_imu(a.port, a.hours, out))
    if a.analyse:
        return analyse(Path(a.analyse))
    ap.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
