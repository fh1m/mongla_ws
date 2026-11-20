"""Score the localization candidates against exact truth, before any water.

Why this exists. The 2026-11-20 survey ranked what to change in the estimator;
three of its candidates can be settled on a trajectory whose truth is known
exactly, and a replay of the bench recording cannot do that (the hull there
never moves). This flies a pool route kinematically, synthesises every input
the node feeds the filter at the node's own rates and sigmas, and scores the
real `RIEKF` against the route.

Sensor numbers are the board's where measured-bars has them (accel bias
0.167 m/s^2 on x, gyro noise 0.01 rad/s, 1 mg accel quantisation, board yaw
drift 0.458 deg / 25 s); the rest are labelled ASSUMED below. A number from
here is a lower bound on water and its value is COMPARATIVE: which arm wins.

The bars, written before the first run (measured-bars 99 records the result):

  A1 accel-bias prior 0.2 (shipped) against 0.01: in MOTION, 0.2 must not be
     worse than 0.01 by more than 10 % on position RMS. At rest it already won.
  A2 flow scale error 0 / 2.5 % / 5 %: characterisation, no pass or fail. It
     sets how much a scale error costs per metre, which is what the pool-day
     scale bar (<= 2 %) has to buy.
  A3 a confidently wrong filter (5 % scale over the route) must ACCEPT one
     correct prop fix (sigma 0.15 m): the error after the fix must be under
     2 x 0.15 m. Refused = the chi-square lockout defect the survey warned of.
  A4 a 12 s flow dropout mid-transit: characterisation; error at the end of
     the dropout, and whether the B-56 gate closed.

`--self-test` must pass first: perfect sensors give under 1 cm, and a 5 %
scale error chosen on purpose must read as about 5 % of the distance flown.
A scorer that cannot see a bias it was handed is not evidence.

    python3 tools/studies/localization_candidates.py --self-test
    python3 tools/studies/localization_candidates.py
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_LOC = Path(__file__).resolve().parents[2] / 'src' / 'mongla_localization'
sys.path.insert(0, str(_LOC))

from mongla_localization.inekf import GRAVITY, RIEKF, State, so3_exp, so3_log  # noqa: E402


def _node_constant(name: str) -> float:
    """A shipped constant, read from the node's source: one truth, no copy."""
    import re
    src = (_LOC / 'mongla_localization' / 'localization_node.py').read_text()
    return float(re.search(rf'^{name} = ([0-9.]+)', src, re.M).group(1))


FLOW_SCALE_SIGMA = _node_constant('FLOW_SCALE_SIGMA')
ACCEL_BIAS_PRIOR = _node_constant('ACCEL_BIAS_PRIOR')

IMU_HZ = 50.0          # measured: SCALED_IMU2 at 50.05 Hz
FLOW_HZ = 50.0
DEPTH_HZ = 10.0
ZUPT_PERIOD_S = 1.0    # the node's 50-sample window, cleared after each fire
SPEED = 0.4            # m/s cruise


@dataclass
class Sensors:
    gyro_noise: float = 0.01                    # measured: sd 8-15 mrad/s
    accel_noise: float = 0.02                   # ASSUMED white part
    accel_quant: float = 0.00981                # measured: 1 mg on the wire
    accel_bias: tuple = (0.167, -0.05, 0.03)    # x measured; y, z ASSUMED
    gyro_bias: tuple = (0.002, -0.001, 0.0015)  # ASSUMED
    att_noise_deg: float = 0.15                 # measured roll/pitch 0.10-0.16
    att_yaw_drift_dps: float = 0.458 / 25.0     # measured
    flow_noise: float = 0.01                    # ASSUMED
    flow_scale: float = 0.0
    depth_noise: float = 0.01
    seed: int = 1

    @classmethod
    def perfect(cls) -> 'Sensors':
        return cls(0.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0), 0.0, 0.0, 0.0)


def _ramp(t, t0, t1):
    """0 before t0, 1 after t1, a cosine between."""
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    return 0.5 - 0.5 * math.cos(math.pi * (t - t0) / (t1 - t0))


def profile(t):
    """(forward speed m/s, yaw rad) of the route at time t."""
    u = SPEED * (_ramp(t, 5, 6) - _ramp(t, 30, 31) + _ramp(t, 40, 41) - _ramp(t, 53, 54))
    yaw = math.radians(90.0) * _ramp(t, 34, 37)
    return u, yaw


STILL = ((0.0, 5.0), (31.5, 34.0), (37.5, 40.0), (54.5, 60.0))
SECONDS = 60.0


def _rot(phi, theta, psi):
    cr, sr, cp, sp, cy, sy = (math.cos(phi), math.sin(phi), math.cos(theta),
                              math.sin(theta), math.cos(psi), math.sin(psi))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def truth():
    """Every 50 Hz instant: R, v (world), p (world), integrated with the
    filter's own step so a perfect filter tracks it exactly."""
    dt = 1.0 / IMU_HZ
    n = int(SECONDS * IMU_HZ) + 1
    Rs, vs = [], []
    for k in range(n):
        t = k * dt
        u, yaw = profile(t)
        R = _rot(math.radians(2.0) * math.sin(0.7 * t),
                 math.radians(1.5) * math.sin(0.5 * t), yaw)
        Rs.append(R)
        vs.append(np.array([u * math.cos(yaw), u * math.sin(yaw), 0.0]))
    ps = [np.array([0.0, 0.0, 1.0])]
    for k in range(n - 1):
        aw = (vs[k + 1] - vs[k]) / dt
        ps.append(ps[k] + vs[k] * dt + 0.5 * aw * dt * dt)
    return Rs, vs, ps


@dataclass
class Score:
    rms: float
    final: float
    at_rest_5s: float
    end_of_dropout: float
    fix_accepted: bool | None
    after_fix: float
    sigma_before_fix: float
    fix_own_error: float
    gated: int
    distance: float

    def __str__(self):
        fx = '' if self.fix_accepted is None else (
            f' | fix {"ACCEPTED" if self.fix_accepted else "REFUSED"}, '
            f'sigma before {self.sigma_before_fix:.3f} m, error after {self.after_fix:.3f} m')
        dd = ('' if math.isnan(self.end_of_dropout)
              else f' | end of dropout {self.end_of_dropout:.3f} m')
        return (f'rms {self.rms:.3f} m  final {self.final:.3f} m '
                f'({100 * self.final / self.distance:.2f} % of {self.distance:.1f} m)  '
                f'rest 0-5 s {self.at_rest_5s:.4f} m  gated {self.gated}{dd}{fx}')


def run(s: Sensors, *, accel_bias_prior=0.2, dropout=(), fix_at=(), scale_sigma=0.0,
        fix_sigma=0.15, perfect_init=False) -> Score:
    rnd = random.Random(s.seed)
    Rs, vs, ps = truth()
    dt = 1.0 / IMU_HZ
    st = State()
    st.p = ps[0].copy()
    st.R = Rs[0].copy()           # the node seeds attitude from the board
    if perfect_init:
        st.v = vs[0].copy()
    f = RIEKF(state=st, P0_accel_bias=accel_bias_prior,
              odometry_scale_sigma=scale_sigma)
    sq, n = 0.0, 0
    rest = drop_end = after = sig_before = own = math.nan
    accepted = None
    last_zupt = -1.0
    dist = sum(float(np.linalg.norm(ps[k + 1][:2] - ps[k][:2])) for k in range(len(ps) - 1))
    for k in range(len(Rs) - 1):
        t = (k + 1) * dt
        R0, R1 = Rs[k], Rs[k + 1]
        w = so3_log(R0.T @ R1) / dt
        aw = (vs[k + 1] - vs[k]) / dt
        spec = R0.T @ (aw - GRAVITY)
        gyro = [w[i] + s.gyro_bias[i] + rnd.gauss(0, s.gyro_noise) for i in range(3)]
        acc = [spec[i] + s.accel_bias[i] + rnd.gauss(0, s.accel_noise) for i in range(3)]
        if s.accel_quant:
            acc = [round(a / s.accel_quant) * s.accel_quant for a in acc]
        f.predict(gyro, acc, dt)

        # Board attitude at 50 Hz, its yaw drifting as measured.
        noise = np.array([rnd.gauss(0, math.radians(s.att_noise_deg)) for _ in range(2)]
                         + [math.radians(s.att_yaw_drift_dps * t)])
        f.update_attitude(so3_exp(noise) @ R1, sigma_deg=0.5)

        vb = R1.T @ vs[k + 1]
        dropped = any(lo <= t < hi for lo, hi in dropout)
        if not dropped and (k + 1) % int(IMU_HZ / FLOW_HZ) == 0:
            fv = vb[:2] * (1.0 + s.flow_scale) + [rnd.gauss(0, s.flow_noise) for _ in range(2)]
            var = max(s.flow_noise, 1e-3) ** 2
            f.update_body_velocity_xy(float(fv[0]), float(fv[1]), var, var)
        if (k + 1) % int(IMU_HZ / DEPTH_HZ) == 0:
            f.update_depth(-(ps[k + 1][2] + rnd.gauss(0, s.depth_noise)), sigma=0.02)
        still = any(lo + 1.0 <= t < hi for lo, hi in STILL)
        if still and t - last_zupt >= ZUPT_PERIOD_S:
            f.update_zero_velocity(sigma=0.01)
            last_zupt = t
        if any(abs(t - fa) < dt / 2 for fa in fix_at):
            # The LAST fix is the one scored; earlier ones shrink sigma first.
            sig_before = math.sqrt(max(f.P[6, 6], f.P[7, 7]))
            z = ps[k + 1][:2] + [rnd.gauss(0, fix_sigma) for _ in range(2)]
            own = float(np.linalg.norm(z - ps[k + 1][:2]))
            accepted = f.update_position(z, sigma=fix_sigma)
            after = float(np.linalg.norm(f.X.p[:2] - ps[k + 1][:2]))

        e = float(np.linalg.norm(f.X.p[:2] - ps[k + 1][:2]))
        sq += e * e
        n += 1
        if abs(t - 5.0) < dt / 2:
            rest = e
        if dropout and abs(t - dropout[0][1]) < dt / 2:
            drop_end = e
    return Score(math.sqrt(sq / n), e, rest, drop_end, accepted, after,
                 sig_before, own, f.gated, dist)


def self_test() -> bool:
    ok = True
    p = run(Sensors.perfect(), perfect_init=True)
    print(f'perfect sensors          {p}')
    ok &= p.final < 0.01 and p.rms < 0.01
    s = Sensors.perfect()
    s.flow_scale = 0.05
    b = run(s, perfect_init=True)
    print(f'5 % scale, chosen        {b}')
    ok &= 0.035 <= b.final / b.distance <= 0.065
    print('self-test', 'PASS' if ok else 'FAIL')
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--seeds', type=int, default=5)
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1
    if not self_test():
        return 1
    seeds = range(1, a.seeds + 1)

    def arm(label, scale=0.0, **kw):
        rs = [run(Sensors(seed=sd, flow_scale=scale), **kw) for sd in seeds]

        def m(attr):
            return float(np.mean([getattr(r, attr) for r in rs]))
        line = (f'{label:<34} rms {m("rms"):.3f}  final {m("final"):.3f} m '
                f'({100 * m("final") / rs[0].distance:.2f} %)  rest5 {m("at_rest_5s"):.4f}  '
                f'gated {m("gated"):.0f}')
        if not math.isnan(rs[0].end_of_dropout):
            line += f'  end-of-dropout {m("end_of_dropout"):.3f}'
        if rs[0].fix_accepted is not None:
            line += (f'  fix accepted {sum(bool(r.fix_accepted) for r in rs)}/{len(rs)}'
                     f'  sigma-before {m("sigma_before_fix"):.3f}  after {m("after_fix"):.3f}')
        print(line)
        return rs

    print(f'\nA1 accel-bias prior, flow perfect in scale, {len(seeds)} seeds')
    a01 = arm('  prior 0.01', accel_bias_prior=0.01)
    a20 = arm(f'  prior {ACCEL_BIAS_PRIOR} (shipped)', accel_bias_prior=ACCEL_BIAS_PRIOR)
    r01 = np.mean([r.rms for r in a01])
    r20 = np.mean([r.rms for r in a20])
    print(f'  bar: 0.2 within +10 % of 0.01 on rms -> {"PASS" if r20 <= 1.1 * r01 else "FAIL"} '
          f'({r20:.3f} vs {r01:.3f})')
    print('\nA2 flow scale error')
    for sc in (0.0, 0.025, 0.05):
        arm(f'  scale {100 * sc:.1f} %', scale=sc)
    print('\nA3 confidently wrong (5 % scale), correct fixes, sigma 0.15 m;'
          f' scale-error floor off (k=0) and as shipped (k={FLOW_SCALE_SIGMA})')
    for k in (0.0, FLOW_SCALE_SIGMA):
        for label, fa in ((f'  k={k} one fix at 59 s', (59.0,)),
                          (f'  k={k} fixes at 20 s, then 59 s', (20.0, 59.0)),
                          (f'  k={k} fixes at 6 s, then 59 s', (6.0, 59.0))):
            a3 = arm(label, scale=0.05, fix_at=fa, scale_sigma=k)
            ok_w = all(r.fix_accepted and r.after_fix < 0.30 for r in a3)
            ok_m = all(r.fix_accepted and r.after_fix <= r.fix_own_error + 0.05 for r in a3)
            print(f'    bar as written (accepted, under 0.30 m after): {"PASS" if ok_w else "FAIL"}'
                  f' | the fix\'s own error, per seed: '
                  + ' '.join(f'{r.fix_own_error:.2f}' for r in a3))
            print('    bar meant (accepted, after <= own error + 0.05 m): '
                  f'{"PASS" if ok_m else "FAIL"}'
                  f' | after, per seed: ' + ' '.join(f'{r.after_fix:.2f}' for r in a3))
    print('\nA4 flow dropout 41-53 s (the second cruise)')
    arm('  no dropout')
    arm('  dropout 12 s', dropout=((41.0, 53.0),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
