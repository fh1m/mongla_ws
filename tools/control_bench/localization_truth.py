"""Fly the plant, synthesise the sensors, and score the EKF against TRUTH.

⛔ WHY THIS EXISTS. `mongla_localization`'s filter has 21 tests and every one of
them is a unit or property test: exp/log round-trip, covariance symmetry,
positive-definiteness, the sign of a single update. Not one drives it along a
DYNAMIC TRAJECTORY and asks how far it drifted -- because until the Fossen plant
existed there was no truth to drive it against.

CLAUDE.md section 9 names the rule this closes: *"Comparing a new estimator
against the incumbent measures agreement and cannot rank them; construct a case
where truth is known."* The plant IS that case. It produces exact position,
attitude and velocity at every tick, so the drift it measures is real error and
not disagreement.

⚠ WHAT A NUMBER FROM HERE MEANS. The plant is a model: its drag is a guess and
it has no currents, no thermal drift and no vibration. So a drift figure is a
LOWER BOUND on what the water will do, and its value is COMPARATIVE -- which
sensor dropout hurts most, whether an update helps at all, how error grows with
time. Never quote it as the vehicle's navigation accuracy.

⭐ ONE CONVENTION WE GOT FOR FREE. The filter's world is NED with z DOWN and its
body is FRD; Fossen is x-forward, y-starboard, z-DOWN. They are the same frame,
so nothing is remapped here -- and `test_the_plant_and_the_filter_share_a_frame`
pins that rather than trusting it.
"""
from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np                                            # noqa: E402

from flight import Vehicle                                    # noqa: E402
from plant import Damping                                     # noqa: E402

_LOC = Path(__file__).resolve().parents[2] / 'src' / 'mongla_localization'
if str(_LOC) not in sys.path:
    sys.path.insert(0, str(_LOC))

from mongla_localization.inekf import GRAVITY, RIEKF, State    # noqa: E402

# Measured on the vehicle, not assumed: `SCALED_IMU2` arrives at 50.05 Hz and
# the flow front-end runs with it. Depth is slower.
IMU_HZ = 50.0
FLOW_HZ = 50.0
DEPTH_HZ = 10.0
YAW_HZ = 10.0
# ⚠ A LANDMARK FIX IS INTERMITTENT BY NATURE -- it exists only while a prop is
# in view and resects. 2 Hz while visible is generous.
FIX_HZ = 2.0


def rot_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """Body(FRD) -> world(NED) for aerospace Z-Y-X Euler, matching the board."""
    cr, sr, cp, sp, cy, sy = (math.cos(phi), math.sin(phi), math.cos(theta),
                              math.sin(theta), math.cos(psi), math.sin(psi))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


@dataclass
class Sensors:
    """What the vehicle would report. ⚠ Noise levels are PLAUSIBLE, not
    measured on this IMU -- treat them as a knob, and sweep them."""
    gyro_noise: float = 0.002          # rad/s
    accel_noise: float = 0.02          # m/s^2
    gyro_bias: tuple = (0.0, 0.0, 0.0)
    accel_bias: tuple = (0.0, 0.0, 0.0)
    flow_noise: float = 0.01           # m/s
    depth_noise: float = 0.01          # m
    yaw_noise_deg: float = 1.0
    fix_noise_m: float = 0.5      # prop resection, world x/y
    seed: int = 1


@dataclass
class Score:
    """Drift against truth. All errors are |estimate - truth|."""
    pos_rms_m: float = 0.0
    pos_final_m: float = 0.0
    yaw_rms_deg: float = 0.0
    yaw_final_deg: float = 0.0
    depth_rms_m: float = 0.0
    vel_rms_ms: float = 0.0
    seconds: float = 0.0
    accepted: int = 0
    rejected: int = 0
    track: list = field(default_factory=list)

    def drift_rate_m_per_min(self) -> float:
        return self.pos_final_m / (self.seconds / 60.0) if self.seconds else 0.0

    def __str__(self) -> str:
        return (f'{self.seconds:5.1f} s   '
                f'pos rms {self.pos_rms_m:6.3f} m  final {self.pos_final_m:6.3f} m '
                f'({self.drift_rate_m_per_min():5.2f} m/min)   '
                f'yaw rms {self.yaw_rms_deg:5.2f} deg   '
                f'depth rms {self.depth_rms_m:5.3f} m   '
                f'acc {self.accepted} rej {self.rejected}')


def run(*, seconds: float = 60.0, sensors: Sensors | None = None,
        use_flow: bool = True, use_depth: bool = True, use_yaw: bool = True,
        flow_dropout: tuple = (), manoeuvre=None,
        gate_world_updates: bool = False,
        fix_windows: tuple = (), use_fix: bool = False,
        initial_pos_error_m: float = 0.0,
        filter_cls=None,
        damping: Damping | None = None) -> Score:
    """Fly a trajectory, feed the filter, score it against the plant's truth.

    `flow_dropout` is a list of (start_s, end_s) windows in which the velocity
    update is withheld -- the question the pool cannot answer cheaply: how far
    does it drift when the bottom camera loses the floor?

    ⭐ `gate_world_updates` withholds the DEPTH and YAW updates during the same
    window. That sounds backwards -- throwing away good measurements -- and it
    is the mitigation the failure mechanism implies. `update_depth`'s Jacobian
    carries `-skew(p)`, an attitude coupling that grows LINEARLY with distance
    from the origin, and the filter's own docstring calls the measurement
    "imperfect for a right-invariant filter". While velocity is pinned by flow
    that coupling is harmless; once it is not, a world-frame update injects
    attitude error, which in a right-invariant filter rotates the whole believed
    trajectory, which moves position further from the origin, which makes the
    coupling larger. Gating breaks that loop.
    """
    s = sensors or Sensors()
    rnd = random.Random(s.seed)
    d = damping or Damping.from_cd(cd_axial=0.25, cd_transverse=1.00,
                                   provenance='band midpoint, UNMEASURED')
    v = Vehicle(damping=d)
    v.plant.nu = [0.0] * 6
    v.plant.eta = [0.0] * 6
    v.board.reset()

    # ⛔ A HARNESS THAT STARTS THE FILTER AT TRUTH IS CHEATING, and the first
    # version of this file did. The filter is constructed believing +-1 m of
    # position uncertainty (P0_position); handing it a perfect origin gives it
    # an accuracy it has no way to know it has, and then any landmark fix looks
    # harmful by comparison. `initial_pos_error_m` gives it the error its own
    # covariance claims, which is the honest starting condition.
    st = State()
    if initial_pos_error_m:
        ang = rnd.uniform(0, 2 * math.pi)
        st.p = np.array([initial_pos_error_m * math.cos(ang),
                         initial_pos_error_m * math.sin(ang), 0.0])
    f = (filter_cls or RIEKF)(state=st)
    dt = 1.0 / IMU_HZ
    manoeuvre = manoeuvre or (lambda t: math.radians(30.0 * math.sin(t / 8.0)))

    sc = Score(seconds=seconds)
    pos_sq = yaw_sq = dep_sq = vel_sq = 0.0
    n = 0
    t = 0.0
    while t < seconds:
        # ── the plant, driven by the board's own cascade ──────────────────
        roll, pitch, yaw = v.plant.attitude
        gx, gy, gz = v.plant.body_rates
        v.board.hold_yaw(manoeuvre(t))
        tq = v.board.stabilize(stick_roll=0.0, stick_pitch=0.0, stick_yaw=0.0,
                               roll=roll, pitch=pitch, yaw=yaw,
                               gx=gx, gy=gy, gz=gz, dt=dt)
        from flight import body_to_cad, dshot_to_fraction
        a = v.alloc.allocate(**body_to_cad(roll=tq[0], pitch=tq[1], yaw=tq[2]))
        u = [dshot_to_fraction(v.board, x) for x in a.thrusts]
        tau = v._wrench(u)
        nu_dot, _ = v.plant.derivative(v.plant.nu, v.plant.eta, tau)
        v.plant.step(tau, dt)
        t += dt

        # ── truth, straight off the plant ─────────────────────────────────
        eta, nu = v.plant.eta, v.plant.nu
        R = rot_ned(eta[3], eta[4], eta[5])

        # ── synthesise the IMU ────────────────────────────────────────────
        # Specific force: inertial acceleration in body frame, less gravity.
        # ⛔ The transport term omega x v is NOT optional -- without it a turning
        # vehicle's accelerometer is wrong by exactly the centripetal term, and
        # the filter would be fed a lie that looks like a bias.
        w = np.array(nu[3:6])
        vb = np.array(nu[0:3])
        a_body = np.array(nu_dot[0:3]) + np.cross(w, vb)
        spec = a_body - R.T @ GRAVITY
        gyro = [w[i] + s.gyro_bias[i] + rnd.gauss(0, s.gyro_noise) for i in range(3)]
        accel = [spec[i] + s.accel_bias[i] + rnd.gauss(0, s.accel_noise)
                 for i in range(3)]
        f.predict(gyro, accel, dt)

        # ── the aiding updates, at their real rates ───────────────────────
        k = int(round(t * IMU_HZ))
        dropped = any(lo <= t < hi for lo, hi in flow_dropout)
        if use_flow and not dropped and k % max(1, int(IMU_HZ / FLOW_HZ)) == 0:
            f.update_body_velocity(
                [vb[i] + rnd.gauss(0, s.flow_noise) for i in range(3)],
                sigma=max(s.flow_noise, 1e-3))
        gated = dropped and gate_world_updates
        if use_depth and not gated and k % max(1, int(IMU_HZ / DEPTH_HZ)) == 0:
            f.update_depth(-eta[2] + rnd.gauss(0, s.depth_noise),
                           sigma=max(s.depth_noise, 1e-3))
        if use_yaw and not gated and k % max(1, int(IMU_HZ / YAW_HZ)) == 0:
            f.update_yaw(math.degrees(eta[5]) + rnd.gauss(0, s.yaw_noise_deg),
                         sigma_deg=max(s.yaw_noise_deg, 0.1))
        # ⭐ The landmark position fix. `update_position` carries the SAME
        # `-skew(p)` attitude coupling as `update_depth` (B-56), so whether it
        # bounds drift or feeds the same loop is a question, not an assumption.
        if use_fix and (not fix_windows
                        or any(lo <= t < hi for lo, hi in fix_windows)):
            if k % max(1, int(IMU_HZ / FIX_HZ)) == 0:
                f.update_position(
                    [eta[0] + rnd.gauss(0, s.fix_noise_m),
                     eta[1] + rnd.gauss(0, s.fix_noise_m)],
                    sigma=max(s.fix_noise_m, 1e-3))

        # ── score ─────────────────────────────────────────────────────────
        pe = math.dist(f.X.p[:2], eta[0:2])
        ye = abs((math.degrees(eta[5]) - f.X.yaw_deg() + 180) % 360 - 180)
        de = abs(f.X.p[2] - eta[2])
        ve = float(np.linalg.norm(R.T @ f.X.v - vb))
        pos_sq += pe * pe; yaw_sq += ye * ye; dep_sq += de * de; vel_sq += ve * ve
        n += 1
        sc.pos_final_m, sc.yaw_final_deg = pe, ye
        if n % 25 == 0:
            sc.track.append((t, pe, ye))

    sc.pos_rms_m = math.sqrt(pos_sq / n)
    sc.yaw_rms_deg = math.sqrt(yaw_sq / n)
    sc.depth_rms_m = math.sqrt(dep_sq / n)
    sc.vel_rms_ms = math.sqrt(vel_sq / n)
    sc.accepted, sc.rejected = f.accepted, f.rejected
    return sc


if __name__ == '__main__':
    print('full aiding      ', run())
    print('no flow          ', run(use_flow=False))
    print('no depth         ', run(use_depth=False))
    print('no yaw           ', run(use_yaw=False))
    print('IMU only         ', run(use_flow=False, use_depth=False, use_yaw=False))
