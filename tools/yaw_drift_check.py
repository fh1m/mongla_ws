"""Yaw drift with an OBJECTIVE stillness gate.

⛔ WHY THIS EXISTS. Every previous drift measurement in this project trusted
"the board is on a bench" as evidence that the board was STILL. It is not
evidence, and it has now been wrong twice: round 26's "+51.9 deg/min" and this
round's "+38.4 deg/min" were both a hull being handled. The operator has just
told us the board was NOT still for the three captures I reported as
"motionless", so those are contaminated too.

The board publishes its own body rates at 50 Hz inside ATTITUDE (fw
mav_stream.cpp:226 packs s.gx/gy/gz). So stillness is MEASURABLE and never has
to be assumed again. This tool:

  * records yaw AND gyro together,
  * reports the gyro RMS so the reader can see how still it actually was,
  * and fits the drift ONLY over samples where the board was quiescent,
    reporting what fraction of the run qualified.

A run that was mostly not-quiet reports that fact instead of a number.
"""
import sys, time, math, json
import rclpy
from rclpy.node import Node
from duburi_interfaces.msg import DuburiState
from geometry_msgs.msg import Vector3Stamped

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 480.0
# Quiescence threshold, rad/s. The firmware's own MOTOR_DETECT settle gate uses
# |gyro| < 0.15 rad/s as "quiet" (calibration.cpp:332); a bench board should be
# far below that, so this is deliberately tight.
QUIET_RADS = 0.02


class W(Node):
    def __init__(self):
        super().__init__('yaw_drift3')
        self.y = []          # (t, yaw_deg)
        self.g = []          # (t, |omega|)
        self.t0 = time.monotonic()
        self.create_subscription(DuburiState, '/duburi/state', self.cb_y, 10)
        self.create_subscription(Vector3Stamped, '/duburi/imu_rates', self.cb_g, 20)

    def cb_y(self, m):
        v = float(m.yaw_deg)
        if not math.isnan(v):
            self.y.append((time.monotonic() - self.t0, v))

    def cb_g(self, m):
        w = math.sqrt(m.vector.x**2 + m.vector.y**2 + m.vector.z**2)
        self.g.append((time.monotonic() - self.t0, w))


def unwrap(v):
    o = [v[0]]
    for x in v[1:]:
        d = x - o[-1]
        while d > 180: d -= 360
        while d < -180: d += 360
        o.append(o[-1] + d)
    return o


def fit(t, y):
    n = len(t)
    if n < 20:
        return None, None
    mt = sum(t)/n; my = sum(y)/n
    den = sum((a-mt)**2 for a in t)
    if den <= 0:
        return None, None
    sl = sum((t[i]-mt)*(y[i]-my) for i in range(n))/den
    r = [y[i]-(my+sl*(t[i]-mt)) for i in range(n)]
    return sl*60, math.sqrt(sum(x*x for x in r)/n)


def stillness_verdict(g_rms, g_max, quiet_frac, *, quiet_rads=QUIET_RADS):
    """('STILL'|'NOT STILL'|'UNKNOWN', text) from measured gyro statistics.

    Pure, so the decision can be tested with no board. `None` anywhere means the
    stillness check did not run, and that is UNKNOWN -- never STILL. Absence of a
    stillness check is not evidence of stillness, which is the exact mistake this
    tool exists to stop.
    """
    if g_rms is None or g_max is None or quiet_frac is None:
        return ('UNKNOWN',
                'no gyro samples -- stillness was not measured. Absence of a check is '
                'not evidence of stillness; refuse to report a drift.')
    if quiet_frac > 0.98 and g_max < 0.10:
        return ('STILL',
                f'quiescent ({quiet_frac*100:.1f}% of samples under {quiet_rads} rad/s, '
                f'max {g_max:.4f}) -- the number is about the SENSOR.')
    return ('NOT STILL',
            f'the board MOVED during this capture (only {quiet_frac*100:.1f}% quiet, '
            f'max {g_max:.4f} rad/s). Treat any drift or wander from it as contaminated.')


rclpy.init(); n = W(); end = time.monotonic() + DUR
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()

if len(n.y) < 50:
    print(f"INSUFFICIENT yaw samples: {len(n.y)}"); sys.exit(1)

ty = [a for a, _ in n.y]; yy = unwrap([b for _, b in n.y])

# --- stillness, MEASURED -------------------------------------------------- #
if not n.g:
    print("NO /duburi/imu_rates -- cannot verify stillness. REFUSING to report a drift.")
    print("Absence of a stillness check is not evidence of stillness.")
    sys.exit(3)
gw = [w for _, w in n.g]
g_rms = math.sqrt(sum(w*w for w in gw)/len(gw))
g_max = max(gw)
quiet_frac = sum(1 for w in gw if w < QUIET_RADS)/len(gw)

print(f"yaw samples {len(n.y)} over {ty[-1]-ty[0]:.0f} s   gyro samples {len(n.g)}")
print(f"GYRO   rms {g_rms:.5f} rad/s   max {g_max:.5f}   quiet(<{QUIET_RADS}) {quiet_frac*100:.1f}% of samples")

# Keep only yaw samples whose nearest gyro sample was quiet.
gi = sorted(n.g)
kt, ky = [], []
j = 0
for i, t in enumerate(ty):
    while j + 1 < len(gi) and gi[j+1][0] <= t:
        j += 1
    if gi and gi[j][1] < QUIET_RADS:
        kt.append(t); ky.append(yy[i])
kept = len(kt)/len(ty) if ty else 0

full, rms = fit(ty, yy)
qslope, qrms = fit(kt, ky)
print(f"ALL SAMPLES      drift {full:+.3f} deg/min   residual {rms:.3f}   p2p {max(yy)-min(yy):.3f} deg")
if qslope is None:
    print(f"QUIESCENT ONLY   too few quiet samples ({kept*100:.1f}% kept) -- NO DRIFT REPORTED")
else:
    print(f"QUIESCENT ONLY   drift {qslope:+.3f} deg/min   residual {qrms:.3f}   ({kept*100:.1f}% of samples kept)")
    print(f"                 p2p over quiet samples {max(ky)-min(ky):.3f} deg")

verdict, vtext = stillness_verdict(g_rms, g_max, quiet_frac)
print(f"VERDICT: {verdict} -- {vtext}")
print("JSON:" + json.dumps({"n_yaw": len(n.y), "dur": ty[-1]-ty[0], "g_rms": g_rms,
                            "g_max": g_max, "quiet_frac": quiet_frac, "kept": kept,
                            "full": full, "quiescent": qslope, "verdict": verdict,
                            "p2p_all": max(yy)-min(yy),
                            "p2p_quiet": (max(ky)-min(ky)) if ky else None}))
