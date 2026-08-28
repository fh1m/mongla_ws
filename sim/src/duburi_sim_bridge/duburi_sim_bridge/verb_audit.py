#!/usr/bin/env python3
"""Measure what each /duburi/move verb PHYSICALLY does, against ground truth.

The stack cannot referee itself. Two defects found this way were invisible to
every unit test, because in both cases the telemetry the verb reports on is the
thing that was wrong:

  surface()          failed while the vehicle HAD surfaced (AHRS2 depth is
                     offset ~0.33 m at the surface, so it never reads 0.00)
  move_forward_dist  drove 2.361 m for a 1.0 m command and reported
                     "OK ... completed" (no DVL, silent open-loop fallback)

So every number here comes from `/duburi/sim/ground_truth`, never from the
verb's own result.

FRAME CONVENTION -- read before adding a check. ROS REP-103 body frame is
x FORWARD, y PORT, z UP. An earlier pass wrongly flagged move_left/move_right as
inverted because the harness had labelled +y starboard. `move_right` emits
Ch6 = 1720 (>1500 = starboard, the pool-verified hull polarity) and moves to
NEGATIVE body y. That is correct.

YAW DIRECTION CANNOT BE CHECKED THIS WAY. The hull carries angular momentum
through the settle and wraps past +/-180 deg, so the measured sign flips between
runs. Verify yaw sense from the RC PWM instead (Ch4 > 1500 = RIGHT).

    ros2 run duburi_sim_bridge verb_audit            # all groups
    ros2 run duburi_sim_bridge verb_audit --group yaw
"""
import argparse
import json
import math
import subprocess
import sys
import time

WS = '/home/fh1m/Envs/dockers/auv-ros2/Ros_workspaces/duburi_ws'
SRC = (f'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
       f'source {WS}/install/setup.bash >/dev/null 2>&1; '
       f'source {WS}/sim/install/setup.bash >/dev/null 2>&1; ')


def sh(cmd, timeout=300):
    return subprocess.run(['bash', '-c', cmd], capture_output=True,
                          text=True, timeout=timeout).stdout


def gt():
    """(x, y, z, yaw_deg) from ground truth. yaw CCW-positive (ENU)."""
    for _ in range(4):
        out = sh(SRC + 'timeout 15 ros2 topic echo /duburi/sim/ground_truth --once 2>/dev/null')
        try:
            pos = out.split('position:')[1].split('orientation:')[0]
            ori = out.split('orientation:')[1].split('\n\n')[0]
            g = lambda b, k: float([l for l in b.splitlines()
                                    if l.strip().startswith(k + ':')][0].split(':')[1])
            qx, qy, qz, qw = g(ori, 'x'), g(ori, 'y'), g(ori, 'z'), g(ori, 'w')
            yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                          1 - 2 * (qy * qy + qz * qz)))
            return g(pos, 'x'), g(pos, 'y'), g(pos, 'z'), yaw
        except Exception:
            time.sleep(1)
    return None


def verb(v, args='', timeout=200):
    out = sh(SRC + f'timeout {timeout} ros2 run duburi_planner duburi {v} {args} 2>&1',
             timeout + 30)
    ok = '-> OK' in out
    msg = ''
    for line in out.splitlines():
        if 'msg=' in line:
            msg = line.split('msg=')[-1].strip().strip('"')
    return ok, msg, out


def body(dx, dy, yaw_deg):
    """World delta -> body frame. Returns (forward, port)."""
    r = math.radians(yaw_deg)
    return dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r)


RESULTS = []


def record(name, verdict, detail):
    RESULTS.append({'verb': name, 'verdict': verdict, 'detail': detail})
    print(f'  {verdict:9s} {name:20s} {detail}')


def measure(name, args='', timeout=200, settle=4.0):
    """Run a verb, return (ok, msg, forward_m, port_m, dz_m, dyaw_deg)."""
    p0 = gt()
    ok, msg, _ = verb(name, args, timeout)
    time.sleep(settle)
    p1 = gt()
    if not (p0 and p1):
        return ok, msg, None, None, None, None
    fwd, port = body(p1[0] - p0[0], p1[1] - p0[1], p0[3])
    dyaw = (p1[3] - p0[3] + 180) % 360 - 180
    return ok, msg, fwd, port, p1[2] - p0[2], dyaw


# ---------------------------------------------------------------- groups ----

def group_motion():
    """arc: curved path -- forward travel AND a heading change together."""
    ok, msg, fwd, port, dz, dyaw = measure('arc', '--duration 6 --gain 55 --target_yaw 60')
    if fwd is None:
        record('arc', 'NO-GT', 'no ground truth')
    elif abs(fwd) > 0.3 and abs(dyaw) > 5.0:
        record('arc', 'OK', f'curved: fwd {fwd:+.2f} m, yaw {dyaw:+.1f} deg, ok={ok}')
    else:
        record('arc', 'SUSPECT',
               f'expected forward AND turn; got fwd {fwd:+.2f} m yaw {dyaw:+.1f} deg (ok={ok}) -- {msg}')

    # style_roll / style_yaw: showpiece manoeuvres, should move something.
    for v in ('style_roll', 'style_yaw'):
        ok, msg, fwd, port, dz, dyaw = measure(v, '', timeout=180)
        if fwd is None:
            record(v, 'NO-GT', 'no ground truth')
        else:
            moved = abs(dyaw) > 5.0 or abs(dz) > 0.05 or abs(fwd) > 0.1 or abs(port) > 0.1
            record(v, 'OK' if moved else 'SUSPECT',
                   f'yaw {dyaw:+.1f} deg, dz {dz:+.2f} m, fwd {fwd:+.2f} m (ok={ok})')


def group_heading():
    """turn: absolute heading. lock_heading: must hold UNDER DISTURBANCE."""
    ok, msg, fwd, port, dz, dyaw = measure('turn', '--target 45', timeout=140)
    record('turn', 'OK' if (dyaw is not None and abs(dyaw) > 10) else 'SUSPECT',
           f'commanded 45 deg absolute, gt moved {dyaw:+.1f} deg (ok={ok})'
           if dyaw is not None else 'no ground truth')

    # A lock only tested at rest is untested: command lateral thrust and see
    # whether the heading actually holds against it.
    verb('lock_heading', '--target 0 --timeout 120')
    p0 = gt()
    verb('move_right', '--duration 5 --gain 60')
    time.sleep(4)
    p1 = gt()
    if p0 and p1:
        drift = (p1[3] - p0[3] + 180) % 360 - 180
        record('lock_heading', 'OK' if abs(drift) < 15 else 'SUSPECT',
               f'heading drift under lateral thrust: {drift:+.1f} deg (want ~0)')
    else:
        record('lock_heading', 'NO-GT', 'no ground truth')

    ok, msg, _, _, _, _ = measure('unlock_heading', '', settle=1.0)
    record('unlock_heading', 'OK' if ok else 'SUSPECT', f'ok={ok} -- {msg}')


def group_stateful():
    """Verbs with no direct physical signature -- prove them another way."""
    for mode in ('ALT_HOLD', 'MANUAL'):
        ok, msg, _ = verb('set_mode', f'--target_name {mode}', timeout=90)
        time.sleep(2)
        out = sh(SRC + 'timeout 15 ros2 topic echo /duburi/state --once 2>/dev/null')
        got = ''
        for line in out.splitlines():
            if line.startswith('mode:'):
                got = line.split(':', 1)[1].strip()
        record(f'set_mode {mode}', 'OK' if got == mode else 'SUSPECT',
               f'/duburi/state reports {got!r} (ok={ok})')

    # `head` takes NO arguments -- it reads the live heading into final_value.
    # So the real check is not "did it run" but "does it agree with truth".
    p = gt()
    _, _, out = verb('head', '', timeout=90)
    reported = None
    for line in out.splitlines():
        if 'final=' in line:
            try:
                reported = float(line.split('final=')[1].split()[0])
            except ValueError:
                pass
    if p is None or reported is None:
        record('head', 'SUSPECT', f'could not read a heading back: {out.strip()[-70:]}')
    else:
        # The stack's heading convention is compass-style (CW positive); ground
        # truth is ENU (CCW positive). Compare magnitudes of the wrapped delta
        # against BOTH conventions and take the better -- the sign convention is
        # a documented difference, not an error.
        gt_yaw = p[3]
        d_same = abs((reported - gt_yaw + 180) % 360 - 180)
        d_flip = abs((reported + gt_yaw + 180) % 360 - 180)
        best = min(d_same, d_flip)
        record('head', 'OK' if best < 10 else 'SUSPECT',
               f'reported {reported:.1f} deg vs ground truth {gt_yaw:.1f} deg '
               f'(best-convention error {best:.1f} deg)')


def group_absent_hw():
    """No DVL-less path and no ESP32 in sim: must fail LOUDLY, not silently."""
    # These now refuse without a DVL; with sim_dvl they should work.
    for v, a in (('move_back_dist', '--distance_m 1.0 --gain 55'),
                 ('move_lateral_dist', '--distance_m 1.0 --gain 40')):
        ok, msg, fwd, port, dz, dyaw = measure(v, a, timeout=180)
        if fwd is None:
            record(v, 'NO-GT', f'ok={ok} -- {msg}')
            continue
        travelled = abs(fwd) if 'back' in v else abs(port)
        record(v, 'OK' if ok and travelled > 0.3 else 'SUSPECT',
               f'gt travelled {travelled:.2f} m of 1.00 (ok={ok}) -- {msg}')

    ok, msg, out = verb('fire', '--fire_channel 1', timeout=90)
    record('fire', 'OK' if not ok else 'SUSPECT',
           f'no payload in sim; ok={ok} (want False) -- {msg or out.strip().splitlines()[-1][:70]}')


def group_vision():
    """Never-fail contract: these return outcome codes, they do not raise."""
    for v, a in (('vision_align', '--camera forward --target_class gate --axes lat --duration 8'),
                 ('vision_move', '--camera forward --target_class gate --fwd_fill 80 --duration 8')):
        ok, msg, _ = verb(v, a, timeout=90)
        record(v, 'OK' if ok else 'SUSPECT',
               f'never-fail contract holds (ok={ok}) -- {msg[:70]}')


GROUPS = {'motion': group_motion, 'heading': group_heading,
          'stateful': group_stateful, 'absent_hw': group_absent_hw,
          'vision': group_vision}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--group', action='append', choices=sorted(GROUPS))
    ap.add_argument('--json', default='')
    args = ap.parse_args()

    if gt() is None:
        sys.exit('no /duburi/sim/ground_truth -- is the sim running?')
    verb('arm')
    verb('set_depth', '--target -0.8', timeout=90)
    time.sleep(3)

    print('=== VERB PHYSICS AUDIT (ground truth) ===\n')
    for name in (args.group or sorted(GROUPS)):
        print(f'-- {name} --')
        try:
            GROUPS[name]()
        except Exception as exc:                       # keep going; report it
            record(name, 'ERROR', f'{type(exc).__name__}: {exc}')
        print()

    verb('disarm')
    bad = [r for r in RESULTS if r['verdict'] != 'OK']
    print(f'=== {len(RESULTS) - len(bad)}/{len(RESULTS)} OK ===')
    for r in bad:
        print(f"  {r['verdict']:9s} {r['verb']}: {r['detail']}")
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(RESULTS, fh, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
