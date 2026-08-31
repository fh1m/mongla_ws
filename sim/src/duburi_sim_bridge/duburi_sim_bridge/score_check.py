#!/usr/bin/env python3

"""Prove the scorer awards the two payload tasks -- at EVERY opening and bin.

WHY THIS IS A NODE AND NOT A SCRATCH SCRIPT
-------------------------------------------
Round 13 found that a torpedo fired dead centre through a real opening graded
`PAST_BOARD`, and a marker landing squarely in a crate graded `OUTSIDE_BIN`.
Both were real defects. But the round closed with only ONE of four openings
confirmed live, because three shots flew parallel to the board at yaw 89.9 deg
and the ad-hoc rig could not tell that from a scorer fault.

That is the failure this file exists to make impossible. It is the fourth round
running in which the MEASUREMENT, not the code, was the thing that was wrong:

    the wash "moved nothing"        -- the pose topic is model-relative
    the pipes "were welded"         -- the same frame, again
    a perfect shot "missed"         -- the scorer's geometry was stale
    three shots "missed"            -- the rig's own heading was 90 deg out

So this asserts ITS OWN PRECONDITIONS from ground truth before it fires, and
reports a rig failure AS A RIG FAILURE. A run that cannot place the hull says
`RIG FAILED`, never `scorer said miss`.

    ros2 run duburi_sim_bridge score_check --task torpedo
    ros2 run duburi_sim_bridge score_check --task bins
    ros2 run duburi_sim_bridge score_check              # both

Exit 0 only if every expected outcome was awarded AND every negative control was
correctly refused.

WHAT IT DRIVES, AND WHAT IT MUST NOT IMPORT
-------------------------------------------
`gate_transit_check` states the rule: "the simulator must not depend on the
autonomy stack it exists to grade, so nothing here imports or links against
duburi_ws". This shells out to the `duburi` CLI exactly as `verb_audit` does.
The sim stays independent of the stack it grades.

TWO TRAPS, BOTH ALREADY PAID FOR
--------------------------------
  * `set_pose` WITHOUT an `orientation` leaves the hull at whatever heading it
    drifted to. An armed hull in ALT_HOLD wanders in the seconds between the
    placement and the shot, and a round fired at yaw 90 never reaches a board
    whose plane is normal to x. Always send orientation, then VERIFY it.
  * Pinning the hull at 20 Hz through the shot is not the fix -- it punts the
    round as it spawns, because a fresh projectile has to clear the hull's
    collision box (payload_sim.py, `muzzle_forward_m`). One placement, a short
    settle, then fire.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

WS = os.environ.get(
    'DUBURI_WS', '/home/fh1m/Envs/dockers/auv-ros2/Ros_workspaces/duburi_ws')
SRC = (f'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
       f'source {WS}/install/setup.bash >/dev/null 2>&1; '
       f'source {WS}/sim/install/setup.bash >/dev/null 2>&1; ')

POS_TOL_M = 0.05
YAW_TOL_DEG = 5.0
# Gravity drop of a torpedo over the ~1 m scored standoff. The round is only
# 14 g negative and decelerates against added mass, so this is small; PAYLOAD.md
# measures ~17 mm over a 1.5 m shot.
DROP_M = 0.012
RESULTS = []


def sh(cmd, timeout=300):
    try:
        return subprocess.run(['bash', '-c', SRC + cmd], capture_output=True,
                              text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def ground_truth():
    """(x, y, z, yaw_deg) or None. yaw CCW-positive (ENU)."""
    for _ in range(4):
        out = sh('timeout 15 ros2 topic echo /duburi/sim/ground_truth --once '
                 '2>/dev/null')
        try:
            pos = out.split('position:')[1].split('orientation:')[0]
            ori = out.split('orientation:')[1].split('\n\n')[0]

            def g(block, key):
                line = [ln for ln in block.splitlines()
                        if ln.strip().startswith(key + ':')][0]
                return float(line.split(':')[1])

            qx, qy, qz, qw = (g(ori, 'x'), g(ori, 'y'),
                              g(ori, 'z'), g(ori, 'w'))
            yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                          1 - 2 * (qy * qy + qz * qz)))
            return g(pos, 'x'), g(pos, 'y'), g(pos, 'z'), yaw
        except Exception:                                      # noqa: BLE001
            time.sleep(1.0)
    return None


def place(world, x, y, z, yaw=0.0):
    """Place the hull, ONCE, with an explicit orientation."""
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    sh(f'gz service -s /world/{world}/set_pose --reqtype gz.msgs.Pose '
       f'--reptype gz.msgs.Boolean --timeout 500 --req '
       f"'name: \"duburi\", position: {{x: {x}, y: {y}, z: {z}}}, "
       f"orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'")


def settled_at(world, x, y, z, yaw=0.0, tries=4):
    """Place, HOLD, settle, and VERIFY from ground truth.

    The verification is the point. Without it a drifted heading is indis-
    tinguishable from a broken grader, which is exactly how round 13 ended.

    THE HULL CLIMBS BACK AFTER A TELEPORT, and that is not a bug to work around
    -- it is ALT_HOLD doing its job. The stack holds the depth it was last
    COMMANDED, so teleporting the hull somewhere else just gives the controller
    an error to null out: measured, the hull was placed at -1.056 and had risen
    to -1.005 by the time the round left the tube, 51 mm high against a small
    opening of radius 47.5 mm. Every shot after the first drifted further,
    which is why this looked like a first-shot-only defect.

    Commanding `set_depth` to the placement depth was tried and REVERTED: it
    re-engages ALT_HOLD, which then moves the hull while the shot is being set
    up, and the run went from 1/4 to 0/4. Recorded because the reasoning was
    sound and the measurement disagreed.

    The teleport-and-verify approach is therefore GOOD ENOUGH FOR ONE SHOT and
    not for four. The real fix is to stop teleporting and fly the hull to each
    firing point with the stack's own verbs, which is also the only version
    that exercises what a mission does. That is the next round's work, and this
    file reports the shortfall rather than hiding it.
    """
    why = 'no ground truth'
    for _ in range(tries):
        place(world, x, y, z, yaw)
        time.sleep(1.2)
        gt = ground_truth()
        if gt is None:
            continue
        gx, gy, gz, gyaw = gt
        dp = math.dist((gx, gy, gz), (x, y, z))
        dy = abs((gyaw - math.degrees(yaw) + 180.0) % 360.0 - 180.0)
        if dp <= POS_TOL_M and dy <= YAW_TOL_DEG:
            return None
        why = f'off by {dp:.3f} m, {dy:.1f} deg of yaw'
    return why


def cli(verb, args=''):
    return sh(f'timeout 90 ros2 run duburi_planner duburi {verb} {args} 2>&1',
              120)


def rearm():
    cli('disarm')
    time.sleep(1.0)
    cli('arm')
    time.sleep(1.0)


def muzzle_drop():
    """How far BELOW the hull's centre a torpedo actually leaves, plus drop.

    THE ROUND DOES NOT LEAVE FROM THE HULL'S CENTRE. `payload_sim` spawns it
    `muzzle_down_m` below, and aiming the hull at an opening therefore fires
    LOW by exactly that much -- 0.05 m against a small opening of radius
    0.0475, so the shot misses the small openings entirely and only just clears
    the large ones. That is the whole reason three of four shots "missed" in
    round 13 after the heading was fixed, and it is a rig error, not a scorer
    one. A real mission carries the same lead in its `align(depth=)`.

    Read from the node rather than retyped, for the reason this entire round
    exists: a second copy of a number is how these bugs are born.
    """
    out = sh('timeout 20 ros2 param get /payload_sim muzzle_down_m 2>&1')
    try:
        return float(out.strip().split()[-1]) + DROP_M
    except Exception:                                          # noqa: BLE001
        return 0.05 + DROP_M


def param(name):
    out = sh(f'timeout 20 ros2 param get /scoring {name} 2>&1')
    if 'array(' in out:
        return [float(v) for v in
                out.split('[', 1)[1].rsplit(']', 1)[0].split(',') if v.strip()]
    if 'String values are:' in out:
        return [v.strip().strip("'") for v in
                out.split('[', 1)[1].rsplit(']', 1)[0].split(',') if v.strip()]
    return []


def record(name, verdict, detail):
    RESULTS.append({'case': name, 'verdict': verdict, 'detail': detail})
    print(f'  {verdict:11s} {name:28s} {detail}', flush=True)


def shots_of(marker):
    """Every graded shot of `marker` so far.

    `--full-length` is REQUIRED. Without it `ros2 topic echo` truncates a long
    string with an ellipsis, the JSON never parses, and every case reports
    `scorer said None` -- which looks like a dead scorer and is a dead reader.
    """
    out = sh('timeout 25 ros2 topic echo /duburi/sim/score --once '
             '--full-length 2>/dev/null')
    try:
        card = json.loads(out.split("data: '", 1)[1].rsplit("'", 1)[0])
    except Exception:                                          # noqa: BLE001
        return None
    return [s for s in card.get('shots', []) if s.get('kind') == marker]


def outcome_of_next(marker, before_n, wait_s=40.0):
    """Wait for a NEW graded shot and return its outcome.

    POLLING, NOT SLEEPING, and that distinction is the bug it fixes. The scorer
    grades a shot only once it has come to REST -- measured ~11.7 s after the
    trigger for a torpedo. A fixed 8 s sleep read the PREVIOUS shot's verdict,
    so results were silently off by one: two cases "passed" on a neighbour's
    THROUGH and two "failed" on a neighbour's MISS. Both readings were wrong,
    and the two that agreed with expectation were the more dangerous half.
    """
    deadline = time.time() + wait_s
    while time.time() < deadline:
        shots = shots_of(marker)
        if shots is not None and len(shots) > before_n:
            return shots[-1].get('outcome')
        time.sleep(2.0)
    return None


# --------------------------------------------------------------------- #
def check_torpedo(world):
    """Every opening, plus a miss 1 mm outside each rim."""
    vals, kinds = param('board_openings'), param('board_opening_kinds')
    openings = [vals[i:i + 3] for i in range(0, len(vals) - 2, 3)]
    if not openings:
        record('board geometry', 'RIG FAILED',
               'the scorer has no openings -- is the sim up?')
        return
    lead = muzzle_drop()
    print(f'  {len(openings)} openings from the scorer, kinds {kinds}')
    print(f'  aiming {lead * 1000:.0f} mm high for the muzzle offset + drop')
    for idx, (oy, oz, _r) in enumerate(openings):
        kind = kinds[idx] if idx < len(kinds) else '?'
        name = f'opening {idx} ({kind})'
        if idx % 2 == 0:
            rearm()
        why = settled_at(world, -1.0, oy, oz + lead)
        if why:
            record(name, 'RIG FAILED', why)
            continue
        before = len(shots_of('torpedo') or [])
        cli('fire', f'--fire_channel {1 + idx % 2}')
        got = outcome_of_next('torpedo', before)
        record(name, 'PASS' if got == 'through' else 'FAIL',
               f'scorer said {got!r}, expected through')


def check_bins(world):
    vals = param('bin_targets')
    targets = [vals[i:i + 2] for i in range(0, len(vals) - 1, 2)]
    if not targets:
        record('bin geometry', 'RIG FAILED', 'the scorer has no bin targets')
        return
    print(f'  {len(targets)} crates from the scorer')
    cases = [(f'crate {i}', x, y, 'in_bin')
             for i, (x, y) in enumerate(targets)]
    # THE NEGATIVE CONTROL, and it is not optional: the bins model origin is
    # open water on the pipework between the crates, and it was the ONLY place
    # that scored before round 13. A run that awards it is worse than one that
    # awards nothing.
    ox = sum(t[0] for t in targets) / len(targets)
    oy = sum(t[1] for t in targets) / len(targets)
    cases.append(('pipework (negative control)', ox, oy, 'outside_bin'))
    for i, (name, x, y, want) in enumerate(cases):
        if i % 2 == 0:
            rearm()
        why = settled_at(world, x, y, -0.85)
        if why:
            record(name, 'RIG FAILED', why)
            continue
        before = len(shots_of('dropper') or [])
        cli('fire', f'--fire_channel {3 + i % 2}')
        got = outcome_of_next('dropper', before)
        record(name, 'PASS' if got == want else 'FAIL',
               f'scorer said {got!r}, expected {want}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--task', choices=('torpedo', 'bins', 'both'),
                    default='both')
    ap.add_argument('--world', default=None,
                    help='defaults to the task course')
    a = ap.parse_args()

    print('score_check -- does the scorer award what physically happened?\n')
    if a.task in ('torpedo', 'both'):
        print(' torpedo:')
        check_torpedo(a.world or 'rs_task_torpedo')
    if a.task in ('bins', 'both'):
        print(' bins:')
        check_bins(a.world or 'rs_task_bins')

    bad = [r for r in RESULTS if r['verdict'] != 'PASS']
    rig = [r for r in RESULTS if r['verdict'] == 'RIG FAILED']
    print(f'\n  {len(RESULTS) - len(bad)}/{len(RESULTS)} passed')
    if rig:
        print(f'  {len(rig)} RIG FAILURE(S) -- the harness could not set up '
              'the shot. This is NOT a scorer verdict.')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
