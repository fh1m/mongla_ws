#!/usr/bin/env python3
"""heading_lock_demo -- show off continuous heading-hold during a square.

Locks the current heading, then runs a strafing square so the lock
thread is the only thing keeping the bow pointed forward. Switches
the lock target via `yaw_left` mid-mission to demonstrate retargeting,
then unlocks and ends the mission.

Run: `ros2 run duburi_planner mission heading_lock_demo`
"""


def _step(client, log, label, cmd, **fields):
    result = client.send(cmd, **fields)
    log.info(
        f'  -> {label}: final={result.final_value:+.3f}  '
        f'err={result.error_value:+.3f}  ({result.message})')


def run(client, log):
    log.info('-' * 52)
    log.info(' HEADING LOCK DEMO -- strafe square w/ active heading-hold')
    log.info('-' * 52)

    _step(client, log, 'arm',       'arm')
    _step(client, log, 'ALT_HOLD',  'set_mode', target_name='ALT_HOLD')
    _step(client, log, 'depth 0.0', 'set_depth', target=-0.0, timeout=40.0)

    # Lock at current heading. timeout is the safety auto-release.
    _step(client, log, 'lock!', 'lock_heading', target=0.0, timeout=120.0)

    for leg in range(4):
        # Strafe square (move_forward + move_left/right). Forward and
        # lateral inject a yaw moment from offset thrust -- the lock
        # thread cancels it via SET_ATTITUDE_TARGET.
        _step(client, log, f'leg {leg} fwd',
              'move_forward', duration=3.0, gain=50.0)
        _step(client, log, f'leg {leg} strafe',
              'move_right' if leg % 2 == 0 else 'move_left',
              duration=3.0, gain=50.0)

    # Mid-mission retarget: yaw_left will retarget the lock to the new heading.
    _step(client, log, 'pivot 45 left', 'yaw_left',  target=45.0)
    _step(client, log, 'fwd post-pivot', 'move_forward',
          duration=3.0, gain=50.0)

    _step(client, log, 'unlock', 'unlock_heading')
    _step(client, log, 'disarm', 'disarm')
