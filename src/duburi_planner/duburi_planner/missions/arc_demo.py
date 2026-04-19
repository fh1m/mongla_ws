#!/usr/bin/env python3
"""arc_demo -- side-by-side comparison of sharp vs curved turns.

Two figure-eights at the surface:
  Phase A (sharp): forward 4s -> stop -> yaw 90 -> stop -> forward 4s -> ...
                   (verifies axis-isolation contract -- each verb settles
                    cleanly before the next runs)
  Phase B (curved): arc(+yaw) -> arc(-yaw) -> arc(+yaw) -> arc(-yaw)
                   (verifies the Ch5 + Ch4 single-packet path)

Run: `ros2 run duburi_planner mission arc_demo`
"""


def _step(client, log, label, cmd, **fields):
    result = client.send(cmd, **fields)
    log.info(
        f'  -> {label}: final={result.final_value:+.3f}  '
        f'err={result.error_value:+.3f}  ({result.message})')


def run(client, log):
    log.info('-' * 52)
    log.info(' ARC DEMO -- sharp vs curved figure-eights')
    log.info('-' * 52)

    _step(client, log, 'arm',       'arm')
    _step(client, log, 'ALT_HOLD',  'set_mode', target_name='ALT_HOLD')
    _step(client, log, 'depth 0.0', 'set_depth', target=-0.0, timeout=40.0)

    # --- Phase A: sharp figure-eight ---------------------------------
    log.info('--- Phase A: sharp turns (yaw_*, settle between)')
    for i in range(2):
        sign = +1 if i == 0 else -1
        _step(client, log, f'A{i}-fwd', 'move_forward', duration=4.0, gain=60.0)
        _step(client, log, f'A{i}-pause', 'pause', duration=1.5)
        _step(client, log, f'A{i}-turn',
              'yaw_right' if sign > 0 else 'yaw_left', target=90.0)
        _step(client, log, f'A{i}-pause', 'pause', duration=1.0)

    _step(client, log, 'A-fwd-final', 'move_forward', duration=4.0, gain=60.0,
          settle=2.0)

    # --- Phase B: curved figure-eight --------------------------------
    log.info('--- Phase B: curved turns (arc, no settle between)')
    for i in range(2):
        rate = 30.0 if i % 2 == 0 else -30.0
        _step(client, log, f'B{i}-arc',
              'arc', duration=4.0, gain=50.0, yaw_rate_pct=rate)

    _step(client, log, 'disarm', 'disarm')
