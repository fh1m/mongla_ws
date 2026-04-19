#!/usr/bin/env python3
"""Square-pattern mission -- the classic Duburi shakedown.

Mission (square pattern at surface):
  arm -> MANUAL -> set_depth 0.0
    -> forward 5s -> yaw_right 90
    -> forward 5s -> yaw_right 90
    -> forward 5s -> yaw_right 90
    -> forward 5s -> disarm

Each `client.send(...)` blocks until the previous command's settle
phase has elapsed -- separation of concerns is enforced by the action
server's single-flight `lock`. We layer an explicit `pause` between
the turn and the next forward to drain residual yaw inertia from the
ArduSub heading-hold integrator (see .claude/context/axis-isolation.md).
"""


def _step(client, log, label, cmd, **fields):
    """Send one command and log its result. Failures propagate."""
    result = client.send(cmd, **fields)
    log.info(
        f'  -> {label}: final={result.final_value:+.3f}  '
        f'err={result.error_value:+.3f}  ({result.message})')


def run(client, log):
    log.info('-' * 52)
    log.info(' SQUARE PATTERN -- 4 legs of 5 s each')
    log.info('-' * 52)

    _step(client, log, 'arm',       'arm')
    _step(client, log, 'MANUAL',    'set_mode', target_name='MANUAL')
    _step(client, log, 'depth 0.0', 'set_depth', target=-0.0, timeout=40.0)

    for leg in range(1, 5):
        _step(client, log, f'leg {leg} fwd', 'move_forward',
              duration=5.0, gain=60.0)
        if leg < 4:
            _step(client, log, f'leg {leg} pause', 'pause', duration=1.0)
            _step(client, log, f'leg {leg} turn', 'yaw_right', target=90.0)
            _step(client, log, f'leg {leg} pause', 'pause', duration=1.0)

    _step(client, log, 'disarm', 'disarm')
