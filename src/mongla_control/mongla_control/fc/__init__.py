"""The board boundary: the SROT board's driver (`srot_fc`), its frozen wire
constants (`srot_protocol`) and the result types every verb returns (`base`).

`srot_fc` imports pymavlink, so it is NOT imported here: `from mongla_control.fc
import srot_protocol` stays import-light for the tools that only need constants.
"""

from .base import (Telemetry, MoveResult, FireResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED,
                   FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED, FIRE_DENIED,
                   FIRE_NO_ACK, FIRE_NOT_READY, FIRE_BUSY)

__all__ = [
    'Telemetry', 'MoveResult', 'FireResult',
    'SUCCEEDED', 'PREEMPTED', 'FAILED', 'DENIED', 'TIMEOUT', 'ABORTED',
    'FIRE_FIRED', 'FIRE_REJECTED_ARM', 'FIRE_DISABLED', 'FIRE_DENIED',
    'FIRE_NO_ACK', 'FIRE_NOT_READY', 'FIRE_BUSY',
]
