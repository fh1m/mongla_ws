"""FlightController HAL -- the autopilot-backend boundary (pixhawk | srot).

Pure modules (`base`, `srot_protocol`) import eagerly; the pymavlink-dependent
backends are built lazily by the factory, so `from duburi_control.fc import
srot_protocol` stays import-light.
"""

from .base import (FlightController, Telemetry, MoveResult, FireResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED,
                   FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED, FIRE_DENIED,
                   FIRE_NO_ACK, FIRE_NOT_READY, FIRE_BUSY)
from .factory import make_flight_controller, BUILDERS

__all__ = [
    'FlightController', 'Telemetry', 'MoveResult', 'FireResult',
    'SUCCEEDED', 'PREEMPTED', 'FAILED', 'DENIED', 'TIMEOUT', 'ABORTED',
    'FIRE_FIRED', 'FIRE_REJECTED_ARM', 'FIRE_DISABLED', 'FIRE_DENIED',
    'FIRE_NO_ACK', 'FIRE_NOT_READY', 'FIRE_BUSY',
    'make_flight_controller', 'BUILDERS',
]
